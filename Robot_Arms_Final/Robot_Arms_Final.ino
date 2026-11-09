/*
  Robot_Arms_Final.ino
  Dual 6-DOF robotic arms driven through an SSC-32U servo controller.

  Hardware assumed by this sketch (change the tables below if yours differs):
    - Arduino pin 10 = RX, pin 11 = TX (SoftwareSerial) wired to the SSC-32U at 9600 baud
    - Servo channel numbers as listed in servos1[] and servos2[]
    - Arduino USB serial at 9600 baud for commands (Serial Monitor line ending must be
      "Newline" or "Both NL & CR")

  Serial command (one line per move):
      <arm> <radius> <height> <gamma>
      <arm>    1 or 2
      <radius> horizontal distance of the tool tip from the shoulder axis (cm)
      <height> vertical distance of the tool tip from the shoulder axis (cm)
      <gamma>  angle of the last link measured from the horizontal (degrees)
      example: 1 20 15 0

  Planar model used for the kinematics (angles in radians inside the code):
      theta_1 = theta_S - 90 deg                  link 1 (shoulder to elbow), L1
      theta_2 = theta_1 + theta_E                 link 2 (elbow to wrist),    L2
      theta_3 = theta_2 + theta_W - 90 deg        link 3 (wrist to tip),      L3
      radius  = L1 cos(theta_1) + L2 cos(theta_2) + L3 cos(theta_3)
      height  = L1 sin(theta_1) + L2 sin(theta_2) + L3 sin(theta_3)
      gamma   = theta_3
    theta_S is the shoulder servo angle, theta_W is the wrist tilt servo angle and
    theta_E = 180 deg - elbow servo angle. The base servo (azimuth), wrist rotation and
    gripper are not part of this planar IK. They stay at whatever moveServo() last set.
*/

#include <SoftwareSerial.h>

SoftwareSerial SSC(10, 11); //Rx Tx

enum ServoIndex /* Converts the Servo Names to numbers 0-5*/
{
    Base,        //0
    Shoulder,    //1
    Elbow,       //2
    Wrist_Tlt,   //3
    Wrist_Rot,   //4
    Gripper      //5
};

/* ---------- Arm geometry (cm) ---------- */
const double L1 = 14.5;   // shoulder to elbow
const double L2 = 18.25;  // elbow to wrist
const double L3 = 10.00;  // wrist to tool tip

/* ---------- SSC-32U pulse range (us) that maps to 0..180 deg ---------- */
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;

/* ---------- Inverse kinematics settings ---------- */
const int    IK_MAX_ITERATIONS = 100;
const double IK_POS_TOL_CM     = 0.10;  // stop when radius and height are this close (cm)
const double IK_ANGLE_TOL_DEG  = 0.50;  // ... and gamma is this close (deg)
const double IK_DAMPING        = 0.20;  // lambda in the damped least squares step
const double IK_MAX_STEP_RAD   = 0.50;  // largest joint change allowed in one step (about 29 deg)

struct ServoData
{
    const char* name;  /*Position on Arm*/
    uint8_t port;      /*Connected Pin on SSC-32U*/
    uint16_t power;    /*Desired Pulse Width (us)*/
    double offset;     /*Angle offset to standardize the position of the arms (deg, can be fractional)*/
};

struct ArmData
{
    uint32_t speed; /*Servo Movement Speed (us/s), not used by the commands yet*/
    uint32_t time;  /*Time to move from a to b (ms, the T value of the SSC-32U)*/

    ArmData(uint32_t s, uint32_t t)
    {
        speed = s;
        time = t;
    }
};

struct ArmPos
{
    double radius;  // cm
    double height;  // cm
    double gamma;   // deg
};

struct RobotArm
{
    ServoData servos[6];
    uint32_t speed;
    uint32_t time;
    RobotArm(const ServoData (&s)[6], const ArmData &m)
    {
        speed = m.speed;
        time = m.time;
        for (int i = 0; i < 6; i++) { servos[i] = s[i]; }
    }
};

ServoData servos1[6] =
{
    {"Base",       2,  1500, 5},
    {"Shoulder",   3,  1500, 8},
    {"Elbow",      4,  1500, 12.5},
    {"Wrist_Tlt",  5,  1500, 0},
    {"Wrist_Rot",  8,  1500, 0},
    {"Gripper",    9,  1500, 0}
};
ArmData motion1(500, 1000);
RobotArm Arm1(servos1, motion1);

ServoData servos2[6] =
{
    {"Base",       16, 1500, 10},
    {"Shoulder",   17, 1500, 15},
    {"Elbow",      20, 1500, 10},
    {"Wrist_Tlt",  21, 1500, 10},
    {"Wrist_Rot",  24, 1500, 0},
    {"Gripper",    25, 1500, 0}
};
ArmData motion2(500, 500);
RobotArm Arm2(servos2, motion2);

/* ====================================================================== */
/* Helpers                                                                */
/* ====================================================================== */

/* Floating-point version of map() so fractional values (e.g. a 12.5 deg offset)
   aren't truncated by Arduino's integer-only map(). */
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Brings an angle difference into -180..180 deg so that 359 deg and -1 deg count as 2 deg apart. */
double wrapDegrees(double d)
{
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

/* Servo angle (0..180 deg) currently stored in the model for one servo. */
double servoDegrees(const ServoData &s)
{
    return mapDouble(s.power, PULSE_MIN, PULSE_MAX, 0, 180);
}

/* Updates the model only. Nothing is sent to the SSC-32U until sendArmCommand()/build().
   Takes a double so IK results keep their fraction (the stored pulse has 1 us resolution,
   about 0.09 deg, instead of being rounded to whole degrees). Out-of-range angles are
   clamped to the servo range. */
void moveServo(RobotArm &arm, ServoIndex servo, double degree)
{
    degree = constrain(degree, 0.0, 180.0);
    arm.servos[servo].power = (uint16_t)(mapDouble(degree, 0, 180, PULSE_MIN, PULSE_MAX) + 0.5);
}

/* Reads the three IK joints out of the model, in RADIANS. */
void getJointAngles(const RobotArm &arm, double &theta_S, double &theta_E, double &theta_W)
{
    theta_S = radians(servoDegrees(arm.servos[Shoulder]));
    theta_E = radians(180.0 - servoDegrees(arm.servos[Elbow]));
    theta_W = radians(servoDegrees(arm.servos[Wrist_Tlt]));
}

/* Writes the three IK joints (RADIANS) back into the model as servo angles. */
void setJointAngles(RobotArm &arm, double theta_S, double theta_E, double theta_W)
{
    moveServo(arm, Shoulder,  degrees(theta_S));
    moveServo(arm, Elbow,     180.0 - degrees(theta_E));
    moveServo(arm, Wrist_Tlt, degrees(theta_W));
}

/* ====================================================================== */
/* Talking to the SSC-32U                                                 */
/* ====================================================================== */

/* Builds and sends ONE SSC-32U command line for a single arm's 6 servos, e.g.
       #2 P1555 #3 P1666 ... #9 P1500 T1000<CR>
   One line with one T value means all six servos of the arm start and finish together.
   Bounded with strncat and a buffer with real headroom, so it cannot overflow. */
void sendArmCommand(RobotArm &arm)
{
    char cmd[128] = "";
    char temp[24];

    for (int i = 0; i < 6; i++)
    {
        /* Pulse = model value + the offset converted to us (-90..90 deg is -1000..1000 us) */
        int pulse = arm.servos[i].power +
                    (int)floor(mapDouble(arm.servos[i].offset, -90, 90, -1000, 1000) + 0.5);
        pulse = constrain(pulse, PULSE_MIN, PULSE_MAX); // the SSC-32U only accepts 500..2500

        snprintf(temp, sizeof(temp), "#%d P%d ", arm.servos[i].port, pulse);
        strncat(cmd, temp, sizeof(cmd) - strlen(cmd) - 1);
    }

    char timeStr[16];
    snprintf(timeStr, sizeof(timeStr), "T%lu\r", (unsigned long)arm.time);
    strncat(cmd, timeStr, sizeof(cmd) - strlen(cmd) - 1);

    SSC.print(cmd); // print(), not println(), because cmd already ends in the \r terminator
}

/* Sends both arms (used once at start-up). */
void build()
{
    sendArmCommand(Arm1);
    sendArmCommand(Arm2);
}

/* ====================================================================== */
/* Forward kinematics and Jacobian                                        */
/* ====================================================================== */

ArmPos fwdKine(RobotArm &arm)
{
    ArmPos pos;

    double theta_S, theta_E, theta_W;
    getJointAngles(arm, theta_S, theta_E, theta_W);

    double theta_1 = theta_S - PI / 2;
    double theta_2 = theta_1 + theta_E;
    double theta_3 = theta_2 + theta_W - PI / 2;

    pos.radius = L3 * cos(theta_3) + L2 * cos(theta_2) + L1 * cos(theta_1);
    pos.height = L3 * sin(theta_3) + L2 * sin(theta_2) + L1 * sin(theta_1);
    pos.gamma  = degrees(theta_3);

    return pos;
}

/* J[0] = d(radius)/d(theta_S, theta_E, theta_W)   (cm per radian)
   J[1] = d(height)/d(theta_S, theta_E, theta_W)   (cm per radian)
   J[2] = d(gamma) /d(theta_S, theta_E, theta_W)   (gamma = theta_3, so every entry is 1) */
void computeJacobian(RobotArm &arm, double J[3][3])
{
    double theta_S, theta_E, theta_W;
    getJointAngles(arm, theta_S, theta_E, theta_W);

    double theta_1 = theta_S - PI / 2;
    double theta_2 = theta_1 + theta_E;
    double theta_3 = theta_2 + theta_W - PI / 2;

    // Row 0: radius
    J[0][0] = -L1 * sin(theta_1) - L2 * sin(theta_2) - L3 * sin(theta_3);
    J[0][1] = -L2 * sin(theta_2) - L3 * sin(theta_3);
    J[0][2] = -L3 * sin(theta_3);

    // Row 1: height
    J[1][0] = L1 * cos(theta_1) + L2 * cos(theta_2) + L3 * cos(theta_3);
    J[1][1] = L2 * cos(theta_2) + L3 * cos(theta_3);
    J[1][2] = L3 * cos(theta_3);

    // Row 2: gamma
    J[2][0] = 1;
    J[2][1] = 1;
    J[2][2] = 1;
}

/* ====================================================================== */
/* Inverse kinematics                                                     */
/* ====================================================================== */

/* Determinant of a 3x3 matrix. */
double det3(const double M[3][3])
{
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

/* Solves A x = b for a 3x3 system with Cramer's rule. Returns false if A is singular. */
bool solve3x3(const double A[3][3], const double b[3], double x[3])
{
    double det = det3(A);
    if (fabs(det) < 1e-9) return false;

    for (int c = 0; c < 3; c++)
    {
        double M[3][3];
        for (int r = 0; r < 3; r++)
            for (int k = 0; k < 3; k++)
                M[r][k] = (k == c) ? b[r] : A[r][k];   // column c replaced by b
        x[c] = det3(M) / det;
    }
    return true;
}

/* True when the arm model is within tolerance of the target pose. */
bool poseReached(const ArmPos &p, double radius, double height, double gamma)
{
    return fabs(radius - p.radius) <= IK_POS_TOL_CM &&
           fabs(height - p.height) <= IK_POS_TOL_CM &&
           fabs(wrapDegrees(gamma - p.gamma)) <= IK_ANGLE_TOL_DEG;
}

/* ONE damped least squares Jacobian step toward the target pose.

       error  e  = [radius error (cm), height error (cm), gamma error (rad)]
       step   dq = J^T (J J^T + lambda^2 I)^-1 e

   The plain "transpose" step dq = alpha J^T e only converges for very small alpha
   (below about 0.0007 for this arm), and slowly. The damped step takes a properly sized
   step and the lambda term keeps it from blowing up near a singular pose (elbow fully
   straight or fully folded). The result is applied to the model; moveServo() clamps it
   to the servo limits. */
void jacobianStep(RobotArm &arm, double targetRadius, double targetHeight, double targetGamma)
{
    ArmPos current = fwdKine(arm);

    double e[3];
    e[0] = targetRadius - current.radius;                          // cm
    e[1] = targetHeight - current.height;                          // cm
    e[2] = radians(wrapDegrees(targetGamma - current.gamma));      // deg error -> radians, to match J's units

    double J[3][3];
    computeJacobian(arm, J);

    double A[3][3];                                                // A = J J^T + lambda^2 I
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            A[i][j] = J[i][0] * J[j][0] + J[i][1] * J[j][1] + J[i][2] * J[j][2]
                    + ((i == j) ? IK_DAMPING * IK_DAMPING : 0.0);

    double y[3];
    if (!solve3x3(A, e, y)) return;                                // cannot happen while lambda > 0

    double dq[3];                                                  // dq = J^T y
    for (int k = 0; k < 3; k++)
        dq[k] = J[0][k] * y[0] + J[1][k] * y[1] + J[2][k] * y[2];

    double biggest = max(fabs(dq[0]), max(fabs(dq[1]), fabs(dq[2])));
    if (biggest > IK_MAX_STEP_RAD)                                 // keep each step small and safe
        for (int k = 0; k < 3; k++) dq[k] *= IK_MAX_STEP_RAD / biggest;

    // Kept in RADIANS throughout, only converted to degrees inside setJointAngles()
    double theta_S, theta_E, theta_W;
    getJointAngles(arm, theta_S, theta_E, theta_W);
    setJointAngles(arm, theta_S + dq[0], theta_E + dq[1], theta_W + dq[2]);
}

/* Repeats jacobianStep() until the pose is within tolerance. Returns false if it never gets there. */
bool refineWithJacobian(RobotArm &arm, double radius, double height, double gamma)
{
    for (int i = 0; i < IK_MAX_ITERATIONS; i++)
    {
        if (poseReached(fwdKine(arm), radius, height, gamma)) return true;
        jacobianStep(arm, radius, height, gamma);
    }
    return poseReached(fwdKine(arm), radius, height, gamma);
}

/* Closed-form starting guess. The wrist point is the tool tip pulled back along link 3.
   From the wrist point, links 1 and 2 are a standard two-link arm:

       cos(theta_E) = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2)
       theta_1      = atan2(y, x) - atan2(L2 sin(theta_E), L1 + L2 cos(theta_E))
       theta_W      = gamma - (theta_1 + theta_E) + 90 deg

   theta_E is taken between 0 and 180 deg, which is the elbow's mechanical range, so there is only
   one answer. Returns false if the wrist point is out of reach, meaning no joint angles can
   reach this target at all. */
bool seedFromClosedForm(RobotArm &arm, double radius, double height, double gamma)
{
    double g = radians(gamma);
    double x = radius - L3 * cos(g);   // wrist point
    double y = height - L3 * sin(g);

    double c = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (c > 1.0001 || c < -1.0001) return false;
    c = constrain(c, -1.0, 1.0);

    double theta_E = acos(c);
    double theta_1 = atan2(y, x) - atan2(L2 * sin(theta_E), L1 + L2 * cos(theta_E));
    theta_1 = radians(wrapDegrees(degrees(theta_1)));   // the difference of two atan2 results can fall outside -180..180 deg
    double theta_S = theta_1 + PI / 2;
    double theta_W = radians(wrapDegrees(degrees(g - (theta_1 + theta_E) + PI / 2)));

    setJointAngles(arm, theta_S, theta_E, theta_W);
    return true;
}

/* Moves the arm model to the requested tool pose and sends it to the SSC-32U.
   Works on a copy first, so if the target cannot be reached the arm is left exactly as it was
   and nothing is sent. Returns true on success. */
bool invKine(RobotArm &arm, double radius, double height, double gamma)
{
    RobotArm trial = arm;

    // 1) Closed-form guess. It is usually already inside the tolerance, and the Jacobian
    //    steps clean up whatever is left (servo pulse rounding, joint limits).
    if (!seedFromClosedForm(trial, radius, height, gamma)) return false;   // out of reach

    if (!refineWithJacobian(trial, radius, height, gamma))
    {
        // 2) Fallback: iterate from where the arm is right now.
        trial = arm;
        if (!refineWithJacobian(trial, radius, height, gamma)) return false;
    }

    arm = trial;
    sendArmCommand(arm); // one transmission of the final pose, for this arm only
    return true;
}

/* ====================================================================== */
/* Serial command handling                                                */
/* ====================================================================== */

const uint8_t LINE_BUF_SIZE = 48;
char lineBuf[LINE_BUF_SIZE];
uint8_t lineLen = 0;
bool lineOverflow = false;

/* Collects characters until the end of a line without ever waiting for more.
   (Serial.parseInt() and parseFloat() block for up to a second per value, and a leftover
   newline from the previous command made them stall again on the next pass.)
   Returns true when a complete line is ready in lineBuf. */
bool readLine()
{
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (lineLen == 0 && !lineOverflow) continue;   // blank line, or the second half of \r\n
            bool tooLong = lineOverflow;
            lineBuf[lineLen] = '\0';
            lineLen = 0;
            lineOverflow = false;
            if (tooLong) { Serial.println(F("Line too long, ignored")); continue; }
            return true;
        }

        if (lineLen < LINE_BUF_SIZE - 1) lineBuf[lineLen++] = c;
        else lineOverflow = true;
    }
    return false;
}

/* Parses "<arm> <radius> <height> <gamma>". Returns false on anything else. */
bool parseCommand(const char *line, int &armNum, double &radius, double &height, double &gamma)
{
    char *end;
    const char *start = line;

    armNum = (int)strtol(start, &end, 10);
    if (end == start) return false;

    double *values[3] = { &radius, &height, &gamma };
    for (int i = 0; i < 3; i++)
    {
        start = end;
        *values[i] = strtod(start, &end);
        if (end == start || isnan(*values[i]) || isinf(*values[i])) return false;
    }

    while (*end == ' ' || *end == '\t') end++;
    return *end == '\0';                                  // nothing extra after the 4th number
}

void setup()
{
    Serial.begin(9600);
    SSC.begin(9600);

    RobotArm *arms[2] = { &Arm1, &Arm2 };
    for (int a = 0; a < 2; a++)
        for (int i = 0; i < 6; i++)
            moveServo(*arms[a], (ServoIndex)i, 90);

    build(); // moveServo only updates the model, so send the starting pose explicitly

    Serial.println(F("Ready. Send: <arm 1-2> <radius cm> <height cm> <gamma deg>"));
}

void loop()
{
    if (!readLine()) return;

    int armNum;
    double radius, height, gamma;

    if (!parseCommand(lineBuf, armNum, radius, height, gamma))
    {
        Serial.println(F("Usage: <arm 1-2> <radius cm> <height cm> <gamma deg>"));
        return;
    }

    RobotArm *arm = nullptr;
    if (armNum == 1)      arm = &Arm1;
    else if (armNum == 2) arm = &Arm2;

    if (arm == nullptr)
    {
        Serial.println(F("Arm must be 1 or 2"));
        return;
    }

    if (invKine(*arm, radius, height, gamma))
    {
        ArmPos reached = fwdKine(*arm);
        Serial.print(F("OK arm "));
        Serial.print(armNum);
        Serial.print(F(": radius "));
        Serial.print(reached.radius, 2);
        Serial.print(F(" cm, height "));
        Serial.print(reached.height, 2);
        Serial.print(F(" cm, gamma "));
        Serial.print(reached.gamma, 1);
        Serial.println(F(" deg"));
    }
    else
    {
        Serial.println(F("Target not reachable, arm not moved"));
    }
}
