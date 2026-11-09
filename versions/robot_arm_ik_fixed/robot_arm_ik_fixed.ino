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

struct ServoData
{
    const char* name;  /*Position on Arm*/
    uint8_t port;      /*Connected Pin on SSC-32U*/
    uint16_t power;    /*Desired Pulse Width (muS)*/
    double offset;     /*Angle offset to standardize the position of the arms (deg, can be fractional)*/
};

struct ArmData
{
    uint32_t speed; /*Servo Movement Speed (muS/S)*/
    uint32_t time;  /*Time to move from a to b (muS)*/

    ArmData(uint32_t s, uint32_t t)
    {
        speed = s;
        time = t;
    }
};

struct ArmPos
{
    double radius;
    double height;
    double gamma;
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

/* Floating-point version of map() so fractional offsets (e.g. 12.5 deg)
   aren't truncated by Arduino's integer-only map(). */
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Builds and sends one SSC-32U command line for a single arm's 6 servos.
   Bounded with strncat + a buffer sized with real headroom, so it can
   never overflow regardless of port/pulse values. */
void sendArmCommand(RobotArm &arm)
{
    char cmd[128] = "";
    char temp[24];

    for (int i = 0; i < 6; i++)
    {
        int pulse = arm.servos[i].power +
                    (int)mapDouble(arm.servos[i].offset, -90, 90, -1000, 1000);

        snprintf(temp, sizeof(temp), "#%d P%d ", arm.servos[i].port, pulse);
        strncat(cmd, temp, sizeof(cmd) - strlen(cmd) - 1);
    }

    char timeStr[16];
    snprintf(timeStr, sizeof(timeStr), "T%lu\r", (unsigned long)arm.time);
    strncat(cmd, timeStr, sizeof(cmd) - strlen(cmd) - 1);

    SSC.print(cmd); // print(), not println() -- cmd already ends in the \r terminator
}

void build()
{
    sendArmCommand(Arm1);
    sendArmCommand(Arm2);
}

ArmPos fwdKine(RobotArm &arm)
{
    ArmPos pos;

    double theta_S = radians((double)map(arm.servos[Shoulder].power, 500, 2500, 0, 180));
    double theta_E = radians(180.0 - (double)map(arm.servos[Elbow].power, 500, 2500, 0, 180));
    double theta_W = radians((double)map(arm.servos[Wrist_Tlt].power, 500, 2500, 0, 180));

    double theta_1 = theta_S - PI / 2;
    double theta_2 = theta_1 + theta_E;
    double theta_3 = theta_2 + theta_W - PI / 2;

    double l1 = 14.5;  //cm
    double l2 = 18.25; //cm
    double l3 = 10.00; //cm

    pos.radius = l3 * cos(theta_3) + l2 * cos(theta_2) + l1 * cos(theta_1);
    pos.height = l3 * sin(theta_3) + l2 * sin(theta_2) + l1 * sin(theta_1);
    pos.gamma  = degrees(theta_3);

    return pos;
}

/* J[0] = d(radius)/d(theta_S, theta_E, theta_W)
   J[1] = d(height)/d(theta_S, theta_E, theta_W)
   J[2] = d(gamma) /d(theta_S, theta_E, theta_W)   (gamma = theta_3, cumulative)
   Rows now actually match these labels, and this always operates on the
   `arm` passed in (no more hardcoded Arm1). */
void computeJacobian(RobotArm &arm, double J[3][3])
{
    double l1 = 14.5;  //cm
    double l2 = 18.25; //cm
    double l3 = 10.00; //cm

    double theta_S = radians((double)map(arm.servos[Shoulder].power, 500, 2500, 0, 180));
    double theta_E = radians(180.0 - (double)map(arm.servos[Elbow].power, 500, 2500, 0, 180));
    double theta_W = radians((double)map(arm.servos[Wrist_Tlt].power, 500, 2500, 0, 180));

    double theta_1 = theta_S - PI / 2;
    double theta_2 = theta_1 + theta_E;
    double theta_3 = theta_2 + theta_W - PI / 2;

    // Row 0: radius
    J[0][0] = -l1 * sin(theta_1) - l2 * sin(theta_2) - l3 * sin(theta_3);
    J[0][1] = -l2 * sin(theta_2) - l3 * sin(theta_3);
    J[0][2] = -l3 * sin(theta_3);

    // Row 1: height
    J[1][0] = l1 * cos(theta_1) + l2 * cos(theta_2) + l3 * cos(theta_3);
    J[1][1] = l2 * cos(theta_2) + l3 * cos(theta_3);
    J[1][2] = l3 * cos(theta_3);

    // Row 2: gamma
    J[2][0] = 1;
    J[2][1] = 1;
    J[2][2] = 1;
}

void jacobianStep(RobotArm &arm, double targetRadius, double targetHeight, double targetGamma)
{
    ArmPos current = fwdKine(arm);

    double er = targetRadius - current.radius;         // cm
    double eh = targetHeight - current.height;         // cm
    double eg = radians(targetGamma - current.gamma);  // deg error -> radians, to match J's units

    double J[3][3];
    computeJacobian(arm, J);

    const double alpha = 0.01;

    double dqS = alpha * (J[0][0] * er + J[1][0] * eh + J[2][0] * eg);
    double dqE = alpha * (J[0][1] * er + J[1][1] * eh + J[2][1] * eg);
    double dqW = alpha * (J[0][2] * er + J[1][2] * eh + J[2][2] * eg);

    // Kept in RADIANS throughout -- only converted to degrees right at the servo boundary
    double theta_S = radians((double)map(arm.servos[Shoulder].power, 500, 2500, 0, 180));
    double theta_E = radians(180.0 - (double)map(arm.servos[Elbow].power, 500, 2500, 0, 180));
    double theta_W = radians((double)map(arm.servos[Wrist_Tlt].power, 500, 2500, 0, 180));

    theta_S += dqS;
    theta_E += dqE;
    theta_W += dqW;

    int shoulderDeg = (int)constrain(degrees(theta_S), 0, 180);
    int elbowDeg    = (int)constrain(180.0 - degrees(theta_E), 0, 180);
    int wristDeg    = (int)constrain(degrees(theta_W), 0, 180);

    moveServo(arm, Shoulder,  shoulderDeg);
    moveServo(arm, Elbow,     elbowDeg);
    moveServo(arm, Wrist_Tlt, wristDeg);
}

void invKine(RobotArm &arm, double radius, double height, double gamma)
{
    for (int i = 0; i < 50; i++)
    {
        jacobianStep(arm, radius, height, gamma);
    }
    build(); // one transmission of the final, converged pose
}

void moveServo(RobotArm &arm, ServoIndex servo, int degree)
{
    int pulse = map(degree, 0, 180, 500, 2500);
    arm.servos[servo].power = pulse; // just update the model; build() is responsible for sending
}

void setup()
{
    Serial.begin(9600);
    SSC.begin(9600);

    moveServo(Arm1, Base, 90);
    moveServo(Arm1, Shoulder, 90);
    moveServo(Arm1, Elbow, 90);
    moveServo(Arm1, Wrist_Tlt, 90);
    moveServo(Arm1, Wrist_Rot, 90);
    moveServo(Arm1, Gripper, 90);
    moveServo(Arm2, Base, 90);
    moveServo(Arm2, Shoulder, 90);
    moveServo(Arm2, Elbow, 90);
    moveServo(Arm2, Wrist_Tlt, 90);
    moveServo(Arm2, Wrist_Rot, 90);
    moveServo(Arm2, Gripper, 90);

    build(); // moveServo no longer auto-sends, so send the starting pose explicitly
}

void loop()
{
    if (Serial.available())
    {
        int armNum = Serial.parseInt();
        double radius = Serial.parseFloat();
        double height = Serial.parseFloat();
        double gamma = Serial.parseFloat();

        RobotArm *arm = nullptr;

        if (armNum == 1)
            arm = &Arm1;
        else if (armNum == 2)
            arm = &Arm2;

        if (arm != nullptr)
        {
            invKine(*arm, radius, height, gamma);
        }
    }

    build();
}
