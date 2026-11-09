#include <SoftwareSerial.h>

SoftwareSerial SSC(10,11); //Rx Tx
enum ServoIndex /* Converts the Servo Names to numbers 0-5*/
{
    Base,//0
    Shoulder,//1    
    Elbow,//2
    Wrist_Tlt,//3
    Wrist_Rot,//4
    Gripper//5
};
struct ServoData
{
    const char* name; /*Positon on Arm*/
    uint8_t port; /*Connected Pin on SSC-32U*/
    uint16_t power; /*Desired Pulse Width (muS)*/
    int offset; /* Angle offset to standardize the position of the arms*/
};
struct ArmData 
{
  uint32_t speed; /*Servo Movement Speed (muS/S)*/
  uint32_t time; /*Time to move from a to b (muS)*/

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
        for (int i = 0; i < 6; i++){ servos[i] = s[i];}   
    }

};

ServoData servos1[6] =
{
    {"Base",        0, 1500,5},
    {"Shoulder",    1, 1500,8},
    {"Elbow",       4, 1500,12.5},
    {"Wrist_Tlt",  5, 1500,0},
    {"Wrist_Rot",   8, 1500,0},
    {"Gripper",     9, 1500,0}
};
ArmData motion1(500, 1000);
RobotArm Arm1(servos1, motion1);
ServoData servos2[6] =
{
    {"Base",        16, 1500,10},
    {"Shoulder",    17, 1500,15},
    {"Elbow",       20, 1500,10},
    {"Wrist_Tlt",  21, 1500,10},
    {"Wrist_Rot",   24, 1500,0},
    {"Gripper",     25, 1500,0}
};
ArmData motion2 (500, 500);
RobotArm Arm2(servos2, motion2);



void build()
{
      /* Main command */ 
    char cmd[256] = "";
    /* Temp storage */   
    char temp[32];

    /* Arm 1 servos */
    for (int i = 0; i < 6; i++)
    {
        snprintf(temp, sizeof(temp),
                 "#%d P%d T%d \r \n",
                 Arm1.servos[i].port,
                 (Arm1.servos[i].power + (map(Arm1.servos[i].offset,-90,90,-1000,1000)) ),
                 
                 Arm1.time);

        strcat(strcat(cmd, temp),"T1000\r");
    }

    /* Arm 2 servos */  
    for (int i = 0; i < 6; i++)
    {
        snprintf(temp, sizeof(temp),
                 "#%d P%d T%d \r \n",
                 Arm2.servos[i].port,
                 (Arm2.servos[i].power + (map(Arm2.servos[i].offset,-90,90,-1000,1000)) ),
                 
                 Arm2.time);

        strcat(strcat(cmd, temp),"T1000\r");
    }


    /* Send command */  
    SSC.println(cmd);

}
ArmPos fwdKine(RobotArm &arm)
{
    ArmPos pos;

    double theta_B = radians((map(arm.servos[Base].power,500,2500, 0,180)));
    double theta_S = radians((map(arm.servos[Shoulder].power,500,2500, 0,180)));
    double theta_E = radians(180 - (map(arm.servos[Elbow].power,500,2500,0,180)));
    double theta_W = radians((map(arm.servos[Wrist_Tlt].power,500,2500 ,0,180)));

    double theta_1 = theta_S - PI/2;
    double theta_2 = theta_1 + theta_E ;
    double theta_3 = theta_2 + theta_W - PI/2;
    


    double l1 = 14.5; //cm
    double l2 = 18.25; //cm
    double l3 = 10.00; //cm
    
    double radius = (l3*cos(theta_3)+ l2*cos(theta_2) + l1*cos(theta_1));
    double height = (l3*sin(theta_3)+ l2*sin(theta_2) + l1*sin(theta_1 ));
    double gamma  = theta_3;
    
    pos.radius = radius;
    pos.height = height;
    pos.gamma  = degrees(theta_3);

    return pos;
}

void fwdKineMove (RobotArm &arm)
{

    double theta_B = radians((map(arm.servos[Base].power,500,2500, 0,180)));
    double theta_S = radians((map(arm.servos[Shoulder].power,500,2500, 0,180)));
    double theta_E = radians(180 - (map(arm.servos[Elbow].power,500,2500,0,180)));
    double theta_W = radians((map(arm.servos[Wrist_Tlt].power,500,2500 ,0,180)));

    double theta_1 = theta_S - PI/2;
    double theta_2 = theta_1 + theta_E ;
    double theta_3 = theta_2 + theta_W - PI/2;
    


    double l1 = 14.5; //cm
    double l2 = 18.25; //cm
    double l3 = 10.00; //cm
    
    double height = (l3*cos(theta_3)+ l2*cos(theta_2) + l1*cos(theta_1));
    double radius = (l3*sin(theta_3)+ l2*sin(theta_2) + l1*sin(theta_1 ));
    double gamma = theta_1 + theta_2+theta_3;
    
    Serial.print(radius);
    Serial.print("\n");
    Serial.print(height);
    Serial.print("\n\n");
    Serial.print(gamma);
    Serial.print("\n\n");
   
    
}

void computeJacobian(RobotArm &arm, double J[3][3])
{
    ArmPos current = fwdKine(Arm1);

    double l1 = 14.5; //cm
    double l2 = 18.25; //cm
    double l3 = 10.00; //cm

    double theta_B = radians((map(arm.servos[Base].power,500,2500, 0,180)));
    double theta_S = radians((map(arm.servos[Shoulder].power,500,2500, 0,180)));
    double theta_E = radians(180 - (map(arm.servos[Elbow].power,500,2500,0,180)));
    double theta_W = radians((map(arm.servos[Wrist_Tlt].power,500,2500 ,0,180)));

    double theta_1 = theta_S - PI/2;
    double theta_2 = theta_1 + theta_E;
    double theta_3 = theta_2 + theta_W - PI/2;

    //Row 1
    J[0][0] = l1*cos(theta_1) +l2*cos(theta_2) +l3*cos(theta_3);
    J[0][1] = l2*cos(theta_2) +l3*cos(theta_3);
    J[0][2] = l3*cos(theta_3);

    //Row 2
    J[1][0] = -l1*sin(theta_1) -l2*sin(theta_2) -l3*sin(theta_3);
    J[1][1] = -l2*sin(theta_2) -l3*sin(theta_3);
    J[1][2] = -l3*sin(theta_3);

    //Row 3
    J[2][0] = 1;
    J[2][1] = 1;
    J[2][2] = 1;

    
    //Convert to Sevo Angles
    int shoulderDeg = degrees(theta_S);
    int elbowDeg    = 180 - degrees(theta_E);
    int wristDeg    = degrees(theta_W);
}

void jacobianStep(RobotArm &arm, double targetRadius, double targetHeight, double targetGamma)
{
    ArmPos current = fwdKine(arm);

    double er = targetRadius - current.radius;
    double eh = targetHeight - current.height;
    double eg = targetGamma - current.gamma;

    double J[3][3];

    computeJacobian(arm, J);

    double alpha = 0.01;

    double dqS = alpha * (J[0][0]*er + J[1][0]*eh + J[2][0]*eg);
    double dqE = alpha * (J[0][1]*er + J[1][1]*eh + J[2][1]*eg);
    double dqW = alpha * (J[0][2]*er + J[1][2]*eh + J[2][2]*eg);

    double theta_B = radians((map(arm.servos[Base].power,500,2500, 0,180)));
    double theta_S = radians((map(arm.servos[Shoulder].power,500,2500, 0,180)));
    double theta_E = radians(180 - (map(arm.servos[Elbow].power,500,2500,0,180)));
    double theta_W = radians((map(arm.servos[Wrist_Tlt].power,500,2500 ,0,180)));

    theta_S += degrees(dqS);
    theta_E += degrees(dqE);
    theta_W += degrees(dqW);

    moveServo(arm, Shoulder, constrain(theta_S,0,180));
    moveServo(arm, Elbow, constrain(180-theta_E, 0, 180));
    moveServo(arm, Wrist_Tlt,constrain(theta_W,0,180));
}
void invKine( RobotArm &arm, double radius, double height, double gamma)
{
    for(int i=0;i<50;i++)
    {
        jacobianStep(arm, radius, height, gamma);
    }

    build();
}

void moveServo(RobotArm &arm, ServoIndex servo, int degree)
{   
    int pulse = map(degree, 0, 180, 500, 2500); /*Converts the Degrees 0-197 to teh power range of the servos*/
    arm.servos[servo].power = pulse; /*Update destination*/
    build(); /* move to the Destination*/
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SSC.begin(9600);

  //Starting setup
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