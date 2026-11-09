enum ServoIndex
{
    Base,
    Shoulder,
    Elbow,
    Wrist_Tlt,
    Wrist_Rot,
    Gripper
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
{};

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
    {"Base",        0, 1500,-5},
    {"Shoulder",    1, 1500,1},
    {"Elbow",       4, 1500,5},
    {"Wrist_Tlt",  5, 1500,-5},
    {"Wrist_Rot",   8, 1500,0},
    {"Gripper",     9, 1500,0}
};
ArmData motion1(500, 1000);
RobotArm Arm1(servos1, motion1);
ServoData servos2[6] =
{
    {"Base",        16, 1500,0},
    {"Shoulder",    17, 1500,9},
    {"Elbow",       20, 1500,5},
    {"Wrist_Tlt",  21, 1500,5},
    {"Wrist_Rot",   24, 1500,-5},
    {"Gripper",     25, 1500,0}
};
ArmData motion2 (500, 500);
RobotArm Arm2(servos2, motion2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

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
                 "#%d P%d T%d \r ",
                 Arm1.servos[i].port,
                 (Arm1.servos[i].power + (map(Arm1.servos[i].offset,-90,90,-1000,1000)) ),
                 
                 Arm1.time);

        strcat(strcat(cmd, temp),"T1000\r");
    }

    /* Arm 2 servos */  
    for (int i = 0; i < 6; i++)
    {
        snprintf(temp, sizeof(temp),
                 "#%d P%d T%d \r ",
                 Arm2.servos[i].port,
                 (Arm2.servos[i].power + (map(Arm2.servos[i].offset,-90,90,-1000,1000)) ),
                 
                 Arm2.time);

        strcat(strcat(cmd, temp),"T1000\r");
    }


    /* Send command */  
    Serial.println(cmd);

}

void moveServo(RobotArm &arm, ServoIndex servo, int degree)
{   
    int pulse = map(degree, 0, 197, 500, 2500);
    arm.servos[servo].power = pulse;
    build();
}
void loop()
{
    build();
    delay(5000);
    moveServo(Arm2, Base, 120);
    moveServo(Arm2, Elbow, 45);
    moveServo(Arm1, Wrist_Rot, 45);

    
    /* Delay time */  
}