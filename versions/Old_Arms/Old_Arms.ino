
struct ServoData
{
    const char* name; /*Positon on Arm*/
    uint8_t port; /*Connected Pin on SSC-32U*/
    uint16_t power; /*Desired Pulse Width (muS)*/
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
    {"Base", 0, 500},
    {"Shoulder", 1, 1500},
    {"Elbow", 4, 1500},
    {"Wrist Tilt", 5, 1500},
    {"Wrist Rotation", 8, 1500},
    {"Gripper", 9, 1500}
};
ArmData motion1(200, 5000);
RobotArm Arm1(servos1, motion1);

ServoData servos2[6] =
{
    {"Base", 16, 2500},
    {"Shoulder", 17, 1500},
    {"Elbow", 20, 1500},
    {"Wrist Tilt", 21, 1500},
    {"Wrist Rotation", 24, 1500},
    {"Gripper", 25, 1500}
};
ArmData motion2 (200, 2000);
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
                 "#%dP%dS%dT%d\r ",
                 Arm1.servos[i].port,
                 Arm1.servos[i].power,
                 Arm1.speed,
                 Arm1.time);

        strcat(cmd, temp);
    }

    /* Arm 2 servos */  
    for (int i = 0; i < 6; i++)
    {
        snprintf(temp, sizeof(temp),
                 "#%dP%dS%dT%d\r ",
                 Arm2.servos[i].port,
                 Arm2.servos[i].power,
                 Arm1.speed,
                 Arm1.time);

        strcat(cmd, temp);
    }


    /* Send command */  
    Serial.println(cmd);

}
void loop()
{
    build();
    /* Delay time */  
    delay(1000);
    servos1[0].power = 500;
    build();
    delay(1000);
    servos1[0].power = 1500;
    build();
    delay(1000);
    servos1[0].power = 2500;
    build();
    delay(1000);
    servos1[0].power = 1500;
    Arm1.servos[0].power
}