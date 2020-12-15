#include <Servo.h>
Servo joint_2;
Servo joint_3;
Servo grip;

int value;

int goalSet[4] = {60, 180, 10};
int initSet[4] = {0,0, 10};

void moveToGoal()
{
  for(int i = 0; i < 15; i++)
  {
    joint_2.write((60-0)/15*i);
    joint_3.write((180-0)/15*i);
//    grip.write((0-40)/15*i+40);
    delay(100);
  }
  delay(1000);
  grip.write(10);
}

void droff()
{
  for(int i = 0; i < 15; i++)
  {
    joint_2.write((60-0)/15*i);
    joint_3.write((180-0)/15*i);
//    grip.write((0-40)/15*i+40);
    delay(100);
  }
  delay(1000);
  grip.write(40);
}

void comeBack()
{
  for(int i = 0; i < 15; i++)
  {
    joint_2.write((0-60)/15*i+60);
    joint_3.write((0-180)/15*i+180);
//    grip.write((40-0)/15*i);
    delay(150);
  }
  delay(1000);
  grip.write(10);
}

void setup() {
  // put your setup code here, to run once:
  joint_2.attach(4);
  joint_3.attach(5);
  grip.attach(6);
  Serial.begin(9600);

  joint_2.write(0);
  joint_3.write(0);
  grip.write(40);
}

void loop() {
//  len = length(joint_2);
//  moveToGoal();
//  comeBack();

    if(Serial.available() > 0)
    {
        String cmd = Serial.readString();
        int a = cmd.toInt();
        switch (a)
        {
          case 1:
          {
            moveToGoal();
            break;
          }
          case 2:
          {
            comeBack();
            break;
          }
          case 3:
          {
            droff();
            break;
          }
        }
    }
    
}
