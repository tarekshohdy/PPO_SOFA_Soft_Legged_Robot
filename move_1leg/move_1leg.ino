#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// #define servo_min 90
// #define servo_max 670
#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servo7 6
#define servo8 7
#define servo9 8

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void Servo_2_Angle(char servo,int angle);

void Reset();
void move_leg(int a1, int a2, int a3);

bool flag = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(PB3, PB10); // PB3 SDA, PB10 SCL
  driver.begin();
  driver.setPWMFreq(50);
  Wire.setClock(400000);
      
  Reset();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Loop");
  // (rand() % 170)

  move_leg(20, 0, 0);
 
}







void Servo_2_Angle(char servo,int angle) {
  int val;
  switch(servo) {
    case servo1:
      val = map(angle, 0, 180, 400, 2770);
      break;

    case servo2:
      val = map(angle, 0, 180, 400, 2770);
      break;

    case servo3:
      val = map(angle, 0, 180, 800, 2770);
      break;

    case servo4:
      val = map(angle, 0, 180, 400, 2775); // 375, 2780
      break;

    case servo5:
      val = map(angle, 0, 180, 600, 2785);
      break;

    case servo6:
      val = map(angle, 0, 180, 400, 2770);
      break;

    case servo7:
      val = map(angle, 0, 180, 600, 2770);
      break;

    case servo8:
      val = map(angle, 0, 180, 400, 2785);
      break;

    case servo9:
      val = map(angle, 0, 180, 400, 2780);
      break;
  }
  driver.writeMicroseconds(servo, val);
}

void Reset() {
   for (int i = 0; i < 9; i++) {
     Servo_2_Angle(i, 0);
   }
}

void move_leg(int a1, int a2, int a3){
  Servo_2_Angle(servo4, a1);
  Servo_2_Angle(servo5, a2);
  Servo_2_Angle(servo6, a3);  
}
