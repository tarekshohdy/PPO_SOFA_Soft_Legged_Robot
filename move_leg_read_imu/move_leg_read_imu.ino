#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


/*Servo numbers*/
#define servo1 0
#define servo2 1
#define servo3 2
#define servo4 3
#define servo5 4
#define servo6 5
#define servo7 6
#define servo8 7
#define servo9 8

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

void Servo_2_Angle(char servo,int angle);

void Reset();
void read_imu();
void move_leg(int a1, int a2, int a3);

bool flag = false;

int a1, a2, a3;

int counter = 0;

void setup() {
  // put your setup code here, to run once:
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Wire.begin(PB3, PB10); // PB3 SDA, PB10 SCL
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  
  driver.begin();
  driver.setPWMFreq(50);

  Serial.begin(115200);
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    // Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    // Serial.println("MPU6050 connection successful");
  }
  
  /* Initializate and configure the DMP*/
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    // Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }


  move_leg(0, 0, 0);

  delay(1000);

  read_imu();

  Serial.print("data");
  Serial.print(",");
  Serial.print(a1);
  Serial.print(",");
  Serial.print(a2);
  Serial.print(",");
  Serial.print(a3);
  Serial.print(",");
  Serial.print(ypr[0] * RAD_TO_DEG);
  Serial.print(",");
  Serial.print(ypr[1] * RAD_TO_DEG);
  Serial.print(",");
  Serial.println(ypr[2] * RAD_TO_DEG);
}

void loop() {

  if (counter < 1000){
    a1 = rand() % 200;
    a2 = rand() % 200;
    a3 = rand() % 200;
    move_leg(a1, a2, a3);
    delay(1000);
    read_imu();
    
    Serial.print("data");
    Serial.print(",");
    Serial.print(a1);
    Serial.print(",");
    Serial.print(a2);
    Serial.print(",");
    Serial.print(a3);
    Serial.print(",");
    Serial.print(ypr[0] * RAD_TO_DEG);
    Serial.print(",");
    Serial.print(ypr[1] * RAD_TO_DEG);
    Serial.print(",");
    Serial.println(ypr[2] * RAD_TO_DEG);
    counter++;
  }
  else{
    Serial.println("D");
    return;
  }
}

void read_imu(){

  if (!DMPReady) return;

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /* Display Euler angles in degrees */
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    
    mpu.dmpGetGravity(&gravity, &q);

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    
    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Serial.print(ypr[0] * RAD_TO_DEG);
    // Serial.print(",");
    // Serial.print(ypr[1] * RAD_TO_DEG);
    // Serial.print(",");
    // Serial.println(ypr[2] * RAD_TO_DEG);
  }
}

void Servo_2_Angle(char servo,int angle) {
  int val;
  switch(servo) {
    case servo1:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo2:
      val = map(angle, 0, 200, 400, 2770);
      break;

    case servo3:
      val = map(angle, 0, 200, 800, 2770);
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
