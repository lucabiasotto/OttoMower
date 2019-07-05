//#include "motors_utils.h"
#include "perimeter_utils.h"

#include <Wire.h> //need by MPU6050
#include <MPU6050.h>//MPU6050byLC24 library by http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html

MPU6050 mpu;

// Pitch, Roll and Yaw values
int pitch = 0;
int roll = 0;
float yaw = 0;
long lastLoopRunningMS = 0;


float frontDistance;
float leftDistance;
float rightDistance;

const float FRONT_LIMIT_CM = 20;
const float LEFT_LIMIT_CM = 5;
const float RIGHT_LIMIT_CM = 5;

const float TURN_LIMIT_CM = 40; // larghezza robot



void setup() {

  Serial.begin(9600);

  //front HC-SR04
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);

  //left HC-SR04
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);

  //riht HC-SR04
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);

  //right BTS7960
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  //left BTS7960
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  Serial.println("Initialize MPU6050");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);

  // Check settings
  checkSettings();
}

void loop() {

  long loopDelay = millis() - lastLoopRunningMS; // i need the loop delay for calculate the correct yaw
  if(lastLoopRunningMS == 0){
    loopDelay = 1;
  }
  lastLoopRunningMS = millis();
  

  /******************************
        GET MOWER POSITION
   ******************************/

  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (normGyro.ZAxis > 1 || normGyro.ZAxis < -1) {
    //normGyro.ZAxis /= 100; // /100 because the delay is 10ms

    normGyro.ZAxis /= (1000 / loopDelay);
    
    yaw += normGyro.ZAxis;
  }

  //Keep our angle between 0-359 degrees
  if (yaw < 0) {
    yaw += 360;
  } else if (yaw > 359) {
    yaw -= 360;
  }

  // print position, like a plane :)
  Serial.print("Delay = ");
  Serial.print(loopDelay);
  Serial.print("\tPitch = ");
  Serial.print(pitch);
  Serial.print("\tRoll = ");
  Serial.print(roll);
  Serial.print("\tYaw = ");
  Serial.print(yaw);
  Serial.println();




  /******************************
        CHECK IF MOWER CAN GO 
   ******************************/

  if (canGo()) {
    //mower can go straight
    forward(yaw);
    
  } else {
    //mower can't go straight
    stop();


    //il roboto è bloccato, lo faccio andare avanti e indietro finchè c'è spazio
    while (!canGo()) {

      if(frontDistance > FRONT_LIMIT_CM && leftDistance < LEFT_LIMIT_CM){
       // front is free, no space to left side

        //TODO
        //forward(50);
       
      }else if(frontDistance > FRONT_LIMIT_CM && rightDistance < RIGHT_LIMIT_CM){
      // front is free, no space to right side

        //TODO
        //forward(-50);
      
      }else if (leftDistance < TURN_LIMIT_CM && rightDistance < TURN_LIMIT_CM) {
        //no space left or right --> go back
        reverse();
      } else if (leftDistance < TURN_LIMIT_CM) {
        //no space left
        right();
      } else {
        //no space right
        left();
      }

      stop();
    } //end while


    //reset the angle
    yaw = 0; 
    lastLoopRunningMS = 0;
  }

  delay(10);
}



/**
   Check and log all settings
*/
void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
}



/**
 * Update mower perimet sensor and retur true if the robot can go straight
 */
boolean canGo(){

  frontDistance = getFrontDistance();
  leftDistance = getLeftDistance();
  rightDistance = getRightDistance();

  return frontDistance > FRONT_LIMIT_CM && leftDistance > LEFT_LIMIT_CM && rightDistance > RIGHT_LIMIT_CM;
}
