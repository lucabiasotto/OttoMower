//#include "motors_utils.h"
#include "perimeter_utils.h"

#include <Wire.h> //need by MPU6050
#include <MPU6050.h>//MPU6050byLC24 library by https://github.com/jarzebski/Arduino-MPU6050

MPU6050 mpu;

//CAUTION!!! DISABLE_EMERGENCY_STOP only for DEBUG
const bool DISABLE_EMERGENCY_STOP = false; //shutdown the robot if it's locked or someone tilted him


const float FRONT_LIMIT_CM = 40;
const float LEFT_LIMIT_CM = 10;
const float RIGHT_LIMIT_CM = 10;
const float LEFT_LIMIT_FOR_DODGE_CM = 20;
const float RIGHT_LIMIT_FOR_DODGE_CM = 20;
const float DEGREE_FOR_DODGE = 45;
const float TURN_LIMIT_CM = 60; // robot width

// Pitch, Roll and Yaw values
int pitch = 0;
int roll = 0;
float yaw = 0;
long lastLoopRunningMS = 0;
const int PITCH_LIMIT = 10; //10 - pitch limit before shutdown
const int ROLL_LIMIT = 10; //10 - roll limit before shutdown
int tiltCount = 0;
const int TILT_LIMIT = 2;

float frontDistance;
float leftDistance;
float rightDistance;

long firstLockDetectionMS = 0;
const float LOCK_TOLLERANCE = 3.5; //tiene conto di eventuali vibrazioni, 3 è un po' troppo permissivo
const int LOCKED_LIMIT_MS = 1000; //if not move for this time try to free the robot
const int LOCKED_LIMIT_SHUTDOWN_MS = 5000 * 3; //if not move for this time shutdown all


const int BUZZER = 9;

unsigned int   resetCount __attribute__ ((section (".noinit")));


/**
   Function for reset arduino
*/
void(* reset)(void) = 0;

/*
   Setup the robot
*/
void setup() {

  Serial.begin(9600);

  pinMode(BUZZER, OUTPUT);


  //front HC-SR04
  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);

  //left HC-SR04
  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);

  //riht HC-SR04
  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);


  //left BTS7960
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  digitalWrite(LEFT_LPWM, LOW);
  digitalWrite(LEFT_RPWM, LOW);

  //right BTS7960
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  digitalWrite(RIGHT_LPWM, LOW);
  digitalWrite(RIGHT_RPWM, LOW);

  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

  Serial.println("Initialize MPU6050");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    digitalWrite(BUZZER, HIGH);
    delay(500);
    digitalWrite(BUZZER, LOW);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);

  // Check settings
  checkSettings();

  Vector normGyro = mpu.readNormalizeGyro();
  int zAxis = normGyro.ZAxis;


  if (resetCount == 1) {
    // 1 is the magic number used to flag when i'm after a reset
    Serial.println("AFTER RESET");
    digitalWrite(BUZZER, HIGH);
    delay(500);
    digitalWrite(BUZZER, LOW);
  } else {
    //i need to do a reset because sometimes the MPU6050 not works at first boot
    Serial.println("FIRST POWER UP");
    resetCount = 1;
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
    reset();
  }
  resetCount = 1;

}


/**
    Check and log all settings
*/
void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  if (mpu.getSleepEnabled()) {
    //invalid state
    digitalWrite(BUZZER, HIGH);
    delay(5000);
    reset();
  }

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

      if (mpu.getClockSource() != MPU6050_CLOCK_PLL_XGYRO) {
        //invalid state
        digitalWrite(BUZZER, HIGH);
        delay(5000);
        reset();
      }
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;

      if (mpu.getScale() != MPU6050_SCALE_2000DPS) {
        //invalid state
        digitalWrite(BUZZER, HIGH);
        delay(5000);
        reset();
      }
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
}


void loop() {

  long loopDelay = millis() - lastLoopRunningMS; // i need the loop delay for calculate the correct yaw
  if (lastLoopRunningMS == 0) {
    loopDelay = 1;
  }
  lastLoopRunningMS = millis();

  Serial.println("");

  /******************************
        GET MOWER POSITION
   ******************************/

  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  /*
    float startAccX = normAccel.XAxis;
    float startAccY = normAccel.YAxis;
    float startAccZ = normAccel.ZAxis;
  */

  // Calculate Pitch & Roll
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

  //Ignore the gyro if our angular velocity does not meet our threshold
  if (normGyro.ZAxis > 1 || normGyro.ZAxis < -1) {
    //normGyro.ZAxis /= 100; // /100 because the delay is 10ms

    normGyro.ZAxis = normGyro.ZAxis / (1000 / loopDelay);
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

  /*
    Serial.print("Acc X = ");
    Serial.print(normAccel.XAxis);
    Serial.print("\tAcc Y = ");
    Serial.print(normAccel.YAxis);
    Serial.print("\tAcc Z = ");
    Serial.print(normAccel.ZAxis);
    Serial.println();
  */



  /******************************
        CHECK IF MOWER CAN GO
   ******************************/

  if (pitch < -PITCH_LIMIT || pitch > PITCH_LIMIT || roll > ROLL_LIMIT || roll < -ROLL_LIMIT ) {
    //robot is tilt

    tiltCount++;
    //TODO stop() stopBlades;
    int stopMS = 500;
    stop(stopMS);
    Serial.println("!!!! TILT DETECTED !!!!");
    firstLockDetectionMS = 0; //when tilted the robot is stopped, i need to reset lock detection
    if(firstLockDetectionMS > 0){
      firstLockDetectionMS = firstLockDetectionMS - stopMS;
    }
  } else {
    tiltCount = 0;
  }


  if (tiltCount >= TILT_LIMIT && !DISABLE_EMERGENCY_STOP) {
    //robot is in tilt more time consecutively...shoutdown
    //TODO stop() stopBlades;
    stop(1);
    Serial.println("");
    Serial.println("!!!! MANY TILT DETECTED --> SHUTDOWN !!!!");
    Serial.println("");

    for (int i = 0; i < 10; i++) {
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);
      delay(100);
    }

    turnOffRobot();
  }

  if (firstLockDetectionMS > 0 &&  millis() - firstLockDetectionMS > LOCKED_LIMIT_SHUTDOWN_MS && !DISABLE_EMERGENCY_STOP) {
    //TODO stop() stopBlades;
    stop(1);
    Serial.println("");
    Serial.println("!!!! LONG TIME LOCK DETECTED --> SHUTDOWN !!!!");
    Serial.println("");

    digitalWrite(BUZZER, HIGH);
    delay(5000);
    digitalWrite(BUZZER, LOW);

    turnOffRobot();
  }


  if (tiltCount > 0) {
    //A. robot can go, wait to see if tilt is over

  } else if (canGo()) {
    //B. no obstacle detected mower can go straight

    if (firstLockDetectionMS > 0 && millis() - firstLockDetectionMS > LOCKED_LIMIT_MS) {
      //robot is "impantanato"
      //TODO stopBlades
      Serial.println("--> LOCK DODGE <--");
      stop(900);
      digitalWrite(BUZZER, HIGH);
      delay(100);
      digitalWrite(BUZZER, LOW);


      //NB: se cambi questi tempi devi tenerne conto nel LOCKED_LIMIT_SHUTDOWN_MS
      reverse(2000);
      if (leftDistance < TURN_LIMIT_CM && rightDistance < TURN_LIMIT_CM) {
        //no space left or right --> go back
        reverse(2000);
      } else if (leftDistance < TURN_LIMIT_CM) {
        //no space left
        right(2000);
      } else if (rightDistance < TURN_LIMIT_CM) {
        //no space right
        left(2000);
      } else {
        if (random(0, 10) < 5) { //random 0-->9
          left(2000);
        } else {
          right(2000);
        }
      }

      yaw = 0; //reset the angle
      lastLoopRunningMS = 0; //need for reset the angle

      //robot must start, else firstLockDetectionMS is not reset
      forward(yaw);
    }
    else if (leftDistance < LEFT_LIMIT_FOR_DODGE_CM) {
      //approaching obstacle from left try to doodge
      Serial.println("<-- LEFT DODGE");
      forward(DEGREE_FOR_DODGE);
    } else if (rightDistance < LEFT_LIMIT_FOR_DODGE_CM) {
      //approaching obstacle from right try to doodge
      Serial.println("RIGHT DODGE -->");
      forward(360 - DEGREE_FOR_DODGE);
    } else {
      //yam is use for balance direction
      forward(yaw);
    }



    //check if robot is locked
    normGyro = mpu.readNormalizeGyro();
    Serial.print("Gyro X = ");
    Serial.print(normGyro.XAxis);
    Serial.print("\tGyro Y = ");
    Serial.print(normGyro.YAxis);
    Serial.print("\tGyro Z = ");
    Serial.print(normGyro.ZAxis);
    Serial.println();

    float gyroX = normGyro.XAxis;
    float gyroY = normGyro.YAxis;
    float gyroZ = normGyro.ZAxis;
    gyroX = gyroX < 0 ? -gyroX : gyroX;
    gyroY = gyroY < 0 ? -gyroY : gyroY;
    gyroZ = gyroZ < 0 ? -gyroZ : gyroZ;


    if (gyroX < LOCK_TOLLERANCE && gyroY < LOCK_TOLLERANCE && gyroZ < LOCK_TOLLERANCE) {
      //the robot is locked
      if (firstLockDetectionMS == 0) {
        firstLockDetectionMS = millis();
      }

      Serial.println("!!!! LOCK DETECTED !!!!");
    } else {
      //robot is moving
      firstLockDetectionMS = 0;
    }
    //END lock check


  } else {
    //C. mower can't go straight
    stop(500);

    reverse(2000);

    if (leftDistance < TURN_LIMIT_CM && rightDistance < TURN_LIMIT_CM) {
      //no space left or right --> go back
      reverse(1000);
    } else if (leftDistance < TURN_LIMIT_CM) {
      //no space left
      right(1500);
    } else if (rightDistance < TURN_LIMIT_CM) {
      //no space right
      left(2000);
    } else {
      if (random(0, 10) < 5) { //random 0-->9
        left(2000);
      } else {
        right(2000);
      }
    }

    stop(500);

    yaw = 0; //reset the angle
    lastLoopRunningMS = 0; //need for reset the angle
    firstLockDetectionMS = 0;
  }


  delay(10);
}




/**
   Update mower perimet sensor and retur true if the robot can go straight
*/
boolean canGo() {

  frontDistance = getFrontDistance();
  leftDistance = getLeftDistance();
  rightDistance = getRightDistance();

  bool canGo = (frontDistance < 0 || frontDistance > FRONT_LIMIT_CM)
               && (leftDistance < 0 || leftDistance > LEFT_LIMIT_CM)
               && (rightDistance < 0 || rightDistance > RIGHT_LIMIT_CM);

  Serial.print("canGo: ");
  Serial.println(canGo);

  return canGo;
}

void turnOffRobot() {
  stop(100);

  for (int i = 0; i < 10; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(1000);
    digitalWrite(BUZZER, LOW);
    delay(1000);
  }

  while (true && !DISABLE_EMERGENCY_STOP) {
    //i'm die
  }
}
