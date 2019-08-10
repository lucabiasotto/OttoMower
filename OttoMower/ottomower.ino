//#include "motors_utils.h"
#include "perimeter_utils.h"

#include <Wire.h> //need by MPU6050
#include <MPU6050.h>//MPU6050byLC24 library by https://github.com/jarzebski/Arduino-MPU6050

MPU6050 mpu;

//CAUTION!!! DISABLE_EMERGENCY_STOP only for DEBUG
const bool DISABLE_EMERGENCY_STOP = true; //shutdown the robot if it's locked or someone tilted him


const float FRONT_LIMIT_CM = 40;
const float LEFT_LIMIT_CM = 10;
const float RIGHT_LIMIT_CM = 10;
const float LEFT_LIMIT_FOR_DODGE_CM = 20;
const float RIGHT_LIMIT_FOR_DODGE_CM = 20;
const float DEGREE_FOR_DODGE = 45;
const float TURN_LIMIT_CM = 60; // larghezza robot

// Pitch, Roll and Yaw values
int pitch = 0;
int roll = 0;
float yaw = 0;
long lastLoopRunningMS = 0;
const int PITCH_LIMIT = 10; //pitch limit before shutdown
const int ROLL_LIMIT = 10; //roll limit before shutdown


float frontDistance;
float leftDistance;
float rightDistance;

float prevAccX = 0;
float prevAccY = 0;
float prevAccZ = 0;

int lockedStartTimeMS = 0;
const int LOCKED_LIMIT_MS = 2000; //if not move for this time try to free the robot
const int LOCKED_LIMIT_SHUTDOWN_MS = 20000; //if not move for this time shutdown all


const int BUZZER = 9;

unsigned int   resetCount __attribute__ ((section (".noinit")));


/**
   Function for reset arduino
*/
void(* reset)(void) = 0;


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


  Serial.print("Acc X = ");
  Serial.print(normAccel.XAxis);
  Serial.print("\tAcc Y = ");
  Serial.print(normAccel.YAxis);
  Serial.print("\tAcc Z = ");
  Serial.print(normAccel.ZAxis);
  Serial.println();


  /******************************
        CHECK IF MOWER CAN GO
   ******************************/


  if (pitch < -PITCH_LIMIT || pitch > PITCH_LIMIT || roll > ROLL_LIMIT || roll < -ROLL_LIMIT || (millis() - lockedStartTimeMS) > LOCKED_LIMIT_SHUTDOWN_MS) {
    //emergency stop
    stop();

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


  if (canGo()) {
    //no obstacle detected mower can go straight

    boolean robotIsLocked = false;

    if (prevAccX != 0 && prevAccY != 0  && prevAccZ != 0) {
      //check acceleration, if accelaration not chage the rover is locked
      float tollerance = 0.1;
      float deltaX = prevAccX - normAccel.XAxis;
      float deltaY = prevAccY - normAccel.YAxis;
      float deltaZ = prevAccZ - normAccel.ZAxis;

      deltaX = deltaX < 0 ? -deltaX : deltaX;
      deltaY = deltaY < 0 ? -deltaY : deltaY;
      deltaZ = deltaZ < 0 ? -deltaZ : deltaZ;

      if (deltaX < tollerance && deltaY < tollerance && deltaZ < tollerance) {
        //the robot is locked
        if(lockedStartTimeMS == 0){
          lockedStartTimeMS = millis();
        }

        Serial.println("");
        Serial.println("!!!! LOCK DETECTED !!!!");
        Serial.println("");
        
        if(millis() - lockedStartTimeMS > LOCKED_LIMIT_MS){
          robotIsLocked = true;
        }

      } else {
        //robot is moving
        lockedStartTimeMS = 0;
        robotIsLocked = false;
      }
    }
    prevAccX = normAccel.XAxis;
    prevAccY = normAccel.YAxis;
    prevAccZ = normAccel.ZAxis;


    if (robotIsLocked) {
      //robot is locked, try to free it
      stop();
      Serial.println("LOCK DODGE");
      reverse();
      if (leftDistance < TURN_LIMIT_CM ) {
        right();
      } else {
        left();
      }
      stop();

    } else if (leftDistance < LEFT_LIMIT_FOR_DODGE_CM) {
      //approaching obstacle from left try to doodge
      Serial.println("LEFT DODGE");
      forward(DEGREE_FOR_DODGE);
    } else if (rightDistance < LEFT_LIMIT_FOR_DODGE_CM) {
      //approaching obstacle from right try to doodge
      Serial.println("RIGHT DODGE");
      forward(360 - DEGREE_FOR_DODGE);
    } else {
      //yam is use for balance direction
      forward(yaw);
    }

  } else {
    //mower can't go straight
    stop();
    lockedStartTimeMS = 0;
    
    reverse();

    //il roboto è bloccato, lo faccio andare avanti e indietro finchè c'è spazio
    do {

      if (leftDistance < TURN_LIMIT_CM && rightDistance < TURN_LIMIT_CM) {
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
    } while (!canGo());


    //reset the angle
    yaw = 0;
    lastLoopRunningMS = 0;
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
