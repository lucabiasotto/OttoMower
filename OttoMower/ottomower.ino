//#include "motors_utils.h"
#include "perimeter_utils.h"

#include <Wire.h> //need by MPU6050
#include <MPU6050.h>//MPU6050byLC24 library by https://github.com/jarzebski/Arduino-MPU6050

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
const float DEGREE_FOR_DODGE = 10;

const float TURN_LIMIT_CM = 40; // larghezza robot

const int BUZZER = 9;

unsigned int   resetCount __attribute__ ((section (".noinit")));  


/**
 * Function for reset arduino
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

  

  if(resetCount == 1){ 
    // 1 is the magic number used to flag when i'm after a reset
    Serial.println("AFTER RESET"); 
    digitalWrite(BUZZER, HIGH);
    delay(500);
    digitalWrite(BUZZER, LOW);
  }else{
    //i need to do a reset because sometimes the MPU6050 not works at first boot
    Serial.print("FIRST POWER UP");
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




  /******************************
        CHECK IF MOWER CAN GO 
   ******************************/

  if(pitch < 0 || pitch > 5 || roll > 2 || roll < -2){
    //emergency stop
    stop();
    
    for(int i=0; i<10; i++){
      digitalWrite(BUZZER, HIGH);
      delay(1000);
      digitalWrite(BUZZER, LOW);
      delay(1000);
    }

    while(true){
      //i'm die  
    }
  }

  

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
       forward(DEGREE_FOR_DODGE);
       
      }else if(frontDistance > FRONT_LIMIT_CM && rightDistance < RIGHT_LIMIT_CM){
        // front is free, no space to right side

        forward(360 - DEGREE_FOR_DODGE);
      
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
 *  Check and log all settings
 */
void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  if(mpu.getSleepEnabled()){
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

    if(mpu.getClockSource() != MPU6050_CLOCK_PLL_XGYRO){
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

    if(mpu.getScale() != MPU6050_SCALE_2000DPS){
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



/**
 * Update mower perimet sensor and retur true if the robot can go straight
 */
boolean canGo(){

  frontDistance = getFrontDistance();
  leftDistance = getLeftDistance();
  rightDistance = getRightDistance();

  bool canGo = (frontDistance < 0 || frontDistance > FRONT_LIMIT_CM) 
                  && (leftDistance < 0 || leftDistance > LEFT_LIMIT_CM)
                  && (rightDistance <0 || rightDistance > RIGHT_LIMIT_CM); 
  
  Serial.print("canGo: ");
  Serial.println(canGo);

  return canGo;
}
