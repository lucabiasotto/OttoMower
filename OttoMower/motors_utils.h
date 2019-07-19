/**
   Classe for control motors
*/

//left motor
const int LEFT_LPWM= 5; // +5V left motor reverse rotation
const int LEFT_RPWM = 6; // +5V left motor forward rotation


//right motor
const int RIGHT_LPWM = 10; // +5V right motor reverse rotation
const int RIGHT_RPWM = 11; // +5V right motor forward rotation

const int MAX_SPEED = 255;
const int TURN_SPEED = 150;

void stop() {
  Serial.println("stop");
  digitalWrite(RIGHT_LPWM, LOW);
  digitalWrite(RIGHT_RPWM, LOW);

  digitalWrite(LEFT_LPWM, LOW);
  digitalWrite(LEFT_RPWM, LOW);
  
  delay(2000); //give the mower the time for stop
}

void forward(float currentAngle) {
 
  
  Serial.println("forward");

  int leftPower = MAX_SPEED;
  int rightPower = MAX_SPEED;
  if(currentAngle < 180){
    //reduce right power
    if(currentAngle > 90){
      currentAngle = 90;
    }
    
    rightPower = 255 - (255*currentAngle) / 90;
    
  }else{
    //reduce left power
    if(currentAngle < 270){
      currentAngle = 270;
    }

    currentAngle = 360 - currentAngle;

    leftPower = 255 - (255*currentAngle) / 90;
  }

  Serial.print("currentAngle = ");
  Serial.print(currentAngle);
  Serial.print("\trightPower = ");
  Serial.print(rightPower);
  Serial.print("\tleftPower = ");
  Serial.print(leftPower);
  Serial.println();

  analogWrite(RIGHT_LPWM, rightPower);
  digitalWrite(RIGHT_RPWM, LOW);

  analogWrite(LEFT_LPWM, leftPower);
  digitalWrite(LEFT_RPWM, LOW);
}

void left() {
  Serial.println("left");
  
  analogWrite(RIGHT_LPWM, TURN_SPEED);
  digitalWrite(RIGHT_RPWM, LOW);

  digitalWrite(LEFT_LPWM, LOW);
  analogWrite(LEFT_RPWM, TURN_SPEED);
  
  delay(5000);
}

void right() {
  Serial.println("right");
  
  digitalWrite(RIGHT_LPWM, LOW);
  analogWrite(RIGHT_RPWM, TURN_SPEED);  

  analogWrite(LEFT_LPWM, TURN_SPEED);
  digitalWrite(LEFT_RPWM, LOW);
  

  delay(5000);
}

void reverse() {
  Serial.println("forward");
  
  digitalWrite(RIGHT_LPWM, LOW);
  analogWrite(RIGHT_RPWM, TURN_SPEED);

  digitalWrite(LEFT_LPWM, LOW);
  analogWrite(LEFT_RPWM, TURN_SPEED);
  
  delay(5000);
}
