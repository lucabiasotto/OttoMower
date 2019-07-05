/**
   Classe for control motors
*/

//MOTORE DX
const int RIGHT_EN = 11; //ENA
const int RIGHT_IN1 = 12; //IN1
const int RIGHT_IN2 = 13; //IN2

//MOTORE SX
const int LEFT_EN = 10; //ENB
const int LEFT_IN1 = 9; //IN3
const int LEFT_IN2 = 8; //IN4

const int MAX_SPEED = 255;
const int TURN_SPEED = 150;

void stop() {
  Serial.println("stop");
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  delay(2000); //give the mower the time for stop
}

void forward(float currentAngle) {
  //TODO use currentAngle
  
  Serial.println("forward");
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, MAX_SPEED); //0-255

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, MAX_SPEED);
}

void left() {
  Serial.println("left");
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(RIGHT_EN, TURN_SPEED);

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  analogWrite(LEFT_EN, TURN_SPEED);
  delay(5000);
}

void right() {
  Serial.println("right");
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(RIGHT_EN, TURN_SPEED);

  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_EN, TURN_SPEED);

  delay(5000);
}

void reverse() {
  Serial.println("forward");
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);
  analogWrite(RIGHT_EN, TURN_SPEED); //0-255

  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  analogWrite(LEFT_EN, TURN_SPEED);
  delay(5000);
}
