/*
 * Class for check perimeter and obstacle
 */

const int US_FRONT_TRIG = 2;
const int US_FRONT_ECHO = 8;
const int US_LEFT_TRIG = 4;
const int US_LEFT_ECHO = 12;
const int US_RIGHT_TRIG = 13;
const int US_RIGHT_ECHO = 7;



/**
 * return the distance for the passed HC-SR04 pin
 */
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float cm = pulseIn(echoPin, HIGH) / 58.0; //The echo time is converted into cm
  cm = (int(cm * 100.0)) / 100.0; //Keep two decimal places
  return cm;
}

/**
 * return front distance in cm
 */
int getFrontDistance() {
  float frontDistance = getDistance(US_FRONT_TRIG, US_FRONT_ECHO);
  Serial.print("frontDistance: ");
  Serial.println(frontDistance);
  return frontDistance;
}

/**
 * return left distance in cm
 */
int getLeftDistance() {
  float  leftDistance = getDistance(US_LEFT_TRIG, US_LEFT_ECHO);
  Serial.print("leftDistance: ");
  Serial.println(leftDistance);
  return leftDistance;
}

/**
 * return right distance in cm
 */
int getRightDistance() {
  float rightDistance = getDistance(US_RIGHT_TRIG, US_RIGHT_ECHO);
  Serial.print("rightDistance: ");
  Serial.println(rightDistance);
  return rightDistance;
}
