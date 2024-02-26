#include <microTuple.h>

// Mode 0: standard joystick input
// Mode 1: drive in circle of specified radius
// Mode 2: drive in rectangle of specified dimensions
// Mode 3: parallel park
#define MODE 0

#if MODE == 1
#define TURN_RADIUS 8.f
#elif MODE == 2 || MODE == 3
#define RECT_WIDTH 12.
#define RECT_HEIGHT 24.
// Parameters for parallel parking (change via experiment)
#define ANGLE 0.61
#define FIRST_DISTANCE -10.51
#define SECOND_DISTANCE 5.f
#endif

// General param - adjust for greater/lower responsiveness
#define TIMEOUT 50

// Robot measurements - determined by construction
#define WHEEL_WIDTH 5.5
#define MAX_IPS 15.89
#define WHEEL_RADIUS 1.25

// Pin information - determined by wiring
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define RIGHT_POW 3
#define RIGHT_POS 5
#define RIGHT_NEG 4
#define LEFT_POW 9
#define LEFT_POS 8
#define LEFT_NEG 7

// Some code utilities
#define setRightMotorPower(power) \
  setMotorPower(RIGHT_POS, RIGHT_NEG, RIGHT_POW, power)
#define setLeftMotorPower(power) \
  setMotorPower(LEFT_POS, LEFT_NEG, LEFT_POW, power)

void setMotorDirection(int posPort, int negPort, float power) {
  if (power == 0) {
    digitalWrite(posPort, LOW);
    digitalWrite(negPort, LOW);
  } else if (power > 0) {
    digitalWrite(posPort, HIGH);
    digitalWrite(negPort, LOW);
  } else {
    digitalWrite(posPort, LOW);
    digitalWrite(negPort, HIGH);
  }
}

void setMotorPower(int posPin, int negPin, int powerPin, float power) {
  setMotorDirection(posPin, negPin, power);
  analogWrite(powerPin, (int)abs(floor(power * 255)));
}

float getRadius(float xPos, float yPos, float theta) {
  float max;
  // If on the right/left edge of rectangle, max value is determined by x
  // If on top/bottom, max value is determined by y
  if (theta < PI / 4 || theta > 3 * PI / 4 && theta < 5 * PI / 4 ||
      theta > 7 * PI / 4)
    max = 512 / cos(theta);
  else
    max = 512 / sin(theta);
  return sqrt(sq(xPos) + sq(yPos)) / abs(max);
}

MicroTuple<float, float> xyToPolar(int xPos, int yPos) {
  float adjustedX = 512 - yPos;
  float adjustedY = 512 - xPos;
  // Get theta in radians
  float theta;
  if (adjustedX == 0) {
    theta = adjustedY > 0 ? PI / 2 : -PI / 2;
  } else {
    theta = atan(adjustedY / adjustedX);
  }
  // Adjust for limitations of arctan
  if (adjustedX < 0) theta += PI;
  // Ensure theta is in range [0, 2pi]
  if (adjustedX >= 0 && adjustedY < 0) theta += 2 * PI;
  // Get radius in range [0, 1]
  float r = getRadius(adjustedX, adjustedY, theta);
  return MicroTuple<float, float>(r, theta);
}

#if MODE == 0

void setRightMotor(float power, float theta) {
  float finalPower = power;
  if (theta < PI / 2) {
    finalPower *= -cos(2 * theta);
  } else if (theta < PI) {
    finalPower *= 1;
  } else if (theta < 3 * PI / 2) {
    finalPower *= cos(2 * theta);
  } else {
    finalPower *= -1;
  }
  setRightMotorPower(finalPower);
}

void setLeftMotor(float power, float theta) {
  float finalPower = power;
  if (theta < PI / 2) {
    finalPower *= 1;
  } else if (theta < PI) {
    finalPower *= -cos(2 * theta);
  } else if (theta < 3 * PI / 2) {
    finalPower *= -1;
  } else {
    finalPower *= cos(2 * theta);
  }
  setLeftMotorPower(finalPower);
}

// Adjust power based on cubic function (allow more control at lower
// acceleration)
float adjustPower(float power) { return pow(power, 3); }

void drive(float rawPower, float theta) {
  float power = adjustPower(rawPower);
  setRightMotor(power, theta);
  setLeftMotor(power, theta);
}

void driveByJoystick() {
  auto polarCoords = xyToPolar(analogRead(JOYSTICK_X), analogRead(JOYSTICK_Y));
  drive(polarCoords.get<0>(), polarCoords.get<1>());
}

#elif MODE == 1

void turnRadius(float radius) {
  auto polarCoords = xyToPolar(analogRead(JOYSTICK_X), analogRead(JOYSTICK_Y));
  if (polarCoords.get<0>() > 0.02) {
    float outerRadius = radius + WHEEL_WIDTH;
    setLeftMotorPower(1.f);
    setRightMotorPower(radius / outerRadius);
  } else {
    setLeftMotorPower(0.f);
    setRightMotorPower(0.f);
  }
}

#elif MODE == 2 || MODE == 3

void driveDistance(float distance) {
  long timeout = (long)abs(1000 * distance / MAX_IPS);
  int distanceSign = distance > 0 ? 1 : -1;
  setRightMotorPower(distanceSign);
  setLeftMotorPower(distanceSign);
  delay(timeout);
  setRightMotorPower(0.f);
  setLeftMotorPower(0.f);
  delay(TIMEOUT * 4);
}

void turnAngle(float theta) {
  long timeout = (long)abs(1000 * theta * WHEEL_WIDTH / 2 / MAX_IPS);
  int thetaSign = theta > 0 ? 1 : -1;
  setRightMotorPower(-thetaSign);
  setLeftMotorPower(thetaSign);
  delay(timeout);
  setRightMotorPower(0.f);
  setLeftMotorPower(0.f);
  delay(TIMEOUT * 4);
}

void driveRectangle(float width, float height) {
  driveDistance(height);
  turnAngle(PI / 2);
  driveDistance(width);
  turnAngle(PI / 2);
  driveDistance(height);
  turnAngle(PI / 2);
  driveDistance(width);
  turnAngle(PI / 2);
}

void parallelPark() {
  turnAngle(-ANGLE);
  driveDistance(FIRST_DISTANCE);
  turnAngle(ANGLE);
  driveDistance(SECOND_DISTANCE);
  delay(5000);
}

#endif

void setup() {
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(RIGHT_POW, OUTPUT);
  pinMode(LEFT_POW, OUTPUT);
  pinMode(RIGHT_POS, OUTPUT);
  pinMode(RIGHT_NEG, OUTPUT);
  pinMode(LEFT_POS, OUTPUT);
  pinMode(LEFT_NEG, OUTPUT);

#if MODE == 0
  // Setup joystick pins and original values
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
#endif

  // Turn off motors - Initial state
  setLeftMotorPower(0.f);
  setRightMotorPower(0.f);
}

void loop() {
#if MODE == 0
  driveByJoystick();
#elif MODE == 1
  turnRadius(TURN_RADIUS);
#elif MODE == 2
  driveRectangle(RECT_WIDTH, RECT_HEIGHT);
#elif MODE == 3
  parallelPark();
#endif
  delay(TIMEOUT);
}