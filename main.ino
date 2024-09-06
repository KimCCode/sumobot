#include <Servo.h>
// PINS
#define leftEchoPin 3
#define leftTrigPin 4
#define middleEchoPin 5  // Middle sensor
#define middleTrigPin 6  // Middle sensor
#define rightEchoPin 7
#define rightTrigPin 8
#define servoPin 11
#define IRPin 12

// SPEEDS
#define MAX_SPEED 255
#define TURN_SPEED 75
#define CRUISE_SPEED 50

// STATES
#define WAITING 0
#define SEARCHING 1
#define WAITING_FOR_ENEMY 2
#define ROLLING_THUNDER 3
#define PUSH_BACK 4

// THRESHOLDS
#define STATIONARY_THRESHOLD 5        // Consider enemy stationary if doesn't move more than 5cm
#define ROLLING_THUNDER_THRESHOLD 17  // 17cm
#define MAX_SENSOR_RANGE 120          // 120cm - Ignore objects outside ring
#define NORMALISE 650

#define LEFT 7
#define RIGHT 8

int RIGHT_SPEED = 9;  // Speed pin, ranges from 0 to 255 (ENA)
int RIGHT_F = 14;     // Pin to move motor forwards (IN1)
int RIGHT_R = 15;     // Pin to move motor backwards (IN2)

int LEFT_SPEED = 10;  // Speed pin, ranges from 0 to 255 (ENB)
int LEFT_R = 16;      // Pin to move motor backwards (IN4)
int LEFT_F = 17;      // Pin to move motor forwards (IN3)

int currentState = WAITING;

double prevLeftDistance = 0;
double prevMiddleDistance = 0;
double prevRightDistance = 0;

double leftDistance = 0;
double middleDistance = 0;
double rightDistance = 0;

unsigned long currTime = 0;
unsigned long prevTime = 0;
unsigned long waitTime = 30000;

Servo servo;

void setup() {
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(middleTrigPin, OUTPUT);
  pinMode(middleEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  servo.attach(servoPin);
  Serial.begin(9600);
  Serial.println("Start!");
}

void loop() {
  currTime = millis();
  // Wait 5s
  if (currentState == WAITING) {
    delay(5000);
    for (int pos = 0; pos <= 180; pos++) {
      servo.write(pos);
      delay(5);
    }
    currentState = SEARCHING;
  }

  if (digitalRead(IRPin) == 1) {
    atEdge();
    currentState = SEARCHING;
  }

  // Must move every 30s - Test this
  if (currTime - prevTime >= waitTime) {
    turnLeft();
    delay(250);
    turnRight();
    stopMotors();
    prevTime = currTime;
  }

  updateDistances();
  handleState();
  delay(250);
}

void updateDistances() {
  leftDistance = getDistance(leftTrigPin, leftEchoPin);
  middleDistance = getDistance(middleTrigPin, middleEchoPin);
  rightDistance = getDistance(rightTrigPin, rightEchoPin);
  Serial.print("Left sensor distance: ");
  Serial.println(leftDistance);
  Serial.print("Middle sensor distance: ");
  Serial.println(middleDistance);
  Serial.print("Right sensor distance: ");
  Serial.println(rightDistance);

  // update prev distances
  prevLeftDistance = leftDistance;
  prevMiddleDistance = middleDistance;
  prevRightDistance = rightDistance;
}


void handleState() {
  switch (currentState) {
    case SEARCHING:
      if (leftDistance <= MAX_SENSOR_RANGE && middleDistance <= MAX_SENSOR_RANGE && rightDistance <= MAX_SENSOR_RANGE) {
        stopMotors();
        if (leftDistance < ROLLING_THUNDER_THRESHOLD || rightDistance < ROLLING_THUNDER_THRESHOLD || middleDistance < ROLLING_THUNDER_THRESHOLD) {
          currentState = ROLLING_THUNDER;
        }
      } else if (leftDistance < rightDistance && leftDistance < MAX_SENSOR_RANGE && rightDistance < MAX_SENSOR_RANGE) {
        turnLeft();
      } else if (rightDistance < leftDistance && rightDistance < MAX_SENSOR_RANGE && leftDistance < MAX_SENSOR_RANGE) {
        turnRight();
      } else {
        turnRight();
      }

      break;

    case ROLLING_THUNDER:
      if (leftDistance < rightDistance) {
        rollingThunder(LEFT);
      } else {
        rollingThunder(RIGHT);
      }
      currentState = SEARCHING;
      break;

    default:
      break;
  }
}
void engage_forward() {
  Serial.println("Moving slightly forward to engage.");
  analogWrite(LEFT_SPEED, CRUISE_SPEED);
  analogWrite(RIGHT_SPEED, CRUISE_SPEED);
  digitalWrite(LEFT_F, HIGH);
  digitalWrite(LEFT_R, LOW);
  digitalWrite(RIGHT_F, HIGH);
  digitalWrite(RIGHT_R, LOW);
}

void RT_comeIn_r() {
  Serial.println("Allowing the opp to come into our right");
  analogWrite(RIGHT_SPEED, MAX_SPEED);
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, HIGH);
  delay(2000);   // change value for however long it takes our bot to rotate 90 degrees
}

void RT_comeIn_l() {
  Serial.println("Allowing the opp to come into our left");
  analogWrite(LEFT_SPEED, MAX_SPEED);
  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, HIGH);
  delay(2000);   // change value for however long it takes our bot to rotate 90 degrees
}

void RT_turnEmOut_r() {
  Serial.println("Fuck you dumbass! Find the edge BITCH!");
  turnRight();
  delay(2000);   // change value for however long it takes our bot to rotate 90 degrees
}

void RT_turnEmOut_l() {
  Serial.println("Fuck you dumbass! Find the edge BITCH!");
  turnLeft();
  delay(2000);   // change value for however long it takes our bot to rotate 90 degrees
}

void rollingThunder(int side) {
  engage_forward();
  delay(3000);  // change value for however long it takes our bot to go forward a little
  stopMotors();

  Serial.println("ROOOOLLLING THUNDER");
  if (side == RIGHT) {
    RT_comeIn_r();
    RT_turnEmOut_r();
    // while (!checkBorder(IRPin)) {
    //   driveForwards();
    // }
    driveForwards();
    delay(2000);
    stopMotors();
  } else {
    RT_comeIn_l();
    RT_turnEmOut_l();
    // while (!checkBorder(IRPin)) {
    //   driveForwards();
    // }
    driveForwards();
    delay(2000);
    stopMotors();
  }
}

void continuousTurnLeft() {
  analogWrite(LEFT_SPEED, MAX_SPEED);
  analogWrite(RIGHT_SPEED, TURN_SPEED);

  digitalWrite(LEFT_F, HIGH);
  digitalWrite(LEFT_R, LOW);
  digitalWrite(RIGHT_F, HIGH);
  digitalWrite(RIGHT_R, LOW);
}

void atEdge() {
  driveForwards();
  delay(2000);
  stopMotors();
}

void stopMotors() {
  Serial.println("Stopping robot");
  analogWrite(LEFT_SPEED, 0);
  analogWrite(RIGHT_SPEED, 0);

  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, LOW);
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, LOW);
}

void driveForwards() {
  Serial.println("Driving forward");
  analogWrite(LEFT_SPEED, MAX_SPEED);
  analogWrite(RIGHT_SPEED, MAX_SPEED);

  digitalWrite(LEFT_F, HIGH);
  digitalWrite(LEFT_R, LOW);
  digitalWrite(RIGHT_F, HIGH);
  digitalWrite(RIGHT_R, LOW);
}

void driveBackwards() {
  Serial.println("Driving backwards");
  analogWrite(LEFT_SPEED, MAX_SPEED);
  analogWrite(RIGHT_SPEED, MAX_SPEED);

  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, HIGH);
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, HIGH);
}

int checkBorder(int irSensorPin) {
  int statusSensor = digitalRead(irSensorPin);
  if (statusSensor == HIGH) {
    Serial.println("DETECTED WHITE!");
    return 1;
  } else {
    Serial.println("DETECTED BLACK!");
    return 0;
  }
}

double getDistance(int trigPin, int echoPin) {
  double distance, duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 100000L);
  distance = (duration * 0.0340) / 2;
  return distance;
}

void turnLeft() {
  Serial.println("Moving left");
  analogWrite(LEFT_SPEED, MAX_SPEED);
  analogWrite(RIGHT_SPEED, MAX_SPEED);

  digitalWrite(LEFT_F, LOW);
  digitalWrite(LEFT_R, HIGH);
  digitalWrite(RIGHT_F, HIGH);
  digitalWrite(RIGHT_R, LOW);
}

void turnRight() {
  Serial.println("Moving left");
  analogWrite(LEFT_SPEED, MAX_SPEED);
  analogWrite(RIGHT_SPEED, MAX_SPEED);

  digitalWrite(LEFT_F, HIGH);
  digitalWrite(LEFT_R, LOW);
  digitalWrite(RIGHT_F, LOW);
  digitalWrite(RIGHT_R, HIGH);
}
