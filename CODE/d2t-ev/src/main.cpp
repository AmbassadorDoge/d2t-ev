#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h>
#include <Stepper.h>
#include <BMI160.h>
#include <BMI160Gen.h>
#include <Wire.h>

const int BMI160_ADDR = 0x68;
BMI160GenClass BMI;

#define BUTTON_1 A0
#define BUTTON_2 A1
#define SDA A4
#define SCL A5

const int STEPPER_IN_1 = 4;
const int STEPPER_IN_2 = 5;
const int STEPPER_IN_3 = 6;
const int STEPPER_IN_4 = 7;

const int MOTOR_1 = 9;
const int MOTOR_2 = 10;

const int BACK_ENCODER_1 = 2;
const int BACK_ENCODER_2 = 11;
const int FRONT_ENCODER_1 = 3;
const int FRONT_ENCODER_2 = 12;

int STEPPER_SPEED = 20;
int STEPPER_POSITION = 0;

float gyroX = 0.0;
float motorSpeed = 0.0;
float motorPosition = 0.0;

float x = 0, y = 0, theta = 0;
float L = 0.25;
float v = 0.2;
float dt = 0.05;
float lookahead = 0.4;

float waypointX = 1.0, waypointY = 0.5;
float finalX = 2.0, finalY = 1.0;
bool passedWaypoint = false;

Encoder backEncoder = Encoder(BACK_ENCODER_1, BACK_ENCODER_2);
Encoder frontEncoder = Encoder(FRONT_ENCODER_1, FRONT_ENCODER_2);

Stepper stepper(200, STEPPER_IN_1, STEPPER_IN_2, STEPPER_IN_3, STEPPER_IN_4);
Adafruit_SSD1306 display(128, 32, &Wire, -1);

class TemporalPager {
  unsigned long lastTime = 0;
  unsigned long interval;
public:
  TemporalPager(unsigned long ms) : interval(ms) {}
  bool ready() {
    unsigned long now = millis();
    if (now - lastTime >= interval) {
      lastTime = now;
      return true;
    }
    return false;
  }
  void reset() { lastTime = millis(); }
  void setInterval(unsigned long ms) { interval = ms; }
};

void initBackEncoder() { backEncoder.write(0); }
void initFrontEncoder() { frontEncoder.write(0); }
void initMotors() {
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  analogWrite(MOTOR_1, 0);
  analogWrite(MOTOR_2, 0);
}
void initStepper() { stepper.setSpeed(STEPPER_SPEED); }
void initBMI160() {
  if (!BMI.begin(BMI160GenClass::I2C_MODE, BMI160_ADDR)) {
    Serial.println(F("BMI160 initialization failed!"));
    while (1);
  } else {
    Serial.println(F("BMI160 initialized successfully!"));
  }
}
void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(F("Display Initialized"));
  display.display();
}
void initButtons() {
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
}
void initHardware() {
  initBackEncoder();
  initFrontEncoder();
  initMotors();
  initStepper();
  initBMI160();
  initDisplay();
  initButtons();
}

float wrapToPi(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

void rearSteerUpdate(float &x, float &y, float &theta, float delta, float v, float L, float dt) {
  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;
  theta += -v * tan(delta) / L * dt;
}

float purePursuitRearSteer(float x, float y, float theta, float targetX, float targetY, float L, float lookahead) {
  float dx = targetX - x;
  float dy = targetY - y;
  float targetAngle = atan2(dy, dx);
  float alpha = wrapToPi(targetAngle - theta);
  float delta = atan2(2.0 * L * sin(alpha), lookahead);
  return delta;
}

float distanceTo(float x1, float y1, float x2, float y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void printPose(float x, float y, float theta) {
  Serial.print("Pose => X: ");
  Serial.print(x, 3);
  Serial.print(" Y: ");
  Serial.print(y, 3);
  Serial.print(" Theta: ");
  Serial.println(theta, 3);
}

TemporalPager steeringPager(50);
float updateSteering() {
  if (steeringPager.ready()) {
    float tx = passedWaypoint ? finalX : waypointX;
    float ty = passedWaypoint ? finalY : waypointY;
    if (!passedWaypoint && distanceTo(x, y, waypointX, waypointY) < 0.2) {
      passedWaypoint = true;
    }
    float delta = purePursuitRearSteer(x, y, theta, tx, ty, L, lookahead);
    rearSteerUpdate(x, y, theta, delta, v, L, dt);
    printPose(x, y, theta);
    return delta;
  }
  return 0.0;
}

void setSteeringAngle(float delta) {
  float maxAngle = 0.6;
  int maxSteps = 100;
  delta = constrain(delta, -maxAngle, maxAngle);
  int targetSteps = map(delta * 1000, -maxAngle * 1000, maxAngle * 1000, -maxSteps, maxSteps);
  int stepDiff = targetSteps - STEPPER_POSITION;
  stepper.step(stepDiff);
  STEPPER_POSITION = targetSteps;
}

void updateMotorSCurve() {
  static unsigned long startTime = 0;
  static bool accelerating = true;
  static bool direction = true;
  static float targetSpeed = 255;
  static float accelTime = 2000; // ms
  static float runDuration = 4000; // ms
  unsigned long now = millis();
  float t = now - startTime;

  if (t < accelTime) {
    float ramp = t / accelTime;
    float sCurve = 3 * pow(ramp, 2) - 2 * pow(ramp, 3);
    float pwm = sCurve * targetSpeed;
    analogWrite(direction ? MOTOR_1 : MOTOR_2, pwm);
    analogWrite(direction ? MOTOR_2 : MOTOR_1, 0);
  } else if (t >= accelTime && t < (accelTime + runDuration)) {
    analogWrite(direction ? MOTOR_1 : MOTOR_2, targetSpeed);
    analogWrite(direction ? MOTOR_2 : MOTOR_1, 0);
  } else if (t >= (accelTime + runDuration) && t < (2 * accelTime + runDuration)) {
    float ramp = (t - accelTime - runDuration) / accelTime;
    float sCurve = 3 * pow(1 - ramp, 2) - 2 * pow(1 - ramp, 3);
    float pwm = sCurve * targetSpeed;
    analogWrite(direction ? MOTOR_1 : MOTOR_2, pwm);
    analogWrite(direction ? MOTOR_2 : MOTOR_1, 0);
  } else {
    analogWrite(MOTOR_1, 0);
    analogWrite(MOTOR_2, 0);
    startTime = now;
    direction = !direction;
  }
}

void updateMotorPosition() {
  long backPos = backEncoder.read();
  long frontPos = frontEncoder.read();
}

void handleButtons() {
  if (digitalRead(BUTTON_1) == LOW) {
    backEncoder.write(0);
    frontEncoder.write(0);
    x = y = theta = 0;
    passedWaypoint = false;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  initHardware();
}

void loop() {
  float delta = updateSteering();
  setSteeringAngle(delta);
  updateMotorSCurve();
  handleButtons();
  updateMotorPosition();
}
