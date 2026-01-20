#define MOTOR_DIRECTION 0 // 0 is forwards 1 is backwards
#define PIN_DIRECTION_RIGHT 3
#define PIN_DIRECTION_LEFT  4
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_MOTOR_PWM_LEFT  6

char command;

void setup() {
  Serial.begin(9600); // Bluetooth module speed
  Serial.println("Bluetooth car ready");
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();
    switch (command) {
      case 'F': // Forwards
        Serial.println("Moving Forward");
        motorRun(120, 120);
        break;
      case 'B': // Backwards
        Serial.println("Moving Backward");
        motorRun(-120, -120);
        break;
      case 'L': // Left
        Serial.println("Turning Left");
        motorRun(-120, 120);
        break;
      case 'R': // Right
        Serial.println("Turning Right");
        motorRun(120, -120);
        break;
      case 'S': // Stop
        Serial.println("STOP");
        motorRun(0, 0);
        break;
    }
  }
}

void motorRun(int speedl, int speedr) { // controls the speed of the two sets of motors (left and right)
  int dirL = 0, dirR = 0; // dirL controls left wheel spin direction and dirR controls right wheel spin direction
  if (speedl > 0) {
    dirL = 0 ^ MOTOR_DIRECTION;
  } else {
    dirL = 1 ^ MOTOR_DIRECTION;
    speedl = -speedl;
  }

  if (speedr > 0) {
    dirR = 1 ^ MOTOR_DIRECTION;
  } else {
    dirR = 0 ^ MOTOR_DIRECTION;
    speedr = -speedr;
  }
  digitalWrite(PIN_DIRECTION_LEFT, dirL); // moves the wheels
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}