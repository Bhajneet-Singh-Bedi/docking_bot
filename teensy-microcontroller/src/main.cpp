#include <Encoder.h>

// mot-1 conn with pwm pin
const int motor1_pin1 = 9;
const int motor1_pin2 = 10;
const int pwm_1 = 6;

// mot-2 conn with pwm pin
const int motor2_pin1 = 11;
const int motor2_pin2 = 12;
const int pwm_2 = 7;

// ENC pins
const int ENC1A = 2;
const int ENC1B = 3;
const int ENC2A = 4;
const int ENC2B = 5;

int pos1 = 0, pos2 = 0;

Encoder motor1Encoder(ENC1A, ENC1B);
Encoder motor2Encoder(ENC2A, ENC2B);

volatile long targetSpeed1 = 0;
volatile long targetSpeed2 = 0;

// using PID to control the speed/position of the motor.
float kp = 1.0;
float ki = 0.01;
float kd = 0.1;

float integral1 = 0;
float integral2 = 0;

float lastError1 = 0;
float lastError2 = 0;

class BOT
{

private:
  long encoderPosition1 = 0;
  long encoderPosition2 = 0;

  long lastEncoderPosition1 = 0;
  long lastEncoderPosition2 = 0;

  unsigned long lastTime1 = 0;
  unsigned long lastTime2 = 0;

public:
  void setup()
  {
    // read pos from encoder.
    pinMode(ENC1A, INPUT);
    pinMode(ENC1B, INPUT);
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);

    // motor-1 setup
    pinMode(motor1_pin1, OUTPUT);
    pinMode(motor1_pin2, OUTPUT);
    pinMode(pwm_1, OUTPUT);

    // motor-2 setup
    pinMode(motor2_pin1, OUTPUT);
    pinMode(motor2_pin2, OUTPUT);
    pinMode(pwm_2, OUTPUT);
  }
  void getCurrentSpeedLinear()
  {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime1;

    encoderPosition1 = motor1Encoder.read();
    long encoderSpeed1 = (encoderPosition1 - lastEncoderPosition1) * (1000.0 / deltaTime);
    lastEncoderPosition1 = encoderPosition1;
  }

  void getCurrentRotationSpeed() {}

  void setSpeedLinear()
  {
  }
  void setSpeedRotational()
  {
  }
};

BOT bot;
void setup()
{

  bot.setup();
  Serial.begin(9600);
}

void loop()
{

  bot.setSpeedLinear();
  bot.setSpeedRotational();
}

// void controlMotor1() {
//   unsigned long currentTime = millis();
//   unsigned long deltaTime = currentTime - lastTime1;

//   encoderPosition1 = motor1Encoder.read();
//   long encoderSpeed1 = (encoderPosition1 - lastEncoderPosition1) * (1000.0 / deltaTime);
//   lastEncoderPosition1 = encoderPosition1;

//   float error1 = targetSpeed1 - encoderSpeed1;
//   integral1 += error1 * deltaTime;
//   float derivative1 = (error1 - lastError1) / deltaTime;
//   float output1 = kp * error1 + ki * integral1 + kd * derivative1;

//   setMotor1Speed(constrain(output1, -255, 255));

//   lastError1 = error1;
//   lastTime1 = currentTime;
// }

// void controlMotor2() {
//   unsigned long currentTime = millis();
//   unsigned long deltaTime = currentTime - lastTime2;

//   encoderPosition2 = motor2Encoder.read();
//   long encoderSpeed2 = (encoderPosition2 - lastEncoderPosition2) * (1000.0 / deltaTime);
//   lastEncoderPosition2 = encoderPosition2;

//   float error2 = targetSpeed2 - encoderSpeed2;
//   integral2 += error2 * deltaTime;
//   float derivative2 = (error2 - lastError2) / deltaTime;
//   float output2 = kp * error2 + ki * integral2 + kd * derivative2;

//   setMotor2Speed(constrain(output2, -255, 255));

//   lastError2 = error2;
//   lastTime2 = currentTime;
// }

void setMotor1Speed(int speed)
{
  if (speed > 0)
  {
    digitalWrite(motor1_pin1, HIGH);
    digitalWrite(motor1_pin2, LOW);
    analogWrite(pwm_1, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, HIGH);
    analogWrite(pwm_1, -speed);
  }
  else
  {
    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, LOW);
    analogWrite(pwm_1, 0);
  }
}

void setMotor2Speed(int speed)
{
  if (speed > 0)
  {
    digitalWrite(motor2_pin1, HIGH);
    digitalWrite(motor2_pin2, LOW);
    analogWrite(pwm_2, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, HIGH);
    analogWrite(pwm_2, -speed);
  }
  else
  {
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, LOW);
    analogWrite(pwm_2, 0);
  }
}
