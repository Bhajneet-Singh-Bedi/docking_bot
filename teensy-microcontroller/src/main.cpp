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

Encoder motor1Encoder(ENC1A, ENC1B);
Encoder motor2Encoder(ENC2A, ENC2B);

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
    long encPos1 = 0;
    long encPos2 = 0;

    long lastEncPos1 = 0;
    long lastEncPos2 = 0;

    unsigned long prevT1 = 0;
    unsigned long prevT2 = 0;

    float WHEELBASE = 20.0; // in m
    float WHEELDIA = 0.1;   // in m
    float encRES = 1000.0;

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

    long getCurrentSpeedLinear() { return (speed1() + speed2()) / 2; }
    long getCurrentRotationSpeed() { return (speed2() - speed1()) / WHEELBASE; }

    long speed1()
    {
      unsigned long currT = millis();
      unsigned long delT = (currT - prevT1)/1000.0;

      encPos1 = motor1Encoder.read();
      long encSpeed1 = (encPos1 - lastEncPos1) / delT;
      long rps1 = encSpeed1 / encRES;
      lastEncPos1 = encPos1;
      return rps1 * WHEELDIA * PI;
    }

    long speed2()
    {
      unsigned long currT = millis();
      unsigned long delT = (currT - prevT2)/1000.0;

      encPos2 = motor2Encoder.read();
      long encSpeed2 = (encPos2 - lastEncPos2) * delT;
      long rps2 = encSpeed2 / encRES;
      lastEncPos2 = encPos2;
      return rps2 * WHEELDIA * PI;
    }

    void setSpeed(){}
};

BOT bot;
void setup()
{
  bot.setup();
}

void loop()
{
  bot.getCurrentSpeedLinear();
  bot.getCurrentRotationSpeed();
  bot.setSpeed();
}

// void controlMotor1() {
//   unsigned long currentTime = millis();
//   unsigned long deltaTime = currentTime - prevT1;

//   encoderPosition1 = motor1Encoder.read();
//   long encoderSpeed1 = (encoderPosition1 - lastEncoderPosition1) * (1000.0 / deltaTime);
//   lastEncoderPosition1 = encoderPosition1;

//   float error1 = targetSpeed1 - encoderSpeed1;
//   integral1 += error1 * deltaTime;
//   float derivative1 = (error1 - lastError1) / deltaTime;
//   float output1 = kp * error1 + ki * integral1 + kd * derivative1;

//   setMotor1Speed(constrain(output1, -255, 255));

//   lastError1 = error1;
//   prevT1 = currentTime;
// }

// void controlMotor2() {
//   unsigned long currentTime = millis();
//   unsigned long deltaTime = currentTime - prevT2;

//   encoderPosition2 = motor2Encoder.read();
//   long encoderSpeed2 = (encoderPosition2 - lastEncoderPosition2) * (1000.0 / deltaTime);
//   lastEncoderPosition2 = encoderPosition2;

//   float error2 = targetSpeed2 - encoderSpeed2;
//   integral2 += error2 * deltaTime;
//   float derivative2 = (error2 - lastError2) / deltaTime;
//   float output2 = kp * error2 + ki * integral2 + kd * derivative2;

//   setMotor2Speed(constrain(output2, -255, 255));

//   lastError2 = error2;
//   prevT2 = currentTime;
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
