#include <Encoder.h>
#include <PID_v1.h>
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
float kp = 1.0, ki = 0.01, kd = 0.1;
double input1, output1, setpoint1;
double input2, output2, setpoint2;

// PID controllers
PID pid1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);

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

    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);
  }

  long getCurrentSpeedLinear() { return (speed1() + speed2()) / 2; }                                               // m/s
  long getCurrentRotationSpeed() { return ((speed2() / (WHEELDIA / 2)) - (speed1() / WHEELDIA / 2)) / WHEELBASE; } // rad/sec

  long speed1()
  {
    unsigned long currT = millis();
    unsigned long delT = (currT - prevT1) / 1000.0;

    encPos1 = motor1Encoder.read();
    long encSpeed1 = (encPos1 - lastEncPos1) / delT;
    long rps1 = encSpeed1 / encRES; // rps.
    lastEncPos1 = encPos1;
    return rps1 * WHEELDIA * PI; // speed (m/s)
  }

  long speed2()
  {
    unsigned long currT = millis();
    unsigned long delT = (currT - prevT2) / 1000.0;

    encPos2 = motor2Encoder.read();
    long encSpeed2 = (encPos2 - lastEncPos2) * delT;
    long rps2 = encSpeed2 / encRES; // ros.
    lastEncPos2 = encPos2;
    return rps2 * WHEELDIA * PI; // speed (m/s)
  }

  void setSpeed(float x, float y)
  {
    // x is linear speed, y is angular speed
    // V-linear, omega-angular, L-WHEELBASE, r-wheelRad
    // linear velocity of wheel
    // Vr = V + (omega * L)/2
    // Vl = V - (omega * L)/2

    // rotational velocity
    // ThetaR = Vr/r;
    // ThetaL = Vl/r;

    // rps = theta/2*PI;
    // rpsR = ThetaR/2*PI;
    // rpsL = ThetaL/2*PI;

    float Vr = x + (y * WHEELBASE) / 2;
    float Vl = x - (y * WHEELBASE) / 2;

    float ThetaR = Vr / (WHEELDIA / 2);
    float ThetaL = Vl / (WHEELDIA / 2);

    float rpsR = ThetaR / 2 * PI;
    float rpsL = ThetaL / 2 * PI;

    setMPS(rpsR, rpsL);
  }

  void setMPS(float rpsR, float rpsL)
  {
    input1 = speed1() / (WHEELDIA * PI); // in rps
    input2 = speed2() / (WHEELDIA * PI); // in rps

    pid1.Compute();
    pid2.Compute();

    setMotor1Speed(constrain(output1, -255, 255));
    setMotor2Speed(constrain(output2, -255, 255));
  }

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
      analogWrite(pwm_2, abs(speed));
    }
    else if (speed < 0)
    {
      digitalWrite(motor2_pin1, LOW);
      digitalWrite(motor2_pin2, HIGH);
      analogWrite(pwm_2, abs(speed));
    }
    else
    {
      digitalWrite(motor2_pin1, LOW);
      digitalWrite(motor2_pin2, LOW);
      analogWrite(pwm_2, 0);
    }
  }
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
  bot.setSpeed(3.0, 3);
}
