#include <Encoder.h>
#include <PID_v1.h>
#include <SPI.h>

#define head1 0xAA
#define head2 0x55
#define sendType_velocity 0x11
#define sendType_pid 0x12
#define sendType_params 0x13

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

const int ss = 8;

double encPos1 = 0;
double encPos2 = 0;

double lastEncPos1 = 0;
double lastEncPos2 = 0;

double prevT1 = 0;
double prevT2 = 0;

float WHEELBASE = 20.0; // in m
float WHEELDIA = 0.1;   // in m
float encRES = 1000.0;

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
public:
  void setup()
  {
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

    pinMode(ss, OUTPUT);
    digitalWrite(ss, LOW);
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  }

  double getCurrentSpeedLinear() { return (speed1() + speed2()) / 2; }                                               // m/s
  double getCurrentRotationSpeed() { return ((speed2() / (WHEELDIA / 2)) - (speed1() / WHEELDIA / 2)) / WHEELBASE; } // rad/sec

  double speed1()
  {
    double currT = millis();
    double delT = (currT - prevT1) / 1000.0;

    encPos1 = motor1Encoder.read();
    double encSpeed1 = (encPos1 - lastEncPos1) / delT;
    double rps1 = encSpeed1 / encRES; // rps.
    lastEncPos1 = encPos1;
    return rps1 * WHEELDIA * PI; // speed (m/s)
  }

  double speed2()
  {
    double currT = millis();
    double delT = (currT - prevT2) / 1000.0;

    encPos2 = motor2Encoder.read();
    double encSpeed2 = (encPos2 - lastEncPos2) * delT;
    double rps2 = encSpeed2 / encRES; // ros.
    lastEncPos2 = encPos2;
    return rps2 * WHEELDIA * PI; // speed (m/s)
  }

  void setSpeed(float x, float yaw)
  {
    // x is linear speed, yaw is angular speed
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

    float Vr = x + (yaw * WHEELBASE) / 2;
    float Vl = x - (yaw * WHEELBASE) / 2;

    float ThetaR = Vr / (WHEELDIA / 2);
    float ThetaL = Vl / (WHEELDIA / 2);

    float rpsR = ThetaR / 2 * PI;
    float rpsL = ThetaL / 2 * PI;

    sebufferS(rpsR, rpsL);
  }

  void sebufferS(float rpsR, float rpsL)
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
      analogWrite(pwm_1, abs(speed));
    }
    else if (speed < 0)
    {
      digitalWrite(motor1_pin1, LOW);
      digitalWrite(motor1_pin2, HIGH);
      analogWrite(pwm_1, abs(speed));
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

uint8_t checksum(uint8_t *buf, size_t len)
{
  uint8_t sum = 0x00;
  for (int i = 0; i < len; i++)
  {
    sum += *(buf + i);
  }

  return sum;
}

BOT bot;
void setup()
{
  bot.setup();
}

void loop()
{
  static uint8_t buffer[9];

  double x = bot.getCurrentSpeedLinear();
  double yaw = bot.getCurrentRotationSpeed();
  // digitalWrite(ss, LOW);
  buffer[0] = head1;
  buffer[1] = head2;
  buffer[2] = 0x09;
  buffer[3] = sendType_velocity;
  buffer[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
  buffer[5] = ((int16_t)(x * 1000)) & 0xff;
  buffer[6] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
  buffer[7] = ((int16_t)(yaw * 1000)) & 0xff;
  buffer[8] = checksum(buffer, 9);
  SPI.transfer(buffer, 9);

  // Same for this one.
  bot.setSpeed(x, yaw);
  // digitalWrite(ss, HIGH);
}
