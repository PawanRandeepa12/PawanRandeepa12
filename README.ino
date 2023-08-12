#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5

#define RMotorA 2
#define RMotorB 3
#define RMotorPWM 9

#define LMotorA 4
#define LMotorB 5
#define LMotorPWM 10

#define MAX_SPEED 150

int MotorBasespeed = 100;

int IR_val[6] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-20,-10,-5,5,10,20};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 5.5;
float Kd = 10;
float Ki = 0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

#define servoPin 9
#define ultrasonicTrigger 10
#define ultrasonicEcho 11

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(servoPin, OUTPUT);
  pinMode(ultrasonicTrigger, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);

  set_forward();
  delay(2000);
}


void loop()

{

  read_IR();
  if (IR_val[0] ==0 && IR_val[1] ==0 && IR_val[2] ==0 && IR_val[3] ==0 && IR_val[4] ==0 && IR_val[5] ==0){
    stop();
    while(1){}
  }
PID_control();
  set_speed();

  // Check if the distance is less than 3cm
  if (distance < 3) {
    // Gripper closed
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(1450); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle
  } else {
    // Gripper open
    digitalWrite(servoPin, LOW);
    delayMicroseconds(2000); // Duration of the pusle in microseconds
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(1800); // 20ms - duration of the pusle
  }
}

void PID_control() {

  error =0;

  for (int i = 0; i < 6; i++)
  {
    error += IR_weights[i] * IR_val[i];
  }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  LMotorSpeed = MotorBasespeed - speedAdjust;
  RMotorSpeed = MotorBasespeed + speedAdjust;

  if (LMotorSpeed < 0)
  {
    LMotorSpeed = 0;
  }
  if (RMotorSpeed < 0)
  {
    RMotorSpeed = 0;
    }
  }
    
