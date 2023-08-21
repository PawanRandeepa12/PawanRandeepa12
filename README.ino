#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3

#define RMotorA 2
#define RMotorB 3
#define RMotorPWM 9

#define LMotorA 4
#define LMotorB 5
#define LMotorPWM 10

#define MAX_SPEED 150

int MotorBasespeed = 100;

int IR_val[4] = {0, 0, 0, 0};
int IR_weights[4] = {-20,-10,-5,5};

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

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  set_forward();
  delay(2000);
}


void loop()

{

  read_IR();
  if (IR_val[0] ==0 && IR_val[1] ==0 && IR_val[2] ==0 && IR_val[3] ==0){
    stop();
    while(1){}
  }
PID_control();
  set_speed();
}

void PID_control() {

  error =0;

  for (int i = 0; i < 4; i++)
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
    
void read_IR() {

  for (int i = 0; i < 4; i++)
  {
    IR_val[i] = analogRead(i);
  }
}

void set_speed() {

  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}

void set_forward() {

  digitalWrite(LMotorA, HIGH);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, HIGH);
  digitalWrite(RMotorB, LOW);
}

void stop() {

  digitalWrite(LMotorA, LOW);
  digitalWrite(LMotorB, LOW);
  digitalWrite(RMotorA, LOW);
  digitalWrite(RMotorB, LOW);
}
