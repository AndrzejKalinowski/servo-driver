#include <Arduino.h> /// Arduino main lib
#include <PID_v1.h>  /// PID liblary
unsigned long time = 0;
unsigned long oldTime = 0;
int value;
int oldValue;
int out;

/* PID definition */
double Setpoint, Input, Output; 
double Kp=0.7, Ki=5, Kd=0.05; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/* PID definition end */

//// function for driving motor
int vel;
void motor(int velocity){
  if (velocity >= 0)
  {
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
  }
  if(velocity < 0)
  {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }
  analogWrite(10, abs(velocity));
}

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;
  //Setpoint = 300;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-254, 254);

  Serial.begin(115200);

  value = analogRead(A0);
  oldValue = value;

  //motor definition
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  Setpoint = analogRead(A1);
  Input = out;
  myPID.Compute();
  motor(Output);
  
  value = analogRead(A0);
  
  Serial.print(out);
  Serial.print(", ");
  Serial.print(value);
  //Serial.print(out);
  Serial.print(", ");
  //motor(Output);
  Serial.print(Output);
  Serial.print(", ");
  Serial.println(Setpoint);
  //filter
  time = millis();
  if ((time - oldTime) >= 1)
  {
    if ((value - oldValue) >= 3 || (oldValue - value) >= 3)
    {
      out = value;
      oldValue = value;
    }
    else
    {
      out = oldValue;
    }
  }
  //filter end

}