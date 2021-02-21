#include <Arduino.h> /// Arduino main lib
#include <PID_v1.h>  /// PID liblary

unsigned long time = 0;     // time variable for filter
unsigned long oldTime = 0;  // time variable for filter
int value;
int oldValue;
int out;   /// filter output

#define AIA 10  // driver input A
#define AIB 11  // driver input B

/* PID definition */
double Setpoint, Input, Output; 
double Kp=0.7, Ki=5, Kd=0.05; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/* PID definition end */

//// function for driving motor
void motor(int velocity){
  if (velocity >= 0)
  {
    digitalWrite(AIA, LOW);
    analogWrite(AIB, abs(velocity));
  }
  if(velocity < 0)
  {
    analogWrite(AIA, abs(velocity));
    digitalWrite(AIB, LOW);
  }
}

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;  /// setting PWM freq. to max

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  Serial.begin(115200);      ///Serial port 

  value = analogRead(A0);
  oldValue = value;

  //motor definition
  pinMode(AIA, OUTPUT);
  pinMode(AIB, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
    // read the incoming byte:
    Setpoint = Serial.read();
  }

  //Setpoint = analogRead(A1);
  Input = out;
  myPID.Compute();
  motor(Output);
  
  value = analogRead(A0);
  
  Serial.print(out);
  Serial.print(", ");
  Serial.print(value);
  Serial.print(", ");
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