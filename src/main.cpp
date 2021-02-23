#include <Arduino.h> /// Arduino main lib

unsigned long time = 0;     // time variable for filter
unsigned long oldTime = 0;  // time variable for filter
int value;
int oldValue;
int out;   /// filter output

#define AIA 10  // driver input A
#define AIB 11  // driver input B

double Setpoint = 300;

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
    Setpoint = Serial.parseInt();
  }

  //Setpoint = analogRead(A1);
  value = analogRead(A0);

  if(Setpoint > out) {
    motor(200);
  }
  else {
    motor(-200);
  }
  
  Serial.print(out);
  Serial.print(", ");
  Serial.print(value);
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