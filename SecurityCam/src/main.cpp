#include <Arduino.h>
const int debugLED = 3;
const int motionPin = 6;
bool initialDelay = false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(debugLED,OUTPUT);
  pinMode(motionPin,INPUT);
}

void loop() {
  if (initialDelay == false){
    initialDelay = true;
    delay(3000);
  }
  // put your main code here, to run repeatedly:
  if(digitalRead(motionPin) == HIGH)
  {
    digitalWrite(debugLED,HIGH);
  }
  else
  {
    digitalWrite(debugLED,LOW);
  }
  
  Serial.println(digitalRead(motionPin));
}