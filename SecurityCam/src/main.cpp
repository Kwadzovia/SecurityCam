#include <Arduino.h>
#include <avr/sleep.h>

const int debugLED = 6;
const int motionInterrupt = 3;
bool initialDelay = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(debugLED,OUTPUT);
  pinMode(motionInterrupt,INPUT);
}

void wakeUp(){
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(3));
  Serial.println("BODY DETECTED");
}

void Go_To_Sleep(){
  Serial.println("GOING TO SLEEP");
  sleep_enable();
  EIFR |= 0x02; //Clear interrupt 1 flag (Datasheet)
  attachInterrupt(digitalPinToInterrupt(3),wakeUp,HIGH);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  digitalWrite(debugLED,LOW);
  delay(1000);
  sleep_cpu();
  Serial.println("SUCCESSFUL WAKEUP");
  digitalWrite(debugLED,HIGH);
}



void loop() {
  digitalWrite(debugLED,HIGH);
  //Go_To_Sleep();


  if (initialDelay == false){
    initialDelay = true;
    delay(3000);
  }

  
  // // put your main code here, to run repeatedly:
  // if(digitalRead(motionInterrupt) == HIGH)
  // {
  //   digitalWrite(debugLED,HIGH);
  // }
  // else
  // {
  //   digitalWrite(debugLED,LOW);
  // }
  
  // Serial.println(digitalRead(motionInterrupt));
}

