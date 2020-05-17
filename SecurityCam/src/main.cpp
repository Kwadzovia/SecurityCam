#include <Arduino.h>
#include <avr/sleep.h>

#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>

// comment out this line if using Arduino V23 or earlier
#include <SoftwareSerial.h>         



const int debugLED = A5;
const int motionInterrupt = 6;
const int motorControlVcc = A4;
const int motorControlENABLE = 5;


bool initialDelay = false;
unsigned long initialTime = 0;
unsigned long currentTime = 0;

 // SD card chip select line varies among boards/shields:
  // Adafruit SD shields and modules: pin 10
  // Arduino Ethernet shield: pin 4
  // Sparkfun SD shield: pin 8
  // Arduino Mega w/hardware SPI: pin 53
  // Teensy 2.0: pin 0
  // Teensy++ 2.0: pin 20
  #define chipSelect 10

  // Using SoftwareSerial (Arduino 1.0+) or NewSoftSerial (Arduino 0023 & prior):
  #if ARDUINO >= 100
  // On Uno: camera TX connected to pin 2, camera RX to pin 3:
  SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
  // On Mega: camera TX connected to pin 69 (A15), camera RX to pin 3:
  //SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
  #else
  NewSoftSerial cameraconnection = NewSoftSerial(2, 3);
  #endif

  Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(debugLED,OUTPUT);
  pinMode(motionInterrupt,INPUT);
  
  cam.setMotionDetect(true);           // turn it on
  //cam.setMotionDetect(false);        // turn it off   (default)

}

void wakeUp(){
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(3));
  Serial.println("BODY DETECTED");
  //Turn on Camera

  
  initialTime = millis();
}

void Go_To_Sleep(){
  Serial.println("GOING TO SLEEP");
  cam.setMotionDetect(false);
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

void Take_Picture(){

  Serial.println("Attempting to Take Picture...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }  
  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }

    // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);          // small

  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");

}

void cameraOFF(){

}

void cameraON(){

}

void loop() {
  currentTime = millis();
  digitalWrite(debugLED,HIGH);
  cam.setMotionDetect(true);

  if (initialDelay == false){
    initialDelay = true;
    delay(5000);
  }

  
  //if(digitalRead(motionInterrupt) == HIGH)
  
  if (cam.motionDetected())
  { 
    cam.setMotionDetect(false);
    Take_Picture();
    cam.setMotionDetect(true);
  }
  else
  {
    if( currentTime-initialTime >= 15000)
    { 
      //15 seconds runtime
      digitalWrite(debugLED,LOW);
      cam.setMotionDetect(false);

      //Turn off camera
      Go_To_Sleep();
    }
  }
  
  // Serial.println(digitalRead(motionInterrupt));
}

