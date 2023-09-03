#include "VescUart.h"
#define stopPin 8
#define startPin 7
VescUart VESCUART;

//const uint8_t PotentiometerPin = A0;
 //int stopPin = 8;
 //int startPin = 7;
 short state = 0;
  uint32_t LastTime = 0; 
  uint32_t readIn = 0;
void setup() {
  Serial.begin(115200);
  pinMode(startPin,INPUT);
  //digitalWrite(stopPin, HIGH);
  pinMode(startPin, INPUT);
  //digitalWrite(startPin, HIGH);
  

  VESCUART.setSerialPort(&Serial);

}

void loop() {


 // Serial.println(digitalRead(startPin));
  

  if((millis() - LastTime) > 10){

    delay(750);
    //VESCUART.nunchuck.valueY = map(analogRead(PotentiometerPin), 0, 1023, 0, 255);
    //Serial
    //Serial.println(VESCUART.data.vrpm);
    readIn = 0;
    readIn= digitalRead(startPin);
    //readIn = readIn<<1;
    //readIn = readIn & digitalRead(stopPin);
    
    if(readIn == 1)
    {
    VESCUART.setRPM(1);
    VESCUART.setCurrent(1);
    }
    else
    {
    VESCUART.setRPM(0);
    VESCUART.setCurrent(0);
    }
   
    //VESCUART.setNunchuckValues();
   // VESCUART.setDuty(0.1);
    
    LastTime = millis();

  } 


} 

 