/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

uint8_t mac[] = { 0x47, 0x32, 0xee, 0x1f, 0xc6, 0x85 };
int scanTime = 5; //In seconds
BLEScan* pBLEScan;
bool isPaused;
byte i;
byte total;
int rssi[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) 
    {
      BLEAddress address = advertisedDevice.getAddress();
      uint8_t* macAddr = (uint8_t*)address.getNative();
      if(advertisedDevice.getName().c_str()[0] == 'P' &&
         advertisedDevice.getName().c_str()[1] == 'i' &&
         advertisedDevice.getName().c_str()[2] == 'x' &&
         advertisedDevice.getName().c_str()[3] == 'e' &&
         advertisedDevice.getName().c_str()[4] == 'l' &&
         advertisedDevice.getName().c_str()[5] == ' ' &&
         advertisedDevice.getName().c_str()[6] == '5')
      {
        Serial.printf("%i\n", advertisedDevice.getRSSI());
        rssi[i] = advertisedDevice.getRSSI();
        i++;
        i = i % 16;
        total++;
      }
    }
};

void setup() {
  Serial.begin(9600);
  Serial.println("Scanning...");
  isPaused = true;
  i = 0;
  total = 0;

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(200);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  // put your main code here, to run repeatedly:
  if(isPaused)
  {
    if(Serial.available() > 0)
    {
      // Pause.
      isPaused = false;
      while(Serial.available() > 0)
      {
        int incomingByte = Serial.read();
      }
      Serial.println("Started");
    }
  }
  else
  {
    if(total >= 16)
    {
      // Pause.
      isPaused = true;

      // Get average.
      Serial.printf("Average Distance: %3.2f\n", (rssi[0] + rssi[1] + rssi[2] + rssi[3] + rssi[4] + rssi[5] + rssi[6] + rssi[7] + rssi[8] + rssi[9] + rssi[10] + rssi[11] + rssi[12] + rssi[13] + rssi[14] + rssi[15]) / 16.0);
      rssi[0] = 0;
      rssi[1] = 0;
      rssi[2] = 0;
      rssi[3] = 0;
      rssi[4] = 0;
      rssi[5] = 0;
      rssi[6] = 0;
      rssi[7] = 0;
      rssi[8] = 0;
      rssi[9] = 0;
      rssi[10] = 0;
      rssi[11] = 0;
      rssi[12] = 0;
      rssi[13] = 0;
      rssi[14] = 0;
      rssi[15] = 0;
      i = 0;
      total = 0;
      Serial.println("Paused");
    }
    else
    {
      BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
      pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    }
  }
  delay(2000);
}
