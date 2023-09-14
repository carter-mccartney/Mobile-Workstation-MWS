/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int8_t rssiReference = -68; 
char deviceName[32];
int scanTime = 5; //In seconds
BLEScan* pBLEScan;
bool isPaused;
byte i;
byte total;
int rssi[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) 
    {
      std::__cxx11::string other = advertisedDevice.getName();
      if(other == deviceName)
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
      String transmission = Serial.readString();
      transmission.trim();
      Serial.printf("%s\n", transmission);

      if(transmission.length() >= 1)
      {
        if(transmission.charAt(0) == 'g' &&
           transmission.charAt(1) == 'o' &&
           transmission.length() == 2)
        {
          isPaused = false;
          Serial.println("Started");
        }
        else if(transmission.charAt(0) == 'n')
        {
          // What follows is the name.
          strcpy(deviceName, transmission.substring(1).c_str());
          Serial.printf("%s", deviceName);
        }
        else if(transmission.charAt(0) == 'r')
        {
          // What follows is the reference RSSI.
          rssiReference = -transmission.substring(1).toInt();
          Serial.printf("%i", rssiReference);
        }
      }
    }
  }
  else
  {
    if(total >= 16)
    {
      // Pause.
      isPaused = true;

      // Get average.
      Serial.printf("Average Distance: %3.2f\n", pow(10, (rssiReference - (rssi[0] + rssi[1] + rssi[2] + rssi[3] + rssi[4] + rssi[5] + rssi[6] + rssi[7] + rssi[8] + rssi[9] + rssi[10] + rssi[11] + rssi[12] + rssi[13] + rssi[14] + rssi[15]) / 16.0) / 30.0));
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
