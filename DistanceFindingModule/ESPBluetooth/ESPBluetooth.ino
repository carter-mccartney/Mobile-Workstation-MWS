/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEUUID.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; //In seconds
BLEScan* pBLEScan;
bool isPaused;
bool isMeasuring;
byte i;
byte total;
String connectedName;
int rssi[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int callibrationConstant = -69;
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) 
    {
      const char* c_name = advertisedDevice.getName().c_str();
      int stringLength = strlen(c_name);

      bool doesNameMatch = stringLength == connectedName.length();
      for(int j = 0; j < stringLength && doesNameMatch; j++)
      {
        doesNameMatch = doesNameMatch && c_name[j] == connectedName.charAt(j);
      }
      
      if(doesNameMatch &&
         advertisedDevice.getServiceUUID().equals(BLEUUID("560D5EE7-2C11-478F-AB58-07211516C261")))
      {
        rssi[i] = advertisedDevice.getRSSI();
        i++;
        i = i % 16;
        total++;
      }
    }
};

void setup() {
  Serial.begin(9600);
  isPaused = true;
  isMeasuring = false;
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
    delay(250);
    if(Serial.available() > 0)
    {
      while(Serial.available() > 0)
      {
        String command = Serial.readStringUntil('\n');
        Serial.print(command);
        if(command.compareTo("ID") == 0)
        {
          Serial.write("\nTwo\n");
          continue;
        }
        else if(command.compareTo("DISTANCE") == 0)
        {
          isPaused = false;
          continue;
        }
        else if(command.compareTo("CALIBRATE") == 0)
        {
          // Computes a new callibration value.
          isPaused = false;
          continue;
        }
        else if(command.compareTo("SETCAL") == 0)
        {
          // Gives a known callibration value.
          callibrationConstant = atoi(command.substring(command.indexOf(('L'), command.length() - 1)).c_str());
          Serial.print("\n");//Might not be needed
          Serial.flush();
        }
        else if(command.indexOf("NAME") > 0)
        {
          connectedName = command.substring(command.indexOf(('E'),command.length() - 1));
          Serial.print("\n");//Might not be needed
          Serial.flush();
        }
      }
    }
  }
  else if(isMeasuring)
  {
    if(total >= 6)
    {
      // Pause.
      isPaused = true;
      isMeasuring = false;

      // Get average. Round by adding 1/2 before dividing.
      Serial.printf("\n%3.2f\n", pow(10, (callibrationConstant - (rssi[0] + rssi[1] + rssi[2] + rssi[3] + rssi[4] + rssi[5] + 3) / 6) / 30.0));
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
    }
    else
    {
      delay(2000);
      BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
      pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    }
  }
  else // Must be callibrating.
  {
    if(total >= 16)
    {
      // Pause.
      isPaused = true;

      // Get average. Round by adding 1/2 before dividing.
      callibrationConstant = (rssi[0] + rssi[1] + rssi[2] + rssi[3] + rssi[4] + rssi[5] + rssi[6] + rssi[7] + rssi[8] + rssi[9] + rssi[10] + rssi[11] + rssi[12] + rssi[13] + rssi[14] + rssi[15] + 8) / 16;
      Serial.printf("\n%i\n", callibrationConstant);
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
    }
    else
    {
      delay(2000);
      BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
      pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    }
  }
}
