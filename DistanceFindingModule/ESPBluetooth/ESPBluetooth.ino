/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEUUID.h>
#include <BLEAdvertisedDevice.h>

// The scanner for bluetooth advertisements.
BLEScan* pBLEScan;

// Whether nothing is running.
bool isPaused;

// Whether measurement is occurring.
bool isMeasuring;

// The current index in the list of RSSI values.
byte i;

// The total number of RSSI values obtained.
byte total;

// The name of the device to look for.
String connectedName;

// The set of RSSI values obtained.
int rssi[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// The constant used for RSSI at one meter for calculating distance.
int calibrationConstant = -69;

// Contains a callback for results of BLE scans.
class LogScanCallback : public BLEAdvertisedDeviceCallbacks 
{
  // The callback for a scan result.
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    // Check for a matching name.
    const char* c_name = advertisedDevice.getName().c_str();
    int stringLength = strlen(c_name);
    bool doesNameMatch = stringLength == connectedName.length();
    for(int j = 0; j < stringLength && doesNameMatch; j++)
    {
      doesNameMatch = doesNameMatch && c_name[j] == connectedName.charAt(j);
    }
    
    if(doesNameMatch )//&&
       // Check fo a matching UUID/
       //advertisedDevice.getServiceUUID().equals(BLEUUID("560D5EE7-2C11-478F-AB58-07211516C261")))
    {
      // Log the RSSI if matching.
      rssi[i] = advertisedDevice.getRSSI();
    
      // Loop around in buffer and log.
      i++;
      i = i % 16;
      if(total < 16)
      {        
        total++;
      }
    }
  }
};

// Sets up the controller.
void setup() 
{
  // Communicates at 9600 baud for I/O.
  Serial.begin(9600);

  // Should not do anything at startup.
  isPaused = true;
  isMeasuring = false;

  // Initialize data.
  i = 0;
  total = 0;

  // Set up bluetooth.
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new LogScanCallback()); // Register the callback.
  pBLEScan->setActiveScan(true); // Actively scan to get faster results.
  pBLEScan->setInterval(200); // Scan for 200ms at a time.
  pBLEScan->setWindow(200); // Actively scan entire interval.
}

// Runs the controller.
void loop() 
{
  // Determine if state has changed and respond to simple requests.
  if(Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    if(command.compareTo("ID") == 0)
    {
      Serial.write("Two\n");
    }
    else if(command.indexOf("NAME") >= 0)
    {
      int splitIndex = command.indexOf('E');
      connectedName = command.substring(splitIndex + 1);
      Serial.flush();
    }
    else if(isPaused)
    {
      if(command.compareTo("DISTANCE") == 0)
      {
        isPaused = false;
        isMeasuring = true;
      }
      else if(command.compareTo("CALIBRATE") == 0)
      {
        isPaused = false;
        isMeasuring = false;
      }
      else if(command.indexOf("SETCAL") >= 0)
      {
        // Gives a known calibration value.
        int splitIndex = command.indexOf('L');
        String result = command.substring(splitIndex + 1);
        calibrationConstant = atoi(result.c_str());
        Serial.flush();
      }
    }
  }

  // Run logic.
  if(isPaused)
  {
    delayMicroseconds(100);
  }
  else if(isMeasuring)
  {
    // Start scan.
    BLEScanResults foundDevices = pBLEScan->start(4, false);
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release 

    // Pause.
    isPaused = true;
    isMeasuring = false;

    // Output reading.
    int result = 0;
    for(int j = 0; j < total; j++)
    {
      result += rssi[j];
    }
    result /= total;
    Serial.printf("%3.2f\n", pow(10, (calibrationConstant - result) / 30.0));
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
  else // Must be calibrating.
  {
    if(total >= 16)
    {
      // Pause.
      isPaused = true;
      isMeasuring = false;

      // Get average. Round by adding 1/2 before dividing.
      calibrationConstant = (rssi[0] + rssi[1] + rssi[2] + rssi[3] + rssi[4] + rssi[5] + rssi[6] + rssi[7] + rssi[8] + rssi[9] + rssi[10] + rssi[11] + rssi[12] + rssi[13] + rssi[14] + rssi[15] + 8) / 16;
      Serial.printf("%i\n", calibrationConstant);
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
      BLEScanResults foundDevices = pBLEScan->start(5, false);
      pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
    }
  }
}
