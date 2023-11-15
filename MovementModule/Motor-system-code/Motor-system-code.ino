/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/
#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;
float RPM_actual;
int eRPM_Current = 0;
int eRPM_Previous = 0;
String CMD = "";
//String RPM_actual_str = "";

int cmd_INDEX = 1;

void setup() {
  Serial.begin(9600, SERIAL_8N1);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);

  while(!Serial) {;}
  
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
  
  //Serial.print(cmd_INDEX);

}

void loop() {
  
   

   if (Serial.available())
   {
        CMD = Serial.readStringUntil('\n');
        

        if (CMD.compareTo("ID") == 0)
        {
           Serial.flush();
           Serial.write("This is Arduino\n");
           
        }

        
        if (CMD.compareTo("MOVE") == 0)
        //if (CMD.startsWith("MOVE"))
        {
            cmd_INDEX = 2;
            eRPM_Current = 1000;

            //Serial.flush();
            //Serial.write("MOVE\n");
            
            //Serial.flush();
            //Serial.write("This is Arduino\n");
        }
        else if (CMD.compareTo("STOP") == 0)//else if (CMD.startsWith("STOP"))
        {
            cmd_INDEX = 1;
        }
        else
        {
            if (CMD.compareTo("") != 0)
            {
                Serial.write("****Not a valid command.****\n");
            }
        }

        if (eRPM_Current != eRPM_Previous) // if the eRPM is changed, then indicate update.
        {
            //RPM_actual = eRPM_Current / 9.5;
            //RPM_actual_str = String(RPM_actual);
            Serial.flush();
            Serial.write("This is Arduino\n");
            //Serial.print(cmd_INDEX);
            //Serial.print(" ");
            //Serial.println(eRPM_Current);

            //Serial.write((char)RPM_actual);
            //Serial.write(" RPM\n");
            //Serial.write("This is Arduino\n");
              
            eRPM_Previous = eRPM_Current;
        }
            

        
   }

   if (cmd_INDEX == 2)
    {
        if (eRPM_Current < 4000)
        {
            eRPM_Current = eRPM_Current + 100;
        }
    }

    if (cmd_INDEX == 1)
    {
        if (eRPM_Current > 900)
        {
            eRPM_Current = eRPM_Current - 100;
        }
        else
        {
            if (eRPM_Current == 900)
            {
                eRPM_Current = 0;
            }
              
        }
    }

   UART.setRPM(eRPM_Current);
   UART.getVescValues();
   //Serial.println(UART.data.rpm);

   delay(100); 
  
}