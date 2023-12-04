/*
  Name:    Attempt_Pmm.ino
  Created: 27-10-2023
  Author:  Carter McCartney
  Description: We start by setting up the ports for two Vesc Motors (left & right). Then, we listen for Serial3 writes from ROS.
                When we receive a serial input, we check if it is a velocity update. If so, we update the velocity of both
                motors accordingly.

                This code will step up (and step down [not yet]) the velocities: we will not just go at the speed ROS gives us immediately.

                Duty cycle should be completely excluded: If we step up (and step down), then we can set the minimum --> we won't
                go lower than the minimum setRPM value.

                The Motors are set to turn off after 1 second of no new commands. We will continue to set the most recent serial
                written command from ROS for 2 seconds and then no more until another command is sent. 

                The velocity will update upon a new command. Upon no new command, the code will continually increment the motor
                velocities towards the expected/new velocities.

                THIS CODE DOES NOT STEP UP OR STEP DOWN. IT SIMPLY GOES TO THE END SPEED IMMEDIATELY.
*/
#include <VescUart.h>

/*
  3 - Convert both new velocities from m/s to erpm
    60      :   converting time - seconds to minutes
    100     :   converting meters to cm
    9.5     :   conversion factor for rpm to erpm
    14.7    :   size of the wheel diameter in cm
    3.14159 :   value of pi

    VELOCITY_CONVERSION is equal to (60*100*9.5)/(14.7*3.14159)
*/
float VELOCITY_CONVERSION = (60*100*9.5)/(14.7*3.14159);

// Declare constants
float VEL_MAX = 1250;     // max speed is [1250 erpm] == [131 rpm] == [<4mph]
float VEL_MAX_NEG = -1250;
float VEL_MIN = 100;      // limit of the minimum speed we will start stepping up from. If the motor is not moving, we will step to this value first.
float VEL_MIN_NEG = -500;
int VEL_COUNTS_MAX = 10;  // this is the vel_count limit (maximum). Ten counts is equivilent to one second.


/** Initiate VescUart objects for the left motor and right motor */
VescUart LeftMotor;
VescUart RightMotor;

// Declare variables for the erpm for both motors, cmd string, current erpm values for both motors, counts for velocity updates
float left_motor_erpm = 0;                  // the erpm value that we are trying to achieve/set the left motor to
float right_motor_erpm = 0;                 // the erpm value that we are trying to achieve/set the right motor to
String CMD = "";                            // the command that is sent via serial to establish a connection with the arduino OR update the velocities of the motors
float left_motor_erpm_current = -1;         // current left motor velocity --> this is going to be stepped up or down each loop iteration to achieve the desired velocity
float right_motor_erpm_current = -1;         // current right motor velocity --> this is going to be stepped up or down each loop iteration to achieve the desired velocity
int vel_count = 0;                          // this count is incremented with each iteration through the loop. It is reset to 0 when a new velocity update comes in. 

float left_motor_velocity_current = -1;     // Current left motor velocity that is sent to serial.
float right_motor_velocity_current = -1;    // Current right motor velocity that is sent to serial.

long left_motor_tachometer_current = -1;     // Current left motor velocity that is sent to serial.
long right_motor_tachometer_current = -1;    // Current right motor velocity that is sent to serial.

long left_motor_tachometer_offset = 0;      // Holds the starting value of the left motor tachometer upon arduino startup. Needed to zero out the tachometer.
long right_motor_tachometer_offset = 0;     // Holds the starting value of the right motor tachometer upon arduino startup. Needed to zero out the tachometer.

int byteReceive = 0;

void setup() {
  Serial.begin(9600);

  /** Setup LeftMotor port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  /** Setup RightMotor port (Serial1 on Atmega32u4) */
  Serial2.begin(115200);
 
  while(!Serial) {;}
  while (!Serial1) {;}
  while (!Serial2) {;}

  /* Define which ports to use as UART */
  LeftMotor.setSerialPort(&Serial1);
  RightMotor.setSerialPort(&Serial2);
}

void loop() {
  
  /* 
    vel_count increments outside of serial.available. If count < 20, set velocities for the motors. else, do not. 
    Increment count inside of serial.available function.
  */
  vel_count++;

  if (Serial.available())
  {
    /* Read in a byte from serial connected to rpi. */
    byteReceive = Serial.read();
    // Serial3.print((char)byteReceive); //Used for debugging by echoing serial port.

    /* We have received a null terminating character from rpi, so process the command. */
    if (byteReceive == '\n')
    {
      // -------------------------------- BEGINNING OF Adjust motor velocities (left & right) ------------------------------ 
      /*
        1 - Check if this serial write is for a velocity update. It should look like: "V[floatForLeftMotorVelocity],[floatForRightMotorVelocity]"
        2 - If it is a velocity update, grab and store the floats. Do this using "toFloat".
        3 - Convert both left and right velocities from m/s to erpm.
        4a - If erpm is 0, apply brake current immediately? or work our way down?
        4b - If erpm is < 900, set duty cycle to the appropriate value.
        4c - If erpm is >= 900, set rpm to the erpms we received.
      */

      // 1 - Check for a "V" at the beginning of the string indicating that the velocity is being updated
      /* Check that the received command starts a 'V' character, is ten bytes long, and has a ',' character in the 5th index.
      * This is to check to see if the 'set velocity' command received is in the correct format of "VLLLL,RRRR" where the 'L' and the 'R' 
      * represent the bytes of the float value for the velocity of the left and right wheel respectively. 
      * If a 'set velocity' command does not adhere to the "VLLLL,RRRR" format, then do not process the command. 
      * This 'if statement' does not check if the floating point values are in a valid velocity range, this is done else-where.
      * NOTE: A float is four bytes of length.
      */
      if (CMD.charAt(0) == 'V' && CMD.indexOf(',') != -1)
      {
        // 2 - Update the left motor speed variable with the new velocity
        int commaIndex = CMD.indexOf(',');
        int lengthOfCMD = CMD.length();

        left_motor_erpm = CMD.substring(1, commaIndex).toFloat();
        // 2 - Update the right motor speed variable with the new velocity
        right_motor_erpm = CMD.substring(commaIndex+1, lengthOfCMD).toFloat();

        //Serial.print(CMD.substring(1, commaIndex)); Serial.print(" "); Serial.print(CMD.substring(commaIndex+1, lengthOfCMD)); Serial.println(""); // Used for debugging recieved velocity values.
        //Serial.print(left_motor_erpm); Serial.print(" "); Serial.print(right_motor_erpm); Serial.println(""); // Used for debugging converted velocity values.
        
        /*
          3 - Convert both new velocities from m/s to erpm
            60      :   converting time - seconds to minutes
            100     :   converting meters to cm
            9.5     :   conversion factor for rpm to erpm
            14.7    :   size of the wheels in diameter
            3.14159 :   value of pi

            VELOCITY_CONVERSION is equal to (60*100*9.5)/(14.7*3.14159)
        */
        left_motor_erpm *= VELOCITY_CONVERSION;
        right_motor_erpm *= VELOCITY_CONVERSION * -1;

        // reset vel_count
        vel_count = 0;
      }

      /* Allows reading of current velocity values and the tachometer values back to the rpi. */
      else if (CMD.charAt(0) == 'R')
      {
        Serial.flush();

        LeftMotor.getVescValues();
        RightMotor.getVescValues();

        /* Send current velocity to the host. 
        * The message should be in the format L.L,R.R\n  where the L.L represents the 
        * floating point number of the left wheel velocity and the R.R represents the 
        * floating point number of the right wheel velocity with both values separated
        * by a comma character and the message is appended with a newline character '\n'
        * as shown above.
        */
        right_motor_velocity_current = RightMotor.data.rpm / VELOCITY_CONVERSION * -1;
        left_motor_velocity_current = LeftMotor.data.rpm / VELOCITY_CONVERSION;
        
        Serial.print(left_motor_velocity_current);
        Serial.print(","); 
        Serial.print(right_motor_velocity_current); 
        Serial.println("");


        /* Send tachometer value to the host. 
        * The message should be in the format LLL,RRR\n  where the LLL represents the 
        * long integer of the left wheel tachometer and the RRR represents the 
        * long integer of the right wheel tachometer with both values separated
        * by a comma character and the message is appended with a newline character '\n'
        * as shown above.
        */
        right_motor_tachometer_current = (RightMotor.data.tachometer * -1) - right_motor_tachometer_offset;
        left_motor_tachometer_current = LeftMotor.data.tachometer - left_motor_tachometer_offset;

        Serial.print(left_motor_tachometer_current);
        Serial.print(","); 
        Serial.print(right_motor_tachometer_current);
        Serial.println("");
      }

      else if (CMD.compareTo("ID") == 0)
      {
        LeftMotor.getVescValues();
        RightMotor.getVescValues();

        left_motor_tachometer_offset = LeftMotor.data.tachometer;
        right_motor_tachometer_offset = RightMotor.data.tachometer * -1;

        Serial.flush();
        Serial.write("This is Arduino\n"); 
      }

      /* Reset the CMD string to an empty state. */ 
      CMD = "";
    }
    else
    {
      /* A null terminating character was not received yet so append the received byte to the CMD string */
      CMD += (char)byteReceive;
    }
  } // end "if Serial3.available"

  if (vel_count <= VEL_COUNTS_MAX)
  {
    // 4 - update the speed: 0 = set brake current ; <500 = output message because something is not right. We aren't setting duty cycle anymore ; >=500 = setRPM   
    if (left_motor_erpm == 0) 
    {
      // TODO MAYBE :: SETUP STEP DOWN FOR BRAKING IF IT JOLTS WHEN BRAKES ARE APPLIED
      // if the motor is already set to velocty=0, then set the brakeCurrent to 0 so that the motors do not spin and no current is applied to lock the motors
      if (left_motor_erpm_current == 0)
      {
        LeftMotor.setBrakeCurrent(0); // This will keep the motors from moving, yet leave them free to move by the user
      }
      else // the left motor is currently moving, so make it brake
      {
        LeftMotor.setBrakeCurrent(5); // This immediately applies brake current
      }

      // adjust the current motor rpm variable to reflect it's new value
      left_motor_erpm_current = left_motor_erpm;
    }
    else // set the new left motor velocity that is nonzero
    {
      // if the new speed is between +-max negative velocity
      if (!(left_motor_erpm > VEL_MIN_NEG && left_motor_erpm < VEL_MIN)) 
      {
        // if the new speed is less than max negative or greater than max.
        if (!(left_motor_erpm < VEL_MAX_NEG || left_motor_erpm > VEL_MAX))
        {
          // TODO :: ===>> if the difference between the current velocity and new velocity is greater than 47, then step up
          LeftMotor.setRPM(left_motor_erpm);
          left_motor_erpm_current = left_motor_erpm;
        }
        //else Serial.println("Left motor velocity was either greater than max or less than neg max.");//LeftMotor.setDuty(left_motor_erpm / (9.5 * 1600)); // dutycycle: [r = 1600x], where 'r' is rpm and 'x' is duty cycle
      }
      //else Serial.println("Left motor velocity was between min and min neg.");//LeftMotor.setDuty(left_motor_erpm / (9.5 * 1600)); // dutycycle: [r = 1600x], where 'r' is rpm and 'x' is duty cycle
    }

    // 4 - update the speed of the right motor
    if (right_motor_erpm == 0) 
    {
      // TODO MAYBE :: SETUP STEP DOWN FOR BRAKING IF IT JOLTS WHEN BRAKES ARE APPLIED
      // if the motor is already set to velocty=0, then set the brakeCurrent to 0 so that the motors do not spin and no current is applied to lock the motors
      if (right_motor_erpm_current == 0)
      {
        RightMotor.setBrakeCurrent(0); // This will keep the motors from moving, yet leave them free to move by the user
      }
      else // the right motor is currently moving, so make it brake
      {
        RightMotor.setBrakeCurrent(5); // This immediately applies brake current
      }

      // adjust the current motor rpm variable to reflect it's new value
      right_motor_erpm_current = right_motor_erpm;
    }
    else // set the new right motor velocity that is nonz
      // if the new speed is between +-max negative velocity
      if (!(left_motor_erpm > VEL_MIN_NEG && left_motor_erpm < VEL_MIN)) 
      {
        // if the new speed is less than max negative or greater than max.
        if (!(left_motor_erpm < VEL_MAX_NEG || left_motor_erpm > VEL_MAX))
        {
          RightMotor.setRPM(right_motor_erpm);
          right_motor_erpm_current = right_motor_erpm;
        }
        //else Serial.println("Right motor velocity was either greater than max or less than neg max.");//LeftMotor.setDuty(left_motor_erpm / (9.5 * 1600)); // dutycycle: [r = 1600x], where 'r' is rpm and 'x' is duty cycle
      }
      //else Serial.println("Right motor velocity was between min and min neg.");//LeftMotor.setDuty(left_motor_erpm / (9.5 * 1600)); // dutycycle: [r = 1600x], where 'r' is rpm and 'x' is duty cycle
    }

    //Serial.print(left_motor_erpm); Serial.print(" "); Serial.print(right_motor_erpm); Serial.println(""); // Used for debugging executed rpm values.
   // Serial.print(left_motor_tachometer_offset); Serial.print(" "); Serial.print(right_motor_tachometer_offset); Serial.println(""); // Used for debugging current velocity (m/s) values.

  } // end of update speed (if vel_count <= VEL_COUNTS_MAX)
  // -------------------------------- END OF Adjust motor velocities (left & right) ------------------------------ 

  // Print out the current erpms of both motors to check where they are at
  /* 
  LeftMotor.getVescValues();
  Serial3.println("Left Motor erpm  : "); Serial3.print(LeftMotor.data.rpm);
  RightMotor.getVescValues();
  Serial3.println("Right Motor erpm : "); Serial3.print(RightMotor.data.rpm);
  */

} // end loop
