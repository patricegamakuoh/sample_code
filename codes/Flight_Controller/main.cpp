/* A flight control sofware based on STM32F1_arduino project 
   Developed by: Patrice Gama */

#include <EEPROM.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include "Globals.h"
#include "Arduino.h"

TwoWire HWire (2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.



//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
float battery_compensation = 40.0;
float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).


const uint8_t gyro_address = 0x68;         //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.




//*********************Declaring global variables*******************


uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;

int32_t channel_1_start,channel_1, channel_1_base, pid_roll_setpoint_base;
int32_t channel_2_start,channel_2, channel_2_base, pid_pitch_setpoint_base;
int32_t channel_3, channel_4, channel_5, channel_6, channel_7, channel_8;
int32_t channel_3_start,channel_4_start,channel_5_start,channel_6_start,channel_7_start,channel_8_start;
int32_t measured_time, measured_time_start, receiver_watchdog;

//GPS variables
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;


float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;


//**************************************setup amd run once*********************************************
void setup() {

#if defined(DEBUG)
  Serial.begin(57600);                                         //Set the serial output to 57600 kbps. (for debugging only)
  delay(2500);                                                 //Give the serial port some time to start to prevent data loss.
  Serial.println(" ON4CRM DRONE CONTROLLER DEBUG");
#endif

  pinMode(4, INPUT_ANALOG);                                     //This is needed for reading the analog value of port A4 (battery voltage)
  Mode(PB0, OUTPUT);                                            //Set PB0 as output for telemetry TX.
  pinMode(PB3, OUTPUT);                                         //Set PB3 as output for red LED.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output for blue LED.
  pinMode(PB5, OUTPUT);                                         //Set PB5 as output for green LED.
  pinMode(STM32_board_LED, OUTPUT);                             //This is the LED on the STM32 board. Used for GPS indication.
  digitalWrite(STM32_board_LED, HIGH);                          //Turn the LED on the STM32 off. The LED function is inverted. Check the STM32 schematic.

  green_led(HIGH);                                              //status Leds check
  delay(500);
  green_led(LOW);  
  
  red_led(HIGH); 
  delay(500);
  red_led(LOW);
  
  blue_led(HIGH); 
  delay(500);
  blue_led(LOW);                                                

  //EEPROM emulation setup
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;

  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.

  gps_setup();                                                  //Set the baud rate and output refreshrate of the GPS module.
  
  //Check if the MPU-6050 is responding.
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(compass_address);                     
  error = HWire.endTransmission();                              
  while (error != 0) {                                          
    error = 2;                                                  
    error_signal();                                            
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }


  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(baro_address);                        //Start communication with the MS5611.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  
  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  
  battery_voltage = (float)analogRead(4) / 112.81;

  //receiver_watchdog = 850;                                      // avoid RTH 
  loop_timer = micros();                                        //Set the timer for the first loop.
}





//********************************Main program loop***************************************
void loop() {
  
  //some subroutines
  flight_mode_signal();                                                            //Show the flight_mode via the green LED.                                                                 //Show the error via the red LED.
  gyro_signalen();                                                                 //Read the gyro and accelerometer data.
  read_gps();

  if (micros() - loop_timer > 4050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  
  loop_timer = micros();                                                           //Set the timer for the next loop.
}