
#include "Globals.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void red_led(int8_t level) {
  digitalWrite(PB5,!level);    //!level voorled op opshield
}
void green_led(int8_t level) {
  digitalWrite(PB3,!level);
}
void blue_led(int8_t level) {
  digitalWrite(PB4,!level);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void) {
  if (error >= 100) red_led(HIGH);                                                  
  else if (error_timer < millis()) {                                                     
    error_timer = millis() + 250;                                                          
    if (error > 0 && error_counter > error + 3) error_counter = 0;                        
    if (error_counter < error && error_led == 0 && error > 0) {                        
      red_led(HIGH);                                                                      
      error_led = 1;                                                                      
    }
    else {                                                                                 
      red_led(LOW);                                                                       
      error_counter++;                                                                    
      error_led = 0;                                                                      
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the flight mode LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                    
    flight_mode_timer = millis() + 250;                                             
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; 
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    
      green_led(HIGH);                                                                     //Turn the LED on.
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                               .
      green_led(LOW);                                                                      
      flight_mode_counter++;                                                              
      flight_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}
