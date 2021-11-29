/*Name: Patrice Gama
  Date: 2019*/

#include "Globals.h"

//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
void handler_channel_1(void) {
  measured_time = TIMER2_BASE->CCR1 - measured_time_start;
  if (measured_time < 0)measured_time += 0xFFFF;
  measured_time_start = TIMER2_BASE->CCR1;
  
  if (measured_time > 3000) {
    channel_select_counter = 0;
    receiver_watchdog = 0;
  }
  else channel_select_counter++;

  if (channel_select_counter == 1)channel_1 = measured_time;
  if (channel_select_counter == 2)channel_2 = measured_time;
  if (channel_select_counter == 3)channel_3 = measured_time;
  if (channel_select_counter == 4)channel_4 = measured_time;
  if (channel_select_counter == 5)channel_5 = measured_time;
  if (channel_select_counter == 6)channel_6 = measured_time;
  if (channel_select_counter == 7)channel_7 = measured_time;
  if (channel_select_counter == 8)channel_8 = measured_time;

  #if defined (DEBUG_ISR) 
      Serial.print(">");
      Serial.print(channel_select_counter);
      Serial.print(" | ");
      Serial.println(measured_time);
           
    #endif
}
