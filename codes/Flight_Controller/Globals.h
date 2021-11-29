///////////////////////////
//  global delcarations  //
///////////////////////////

#define STM32_board_LED PA1

extern class TwoWire HWire;

//GPS variables
extern uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
extern int16_t gps_add_counter;
extern int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
extern int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;

//declarations
void handler_channel_1(void);
void gyro_setup(void);
void gyro_signalen(void);
void calibrate_gyro(void);
void red_led(int8_t level);
void green_led(int8_t level);
void blue_led(int8_t level);
void gps_setup(void);
void read_gps(void);
