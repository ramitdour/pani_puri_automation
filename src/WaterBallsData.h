#include <Arduino.h>

#ifndef WaterBallsData_h
#define WaterBallsData_h

// #define DEBUG_CODE 1 // TODO:comment in productions
#define INVERTED_LOGIC 1 // TODO:Uncomment for inverted logic

#ifdef DEBUG_CODE

#define serial_baud_rate 115200

#define nos_channel 3
const uint8_t echoPins[] = { 5, 7, 9};
const uint8_t trigPins[] = { 4, 6, 8};
const uint8_t output_channels[] = {11, 12, 13};

// #define nos_channel 1
// const uint8_t echoPins[] = {9};
// const uint8_t trigPins[] = {8};
// const uint8_t output_channels[] = {13};

unsigned long previousMillis_cool_off_print = 0;   // For Testing purpose
#define previousMillis_cool_off_print_interval 250 // For Testing purpose ,Remove hand/cool off

#else

#define nos_channel 4
const uint8_t echoPins[] = {1, 5, 7, 9};
const uint8_t trigPins[] = {0, 4, 6, 8};
const uint8_t output_channels[] = {10, 11, 12, 13};

#endif

#ifdef INVERTED_LOGIC

#define initial_state_of_output_channels true // Initial state for the output channel , false = pull_down,true = pullup
# define water_true false  // for inverted logic
# define water_false true  // for inverted logic

#else

#define initial_state_of_output_channels false // Initial state for the outpu channel , false = pull_down,true = pullup
# define water_true true
# define water_false false

#endif

#define analog_read_pot_pin A1    // 10K pot connected to this pin , and udes to adjust , water fall time
#define analog_read_pot_LED_pin 3 // PWM pin , this brightness will be adjusted according to the value of Pot at 'analog_read_pot_pin'

const uint16_t debounceDuration = 300; // Debounce duration for any interrupt
const uint8_t builtInButton = 2;       // Push button for the ajdustment of water fall time , push once to update to current value

#define activation_min_dist 10      // in cms , refer diagram
#define activation_safe_zone_dist 2 // in cms , refer diagram
//#define water_falling_time_ms 5000  // in ms

#define water_falling_repeats 1   // Output channel will be activated only once
#define dealy_in_action_ms 600   // ms , Buffer time after , reading valid input from the Ultasonic Sensor
#define cool_off_duration_ms 1200 // ms , Buffer time after , successful completion of action , this will give user some time to remove hand

#define default_min_dist 20       // in cms ,refer diagram
#define default_safe_zone_dist 15 // in cms ,refer diagram

#define water_fall_map_min_time 500
#define water_fall_map_max_time 3600

// constants won't change:
// const long interval_bw_reading = 1000; // interval at which to blink (milliseconds) // NOT USED

#define start_index_add 0            // initial address in EEPROM memory of arduino , 10 bit data will be stored in 4 parts.
#define analog_reading_nos_to_avg 10 // Reading multiple values of 10K pot to remove noises.

// DO NOT CHANGE , ultra_sonic_settings
#define US_start_delay_ms 2
#define US_pulse_duration_ms 10
#define US_sound_speed 0.034

#ifdef DEBUG_CODE

#endif

#endif