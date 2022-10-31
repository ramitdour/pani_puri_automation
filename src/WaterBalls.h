
#include <Arduino.h>
#include <WaterBallsData.h>
#ifndef WaterBalls_h
#define WaterBalls_h

/**/



/*Interrup service routine , this should only be used for sttting up flag , for ISR method*/
void myIRS1();

/*All the required tas/action of Interrupt will be done here */
void myIRS1_method();

/*This will check the flagsof ISR , will excecute correspondin method*/
void myIRS_check();

/*Intitial setup of  Interrups , and attachemnt of PINs , RISE/FALL/CHANGE*/
void setupISR1();

/*Storing analog read values to EEPROM for next boot , to start from same value/time*/
void write_analog_store_value_to_EEPROM();

/*Reading store analog value from EEPROM , for water fall time */
void read_analog_value_from_EEPROM();

/* Setting up Trigger and Echo pin and theor Modes*/
void setup_ultrasonic_sensors();

/*Note : Cross check pull-up/pull-down logic of Output*/
void setup_output_channels();

/*Not used , Interrups was used*/
void setup_push_buttons();

/*Not used , */
void setup_analog_read();

/*Will read analog Input(10k Pot) from 10bit-ADC at given pin 0-1023*/
void read_analog_value_from_sensor();

/*Toggle the state and turn off Ticker ,Will execute/trigger when Ticker ticks*/
void callbackFunction_0();

/*Toggle the state and turn off Ticker ,Will execute/trigger when Ticker ticks*/
void callbackFunction_1();

/*Toggle the state and turn off Ticker ,Will execute/trigger when Ticker ticks*/
void callbackFunction_2();

/*Toggle the state and turn off Ticker ,Will execute/trigger when Ticker ticks*/
void callbackFunction_3();

/* set or reset channel*/
void on_off_set_ith_output_channel(uint8_t channel_no, bool channel_state);

/*logic based on on_off_set_ith_output_channel*/
void on_off_check_ith_channel_distace_and_take_action(uint8_t channel_no);

/*Invert the current state of the output Channel*/
void toggle_ith_output_channel(uint8_t channel_no);

/*Turn off the given ticker*/
void stop_ith_ticker_timer(uint8_t channel_no);

/*Not used , */
void setup_ticker_timers();



/*Start timer of given Ticker*/
void start_ith_ticker_timer(uint8_t channel_no);

/*Stop timer of given Ticker , And update Cool-off millis*/
void stop_ith_ticker_timer(uint8_t channel_no);

/*Mapping -> Update time intervals of the all tickers , in proportional the analog read value*/
void update_time_of_all_tickers(uint32_t time_interval_ticker);

/*
Step 1: Check if hand is under the sensor,
Step 2: Output Channel should be free to use,
Step 3.1: Cool off duration should be passed.
Step 3.2: If work of the output channel is done , the hand should be removed , atleast once.
Step 4: If all things are right , then toggle the channel, 

Step 5: If hand was removed during usage , then reset the hand put state, and stop the timer
Step 6: If hand is removed then , rest the hand put state,
Step 4:
*/
void check_ith_channel_distace_and_take_action(uint8_t channel_no);

/*Individually call fuction to take actions , based of distances*/
void check_all_channels_distace_and_take_action();

/*Read Distance of given sensor*/
void read_ith_sensor_distance(uint8_t sensor_no);

/*Individually call fuction to read distances from Ulrasonic Sensors*/
void read_all_sensor_data();

/*Update all Tickers to check weather their time is up or not*/
void loop_ticker_timers();

#endif