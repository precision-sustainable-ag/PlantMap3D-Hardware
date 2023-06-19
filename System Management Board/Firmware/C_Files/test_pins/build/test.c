#include "stdio.h"
#include "string.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"

#define MAIN_RELAY 2

uint32_t output_pins = 0x1a7cd01f;
//0b1 1010 0111 1100 1101 0000 0001 1111
uint32_t delay_time_us = 2000000;
uint32_t current_pin_mask = 1;
uint32_t current_pin_num = 0;


void next_pin(){
    bool print = true; 
    while ((current_pin_mask & output_pins) == 0){
        current_pin_mask = current_pin_mask << 1;
        current_pin_num = current_pin_num + 1;
        if (current_pin_mask > output_pins){
            //print = false;
            current_pin_mask = 1;
            current_pin_num = 0;
            break;
        }
    }
    if (print){
        printf("Setting Pin %d\n",current_pin_num);
        gpio_put_masked(output_pins,(current_pin_mask | (1 << MAIN_RELAY)));
    }
    current_pin_mask = current_pin_mask << 1;
    current_pin_num = current_pin_num + 1;
}

int main(){
    stdio_init_all();
    //The USB setup is non-blocking, but it needs time. 
    sleep_ms(1000);
    gpio_init_mask(output_pins);
    gpio_set_dir_out_masked(output_pins);
    gpio_put_masked(output_pins,0);
    bool hold = false;
    while (true){
        uint64_t time = time_us_64();
        if ((time % delay_time_us)<(delay_time_us/5)){
            if (!hold){
                next_pin();
                hold = true;
            }  
        }
        else {
            hold = false;
        }
    }
}