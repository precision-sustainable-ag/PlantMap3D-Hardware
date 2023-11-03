#include "stdio.h"
#include "string.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "math.h"

#define BLINKER_COMPLEXITY 10

#define IN0 12
#define IN1 10
#define IN2 11
#define OUT0 7
#define OUT1 5
#define OUT2 6
#define LEDA 22
#define LEDB 21
#define SWITCH_PWR_EN 0
#define COMP_PWR_EN 1
#define MAIN_RELAY 2
#define LIGHT_A 3
#define LIGHT_B 4
#define MUX_S2 20
#define MUX_S1 19
#define MUX_S0 18
#define AUX_SW 13
#define ADC_MUX 26
#define ADC_MUX_CHANNEL 0
#define JET_ON 15
#define BUILT_IN_LED 25
#define COMP_I_MONITOR 28
#define SWITCH_I_MONITOR 27

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UARTID uart0
#define BAUDRATE 115200

#define mask 0xffffffe0
#define V_REF 3.25
#define PRIORITY_CONST 50000

#define InitialPower 0x00000000
#define MainRelay 0x00000004
#define CompAndSwitch 0x00000003
uint32_t current_state = 0;
uint32_t all_pins = (
  (1<<IN0) | (1<<IN1) | (1<<IN2) | (1<<OUT0) | (1<<OUT1) | 
  (1<<OUT2) | (1<<LEDA) | (1<<LEDB) | (1<<SWITCH_PWR_EN) | 
  (1<<COMP_PWR_EN) | (1<<MAIN_RELAY) | (1<<LIGHT_A) | 
  (1<<LIGHT_B) | (1<<MUX_S2) | (1<<MUX_S1) | (1<<MUX_S0) | 
  (1<<ADC_MUX) | (1<<JET_ON) | (1<<BUILT_IN_LED) | 
  (1<<COMP_I_MONITOR) | (1<<SWITCH_I_MONITOR) | (AUX_SW)
);
uint32_t output_pins = (
  (1<<OUT0) | (1<<OUT1) | (1<<OUT2) | (1<<LEDA) | 
  (1<<LEDB) | (1<<SWITCH_PWR_EN) | (1<<COMP_PWR_EN) | 
  (1<<MAIN_RELAY) | (1<<LIGHT_A) | (1<<LIGHT_B) | 
  (1<<MUX_S2) | (1<<MUX_S1) | (1<<MUX_S0) | (1<<JET_ON) |
  (1<<BUILT_IN_LED)
);
uint32_t input_pins = (
  (1<<IN0) | (1<<IN1) | (1<<IN2) | (1<<ADC_MUX) | 
  (1<<COMP_I_MONITOR) | (1<<SWITCH_I_MONITOR) | (1<<AUX_SW)
);

bool last_aux_sw_state = false;

int volt_threshold = 1000;

typedef struct bit_holder{
    int S2, S1, S0;
} bits;
const bits KEY_Voltage = {0,0,0};
const bits Temp_Sensor1 = {1,1,0};
const bits Temp_Sensor2 = {1,1,1};

const uint32_t Lights_Pin = IN0;
const uint32_t SD_Finish_Pin = IN2;

typedef struct process_monitor{
    bool in_process;
    uint64_t start_time;
} monitor;
monitor sd_now = {false, 0};
bool end_sd = false;
bool early_start = true;
//Time in seconds between cutting power and shutdown sequence start. 
int shutdown_delay = 10; 
monitor debug = {false, 0};
monitor engage = {true, 0};
uint64_t debug_time = 0;
bool debug_force_sd = false;

typedef struct blinker{
  int pulses; 
  double times[BLINKER_COMPLEXITY];
  int states[BLINKER_COMPLEXITY];
  double length;
} blink_type;

blink_type standard_blink = {1,{0.0,0.5},{1,0},1.0};
blink_type debug_blink = {2,{0.0,0.15,0.25,0.4},{1,0,1,0},1.0};
blink_type sd_blink = {1,{0.0,0.5},{1,0},0.2};
blink_type end_sd_blink = {3,{0.0,0.05,0.1,0.15,0.2,0.25},{1,0,1,0,1,0},1.0};
blink_type early_startup = {2,{0.0,0.43,0.5,0.93,1.0},{1,0,1,0},1.5};

//A simple function to set the selector pins of the ADC mux. 
void set_mux(bits pins){
  sleep_us(10);
  gpio_put(MUX_S2,pins.S2);
  gpio_put(MUX_S1,pins.S1);
  gpio_put(MUX_S0,pins.S0);
  sleep_us(10);
}

//A function to return the ADC channel on the Pico from
//the input pin number. 
uint get_channel_from_pin(uint pin){
  if (pin == 26){
    return 0;
  } else if (pin == 27){
    return 1;
  } else if (pin == 28){
    return 2;
  } else {
    return 0;
  }
}

//A function to take the raw uint64_t time in microseconds
//from system clock methods and return a double in milliseconds. 
double convt_time(uint64_t time){
  double millis = (double)time/1000.0;
  return millis;
}

//Calls the function to set the select bits of the ADC mux
//and then reads and returns the raw ADC value. 
uint read_ADC_MUX(bits pins){
  adc_select_input(get_channel_from_pin(ADC_MUX));
  set_mux(pins);
  uint data = adc_read();
  return data;
}

//This function compares the system input voltage from the key
//to the threshold and returns 1 if greater. 
int check_pow(){
  uint voltage = read_ADC_MUX(KEY_Voltage);
  if (voltage > volt_threshold){
    return 1;
  }
  else {
    return 0;
  }
}

//The normal operating function. This function controls the startup
//and basic monitoring of the system power input. After the Pico 
//engages, it waits for 10 seconds, and then turns on the main relay.
//After 10 more seconds, it turns on the Jetson and the POE Switch. 
void evaluate_state(uint64_t time){
  time -= engage.start_time;
  bool holder = (bool)check_pow();
  if (!holder){
    sd_now.start_time = (time_us_64()-debug_time);
    if (time < (20000000-1000)) {
      sd_now.start_time -= 19000000;
      if (time > (10000000-1000)) {
        sd_now.in_process = true;
        end_sd = true;
        early_start = false;
      }
      else {
        engage.start_time = time_us_64() - debug_time;
        sd_now.in_process = false;
        end_sd = false;
        early_start = true;
      }
    }
    else {
      sd_now.in_process = true;
      end_sd = false;
      early_start = false;
    }
  }
  else if ((time > 20000000) && holder){
    current_state |= MainRelay | CompAndSwitch;
    early_start = false;
  }
  else if ((time > 10000000) && holder) {
    current_state |= MainRelay;
    early_start = false;
  }
  else if ((time > 0) && holder) {
    current_state = InitialPower;
    early_start = false;
  }
}

//This is the controlled shutdown function. It initially waits
//for 10 seconds to make sure the system is actually supposed
//to turn off and there was no momentary lapse in power.
//  PLACEHOLDER BEHAVIOR
//Then, it "presses" the Jetson ON pin for 1 second and then waits
//another 9 seconds before cutting power to the system. 
void shutdown_process(uint64_t input_time){
  uint64_t relative_time = input_time - (sd_now.start_time);
  uint64_t delay_time = (uint64_t)(1000000*shutdown_delay);
  engage.in_process = (bool)check_pow();
  if (debug_force_sd){
    engage.in_process = false;
    printf("Relative Time: %lu\n",((uint32_t)relative_time));
  }
  if (engage.in_process){
    if ((input_time - engage.start_time) > 20000000){
      engage.start_time = input_time-20000000;
    }
    else if ((input_time - engage.start_time) > 10000000){
      engage.start_time = input_time - 10000000;
    }
    else {
      engage.start_time = input_time;
    }
  }
  if (relative_time > (delay_time + 10000000)){
    current_state = InitialPower;
    watchdog_enable(500,1);
    //Once this passes to the gpio_puts_masked, this will be end of program.
    //If it does not shutdown, then a watchdog is enabled
    //to force a reboot because an error has likely occured.
    gpio_put_masked(output_pins,current_state);
    //Watchdog is having an issue, so as a substitute
    sd_now.in_process = false;
    sd_now.start_time = 0;
    engage.in_process = true;
    engage.start_time = input_time;
    debug_force_sd = false;
    //Acts as universal offset in this code, effectively resetting the hardware clock
    debug_time = input_time;
  }
  else if (relative_time > (delay_time + 1000000)){
    current_state = current_state & ~(1<<JET_ON);
  }
  else if ((relative_time > delay_time)&&(!engage.in_process)){    
    current_state = current_state | (1<<JET_ON);
    end_sd = true;
  }
  else if ((relative_time <= delay_time) && (engage.in_process)){
    sd_now.in_process = false;
    sd_now.start_time = 0; 
  }
}

//Inverts the state of the pin by clearing or setting the
//corresponding bit in the current_state global variable. 
void toggle_pin(int pin){
  uint32_t pin_mask = (1 << pin);
  current_state = current_state ^ pin_mask;
  gpio_put_masked(output_pins,current_state);
}

//Converts ADC value to temperature using datasheet-given equation. 
float convt_temp(float temp){
  //The MCP9700T is supposed to have temp coeff Vc = 10mV/˚C and V(0˚) = 400mv. 
  //The datasheet gives the output equation Vout = Va*Vc+V(0˚), where Va = 
  //the ambient temperature. 
  //4096 is based on the RPi Pico's 12-bit ADC readings
  float Vc = 10.0/1000.0;
  float V0 = 500.0/1000.0;
  temp = temp*(V_REF/4096.0);
  temp = temp-V0;
  temp = temp/Vc;
  return temp;
}

//Wrapper linking the ADC MUX and temperature calculation
float check_temp(int sensor){
  float temp = 0.0;
  if (sensor == 1){
    temp = (float)read_ADC_MUX(Temp_Sensor1);
  }
  else if (sensor == 2){
    temp = (float)read_ADC_MUX(Temp_Sensor2);
  }
  temp = convt_temp(temp);
  return temp;
}

//Function to read the current monitor pin on the voltage
//regulators for the Jetson and POE Switch and convert the
//raw ADC value to the current using the formula from the 
//regulator datasheet. 
double current_monitor_read(int pin){
  adc_select_input(get_channel_from_pin(pin));
  uint data = adc_read();
  double voltage = (double)data*((double)V_REF/4096.0);
  return ((voltage-0.23)/0.055);
}

//The core of debug mode that parses the char input and 
//determines the proper test that has been requested. Char
//command usages are listed next to each check below. 
void parser(int input_char){
  bool valid_command = false;
  switch (input_char) {
    //"M" toggles the main relay
    case 77:
      toggle_pin(MAIN_RELAY);
      valid_command = true;
    break;
    case 83:
      //"S" toggles the switch relay
      toggle_pin(SWITCH_PWR_EN);
      valid_command = true;
    break;
    //"C" toggles the computer relay
    case 67:
      toggle_pin(COMP_PWR_EN);
      valid_command = true;
    break;
    //"U" reads current monitor pins
    case 85:{
      double holder[2];
      valid_command = true;
      holder[0] = current_monitor_read(COMP_I_MONITOR);
      holder[1] = current_monitor_read(SWITCH_I_MONITOR);
      printf("Comp I-Monitor: %.2fA\nSwitch I-Monitor: %.2fA\n",holder[0],holder[1]);
    }
    break;
    //"J" toggles the Jetson on pin
    case 74:
      toggle_pin(JET_ON);
      valid_command = true;
    break;
    //"j" provides an interface for the testing mode of the Jetson
    case 106:{
        check_input_pattern();
    }
    //"T" prints temperatures over serial
    case 84:{
      printf("%.4f°C  ",check_temp(1));
      printf("%.4f°C\n",check_temp(2));
      valid_command = true;
    break;}
    //"a" toggles LEDA
    case 97:
      toggle_pin(LEDA);
      valid_command = true;
    break;
    //"b" toggles LEDB
    case 98:
      toggle_pin(LEDB);
      valid_command = true;
    break;
    //"A" toggles LIGHT_A
    case 65:
      toggle_pin(LIGHT_A);
      valid_command = true;
    break;
    //"B" toggles LIGHT_B
    case 66:
      toggle_pin(LIGHT_B);
      valid_command = true;
    break;
    //"O" enables output pin control parsing expects 4 chars like "O001" MSB (2) to LSB (0)
    case 79:{
      uint32_t input_string[3];
      valid_command = true;
      input_string[2] = getchar_timeout_us(5000000)-48;
      input_string[1] = getchar_timeout_us(5000000)-48;
      input_string[0] = getchar_timeout_us(5000000)-48;
      if ((input_string[2]==(uint32_t)-49)||(input_string[1]==(uint32_t)-49)||(input_string[0]==(uint32_t)-49)) break;
      uint32_t state_update = (input_string[0] << OUT0) + (input_string[1] << OUT1) + (input_string[2] << OUT2);
      uint32_t outputs = (1 << OUT0) + (1 << OUT1) + (1 << OUT2);
      current_state = (current_state & (~outputs))+state_update;
      gpio_put_masked(output_pins,current_state);
      printf("\n");
    }
    break;
    //"I" enables input pin value reading
    case 73:{
      int holder[3];
      holder[0] = gpio_get(IN0);
      holder[1] = gpio_get(IN1);
      holder[2] = gpio_get(IN2);
      printf("I%1d%1d%1d\n",holder[2],holder[1],holder[0]);
      valid_command = true;
    }
    break;
    //"K" runs shutdown procedure
    case 75:
      debug.in_process = false;
      sd_now.in_process = true;
      debug_force_sd = true;
      end_sd = true;
      valid_command = true;
      break;
    //"V" reads voltage of input from ADC mux with optional choice of which
    //pin to read. Defaults to reading Pin 1 on the mux. 
    case 86:
    {
      int holder = getchar_timeout_us(5000000);
      if (holder == PICO_ERROR_TIMEOUT){
        uint voltage = read_ADC_MUX(KEY_Voltage);
        printf("%d\n",voltage);
      }
      else{
        if ((holder >= 48)&&(holder <= 57)){
          holder -= 48;
          bits selector = {0,0,0};
          selector.S2 = ((holder&4) >> 2);
          selector.S1 = ((holder&2) >> 1);
          selector.S0 = (holder&1);
          uint voltage = read_ADC_MUX(selector);
          printf("MUX Pin %d: %d\n",holder,voltage);
        }
      }
      valid_command = true;
    }
    
    break;
    case 100:
      debug.in_process = false;
      valid_command = true;
      break;
    }
  if (valid_command){
    printf("Input \"%c\": done\n", input_char);
  }
}

//This is a function to handle inputs for the AUX switch on the 
//power board. Currently with PLACEHOLDER behavior
void check_aux_switch(){
  gpio_set_pulls(AUX_SW,true,false);
  bool holder = gpio_get(AUX_SW);
  if ((holder != last_aux_sw_state) && (holder == false)){
    printf("Auxiliary switch pressed. That's neat!\n");
    last_aux_sw_state = false;
  } else {
    last_aux_sw_state = holder;
  }
}

//This reads the input pins to determine if the Jetson wants the
//Pico to enable the lights, or it also can detect if the Jetson
//is ready to be shutdown, with PLACEHOLDER behavior. 
void check_input_pattern(){
  int lights_holder = gpio_get(Lights_Pin);
  int cur_lights_holder = current_state & (1<<LIGHT_A);
  if (lights_holder){
    current_state |= ((1 << LIGHT_A) | (1 << LIGHT_B));
  }
  else{
    current_state &= (~(1<<LIGHT_A))&(~(1<<LIGHT_B));
  }
  int SD_Finish = gpio_get(SD_Finish_Pin);
  if (SD_Finish){
    //This needs to be integrated with the shutdown protocol. 
    printf("Placeholder for shutdown integration. ");
  }
  if (debug.in_process && SD_Finish){
    printf("Shutdown Done");
  }
  else if (debug.in_process && (!lights_holder & cur_lights_holder)){
    printf("Lights Off");
  }
  else if (debug.in_process && (lights_holder && !cur_lights_holder)){
    printf("Lights On");
  }
  else if (debug.in_process){
    printf("No Commands");
  }
}

//This function takes the current time and determines the time within 
//the pulses of the pre-defined blink patterns and whether LED should 
//be set high or low given the time within the period of the signal. 
void blink_pattern(){
  int state_changes;
  double time = (double)(time_us_64())/1000000.0;
  double time_in_pulse;
  blink_type holder;
  if (debug.in_process){
    state_changes = debug_blink.pulses * 2;
    time_in_pulse = time / debug_blink.length;
    time_in_pulse = (double)(time_in_pulse - (double)floor(time_in_pulse))*debug_blink.length;
    holder = debug_blink;
  } 
  else if (end_sd){
    state_changes = end_sd_blink.pulses *2;
    time_in_pulse = time / end_sd_blink.length;
    time_in_pulse = (double)(time_in_pulse - (double)floor(time_in_pulse))*end_sd_blink.length;
    holder = end_sd_blink;
  }
  else if (sd_now.in_process){
    state_changes = sd_blink.pulses *2;
    time_in_pulse = time / sd_blink.length;
    time_in_pulse = (double)(time_in_pulse - (double)floor(time_in_pulse))*sd_blink.length;
    holder = sd_blink;
  }
  else if (early_start){
    state_changes = early_startup.pulses *2;
    time_in_pulse = time / early_startup.length;
    time_in_pulse = (double)(time_in_pulse - (double)floor(time_in_pulse))*early_startup.length;
    holder = early_startup;
  }
  else {
    state_changes = standard_blink.pulses * 2;
    time_in_pulse = time / standard_blink.length;
    time_in_pulse = (double)(time_in_pulse - (double)floor(time_in_pulse))*standard_blink.length;
    holder = standard_blink;
  }
  for (int i = (state_changes-1); i >=0; i--){
      //printf("%f   %f   %d   i:%d\n",(holder.times[i]*holder.length),time_in_pulse, holder.states[i],i);
      if (time_in_pulse > (holder.times[i]*holder.length)){
        current_state = current_state & (~(1 << BUILT_IN_LED));
        current_state = current_state | ((holder.states[i]) << BUILT_IN_LED);
        break;
      }
    }
}

//Separate loop from the main loop to listen for input
//over serial. 
uint64_t debug_mode(){
    printf("Ready!\n");
    while (debug.in_process) {
        int holder;
        holder = getchar_timeout_us(0);
        if (holder != -1){
            parser(holder);
        }
        blink_pattern();
        gpio_put_masked(output_pins,current_state);
        check_aux_switch();
    }
    printf("Exiting debug mode\n");
    return time_us_64() - debug_time;
}

//Function to setup the UART communication with the Jetson. 
uint8_t init_uart_jetson(){
    uart_init(UARTID, BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

//Main function to initialize all functions and then enter main
//operation loop. 
int main(){
  stdio_init_all();
  //Found these neat predefined functions in the SDK
  //for setting pins from a bitmap.
  gpio_init_mask(all_pins);
  gpio_set_dir_out_masked(output_pins);
  gpio_set_dir_in_masked(input_pins);
  //Configuring ADC input is separate
  adc_init();
  adc_gpio_init(ADC_MUX);
  bool checked_priority = false;
  while (1) {
    uint64_t time_ref = time_us_64() - debug_time;
    if (debug.in_process) {
      bool holder = sd_now.in_process; 
      printf("Entering debug mode\n");
      debug.start_time = time_ref;
      debug_time += debug_mode() - debug.start_time;
      if ((!holder)&&(sd_now.in_process)){
        sd_now.start_time = time_us_64()-debug_time;
      }
    }
    if (((time_ref % PRIORITY_CONST)>(PRIORITY_CONST/2)) && (!checked_priority)){
      if (!sd_now.in_process){
        evaluate_state(time_ref);
      }
      else {
        shutdown_process(time_ref);
      }
      check_aux_switch();
      check_input_pattern();
      checked_priority = true;
    } else if ((time_ref % PRIORITY_CONST)<(PRIORITY_CONST/4)){
      checked_priority = false;
    }
    blink_pattern();
    gpio_put_masked(output_pins,current_state);
    int char_holder = getchar_timeout_us(0);
    if (char_holder==100){
      debug.in_process = true;
    } else if (char_holder == 84){
      printf("Reference time: %lu\n", (uint32_t)(time_ref&0xffffffff));
    } else if (char_holder == 82){
      //This has the effect of resetting time references
      debug_time = time_us_64();
    }
  }
  //Code should NEVER go beyond here. If it does, reboot. 
  watchdog_enable(1,1);
}