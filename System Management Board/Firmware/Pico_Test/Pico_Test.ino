#define IN0 7
#define IN1 10
#define IN2 11
#define OUT0 12
#define OUT1 27
#define OUT2 28
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
#define ADC_MUX 26
#define JET_ON 15

int COMP_PWR_STATE = 0;
int SWITCH_PWR_STATE = 0;
int MAIN_RELAY_STATE = 0;
int LEDA_STATE = 0;
int LEDB_STATE = 0;
int LIGHT_A_STATE = 0;
int LIGHT_B_STATE = 0;
int JET_ON_STATE = 0;

//The threshold for deciding if power is on
//If voltage reading it greater, it is on, if less, off. 
int threshold = 100;

bool American = false; 

//Since there are several 3-bit interfaces, a dedicated structure
//seemed useful. 
struct bits{
  int S2,S1,S0;
};

struct bits AUX_Voltage = {0,0,1};
struct bits Temp_Sensor1 = {1,1,0};
struct bits Temp_Sensor2 = {1,1,1};

void set_mux(struct bits pins){
  delay(10);
  digitalWrite(MUX_S2,pins.S2);
  digitalWrite(MUX_S1,pins.S1);
  digitalWrite(MUX_S0,pins.S0);
  delay(10);
}

int read_ADC_MUX(struct bits pins){
  set_mux(pins);
  int data = analogRead(ADC_MUX);
  //set_mux({0,0,0});
  return data;
}

float convt_temp(float temp){
  //The MCP9700T is supposed to have temp coeff Vc = 10mV/˚C and V(0˚) = 400mv. 
  //The datasheet gives the output equation Vout = Va*Vc+V(0˚), where Va = 
  //the ambient temperature. 
  //4096 is based on the RPi Pico's 12-bit ADC readings
  float Vc = 10.0/1000.0;
  float V0 = 400.0/1000.0;
  temp = temp*(5.0/4096.0);
  temp = temp-V0;
  temp = temp/Vc;
  if (American){
    //Fahrenheit conversion
    temp = (temp*1.8)+32.0;
  }
  return temp;
}

float check_temp(int sensor){
  struct bits temp_mux1 = {1,1,0};
  struct bits temp_mux2 = {1,1,1};
  float temp = 0.0;
  if (sensor == 1){
    temp = (float)read_ADC_MUX(temp_mux1);
  }
  else if (sensor == 2){
    temp = (float)read_ADC_MUX(temp_mux2);
  }
  temp = convt_temp(temp);
  return temp;
}

void toggle(int pin_to_toggle, bool state){
  digitalWrite(pin_to_toggle,!state);
}

void parser(String input_string){
  int input_char = (int)input_string[0];
  switch (input_char) {
    //"M" toggles the main relay
    case 77:
      MAIN_RELAY_STATE = !MAIN_RELAY_STATE;
      digitalWrite(MAIN_RELAY,MAIN_RELAY_STATE);
    break;
    case 83:
      //"S" toggles the switch relay
      SWITCH_PWR_STATE = !SWITCH_PWR_STATE;
      digitalWrite(SWITCH_PWR_EN,SWITCH_PWR_STATE);
    break;
    //"C" toggles the computer relay
    case 67:
      COMP_PWR_STATE = !COMP_PWR_STATE;
      digitalWrite(COMP_PWR_EN,COMP_PWR_STATE);
    break;
    //"J" toggles the Jetson on pin
    case 74:
      JET_ON_STATE = !JET_ON_STATE;
      digitalWrite(JET_ON,JET_ON_STATE);
    break;
    //"T" prints temperatures over serial
    case 84:
      Serial.println(check_temp(1));
      Serial.println(check_temp(2));
    break;
    //"a" toggles LEDA
    case 97:
      LEDA_STATE = !LEDA_STATE;
      digitalWrite(LEDA,LEDA_STATE);
    break;
    //"b" toggles LEDB
    case 98:
      LEDB_STATE = !LEDB_STATE;
      digitalWrite(LEDB,LEDB_STATE);
    break;
    //"A" toggles LIGHT_A
    case 65:
      LIGHT_A_STATE = !LIGHT_A_STATE;
      digitalWrite(LIGHT_A,LIGHT_A_STATE);
    break;
    //"B" toggles LIGHT_B
    case 66:
      LIGHT_B_STATE = !LIGHT_B_STATE;
      digitalWrite(LIGHT_B,LIGHT_B_STATE);
    break;
    //"O" enables output pin control parsing expects 4 chars like "O001" MSB (2) to LSB (0)
    case 79:
      digitalWrite(OUT0,((int)input_string[3])-48);
      digitalWrite(OUT1,((int)input_string[2])-48);
      digitalWrite(OUT2,((int)input_string[1])-48);
    break;
    //"I" enables input pin value reading
    case 73:{
      String holder = "";
      holder.concat(digitalRead(IN2));
      holder.concat(digitalRead(IN1));
      holder.concat(digitalRead(IN0));
      Serial.println(holder);
    }
    break;
    //"K" runs shutdown procedure
    case 75:
      shutdown();
    break;
    //"V" reads voltage of input from ADC mux
    case 86:
    {
      struct bits mux_selector = {0,0,1};
      int voltage = read_ADC_MUX(mux_selector);
      Serial.println(voltage);
    }
  }
  Serial.print((char)input_char);
  Serial.print(" ");
  Serial.println("Done");
}

void check_pow(){
  struct bits mux_selector = {0,0,1};
  int voltage = read_ADC_MUX(mux_selector);
  if (voltage >= threshold){
    digitalWrite(SWITCH_PWR_EN,1);
    delay(50);
    digitalWrite(COMP_PWR_EN,1);
  }
  else {
    shutdown();
  }
}

void shutdown(){
  pinMode(JET_ON,OUTPUT);
  digitalWrite(JET_ON,1);
  delay(1000);
  digitalWrite(JET_ON,0);
  //Wait before cutting power
  delay(30000);
  digitalWrite(SWITCH_PWR_EN,0);
  delay(50);
  digitalWrite(COMP_PWR_EN,0);
  delay(50);
  digitalWrite(LIGHT_A,0);
  delay(50);
  digitalWrite(LIGHT_B,0);
  delay(200);
  digitalWrite(MAIN_RELAY,0);
  //End of code
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IN0, INPUT);
  pinMode(IN1, INPUT);
  pinMode(IN2, INPUT);
  pinMode(OUT0, OUTPUT);
  digitalWrite(OUT0,0);
  pinMode(OUT1, OUTPUT);
  digitalWrite(OUT1,0);
  pinMode(OUT2, OUTPUT);
  digitalWrite(OUT2,0);
  pinMode(LIGHT_A,OUTPUT);
  digitalWrite(LIGHT_A,0);
  pinMode(LIGHT_B,OUTPUT);
  digitalWrite(LIGHT_B,0);
  pinMode(LEDA, OUTPUT);
  digitalWrite(LEDA,0);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB,0);
  pinMode(MAIN_RELAY, OUTPUT);
  digitalWrite(MAIN_RELAY,0);
  pinMode(SWITCH_PWR_EN, OUTPUT);
  digitalWrite(SWITCH_PWR_EN,0);
  pinMode(COMP_PWR_EN, OUTPUT);
  digitalWrite(COMP_PWR_EN, 0);
  Serial.begin(115200);
  for (int i = 0; i<10;i++){
    delay(1000);
    if (Serial.available()>0){
      String data = Serial.readStringUntil(10);
      data.trim();
      parser(data);
    }
  }
  digitalWrite(MAIN_RELAY,1);
  //Pi is in control
  for (int i = 0; i<10;i++){
    delay(1000);
    if (Serial.available()>0){
      String data = Serial.readStringUntil(10);
      data.trim();
      parser(data);
    }
  }
  //check_pow();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Analog read is rather inefficient timewise, so only check power once every
  //5 seconds. 
  if ((millis() % 5000) <= 50){
    struct bits input_bits;
    input_bits = {digitalRead(IN0),digitalRead(IN1),digitalRead(IN2)};
    //Input 11* Light A, 1*1 Light B, 01* LED A, 0*1 LED B
    digitalWrite(input_bits.S0 && input_bits.S1, LIGHT_A);
    digitalWrite(input_bits.S0 && input_bits.S2, LIGHT_B);
    digitalWrite(!input_bits.S0&&input_bits.S1,LEDA);
    digitalWrite(!input_bits.S0&&input_bits.S2,LEDB);
    struct bits mux_selector;
    mux_selector = {0,0,1};
    int voltage = read_ADC_MUX(mux_selector);
    delay(100);
    if (voltage < threshold){
      //shutdown();
    }
  }
  if (Serial.available()>0){
    String data = Serial.readStringUntil(10);
    data.trim();
    parser(data); 
  }
}
