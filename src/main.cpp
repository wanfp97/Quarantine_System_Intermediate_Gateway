#define DEBUG

#define E22_30    //EBYTE E22_30 series
#define FREQUENCY_850     //base frequency of E22 module
#define MY_CRC8_POLY 0xAB     //used for 8bit crc
#define MY_CRC16_POLY 0xABAB    //used for 16 bit crc

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <SoftwareSerial.h>
#include "Keypad.h"
#include "Stepper.h"
#include "LiquidCrystal_I2C.h"
#include "RF24.h"
#include "TinyGPS++.h"
#include "LoRa_E22.h"
#include "CRC.h"
#include "CRC8.h"
#include "CRC16.h"

#include "MyString.h"
#include "MyLora_E22.h"

MyString keypad_input();
bool check_password(MyString input);
void admin_mode1();
void admin_mode2();
bool new_qb_sync();
void qb_setting();
bool get_gps_info();
bool send_to_qb(uint32_t max_time_ms, char* message, unsigned int message_size, char* ack, unsigned int ack_size);
uint8_t scan_and_choose_channel(RF24& nrf24);
bool wait_qb_update();
void qb_stat_view();
uint8_t update_qb_info();
bool sync_to_mg();
bool sync_qb_mg();
bool full_update_to_mg(uint8_t qb_num);
bool update_to_mg(uint8_t qb_num);

uint8_t pipe0;
bool scanned_nrf24_channel = false;
bool update_gps = false;
bool check_time = false;
uint8_t timer_counter = 0;

enum QUARANTINE_BAND_STATUS {
  NOT_INIT  = 0,
  PRESENT   = 1,
  OPENED    = 2,
  MISSING   = 3,
  EMERGENCY = 4,
  MOVED     = 5
};

enum E22_MESSAGE_TYPE {
  REQUEST_E22_ADDR        = 0,
  REQUEST_GOOGLESHEET_ROW = 1,
  FULL_UPDATE_INFO        = 2,
  UPDATE_INFO             = 3,

  RETURN_E22_ADDR         = 4,
  RETURN_GOOGLESHEET_ROW  = 5,

  ACK                     = 6
};

enum E22_ACK_PAYLOAD {
  ACK_ONLY    = 0,
  ACK_PAYLOAD = 1
};

enum CRC_LENGTH {
  CRC_8   = 0,
  CRC_16  = 1,
};

struct qb_sync_message {
  uint8_t ig_channel;
  uint32_t ig_addr;
  uint32_t qb_addr;
  uint8_t qb_num;
};

struct qb_message
{
  uint8_t qb_num;
  uint8_t status;
};

struct e22_message_header{  //10bytes
  uint8_t message_type;
  uint8_t sender_e22_addr_h;
  uint8_t sender_e22_addr_l;
  uint8_t ack_payload;
  uint8_t sender_sequence;
  uint8_t receiver_sequence;
  uint16_t crc_poly;
  uint8_t crc_length;
  uint8_t padding;    //padding for similar alignment between arduino and esp8266
  uint8_t padding1;    //padding for similar alignment between arduino and esp8266
  uint8_t padding2;    //padding for similar alignment between arduino and esp8266
};

struct ig_sync_message{   //16bytes
  e22_message_header ig_header;
  
  uint8_t padding;    //padding for similar alignment between arduino and esp8266
  uint8_t padding1;    //padding for similar alignment between arduino and esp8266
  uint8_t padding2;    //padding for similar alignment between arduino and esp8266

  uint8_t crc;
};

struct ig_ack_message{    //16bytes
  e22_message_header ig_ack_header;

  uint8_t padding;    //padding for similar alignment between arduino and esp8266
  uint8_t padding1;    //padding for similar alignment between arduino and esp8266
  uint8_t padding2;    //padding for similar alignment between arduino and esp8266

  uint8_t crc;
};

struct ig_full_message{   //40bytes
  e22_message_header ig_header;
  uint32_t qb_addr;
  uint16_t mg_mem_addr;
  uint8_t padding;    //padding for similar alignment between arduino and esp8266
  uint8_t padding1;    //padding for similar alignment between arduino and esp8266
  float initial_latitude;
  float initial_longitude;
  char ic[16];      //included padding
  char hp_num[12];      //included padding
  uint8_t status;
  uint8_t padding2;    //padding for similar alignment between arduino and esp8266

  uint16_t crc;
};

struct ig_update_message{   //28bytes 
  e22_message_header ig_header;
  uint16_t mg_mem_addr;
  uint8_t padding;    //padding for similar alignment between arduino and esp8266
  uint8_t padding1;    //padding for similar alignment between arduino and esp8266
  float current_latitude;
  float current_longitude;
  uint8_t status;
  uint8_t padding2;    //padding for similar alignment between arduino and esp8266

  uint16_t crc;
};

struct mg_sync_message{
  e22_message_header mg_header;

  uint8_t ig_e22_addr_h;
  uint8_t ig_e22_addr_l;

  uint8_t padding;    //padding for similar alignment between arduino and esp8266

  uint8_t crc;
};

struct mg_sync_message_qb_mem{
  e22_message_header mg_header;

  uint16_t qb_mem_addr;

  uint8_t padding;    //padding for similar alignment between arduino and esp8266

  uint8_t crc;
};

struct user {
  uint16_t mg_mem_addr;
  uint8_t date_dd;
  uint8_t date_mm;
  uint16_t date_yyyy;
  uint8_t time_h;
  uint8_t time_m;
  char ic[16];    //included padding
  char hp_num[12];      //included padding
  uint8_t status;

  uint32_t qb_addr;

}person[10];

struct intermediate_gateway {
  MyString my_password = "1234";

  uint32_t my_nrf24_addr;
  uint8_t my_nrf24_channel;

  uint8_t current_qb_number = 0;

  bool sync_to_mg = 0;
  uint8_t my_e22_addr_h;
  uint8_t my_e22_addr_l;
  uint8_t e22_channel = 70;     //920.125 MHz
  uint8_t e22_crypt_h = 0xAB;   //used for encryption and decryption
  uint8_t e22_crypt_l = 0xAB;
  
  uint8_t date_dd;
  uint8_t date_mm;
  uint16_t date_yyyy;
  uint8_t time_h;
  uint8_t time_m;

  float initial_latitude;
  float initial_longitude;

  float latitude;
  float longitude;
}ig;

struct main_gateway {
  uint8_t mg_e22_addr_h;
  uint8_t mg_e22_addr_l;
}mg;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

RF24 nrf24(47, 49);

Stepper qb_lock(100, 7, 6, 4, 5);   //initialize the pin used to control the lock in QB

const byte ROWS = 5; //five rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'A','B','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','-'},
  {'L','0','R','+'}
};
byte rowPins[ROWS] = {23, 25, 27, 29, 31}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {39, 37, 35, 33}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

MyLoRa_E22 e22(&Serial3, 26, 22, 24);    // Arduino RX <-- e22 TX, Arduino TX --> e22 RX AUX M0 M1

void setup(){
  pinMode(A0, INPUT);     //used for randomSeed

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.cursor();

  lcd.clear();
  lcd.print(F("Starting up"));
  lcd.setCursor(0,1);
  lcd.print(F("Serial ..."));

  Serial.begin(9600);   //for debugging
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial1.begin(9600);    //for QB
  while (!Serial1) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial2.begin(GPSBaud);   //for gps
  while (!Serial2) {
    // some boards need to wait to ensure access to serial over USB
  }
  delay(500);

  lcd.clear();
  lcd.print(F("Starting up"));
  lcd.setCursor(0,1);
  lcd.print(F("E22 ..."));

  e22.begin();    //init e22 lora module
  delay(500);

  randomSeed(analogRead(A0));       //read floting pin and use as random seed
  ig.my_nrf24_addr = random(1, 0x7FFFFFFE);       //MSbit must be 0 for random() to work with 32bits
  ig.my_e22_addr_h = random(0, 0xFE);     //avoid 0xFF (0xFF and 0 for monitor mode)
  ig.my_e22_addr_l = random(1,0xFF);     //avoid 0 

  if(!e22.set_e22_configuration(ig.my_e22_addr_h, ig.my_e22_addr_l, ig.e22_channel, true, POWER_21, 
    AIR_DATA_RATE_000_03, ig.e22_crypt_h, ig.e22_crypt_l, false)) {
    lcd.clear();
    lcd.print(F("E22 Error"));
    lcd.setCursor(0,1);
    lcd.print(F("check wiring"));
    while(1);
  }
  delay(500);
  #ifdef DEBUG
    Serial.print(F("ig_e22_addr: "));
    Serial.print(ig.my_e22_addr_h,16);
    Serial.println(ig.my_e22_addr_l, 16);
  #endif

  lcd.clear();
  lcd.print(F("Generating"));
  lcd.setCursor(0,1);
  lcd.print(F("random qb addr"));

  randomSeed(analogRead(A0));       //read floting pin and use as random seed
  person[0].qb_addr = random(1, 0x7FFFFFFE);    //MSbit must be zero for random to function correctly
  #ifdef DEBUG
      Serial.print(F("qb"));
      Serial.print(0,10);
      Serial.print(F(" addr = "));
      Serial.println(person[0].qb_addr, 16);
  #endif
  delay(50);     //
  for(int i = 1; i < 10; i++) {
    Reassign:
    randomSeed(analogRead(A0));   //read floting pin and use as random seed
    person[i].qb_addr = random(1, 0x7FFFFFFE);    //MSbit must be 0 for random() to work
    #ifdef DEBUG
      Serial.print(F("qb"));
      Serial.print(i,10);
      Serial.print(F(" addr = "));
      Serial.println(person[i].qb_addr, 16);
    #endif
    delay(50);       //delay so that next random seed will read different analog rading
    for(int j = i-1; j >= 0; j--) {
      if(person[i].qb_addr == person[j].qb_addr) {
        goto Reassign;
      }
    }
  }

  lcd.clear();
  lcd.print(F("Starting up"));
  lcd.setCursor(0,1);
  lcd.print(F("NRF24 ..."));

  delay(100);

  if (!nrf24.begin()) {
    lcd.clear();
    lcd.println(F("NRF24 ERROR!!"));
    while (1) {} // hold in infinite loop
  }

  lcd.clear();
  lcd.print(F("Waiting GPS"));
  lcd.setCursor(0,1);
  lcd.print(F("signal..."));
  delay(1000);

  while(!get_gps_info());
  ig.initial_latitude = ig.latitude;
  ig.initial_longitude = ig.longitude;

  #ifdef DEBUG
    Serial.print(F("initial location = "));
    Serial.print(ig.initial_latitude,6);
    Serial.print(F(", "));
    Serial.println(ig.initial_longitude, 6);
    Serial.print(F("time = "));
    Serial.print(ig.time_h,10);
    Serial.print(F(":"));
    Serial.println(ig.time_m, 10);
  #endif

  lcd.clear();
  lcd.print(F("GPS ready"));

  delay(1000);
  lcd.clear();
  lcd.print(F("F1: Admin mode1"));
  lcd.setCursor(0,1);
  lcd.print(F("F2: Admin mode2"));

  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK5 |= (1<<TOIE5);     //enable overflow interrupt, will overflow every 4.2s
  TCNT5 = 0;
  sei();    //set interrupt
}
  
void loop(){
  

  if(update_gps) {
    update_gps = false;
    get_gps_info();
    // 0.001 approximate to 111 m on equator
    if((((ig.current_qb_number && ig.latitude-ig.initial_latitude)>0.001) || ((ig.longitude-ig.initial_longitude)>0.001))) {
      for(int i = ig.current_qb_number; i >= 0; i--) {
        person[i].status = MOVED;
        update_to_mg(i);
      }
      lcd.clear();
      lcd.print(F("F1: Admin mode1"));
      lcd.setCursor(0,1);
      lcd.print(F("F2: Admin mode2"));
    }
  }

  if (nrf24.available(&pipe0)) {             // check for all rx pipe and return pipe number if data available
    uint8_t qb_num = update_qb_info();     //update qb status and time according to data received
    update_to_mg(qb_num);
  }
  char customKey = customKeypad.getKey();
  if (customKey){
    switch(customKey)
    {
      case 'A':
      {
        admin_mode1();
        break;
      }

      case 'B':
      {
        admin_mode2();
        break;
      }

      default:
      {
        break;
      }
    }
  }

  if(check_time) {
    check_time = false;
    get_gps_info();
    if(ig.current_qb_number) {
      int8_t current_minute = ig.time_m;
      int8_t current_hour = ig.time_h;
      for(int i = ig.current_qb_number; i>=0; i--) {
        if(current_hour != person[i].time_h) {
          current_minute += 60;
        }
        if((current_minute - person[i].time_m)>10) {
          person[i].status = MISSING;
          update_to_mg(i);
        }
      }
    }
  }
}

MyString keypad_input() {
  MyString input;
  char key = 0;
  uint8_t length = 0;
  while(key != '+') {
    while(!(key = customKeypad.getKey()));  //ignore null
    switch (key) {
      case '+':
      {
        break;
      }
      case '-':
      {
        if(length >0) {
          length = length-1;
          input.remove(length,1);     //delete last input
          lcd.setCursor(0,1);
          lcd.print(input);
          lcd.print(" ");
          lcd.setCursor(length,1);
        }
        break;
      }

      case 'L':
      {
        if(length >0) {
          length = length-1;
          lcd.setCursor(length,1);
          break;
        }
      }

      case 'R':
      {
        if(length <16) {
          length = length+1;
          lcd.setCursor(length,1);
          break;
        }
      }

      case 'U':
      {
        break;
      }

      case 'D':
      {
        break;
      }

      default:
      {
        if(length <16) {
          length = length+1;
          lcd.print(key);
          if(input.length() >= length) {
            input.modify_string(length,key);
          }
          else {
            input += key;
          }
        }
        break;
      }
    }
  }
  return input;
}

bool check_password(MyString input) {
  lcd.clear();
  if(input == ig.my_password) {
    lcd.print(F("Password"));
    lcd.setCursor(0,1);
    lcd.print(F("Correct"));
    delay(1000);
    return true;
  }
  else {
    lcd.print(F("Password"));
    lcd.setCursor(0,1);
    lcd.print(F("Wrong"));
    return false;
  }
}

void admin_mode1() {
  lcd.clear();
  lcd.print(F("Enter Password:"));
  lcd.setCursor(0,1);
  bool temp = check_password(keypad_input());

  if(!temp) {
    return;
  }

  lcd.clear();
  lcd.print(F("Admin mode 1"));
  delay(1000);

  uint8_t page = 1;
  uint8_t max_page = 2;

  Select_mode_1:
  switch(page){
    case 1:
    {
      lcd.clear();
      lcd.print(F("1: Scan Channel"));
      lcd.setCursor(0,1);
      lcd.print(F("2: Syn to MG"));
      break;
    }

    case 2:
    {
      lcd.clear();
      lcd.print(F("3: Show MG info"));
      break;
    }
  }
  
  char selection;
  while(!(selection = customKeypad.getKey()));  //ignore null
  switch (selection)
  {
    case '1':
    { 
      if(scanned_nrf24_channel) {
        lcd.clear();
        lcd.print(F("Already scanned"));
        lcd.setCursor(0,1);
        lcd.print(F("NRF24 channel"));
        delay(1000);
        goto Select_mode_1;
      }
      lcd.clear();
      lcd.print(F("Scanning for"));
      lcd.setCursor(0,1);
      lcd.print(F("light channel"));

      uint8_t channel = scan_and_choose_channel(nrf24);
      if(channel != -1) {
        ig.my_nrf24_channel = channel;
        #ifdef DEBUG
          Serial.print(F("ig.my_nrf24_channel = "));
          Serial.println(ig.my_nrf24_channel);
          Serial.print(F("Channel Frequency = "));
          Serial.print(2400 + ig.my_nrf24_channel);
          Serial.println(F(" MHz"));
        #endif

        nrf24.setPALevel(RF24_PA_MAX);      //set max signal amplification
        nrf24.setCRCLength(RF24_CRC_8);     //set 8 bit crc
        nrf24.setPayloadSize(2);      //1byte user number + 1 byte status byte
        nrf24.setAutoAck(true);       //enable auto acknoledgement
        nrf24.setChannel(channel);      //set channel
        nrf24.setAddressWidth(4);        //set addr width

        nrf24.openReadingPipe(0, ig.my_nrf24_addr);     //set receiving addr
        #ifdef DEBUG
          Serial.print(F("ig.my_nrf24_addr = "));
          Serial.println(ig.my_nrf24_addr, 16);
        #endif

        nrf24.startListening();     //start receiving

        lcd.clear();
        lcd.print(F("Set Channel"));
        lcd.setCursor(0,1);
        lcd.print(F("Success"));
        scanned_nrf24_channel = true;
        delay(1000);
        goto Select_mode_1;
      }
      else {
        lcd.clear();
        lcd.print(F("Set Channel"));
        lcd.setCursor(0,1);
        lcd.print(F("Failed"));
        delay(1000);
        goto Select_mode_1;
      }
    }

    case '2':
    {
      if(!ig.sync_to_mg) {
        lcd.clear();
        lcd.print(F("Sending Sync"));
        lcd.setCursor(0,1);
        lcd.print(F("message to MG"));
        if(sync_to_mg()) {
          ig.sync_to_mg = true;
          goto Select_mode_1;
        }
        else {
          lcd.clear();
          lcd.print(F("Sync to MG"));
          lcd.setCursor(0,1);
          lcd.print(F("Failed"));
          delay(1000);
          goto Select_mode_1;
        }
      }

      else {
        lcd.clear();
        lcd.print(F("IG-MG Sync"));
        lcd.setCursor(0,1);
        lcd.print(F("already done"));
        delay(1000);
        goto Select_mode_1;
      }
    }

    case '3':
    {
      lcd.clear();
      lcd.print(F("NRF24 CHAN="));
      lcd.print(2400 + ig.my_nrf24_channel);
      lcd.setCursor(0,1);

      lcd.print(F("MG addr = "));
      lcd.print(mg.mg_e22_addr_h, 16);
      lcd.print(mg.mg_e22_addr_l, 16);
      delay(3000);
      goto Select_mode_1;
    }
    
    case 'L': case 'U':
    {
      if(page > 1){
        page--;
      }
      goto Select_mode_1;
    }

    case 'R': case 'D':
    {
      if(page < max_page){
        page++;
      }
      goto Select_mode_1;
    }

    case '-':
    {
      lcd.clear();
      lcd.print(F("F1: Admin mode1"));
      lcd.setCursor(0,1);
      lcd.print(F("F2: Admin mode2"));
      return;
    }

    default:
    {
      goto Select_mode_1;
    }
  }
}

void admin_mode2() {
  lcd.clear();
  lcd.print(F("Enter Password:"));
  lcd.setCursor(0, 1);
  bool temp = check_password(keypad_input());
  
  if(!temp) {
    return;
  }

  lcd.clear();
  lcd.print(F("Admin mode 2"));
  delay(1000);

  uint8_t page = 1;
  uint8_t max_page = 4;
  
  Select_mode_2:
  switch(page){
    case 1:
    {
      lcd.clear();
      lcd.print(F("Total QB:"));
      lcd.print(ig.current_qb_number);

      lcd.setCursor(0,1);
      lcd.print(F("1: Add New User"));
      break;
    }

    case 2:
    {
      lcd.clear();
      lcd.print(F("2: New QB sync"));
      lcd.setCursor(0,1);
      lcd.print(F("3: QB lock adj"));
      break;
    }

    case 3:
    {
      lcd.clear();
      lcd.print(F("4: Show QB Stat"));
      lcd.setCursor(0,1);
      lcd.print(F("5: Sync QB-MG"));
      break;
    }

    case 4:
    {
      lcd.clear();
      lcd.print(F("6: Update QB-MG"));
      break;
    }

    default:
    {
      break;
    }
  }

  char selection;
  while(!(selection = customKeypad.getKey()));  //ignore null
  switch (selection)
  {
    case '1':
    {
      qb_setting();
      goto Select_mode_2;
    }
    
    case '2':
    {
      if(new_qb_sync()){
        lcd.clear();
        lcd.print(F("QB Sync"));
        lcd.setCursor(0,1);
        lcd.print(F("Completed"));
        #ifdef DEBUG
          Serial.print(F("QB Sync Completed"));
        #endif
        delay(1000);
      }

      else{
        lcd.clear();
        lcd.print(F("QB Sync"));
        lcd.setCursor(0,1);
        lcd.print(F("Failed"));
        #ifdef DEBUG
          Serial.print(F("QB Sync Failed"));
        #endif
        delay(1000);
      }
      goto Select_mode_2;
    }

    case '3':
    {
      lcd.clear();
      lcd.print(F("Control QB lock"));
      lcd.setCursor(0,1);
      lcd.print(F("Using Arrows"));

      QB_lock_adj:
      if(customKeypad.isPressed('U')||customKeypad.isPressed('R')) {
        while ((customKeypad.getKey()=='U') ||(customKeypad.getKey()=='R')) {
          qb_lock.step(1);    //tighthening
        }
      }
      else if(customKeypad.isPressed('D')||customKeypad.isPressed('L')) {
        while ((customKeypad.getKey()=='D') ||(customKeypad.getKey()=='L')) {
          qb_lock.step(-1);    //loosing
        }
      }
      else if(customKeypad.isPressed('-')) {
        goto Select_mode_2;
      }
      goto QB_lock_adj;
    }

    case '4':
    {
      qb_stat_view();
      goto Select_mode_2;
    }

    case '5':
    {
      if (sync_qb_mg()) {
        ig.current_qb_number ++;
      };
      goto Select_mode_2;
    }

    case '6':
    {
      char message[] = "update";
      char ack[] = "start";
      if(send_to_qb(300, message, sizeof(message)-1, ack, (sizeof(ack)-1))) {
        lcd.clear();
        lcd.print(F("Waiting QB send"));
        lcd.setCursor(0,1);
        lcd.print(F("data..."));

        #ifdef DEBUG
          Serial.println(F("Waiting QB send data..."));
        #endif

        if(wait_qb_update()) {
          uint8_t qb_num = update_qb_info();
          full_update_to_mg(qb_num);
        }
        else {
          lcd.clear();
          lcd.print(F("No response"));
          lcd.setCursor(0,1);
          lcd.print(F("from QB"));

        #ifdef DEBUG
          Serial.println(F("No data received from QB"));
        #endif
        }
      }
      goto Select_mode_2;
    }

    case 'L': case 'U':
    {
      if(page > 1){
        page--;
        goto Select_mode_2;
      }
    }

    case 'R': case 'D':
    {
      if(page < max_page){
        page++;
        goto Select_mode_2;
      }
    }

    case '-':
    {
      lcd.clear();
      lcd.print(F("F1: Admin mode1"));
      lcd.setCursor(0,1);
      lcd.print(F("F2: Admin mode2"));
      return;
    }

    default:
    {
      break;
    }
  }
  goto Select_mode_2;

}

bool new_qb_sync() {
  lcd.clear();
  lcd.print(F("Please connect"));
  lcd.setCursor(0,1);
  lcd.print(F("the QB"));
  delay(1000);

  uint8_t count = 0;

  char message[] = "connect";
  char ack[] = "connected";

  Retransmission:
  #ifdef DEBUG
    Serial.print(F("Sending: "));
    Serial.println(message);
  #endif
  bool connection = send_to_qb(500, message, sizeof(message), ack, sizeof(ack));       ///here
  if(connection) {
    delay(100);
    lcd.clear();
    lcd.print(F("QB Connection"));
    lcd.setCursor(0,1);
    lcd.print(F("Success"));

    #ifdef DEBUG
      Serial.println(F("QB connnection Success"));
    #endif
    delay(1000);

    lcd.clear();
    lcd.print(F("Sending Channel"));
    lcd.setCursor(0,1);
    lcd.print(F(",addr & usr num"));

    delay(1000);

    qb_sync_message qb_sync_info;

    qb_sync_info.ig_channel = ig.my_nrf24_channel;
    qb_sync_info.ig_addr = ig.my_nrf24_addr;
    qb_sync_info.qb_addr = person[ig.current_qb_number].qb_addr;
    qb_sync_info.qb_num = ig.current_qb_number;

    #ifdef DEBUG
      Serial.println(F("Sending: "));
      Serial.print(F("ig_channel = "));
      Serial.println(qb_sync_info.ig_channel);
      Serial.print(F("ig_addr = "));
      Serial.println(qb_sync_info.ig_addr, 16);
      Serial.print(F("qb_addr = "));
      Serial.println(qb_sync_info.qb_addr, 16);
      Serial.print(F("qb_num = "));
      Serial.println(qb_sync_info.qb_num);
    #endif
    bool qb_sync = send_to_qb(500, reinterpret_cast<char*>(&qb_sync_info), sizeof(qb_sync_message), \
                    reinterpret_cast<char*>(&qb_sync_info), sizeof(qb_sync_message));   //send channel, addr, & user number
    
    if(qb_sync){
      lcd.clear();
      lcd.print(F("Waiting QB send"));
      lcd.setCursor(0,1);
      lcd.print(F("data..."));
      
      #ifdef DEBUG
        Serial.println(F("Waiting QB send data..."));
      #endif

      if(wait_qb_update()) {
        update_qb_info();
        return true;
      }
      else {
        goto QB_connection_failed;
      }
    }
    else {
      goto QB_connection_failed;
    }
  }
  else {
    count++;
    if(count<5){
      delay(100);
      goto Retransmission;
    }
    else {
      QB_connection_failed:
      return false;
    }
  }
}

void qb_setting() {
  lcd.clear();
  lcd.print(F("Enter IC No.:"));
  lcd.setCursor(0,1);
  String input_str = keypad_input();
  
  #ifdef DEBUG
    Serial.print(F("input ic = "));
    Serial.println(input_str.begin());
  #endif

  strcpy(person[ig.current_qb_number].ic, input_str.begin());

  /////debuging here
  #ifdef DEBUG
    Serial.print(F("saved ic = "));
    Serial.println(person[ig.current_qb_number].ic);
  #endif

  lcd.clear();
  lcd.print(F("Enter H/P No.:"));
  lcd.setCursor(0,1);
  String input_hp_str = keypad_input();
  strcpy(person[ig.current_qb_number].hp_num, input_hp_str.begin());  

  #ifdef DEBUG
    Serial.print(F("saved hp = "));
    Serial.println(person[ig.current_qb_number].hp_num);
  #endif

  lcd.clear();
  lcd.print(F("Done QB setting"));
  delay(1000);
}

bool send_to_qb(uint32_t max_time_ms, char* message, unsigned int message_size, char* ack, unsigned int ack_size) {
  // initialize timer4 
  unsigned long max_wait_millis = 10000;    //max wait for 3s
  unsigned long start_millis = millis();
  unsigned long current_millis;

  bool ack_timeout = 0;

  Serial1.write(message, message_size);
  
  while(Serial1.available()<=0) {
    current_millis = millis();
    if(current_millis - start_millis >= max_wait_millis) {
      ack_timeout = 1;
      break;
    }
  }
  if(ack_timeout) {
    lcd.clear();
    lcd.print(F("Ack timeout"));
    lcd.setCursor(0,1);
    lcd.print(F("Try again"));

    #ifdef DEBUG
      Serial.println(F("Ack timeout, try again"));
    #endif
    return false;
  }

  char* incoming_ack = (char*)malloc(ack_size);
  Serial1.readBytes(incoming_ack, ack_size);

  if(strncmp(incoming_ack, ack, ack_size) == 0) {
    lcd.clear();
    lcd.print(F("Ack Received"));
    #ifdef DEBUG
      Serial.println(F("Ack Received"));
    #endif
    delay(1000);
    free(incoming_ack);
    return true;
  }
  else {
    lcd.clear();
    lcd.print(F("Wrong Ack"));
    lcd.setCursor(0,1);
    lcd.print(F("Try again"));
    delay(500);
    #ifdef DEBUG
      Serial.print(F("Wrong Ack, try again"));
    #endif
    free(incoming_ack);
    return false;
  }
}

uint8_t scan_and_choose_channel(RF24& nrf24) {
  //scan for channel with low interference
  nrf24.setPALevel(RF24_PA_MAX);      //set max signal amplification
  nrf24.setAutoAck(false);
  nrf24.stopListening();    //standby mode 

  const uint8_t num_channels = 100;
  const unsigned int scan_time_ms = 1000;
  
  uint8_t i = num_channels;
  while (i--)
  {
    // Select this channel
    nrf24.setChannel(i);

    // Listen for a little
    nrf24.startListening();
    delay(scan_time_ms);    //scan for 300ms
    nrf24.stopListening();

    // Did we get a carrier?
    if (!nrf24.testCarrier() ) {
      return i;
    }
  }
  return -1;
}

///here
bool wait_qb_update() {
  unsigned long max_wait_millis = 5000;    //max wait for 3s
  unsigned long start_millis = millis();
  unsigned long current_millis;
	
	uint8_t pipe;
	while(!nrf24.available(&pipe)){
		current_millis = millis();
		if(current_millis - start_millis >= max_wait_millis) {
			return false;
		}
	}

  #ifdef DEBUG
    Serial.println(F("Received update from QB"));
  #endif
	return true;
}

void qb_stat_view() {
  uint8_t stat_page = 1;
  uint8_t max_stat_page = ig.current_qb_number+1;

  QB_stat_view:
  lcd.clear();
  lcd.print(F("QB"));
  lcd.print(stat_page);
  lcd.print(F(" "));
  uint8_t stat = person[stat_page-1].status;
  switch (stat)
  {
    case NOT_INIT:
    {
      lcd.print(F("NOT_INIT"));
      break;
    }

    case PRESENT:
    {
      lcd.print(F("PRESENT"));
      break;
    }
      
    case OPENED:
    {
      lcd.print(F("OPENED"));
      break;
    }

    case MISSING:
    {
      lcd.print(F("MISSING"));
      break;
    }

    case EMERGENCY:
    {
      lcd.print(F("EMERGENCY"));
      break;
    }

    default:
    {
      break;
    }
  }
  lcd.setCursor(0,1);
  lcd.print(person[stat_page-1].ic);

  char key = 0;
  while(!(key = customKeypad.getKey()));  //ignore null
  switch (key)
  {
    case 'L': case 'U':
    {
      if(stat_page > 1){
        stat_page--;
      }
      goto QB_stat_view;
    }

    case 'R': case 'D':
    {
      if(stat_page < max_stat_page){
        stat_page++;
      }
      goto QB_stat_view;
    }

    case '-':
    {
      return;
    }
    
    default:
    {
      goto QB_stat_view;
    }
    
  }
}

uint8_t update_qb_info() {
  qb_message qb_msg;
  nrf24.read(&qb_msg, 2);            // fetch payload from FIFO

  uint8_t qb_id = qb_msg.qb_num;    //extract qb_id
  uint8_t stat = qb_msg.status;   //extract status
  person[qb_id].status = stat;

  get_gps_info();

  person[qb_id].time_h = gps.time.hour();     //save incomming time
  person[qb_id].time_m = gps.time.minute();
  person[qb_id].date_dd = gps.date.day();
  person[qb_id].date_mm = gps.date.month();
  person[qb_id].date_yyyy = gps.date.year();

  #ifdef DEBUG
    Serial.println(F("QB info updated:"));
    Serial.print(F("QB ID: "));
    Serial.println(qb_id);
    Serial.print(F("QB status: "));
    switch (stat)
    {
      case NOT_INIT:
      {
        Serial.println(F("NOT_INIT"));
        break;
      }
      
      case PRESENT:
      {
        Serial.println(F("PRESENT"));
        break;
      }

      case OPENED:
      {
        Serial.println(F("OPENED"));
        break;
      }

      case MISSING:
      {
        Serial.println(F("MISSING"));
        break;
      }

      case EMERGENCY:
      {
        Serial.println(F("EMERGENCY"));
        break;
      }

      default:
      {
        Serial.println(F("ERROR"));
        break;
      }
    }
  #endif

  return qb_id;
}

bool sync_to_mg() {
  ig_sync_message ig_to_mg;

  ig_to_mg.ig_header.message_type = REQUEST_E22_ADDR;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_8;
  ig_to_mg.ig_header.crc_poly = MY_CRC8_POLY;

  ig_to_mg.crc = crc8(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(ig_sync_message)-1, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  uint8_t retry = 0;
  unsigned long max_wait_millis = 10000;    //max wait for 3s
  unsigned long start_millis;
  unsigned long current_millis;

  Retry:
  #ifdef DEBUG
    Serial.println(F("sending REQUEST_E22_ADDR message"));
  #endif
  
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_sync_message));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync to MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  start_millis = millis();

  #ifdef DEBUG
      Serial.println(F("Waiting response"));
    #endif
  while(e22.available()<=1) {
    current_millis = millis();
    if(current_millis - start_millis >= max_wait_millis) {
      if(retry<10) {
        retry++;    
        delay(random(0,pow(2,retry)*5));    //random delay if collision happens (0 to 5ms * 2^retry)
        goto Retry;
      }
      else {
        lcd.clear();
        lcd.print(F("Sync to MG"));
        lcd.setCursor(0,1);
        lcd.print(F("Failed"));
        delay(1000);
        return false;
      }
    }
  }

  #ifdef DEBUG
    Serial.println(F("Received response"));
  #endif
  //e22.avalable
  mg_sync_message mg_to_ig;
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_sync_message));   ///debugging here
  if (rsc.status.code!=1){
    retry++;
    goto Retry;
  }
  mg_to_ig = *(mg_sync_message*)rsc.data;
  if((mg_to_ig.mg_header.message_type != RETURN_E22_ADDR) || (mg_to_ig.mg_header.ack_payload != ACK_PAYLOAD)
  || (mg_to_ig.mg_header.sender_sequence != ig_to_mg.ig_header.receiver_sequence) 
  || (mg_to_ig.mg_header.receiver_sequence != (ig_to_mg.ig_header.sender_sequence ^ 0x01))
  || (mg_to_ig.mg_header.crc_length != CRC_8)) {
    retry++;
    goto Retry;
  }
  uint8_t crc = crc8(reinterpret_cast<uint8_t*>(&mg_to_ig), sizeof(mg_sync_message)-1, mg_to_ig.mg_header.crc_poly);
  if(crc != mg_to_ig.crc) {
    retry++;
    goto Retry;
  }
  else {
    mg.mg_e22_addr_h = mg_to_ig.mg_header.sender_e22_addr_h;
    mg.mg_e22_addr_l = mg_to_ig.mg_header.sender_e22_addr_l;
    ig.my_e22_addr_h = mg_to_ig.ig_e22_addr_h;
    ig.my_e22_addr_l = mg_to_ig.ig_e22_addr_l;
    if(e22.set_e22_configuration(ig.my_e22_addr_h, ig.my_e22_addr_l, ig.e22_channel, true, POWER_21, AIR_DATA_RATE_000_03, 
    ig.e22_crypt_h, ig.e22_crypt_l, true)) {
      lcd.clear();
      lcd.print(F("Sync to MG"));
      lcd.setCursor(0,1);
      lcd.print(F("Success"));

      #ifdef DEBUG
        Serial.println(F("Sync to MG Success"));
        Serial.print(F("MG E22 addr = "));
        Serial.print(mg.mg_e22_addr_h, 16);
        Serial.println(mg.mg_e22_addr_l, 16);
      #endif

      ig_ack_message ig_ack;

      
      
      ig_ack.ig_ack_header.message_type = ACK;
      ig_ack.ig_ack_header.ack_payload = ACK_ONLY;
      ig_ack.ig_ack_header.sender_e22_addr_h = ig.my_e22_addr_h;
      ig_ack.ig_ack_header.sender_e22_addr_l = ig.my_e22_addr_l;
      ig_ack.ig_ack_header.sender_sequence = ig_to_mg.ig_header.sender_sequence ^ 1;
      ig_ack.ig_ack_header.receiver_sequence = ig_to_mg.ig_header.receiver_sequence ^ 1;
      ig_ack.ig_ack_header.crc_length = CRC_8;
      ig_ack.ig_ack_header.crc_poly = MY_CRC8_POLY;

      ig_ack.crc = crc8(reinterpret_cast<uint8_t*>(&ig_ack), sizeof(ig_ack_message)-1, ig_ack.ig_ack_header.crc_poly);

      msg_str = reinterpret_cast<char*>(&ig_ack);

      #ifdef DEBUG
        Serial.println(F("Sending ack message to MG"));
      #endif
      //send ack message 3 times, no need to wait response
      for(int i = 0; i<3; i++) {
        e22.sendFixedMessage(mg.mg_e22_addr_h, mg.mg_e22_addr_l, ig.e22_channel, msg_str, sizeof(ig_ack_message));
        delay(500);
        randomSeed(analogRead(A0));
        delay(random(200));
      }
      #ifdef DEBUG
        Serial.println(F("Sync to MG completed"));
      #endif
      lcd.clear();
      lcd.print(F("Sync to MG"));
      lcd.setCursor(0,1);
      lcd.print(F("Completed"));
      delay(1000);
      return true;
    }
    else {
      lcd.clear();
      lcd.print(F("Sync to MG"));
      lcd.setCursor(0,1);
      lcd.print(F("Failed"));
      delay(1000);
      return false;
    }
  }
}

bool get_gps_info() {
  unsigned long max_wait_millis = 3000;
  uint8_t retry = 0;

  #ifdef DEBUG
    Serial.println(F("Getting GPS info"));
  #endif

  Retry:
  if(retry >10 ) {
    #ifdef DEBUG
      Serial.println(F("Fail to get GPS info"));
    #endif
    return false;
  }
  unsigned long start_millis = millis();
  while (Serial2.available() <= 0) {
    unsigned long current_millis = millis();
    if(current_millis - start_millis>= max_wait_millis) { 
      retry++;
      goto Retry;
    }
  }

  start_millis = millis();
  unsigned long current_millis = start_millis;
  while(current_millis - start_millis <= 5000) {
    while(Serial2.available() > 0){
      if(gps.encode(Serial2.read())){
        ig.latitude = (float)gps.location.lat();
        ig.longitude = (float)gps.location.lng();
        
        ig.date_dd = gps.date.day();
        ig.date_mm = gps.date.month();
        ig.date_yyyy = gps.date.year();

        ig.time_h = gps.time.hour();
        ig.time_m = gps.time.minute();
      }
    }
    current_millis = millis();
  }
  
  #ifdef DEBUG
      Serial.print(F("latitude = "));
      Serial.println(ig.latitude, 6);
      Serial.print(F("longitude = "));
      Serial.println(ig.longitude, 6);
  #endif

  delay(500);

  //checking gps signal
  if(ig.latitude == 0 || ig.longitude == 0){
    retry++;
    goto Retry;
  }

  return true;

}

bool sync_qb_mg() {
  ig_sync_message ig_to_mg;

  ig_to_mg.ig_header.message_type = REQUEST_GOOGLESHEET_ROW;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_8;
  ig_to_mg.ig_header.crc_poly = MY_CRC8_POLY;

  ig_to_mg.crc = crc8(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(ig_sync_message)-1, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  uint8_t retry = 0;
  unsigned long max_wait_millis = 10000;    //max wait for 10s
  unsigned long start_millis;
  unsigned long current_millis;

  Retry:
  #ifdef DEBUG
    Serial.println(F("sending REQUEST_GOOGLESHEET_ROW message"));
  #endif
  
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_sync_message));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  start_millis = millis();

  #ifdef DEBUG
      Serial.println(F("Waiting response"));
    #endif

  while(e22.available()<=1) {
    current_millis = millis();
    if(current_millis - start_millis >= max_wait_millis) {
      if(retry<10) {
        retry++;    
        delay(random(0,pow(2,retry)*5));    //random delay if collision happens (0 to 5ms * 2^retry)
        goto Retry;
      }
      else {
        lcd.clear();
        lcd.print(F("Sync QB MG"));
        lcd.setCursor(0,1);
        lcd.print(F("Failed"));
        delay(1000);
        return false;
      }
    }
  }
 
  //e22.avalable
  mg_sync_message_qb_mem mg_to_ig;
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_sync_message_qb_mem));
  if (rsc.status.code!=1){
    return false;
  }
  mg_to_ig = *(mg_sync_message_qb_mem*)rsc.data;
  if((mg_to_ig.mg_header.message_type != RETURN_GOOGLESHEET_ROW) || (mg_to_ig.mg_header.ack_payload != ACK_PAYLOAD)
  || (mg_to_ig.mg_header.sender_sequence != ig_to_mg.ig_header.receiver_sequence) 
  || (mg_to_ig.mg_header.receiver_sequence != (ig_to_mg.ig_header.sender_sequence ^ 0x01))
  || (mg_to_ig.mg_header.crc_length != CRC_8)) {
    retry++;
    goto Retry;
  }
  uint8_t crc = crc8(reinterpret_cast<uint8_t*>(&mg_to_ig), sizeof(mg_sync_message_qb_mem)-1, mg_to_ig.mg_header.crc_poly);
  if(crc != mg_to_ig.crc) {
    retry++;
    goto Retry;
  }
  else {
    person[ig.current_qb_number].mg_mem_addr = mg_to_ig.qb_mem_addr;

    ig_ack_message ig_ack;

    ig_ack.ig_ack_header.message_type = ACK;
    ig_ack.ig_ack_header.ack_payload = ACK_ONLY;
    ig_ack.ig_ack_header.sender_e22_addr_h = ig.my_e22_addr_h;
    ig_ack.ig_ack_header.sender_e22_addr_l = ig.my_e22_addr_l;

    ig_ack.ig_ack_header.sender_sequence = ig_to_mg.ig_header.sender_sequence ^ 0x01;
    ig_ack.ig_ack_header.receiver_sequence = ig_to_mg.ig_header.receiver_sequence ^ 0x01;

    ig_ack.ig_ack_header.crc_length = CRC_8;
    ig_ack.ig_ack_header.crc_poly = MY_CRC8_POLY;

    ig_ack.crc = crc8(reinterpret_cast<uint8_t*>(&ig_ack), sizeof(ig_ack_message)-1, ig_ack.ig_ack_header.crc_poly);

    msg_str = reinterpret_cast<char*>(&ig_ack);

    //send ack message 3 times, no need to wait response
    for(int i = 0; i<3; i++) {
      e22.sendFixedMessage(mg.mg_e22_addr_h, mg.mg_e22_addr_l, ig.e22_channel, msg_str, sizeof(ig_ack_message));
      delay(500);
      randomSeed(analogRead(A0));
      delay(random(200));
    }

    #ifdef DEBUG
      Serial.println(F("Sync QB MG Completed"));
    #endif
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Completed"));
    delay(1000);
    return true;
  }
}

bool full_update_to_mg(uint8_t qb_num) {
  ig_full_message ig_to_mg;

  ig_to_mg.ig_header.message_type = FULL_UPDATE_INFO;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_16;
  ig_to_mg.ig_header.crc_poly = MY_CRC16_POLY;

  ig_to_mg.qb_addr = person[qb_num].qb_addr;
  ig_to_mg.mg_mem_addr = person[qb_num].mg_mem_addr;
  ig_to_mg.initial_latitude = ig.initial_latitude;
  ig_to_mg.initial_longitude = ig.initial_longitude;

  strcpy(ig_to_mg.ic, person[qb_num].ic);
  strcpy(ig_to_mg.hp_num, person[qb_num].hp_num);

  ig_to_mg.status = person[qb_num].status;

  ig_to_mg.crc = crc16(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(ig_full_message)-2, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  unsigned long max_wait_millis = 10000;    //max wait for 10s
  uint8_t retry = 0;

  Retry:
  unsigned long start_millis = millis();
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_full_message));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  while(e22.available()<=1) {
    unsigned long current_millis = millis();
    if(current_millis - start_millis >= max_wait_millis) {
      if(retry<10) {
        retry++;
        delay(random(0,pow(2,retry)*5));    //random delay if collision happens (0 to 5ms * 2^retry)
        goto Retry;
      }
      else {
        lcd.clear();
        lcd.print(F("Sync QB MG"));
        lcd.setCursor(0,1);
        lcd.print(F("Failed"));
        delay(1000);
        return false;
      }
    }
  }
  //e22.avalable
  struct mg_message{
    e22_message_header mg_header;

    uint8_t crc;
  };
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_message));
  if (rsc.status.code!=1){
    return false;
  }
  mg_message mg_to_ig = *(mg_message*)rsc.data;
  if((mg_to_ig.mg_header.message_type != ACK) || (mg_to_ig.mg_header.ack_payload != ACK_ONLY)
  || (mg_to_ig.mg_header.sender_sequence != ig_to_mg.ig_header.receiver_sequence) 
  || (mg_to_ig.mg_header.receiver_sequence != (ig_to_mg.ig_header.sender_sequence ^ 0x01))
  || (mg_to_ig.mg_header.crc_length != CRC_8)) {
    retry++;
    goto Retry;
  }
  uint8_t crc = crc8(reinterpret_cast<uint8_t*>(&mg_to_ig), sizeof(mg_message)-1, mg_to_ig.mg_header.crc_poly);
  if(crc != mg_to_ig.crc) {
    retry++;
    goto Retry;
  }
  lcd.clear();
  lcd.print(F("QB info updated"));
  lcd.setCursor(0,1);
  lcd.print(F("to MG"));
  delay(1000);
  return true;
}

bool update_to_mg(uint8_t qb_num) {
  ig_update_message update;

  update.ig_header.message_type = UPDATE_INFO;
  update.ig_header.ack_payload = ACK_PAYLOAD;
  update.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  update.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  update.ig_header.sender_sequence = 0;
  update.ig_header.receiver_sequence = 0;
  update.ig_header.crc_length = CRC_16;
  update.ig_header.crc_poly = MY_CRC16_POLY;

  update.mg_mem_addr = person[qb_num].mg_mem_addr;
  update.current_latitude = ig.latitude;
  update.current_longitude = ig.longitude;
  update.status = person[qb_num].status;

  update.crc = crc16(reinterpret_cast<uint8_t*>(&update), sizeof(ig_update_message)-2, update.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&update);

  uint8_t retry = 0;
  unsigned long max_wait_millis = 10000;    //max wait for 10s

  Retry:
  unsigned long start_millis = millis();
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_update_message));    //boardcast
  if(rs.code!=1) {
    if(retry<10) {
      retry++;
      delay(random(0,pow(2,retry)*5));    //random delay if collision happens (0 to 5ms * 2^retry)
      goto Retry;
    }
    else {
      lcd.clear();
      lcd.print(F("Updating QB"));
      lcd.print(qb_num);
      lcd.setCursor(0,1);
      lcd.print(F("to MG Failed"));
      delay(1000);
      return false;
    }
  }
  while(e22.available()<=1) {
    unsigned long current_millis  = millis();
    if(current_millis - start_millis>= max_wait_millis) {
      if(retry<10) {
        retry++;
        delay(random(0,pow(2,retry)*5));    //random delay if collision happens (0 to 5ms * 2^retry)
        goto Retry;
      }
      else {
        lcd.clear();
        lcd.print(F("Updating QB"));
        lcd.print(qb_num);
        lcd.setCursor(0,1);
        lcd.print(F("to MG Failed"));
        delay(1000);
        return false;
      }
    }
  }

  //e22.avalable
  struct mg_message{
    e22_message_header mg_header;

    uint8_t crc;
  };
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_message));
  if (rsc.status.code!=1){
    return false;
  }
  mg_message mg_to_ig = *(mg_message*)rsc.data;
  if((mg_to_ig.mg_header.message_type != ACK) || (mg_to_ig.mg_header.ack_payload != ACK_ONLY)
  || (mg_to_ig.mg_header.sender_sequence != update.ig_header.receiver_sequence) 
  || (mg_to_ig.mg_header.receiver_sequence != (update.ig_header.sender_sequence ^ 0x01))
  || (mg_to_ig.mg_header.crc_length != CRC_8)) {
    retry++;
    goto Retry;
  }
  uint8_t crc = crc8(reinterpret_cast<uint8_t*>(&mg_to_ig), sizeof(mg_message)-1, mg_to_ig.mg_header.crc_poly);
  if(crc != mg_to_ig.crc) {
    retry++;
    goto Retry;
  }
  lcd.clear();
  lcd.print(F("Updating QB"));
  lcd.print(qb_num);
  lcd.setCursor(0,1);
  lcd.print(F("to MG Success"));
  delay(1000);
  return true;
}

ISR(TIMER5_OVF_vect)
{
  if(timer_counter < 72) {    //72 * 4.2s = 5mins
    timer_counter ++;
  }
  else {
    timer_counter = 0;
    update_gps = true;
    check_time = true;
  }
}


