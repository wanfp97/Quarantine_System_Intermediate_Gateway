#define DEBUG

#define E22_30    //EBYTE E22_30 series
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
bool send_to_qb(uint32_t max_time_ms, String message, String ack);
uint8_t scan_and_choose_channel(RF24& nrf24);
bool wait_qb_update();
void qb_stat_view();
uint8_t update_qb_info();
bool get_gps_info(uint16_t max_wait_time_ms);
void set_timer4_count_ms(uint16_t time_ms, uint16_t& steps);
bool sync_to_mg();
bool sync_qb_mg();
bool update_to_mg(uint8_t qb_num);

uint8_t pipe0;

enum QUARANTINE_BAND_STATUS {
  NOT_INIT  = 0,
  PRESENT   = 1,
  OPENED    = 2,
  MISSING   = 3,
  EMERGENCY = 4
};

enum E22_MESSAGE_TYPE {
  REQUEST_E22_ADDR        = 0,
  REQUEST_MEM_ADDR        = 1,
  UPDATE_INFO             = 2,

  RETURN_E22_ADDR         = 3,
  RETURN_MEM_ADDR         = 4,

  ACK                     = 5
};

enum E22_ACK_PAYLOAD {
  ACK_ONLY    = 0,
  ACK_PAYLOAD = 1
};

enum CRC_LENGTH {
  CRC_8   = 0,
  CRC_16  = 1,
};

struct e22_message_header{  //11 bytes
  uint8_t message_type;
  uint8_t sender_e22_addr_h;
  uint8_t sender_e22_addr_l;
  uint8_t ack_payload;
  uint8_t sender_sequence;
  uint8_t receiver_sequence;
  uint8_t crc_length;
  uint16_t crc_poly;
};



struct user {
  uint16_t mg_mem_addr;
  uint8_t date_dd;
  uint8_t date_mm;
  uint16_t date_yyyy;
  uint8_t time_h;
  uint8_t time_m;
  uint32_t ic;
  uint8_t status;

  uint32_t qb_addr;

}person[10];

struct intermediate_gateway {
  MyString my_password = "1234";
  
  uint32_t my_nrf24_addr = random(1, 0x7FFFFFFE);       //MSbit must be 0 for random() to work with 32bits
  uint8_t my_nrf24_channel;

  uint8_t current_qb_number = 0;

  bool sync_to_mg = 0;
  uint8_t my_e22_addr_h = random(0,pow(2,8)-2);     //avoid 0xFF (0xFF and 0 for monitor mode)
  uint8_t my_e22_addr_l = random(1,pow(2,8)-1);     //avoid 0 
  uint8_t e22_channel = 70;     //920.125 MHz
  uint8_t e22_crypt_h = 0xAB;   //used for encryption and decryption
  uint8_t e22_crypt_l = 0xAB;
  
  uint8_t date_dd;
  uint8_t date_mm;
  uint16_t date_yyyy;
  uint8_t time_h;
  uint8_t time_m;
  long latitude;
  long longitude;
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

static const uint32_t GPSBaud = 4800;

// The TinyGPS++ object
TinyGPSPlus gps;

MyLoRa_E22 e22(26, 28, 30, 22, 24);    // Arduino RX <-- e22 TX, Arduino TX --> e22 RX AUX M0 M1

void setup(){
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.cursor();

  lcd.clear();
  lcd.print(F("Starting up"));
  lcd.setCursor(0,1);
  lcd.print(F("Serial ..."));

  Serial.begin(115200);   //for debugging
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

  lcd.clear();
  lcd.print(F("Generating"));
  lcd.setCursor(0,1);
  lcd.print(F("random qb addr"));

  randomSeed(analogRead(A0));       //read floting pin
  person[0].qb_addr = random(1, 0x7FFFFFFE);    //MSbit must be zero for random to function correctly
  #ifdef DEBUG
      Serial.print(F("qb"));
      Serial.print(0,10);
      Serial.print(F(" addr = "));
      Serial.println(person[0].qb_addr, 16);
  #endif
  delay(200);     //
  for(int i = 1; i < 10; i++) {
    Reassign:
    randomSeed(analogRead(A0));
    person[i].qb_addr = random(1, 0x7FFFFFFE);    //MSbit must be 0 for random() to work
    #ifdef DEBUG
      Serial.print(F("qb"));
      Serial.print(i,10);
      Serial.print(F(" addr = "));
      Serial.println(person[i].qb_addr, 16);
    #endif
    delay(200);       //delay so that next random seed will read different analog rading
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
  lcd.print(F("IG Main Menu"));
}
  
void loop(){
  
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
      lcd.print(F("3: Show IG info"));
      break;
    }
  }
  
  char selection;
  while(!(selection = customKeypad.getKey()));  //ignore null
  switch (selection)
  {
    case '1':
    {
      uint8_t channel = scan_and_choose_channel(nrf24);
      if(channel != -1) {
        ig.my_nrf24_channel = channel;

        nrf24.setPALevel(RF24_PA_MAX);      //set max signal amplification
        nrf24.setCRCLength(RF24_CRC_8);     //set 8 bit crc
        nrf24.setPayloadSize(2);      //1byte user number + 1 byte status byte
        nrf24.setAutoAck(true);       //enable auto acknoledgement
        nrf24.setChannel(channel);      //set channel
        nrf24.setAddressWidth(4);        //set addr width
        nrf24.openReadingPipe(0, ig.my_nrf24_addr);     //set receiving addr
        nrf24.startListening();     //start receiving

        lcd.clear();
        lcd.print(F("Set Channel"));
        lcd.setCursor(0,1);
        lcd.print(F("Success"));
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
        if(e22.set_e22_configuration(ig.my_e22_addr_h, ig.my_e22_addr_l, ig.e22_channel, true, POWER_21, 
        AIR_DATA_RATE_000_03, ig.e22_crypt_h, ig.e22_crypt_l, false)) {
          if(sync_to_mg()) {
            ig.sync_to_mg = true;
            goto Select_mode_1;
          }
          else {
            goto Select_mode_1;
          }
        }
        else {
          lcd.clear();
          lcd.print(F("IG-MG Sync"));
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
      lcd.print(F("IG Main Menu"));
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
      lcd.print(F("4: Show QB stat"));
      lcd.setCursor(0,1);
      lcd.print(F("5: Sync QB-MG"));
      break;
    }

    case 4:
    {
      lcd.clear();
      lcd.print(F("6: Update QB-MG"));
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
      new_qb_sync();
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
      if(send_to_qb(300, "update", "start")) {
        if(wait_qb_update()) {
          uint8_t qb_num = update_qb_info();
          update_to_mg(qb_num);
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
      lcd.print(F("IG Main Menu"));
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

  uint8_t count = 0;

  Retransmission:
  bool connection = send_to_qb(500, "connect", "connected");       ///here
  if(connection) {
    lcd.clear();
    lcd.print(F("QB Connection"));
    lcd.setCursor(0,1);
    lcd.print(F("Success"));
    delay(1000);

    lcd.clear();
    lcd.print(F("Sending Channel"));
    lcd.setCursor(0,1);
    lcd.print(F(",addr & usr num"));

    String combined_str = String(ig.my_nrf24_channel,16);   ///testing
    String temp_str = String(ig.my_nrf24_addr,16);
    combined_str.concat(temp_str);
    temp_str = String(person[ig.current_qb_number].qb_addr,16);
    combined_str.concat(temp_str);
    temp_str = String(ig.current_qb_number, 16); 
    combined_str.concat(temp_str);
    
    bool qb_sync = send_to_qb(500, combined_str, combined_str);   //send channel & user number
    
    if(qb_sync){
      lcd.clear();
      lcd.print(F("Waiting QB"));
      lcd.setCursor(0,1);
      lcd.print(F("transmission"));
      delay(500);

      if(wait_qb_update()) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      goto QB_connection_failed;
    }
  }
  else {
    count++;
    if(count<5){
      goto Retransmission;
    }
    else {
      QB_connection_failed:
      lcd.clear();
      lcd.print(F("QB Sync"));
      lcd.setCursor(0,1);
      lcd.print(F("Failed"));
      delay(1000);
      return false;
    }
  }
}

void qb_setting() {
  lcd.clear();
  lcd.print(F("Enter IC No.:"));
  lcd.setCursor(0,1);
  String ic_str = keypad_input();
  char *ic = reinterpret_cast<char *> (&ic_str);    ///testing
  person[ig.current_qb_number].ic = atoi(ic);   

  lcd.clear();
  lcd.print(F("Done QB setting"));
  delay(1000);
}

bool send_to_qb(uint32_t max_time_ms, String message, String ack) {
  // initialize timer4 
  uint16_t steps;
  set_timer4_count_ms(max_time_ms, steps);

  bool ack_timeout = 0;
  Serial1.print(message);
  
  while(Serial1.available()<=0) {
    uint16_t current = TCNT4;
    if(current >= steps) {
      ack_timeout = 1;
      break;
    }
  }
  if(ack_timeout) {
    lcd.clear();
    lcd.print(F("Ack timeout"));
    lcd.setCursor(0,1);
    lcd.print(F("Try again"));
    delay(500);
    return false;
  }

  String incoming = Serial1.readString();
  if(incoming == ack) {
    lcd.clear();
    lcd.print(F("Ack Received"));
    delay(1000);
    return true;
  }
  else {
    lcd.clear();
    lcd.print(F("Wrong Ack"));
    lcd.setCursor(0,1);
    lcd.print(F("Try again"));
    delay(500);
    return false;
  }
}

uint8_t scan_and_choose_channel(RF24& nrf24) {
  //scan for channel with low interference
  nrf24.setAutoAck(false);
  nrf24.stopListening();    //standby mode
  #ifdef DEBUG
  nrf24.printDetails();   //print nrf24 details to UART0 terminal
  #endif  

  const uint8_t num_channels = 100;
  const int scan_time_ms = 300;
  
  uint8_t i = num_channels;
  while (i--)
  {
    // Select this channel
    nrf24.setChannel(i);

    // Listen for a little
    nrf24.startListening();
    delayMicroseconds(scan_time_ms);    //scan for 300ms
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
  uint8_t count = 0;
  const int max_wait_count = 5;

  Continue_waiting:
  uint8_t pipe;
  if (nrf24.available(&pipe)) {             // check for all rx pipe and return pipe number if data available
    update_qb_info();     //update qb status and time according to data received

    lcd.clear();
    lcd.print(F("QB Sync"));
    lcd.setCursor(0,1);
    lcd.print(F("Success"));
    delay(1000);
    return true;
  }
  else {
    if(count<max_wait_count){
      count = count + 1;
      delay(300);
      goto Continue_waiting;  ///here
    }
    else {
      lcd.clear();
      lcd.print(F("QB Sync"));
      lcd.setCursor(0,1);
      lcd.print(F("Failed"));
      delay(1000);
      return false;
    }
  }
}

void qb_stat_view() {
  uint8_t stat_page = 1;
  uint8_t max_stat_page = ig.current_qb_number+1;

  QB_stat_view:
  lcd.clear();
  lcd.print(F("QB"));
  char* qb = itoa(stat_page, qb, 16);
  lcd.print(*qb);
  lcd.print(F(" "));
  switch (person[stat_page].status)
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
  lcd.print(person[stat_page].ic,10);
  char key = 0;
  while(!(key = customKeypad.getKey()));  //ignore null
  switch (key)
  {
    case 'L': case 'U':
    {
      if(stat_page > 1){
        stat_page--;
        goto QB_stat_view;
      }
    }

    case 'R': case 'D':
    {
      if(stat_page < max_stat_page){
        stat_page++;
        goto QB_stat_view;
      }
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
  uint8_t bytes = nrf24.getPayloadSize(); // get the size of the payload
  uint16_t received = 0;
  nrf24.read(&received, bytes);            // fetch payload from FIFO
  uint8_t qb_id = (received & 0xF0)>>8;    //extract qb_id
  person[qb_id].status = received & 0x0F;   //extract status

  person[qb_id].time_h = gps.time.hour();     //save incomming time
  person[qb_id].time_m = gps.time.minute();
  person[qb_id].date_dd = gps.date.day();
  person[qb_id].date_mm = gps.date.month();
  person[qb_id].date_yyyy = gps.date.year();

  return qb_id;
}

bool sync_to_mg() {
  struct IG_Message{
    e22_message_header ig_header;
    uint8_t crc;
  } ig_to_mg;

  ig_to_mg.ig_header.message_type = REQUEST_E22_ADDR;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_8;
  ig_to_mg.ig_header.crc_poly = MY_CRC8_POLY;


  ig_to_mg.crc = crc8(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(IG_Message)-1, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  uint8_t retry = 0;
  uint16_t steps;
  // initialize timer4 
  set_timer4_count_ms(3000, steps);

  Retry:
  TCNT4 = 0;
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_to_mg));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync to MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  while(e22.available()<=1) {
    uint16_t current = TCNT4;
    if(current >= steps) {
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
  //e22.avalable
  struct mg_message{
    e22_message_header mg_header;

    uint8_t ig_e22_addr_h;
    uint8_t ig_e22_addr_l;
    uint8_t crc;
  };
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_message));
  if (rsc.status.code!=1){
    return false;
  }
  mg_message mg_to_ig = *(mg_message*)rsc.data;
  if((mg_to_ig.mg_header.message_type != RETURN_E22_ADDR) || (mg_to_ig.mg_header.ack_payload != ACK_PAYLOAD)
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

      ig_to_mg.ig_header.sender_sequence ^= 1;
      ig_to_mg.ig_header.receiver_sequence ^= 1;

      struct ig_ack_message{
        e22_message_header ig_ack_header;
        uint8_t crc;
      } ig_ack;

      ig_ack.ig_ack_header.sender_sequence = ig_to_mg.ig_header.sender_sequence ^ 1;
      ig_ack.ig_ack_header.receiver_sequence = ig_to_mg.ig_header.receiver_sequence ^ 1;
      
      ig_ack.ig_ack_header.message_type = ACK;
      ig_ack.ig_ack_header.ack_payload = ACK_ONLY;
      ig_ack.ig_ack_header.sender_e22_addr_h = ig.my_e22_addr_h;
      ig_ack.ig_ack_header.sender_e22_addr_l = ig.my_e22_addr_l;
      ig_ack.ig_ack_header.sender_sequence = 0;
      ig_ack.ig_ack_header.receiver_sequence = 0;
      ig_ack.ig_ack_header.crc_length = CRC_8;
      ig_ack.ig_ack_header.crc_poly = MY_CRC8_POLY;

      ig_ack.crc = crc8(reinterpret_cast<uint8_t*>(&ig_ack), sizeof(ig_ack_message)-1, ig_ack.ig_ack_header.crc_poly);

      //send ack message 3 times, no need to wait response
      for(int i = 0; i<3; i++) {
        e22.sendFixedMessage(mg.mg_e22_addr_h, mg.mg_e22_addr_l, ig.e22_channel, msg_str, sizeof(ig_to_mg));    //boardcast
      }
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

bool get_gps_info(uint16_t max_wait_time_ms) {
  uint16_t steps;
  set_timer4_count_ms(max_wait_time_ms, steps);
  while (Serial2.available() <= 0) {
    uint16_t current = TCNT4;
    if(current>=steps) {
      return false;
    }
  }
  if(gps.encode(Serial2.read())) {
    ig.date_dd = gps.date.day();
    ig.date_mm = gps.date.month();
    ig.date_yyyy = gps.date.year();

    ig.time_h = gps.time.hour();
    ig.time_m = gps.time.minute();
    return true;
  }
  else {
    return false;
  }
}

void set_timer4_count_ms(uint16_t time_ms, uint16_t& steps) {
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler
  steps = 65536 - (float)(16000)/1024*time_ms;  // preload timer 65536-(16MHz/1024)*(interval_ms/1000)
  TCNT4 = 0;
}

bool sync_qb_mg() {
  struct IG_Message{
    e22_message_header ig_header;
    uint8_t crc;
  } ig_to_mg;

  ig_to_mg.ig_header.message_type = REQUEST_MEM_ADDR;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_8;
  ig_to_mg.ig_header.crc_poly = MY_CRC8_POLY;

  ig_to_mg.crc = crc8(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(IG_Message)-1, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  uint8_t retry = 0;
  uint16_t steps;
  // initialize timer4 
  set_timer4_count_ms(3000, steps);

  Retry:
  TCNT4 = 0;
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_to_mg));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  while(e22.available()<=1) {
    uint16_t current = TCNT4;
    if(current >= steps) {
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

    uint16_t qb_mem_addr;

    uint8_t crc;
  };
  ResponseStructContainer rsc = e22.receiveMessage(sizeof(mg_message));
  if (rsc.status.code!=1){
    return false;
  }
  mg_message mg_to_ig = *(mg_message*)rsc.data;
  if((mg_to_ig.mg_header.message_type != RETURN_MEM_ADDR) || (mg_to_ig.mg_header.ack_payload != ACK_PAYLOAD)
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
  else {
    person[ig.current_qb_number].mg_mem_addr = mg_to_ig.qb_mem_addr;

    struct ig_ack_message{
      e22_message_header ig_ack_header;
      uint8_t crc;
    } ig_ack;

    ig_ack.ig_ack_header.message_type = ACK;
    ig_ack.ig_ack_header.ack_payload = ACK_ONLY;
    ig_ack.ig_ack_header.sender_e22_addr_h = ig.my_e22_addr_h;
    ig_ack.ig_ack_header.sender_e22_addr_l = ig.my_e22_addr_l;

    ig_ack.ig_ack_header.sender_sequence = ig_to_mg.ig_header.sender_sequence ^ 1;
    ig_ack.ig_ack_header.receiver_sequence = ig_to_mg.ig_header.receiver_sequence ^ 1;

    ig_ack.ig_ack_header.crc_length = CRC_8;
    ig_ack.ig_ack_header.crc_poly = MY_CRC8_POLY;

    ig_ack.crc = crc8(reinterpret_cast<uint8_t*>(&ig_ack), sizeof(ig_ack_message)-1, ig_ack.ig_ack_header.crc_poly);

    //send ack message 3 times, no need to wait response
    for(int i = 0; i<3; i++) {
      e22.sendFixedMessage(mg.mg_e22_addr_h, mg.mg_e22_addr_l, ig.e22_channel, msg_str, sizeof(ig_to_mg));    //boardcast
    }
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Completed"));
    delay(1000);
    return true;
  }
}

bool update_to_mg(uint8_t qb_num) {
  struct IG_Message{
    e22_message_header ig_header;
    uint16_t mg_mem_addr;
    uint8_t date_dd;
    uint8_t date_mm;
    uint16_t date_yyyy;
    uint8_t time_h;
    uint8_t time_m;
    uint32_t ic;
    uint8_t status;
    uint16_t crc;
  } ig_to_mg;

  ig_to_mg.ig_header.message_type = UPDATE_INFO;
  ig_to_mg.ig_header.ack_payload = ACK_PAYLOAD;
  ig_to_mg.ig_header.sender_e22_addr_h = ig.my_e22_addr_h;
  ig_to_mg.ig_header.sender_e22_addr_l = ig.my_e22_addr_l;
  ig_to_mg.ig_header.sender_sequence = 0;
  ig_to_mg.ig_header.receiver_sequence = 0;
  ig_to_mg.ig_header.crc_length = CRC_16;
  ig_to_mg.ig_header.crc_poly = MY_CRC16_POLY;

  ig_to_mg.mg_mem_addr = person[qb_num].mg_mem_addr;
  ig_to_mg.date_dd = person[qb_num].date_dd;
  ig_to_mg.date_mm = person[qb_num].date_mm;
  ig_to_mg.date_yyyy = person[qb_num].date_yyyy;
  ig_to_mg.time_h = person[qb_num].time_h;
  ig_to_mg.time_m = person[qb_num].time_m;
  ig_to_mg.ic = person[qb_num].ic;
  ig_to_mg.status = person[qb_num].status;

  ig_to_mg.crc = crc16(reinterpret_cast<uint8_t*>(&ig_to_mg), sizeof(IG_Message)-1, ig_to_mg.ig_header.crc_poly);

  char* msg_str = reinterpret_cast<char*>(&ig_to_mg);

  uint8_t retry = 0;
  uint16_t steps;
  // initialize timer4 
  set_timer4_count_ms(3000, steps);

  Retry:
  TCNT4 = 0;
  ResponseStatus rs = e22.sendFixedMessage(0xFF, 0xFF, ig.e22_channel, msg_str, sizeof(ig_to_mg));    //boardcast
  if(rs.code!=1) {
    lcd.clear();
    lcd.print(F("Sync QB MG"));
    lcd.setCursor(0,1);
    lcd.print(F("Failed"));
    delay(1000);
    return false;
  }
  while(e22.available()<=1) {
    uint16_t current = TCNT4;
    if(current >= steps) {
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


