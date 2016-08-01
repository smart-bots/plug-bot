#include <EEPROM.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <avr/wdt.h>
#include <printf.h>

RF24 radio(7,8);

#define nrfIrqPin 2
#define btnPin 3
#define relayPin 4
#define buzzerPin 6
#define currPin A0

const char token[] = "abcabcabc0"; // token

const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};
// pipe.1: read | pipe.2: write -> oposite with hub

int state,buzzerVal;

unsigned long time1;

bool timeToSleep = false;

struct smartbots {
  char token[11];
  boolean state;
  float curr;
};

smartbots trans,recei;

// watchdog interrupt ----------------------------------------------------------------------------------------
ISR(WDT_vect) {
  wdt_disable();  // disable watchdog
}

void myWatchdogEnable(const byte interval) {
//  sleep bit patterns:
//  1 second:  0b000110
//  2 seconds: 0b000111
//  4 seconds: 0b100000
//  8 seconds: 0b100001
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  wdt_reset();
} 

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

//----------------------------------------------------------------------------------------

void go_to_sleep() {
  Serial.println("Sleeping...");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}

//----------------------------------------------------------------------------------------

void turn_bot(bool xState) {
  if (xState != state) {

    state = xState;
    
    if (state) {
      Serial.println("ON");
      digitalWrite(relayPin, HIGH);
    } else {
      Serial.println("OFF");
      digitalWrite(relayPin, LOW);
    }
    
    buzzerVal = 1;
    
    EEPROM.update(1,state);

    timeToSleep = false;
  } else {
    if (state) {
      Serial.println("Already ON");
    } else {
      Serial.println("Already OFF");
    }
  }
}

//----------------------------------------------------------------------------------------

void send_msg(bool xState, bool type) {

  strncpy(trans.token,token,10);
  trans.state = xState;

  if (type) {
    radio.stopListening();
    radio.write(&trans,sizeof(trans)); // Check Ack? Send till success?
    radio.startListening();
  } else {
    radio.writeAckPayload(1,&trans,sizeof(trans));
  }

  Serial.println("Msg sent!");
}

//----------------------------------------------------------------------------------------

void handle_bot(bool xState, bool type) {
  turn_bot(xState);
  send_msg(xState,type);
  Serial.println("Handled bot!");
}

//----------------------------------------------------------------------------------------

void check_radio(void) {
  
  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);
  
  if (rx || radio.available()) {

    radio.read(&recei,sizeof(recei));

    Serial.println("Got: ");
    Serial.println(recei.token);
    Serial.println(recei.state);
    Serial.println("END");
 
    if (strcmp(recei.token,token) == 0) {
      handle_bot(recei.state,0);
    }
  }
}

//----------------------------------------------------------------------------------------

void toggle_bot() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time >= 100) 
  {
    Serial.println("Btn pushed!");
    handle_bot(!state,1);
    last_interrupt_time = interrupt_time;
  }
}

//----------------------------------------------------------------------------------------

void read_state() {
  if (EEPROM.read(0) != 1)
  {
    EEPROM.update(0,1);
    EEPROM.update(1,1);
  }
  state = EEPROM.read(1);
  Serial.print("Read state: ");
  Serial.println(state);
}

void setup()
{
  Serial.begin(9600);

  read_state();

  radio.begin();
//  printf_begin();
//  radio.printDetails();
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(15,15);
  radio.openReadingPipe(1,pipes[0]);
  radio.openWritingPipe(pipes[1]);
  radio.startListening();

  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(nrfIrqPin), check_radio, LOW);
  attachInterrupt(digitalPinToInterrupt(btnPin), toggle_bot, FALLING);
  
  timeToSleep = true;
}

//----------------------------------------------------------------------------------------

void handle_current() {
  float current = abs(mapfloat(analogRead(currPin),0,1024,-30,30));
  Serial.print("Read current: ");
  Serial.println(current);
}

//----------------------------------------------------------------------------------------

void handle_buzzer() {
  if (buzzerVal == 1) {
    analogWrite(buzzerPin, 255);
    buzzerVal = 0;
    Serial.println("BEEP");
  } else {
    analogWrite(buzzerPin, 0);
    timeToSleep = true;
  }
}

void loop() {
  
  //----------------------------------------------------------------------------------------
  if ((unsigned long)(millis()-time1) > 100)
  {
    handle_buzzer();
    time1 = millis();
  }
  
  //----------------------------------------------------------------------------------------
  if (timeToSleep) {
    for (byte i = 0; i <1; i++) {
      myWatchdogEnable(0b100000);
    }

    handle_current();
  
    delay(200);
    go_to_sleep();
  }
}

