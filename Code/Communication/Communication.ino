/*
 CUBEX V1
 By: Arko
 
 FUSE SETTINGS: EXTENDED 0xFD
                HIGH     0xD8
                LOW      0xE2  (Internal 8Mhz osc)
 
 AVA 70cms Tracker
 
 By Anthony Stirk M0UPU 
 
 October 2012 Version 3
 Subversion 3.35 FLIGHT READY
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Thanks to : 
 http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#define F_CPU (8000000)

#include "config.h"
#include "radio_si446x.h"
#include "rtty.h"
#include "ssdv.h"

#include <SoftwareSerial.h>  

//#define POWERSAVING      // Comment out to turn power saving off

uint8_t buf[60]; 
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int navmode = 0, battv=0, rawbattv=0, GPSerror = 0, lat_int=0,lon_int=0,txnulls=10;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0, battvaverage=0;
int psm_status = 0, radiostatus=0, countreset=0, aprs_attempts=0, aprs_tx_status=0;
unsigned long aprs_startTime;
int32_t tslf=0;
int errorstatus=0; 
/* Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = RFM22B Error Condition Noted, RFM22B Power Cycled
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 */



void setup() {
  
  //Setup
  pinMode(STATUS_LED, OUTPUT); 

  pinMode(SI446x_GPIO_PIN, OUTPUT);
  digitalWrite(SI446x_GPIO_PIN, LOW);
  
  // THE FOLLOWING PINS HAVE NOT YET BEEN CONFIGURED!!
    // START
    //pinMode(CAM_TX_PIN, INPUT); 
    //pinMode(CAM_RX_PIN, INPUT);
    pinMode(VBATT_PIN, INPUT);
    pinMode(VPANEL_PIN, INPUT);
    pinMode(DIO1_PIN, INPUT);
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    //END
 
  // Start the radio
  pinMode(SI446x_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SI446x_SHUTDOWN_PIN, LOW);

  radioStartup();
  ptt_on();

  digitalWrite(SI446x_GPIO_PIN, HIGH);
  blinkled(2);
  //initialise_interrupt();  // This is the old TIMER0 interrupt
  
#ifdef POWERSAVING
  ADCSRA = 0;
#endif
  
  rtx_init();    // Setup RTTY/Interrupt
}

void loop()
{
  delay(10);
  
  txstringlength=strlen(txstring);
  rtx_string(txstring);
  count++;
}    


void prepare_data() {
  
}

void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(STATUS_LED,HIGH);
    wait(100);
    digitalWrite(STATUS_LED,LOW);
    wait(100);
  }    
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}































