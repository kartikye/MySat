/**
  * Loop Actions
  * 
  * After the software started, it will remain in a loop doing folloing steps:
  *     - Transmitting 30 single RTTY Beeps every 5 seconds        150 seconds
  *     - Switch on GPS
  *     - Transmitting Double RTTY Beeps every 5 seconds until    ~ 15 seconds
  *       GPS locks but 30 Beeps max.
  *     - Switch off
  *     - Triple RTTY Beep one time                                  5 seconds
  *     - Transmitting one packet of acquired data                  10 seconds
  *                                                              = 180 seconds for one cycle
  * --------------------------------------------------------------------------------
  * @file Communication.ino
  * @author Kartikye Mittal
  * 
  * Some other authors created parts of the code before.
  * @author Anthony Stirk   Interrupt functions
  * @author Thomas Krahn    First version, Interrupt functions
  */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <math.h>

#include "Si446x.h"

//Tracker Configuration
#define CALLSIGN "D-2"                  //Callsign
#define SEC_CALLSIGN "AF5LI"            //Secondary Callsign which is sent but not included in the package
#define ASCII 7                         //ASCII 7 or 8
#define STOPBITS 2                      //Either 1 or 2
#define TXDELAY 25                      //Transmit-Delay in bits
#define RTTY_BAUD 50                    //Baud rate (600 max)
#define RADIO_FREQUENCY 145.300         //Transmit frequency in MHz (119 - 1050 Mhz)
#define RTTY_SHIFT 440                  //RTTY shift in Hz (varies, differs also on 2m and 70cm)
                                        //490 = 450 Hz @ 434.500 MHz
                                        //440 = 425 Hz @ 145.300 MHz
										
//PCB Configuration
#define RADIO_PIN 10                    //CS pin that defines the SPI slave
#define RADIO_SDN RADIO_SDN_PIN         //Pin to power off the transmitter

#define STATUS_LED 13                   //Status LED (Green)

#define RADIO_POWER 20                  //6    0dBm  (1mW)
                                        //16   8dBm  (6mW)
                                        //20  10dBm  (10mW)
                                        //32  14dBm  (25mW)
                                        //40  17dBm  (50mW)
                                        //127 20dBm  (100mW max)								
										
//#define DEBUG                         //Debug mode (Status LED active when in Power Down Mode)
									
//Global Variables
Si446x radio(RADIO_PIN);                //Radio object
char txstring[150];                     //Transmitting buffer
volatile int txstringlength =  0;       //Transmitting buffer length
volatile int txstatus = 0;              //Current TX state

volatile char txc;                      //Current Char to be transmitted
volatile int txi;                       //
volatile int txj;                       //
volatile long count = 1;                //Incremental number of packets transmitted

/**
  * Setup function of the program. Initializes hardware components.
  * Will be called once at the beginning.
  */
void setup() {
  //Set Pin mode
  pinMode(STATUS_LED, OUTPUT);          //Status LED (Green)
  pinMode(RADIO_SDN, OUTPUT);           //Radio
  
  Serial.begin(9600);                   //Start Serial
    
  digitalWrite(RADIO_SDN, HIGH);        //Power on Radio
  setupRadio();                         //Setup radio
  
  setup_watchdog(8);                    //Setup watchdog (configure 4 sec interrupt)
  
  initialise_interrupt();               //Initialize interrupt
}

/**
  * This will turn off brown-out detection while
  * sleeping. Unfortunately this won't work in IDLE mode.
  * Relevant info about BOD disabling: datasheet p.44
  *
  * Procedure to disable the BOD:
  *
  * 1. BODSE and BODS must be set to 1
  * 2. Turn BODSE to 0
  * 3. BODS will automatically turn 0 after 4 cycles
  *
  * The catch is that we *must* go to sleep between 2
  * and 3, ie. just before BODS turns 0.
  */
void disable_bod_and_sleep() {
  unsigned char mcucr;

  cli();
  mcucr = MCUCR | (_BV(BODS) | _BV(BODSE));
  MCUCR = mcucr;
  MCUCR = mcucr & (~_BV(BODSE));
  sei();
  
  sleep_mode(); //Go to sleep
}

/**
  * Enter power saving mode while waiting. This function is
  * using the Power Down Sleep Mode. Only the watchdog interrupt
  * will wake it up.
  */
void power_save() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
  
  #ifdef DEBUG
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  #endif
  
  disable_bod_and_sleep(); //Go to sleep
  
  #ifdef DEBUG
  digitalWrite(STATUS_LED, HIGH);
  #endif
  
  sleep_disable(); //Resume after wake up
  power_all_enable();
}

/**
  * Set up watchdog timer. Defines a sleeping time interval.
  * Interval Modes: 0 = 16 ms
  *                 1 = 32 ms
  *                 2 = 64 ms
  *                 3 = 128 ms
  *                 4 = 250 ms
  *                 5 = 500 ms
  *                 6 = 1 sec
  *                 7 = 2 sec
  *                 8 = 4 sec
  *                 9 = 8 sec
  *
  * @param ii Interval Mode
  */
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  //Start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  //Set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}


/**
  * Watchdog Interrupt Service. This routine is executed when watchdog timed out.
  */
ISR(WDT_vect) {}

/**
  * Function which is called by the microcontroller first time, when setup is completed and
  * when this function is finished.
  */
void loop() {
  //Disable TX interrupt routine
  disable_tx_interrupt();
  
  //Single beep on radio
  for(int i=0; i<38; i++) {
    //Beep
    radio.ptt_on();
    radio.setHighTone();
    delay(20);
    radio.setLowTone();
    delay(80);
    radio.ptt_off();
    
    //Delay
    power_save();
  }
  //Triple beep on radio
  radio.ptt_on();
  radio.setHighTone();
  delay(20);
  radio.setLowTone();
  delay(80);
  radio.setHighTone();
  delay(80);
  radio.setLowTone();
  delay(80);
  radio.setHighTone();
  delay(80);
  radio.setLowTone();
  delay(80);
  radio.ptt_off();
  
  //Delay
  delay(3300);
  
  //Forming Transmission String
  sprintf(
    txstring,
    "Hello"
  );
  txstringlength = strlen(txstring);
  
  //Transmitting data by interrupt function
  radio.ptt_on();
  txj = 0;
  txstatus = 6;
  
  //Enable TX interrupt routine
  enable_tx_interrupt();
  
  //Set sleep mode to Ilde
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  
  //Wait until data is sent by TX interrupt routine
  while(txstatus != 0)
    sleep_mode();
	
  sleep_disable();
  disable_tx_interrupt();
  power_save();
}


/**
  * This is the transmitting interrupt routine. It is called 50 times
  * a second, when 50 baud Transmitting rate is set.
  * The state of the function is defined by the global variable txstatus.
  * In general this variable is set to 0. Zero defines the state, in
  * which no action is made (no transmission). The transmission is
  * initialized with setting the global variable to 6. When transmission
  * is finished, it will automatically fall back into state 0.
  * This interrupt routine can be enabled and disabled by the functions
  * disable_tx_interrupt and enable_tx_interrupt. This is neccessary
  * due to Power Down Sleeping Mode.
  */
ISR(TIMER1_COMPA_vect) {
  switch(txstatus) {      
    case 6: //TX-delay
      txj++;
      if(txj > TXDELAY) { 
        txj = 0;
        txstatus = 7;
      }
      break;
    
    case 7: //Transmit a single char
      if(txj < txstringlength) {
        txc = txstring[txj]; //Select char
        txj++;
        radio.setLowTone(); //Start Bit (Synchronizing)
        txi = 0;
        txstatus = 8;
      } else {
        txj = 0;
        count++;
        txstatus = 0; //Finished to transmit char
      }
      break;
    
    case 8:
      if(txi < ASCII) {
        txi++;
        if(txc & 1) {
          radio.setHighTone();
        } else {
          radio.setLowTone();
        }
        txc = txc >> 1;
      } else {
        radio.setHighTone(); //Stop Bit
        txi = 0;
        txstatus = 9;
      }
      break;
    
    case 9:
      if(STOPBITS == 2)
        radio.setHighTone(); //Stop Bit
      txstatus = 7;
  }
}

/**
  * Initializes the radio and set its specific frequency, shift and
  * transmission power.
  */
void setupRadio() {
  radio.initSPI();
  radio.setFrequency(RADIO_FREQUENCY);
  radio.setShift(RTTY_SHIFT);
  radio.setPowerLevel(RADIO_POWER);
  radio.init();
}

/**
  * Initializes the interrupt routine for transmission of data.
  */
void initialise_interrupt() {
  //initialize Timer1
  cli(); //Disable global interrupts
  TCCR1A = 0; //Set entire TCCR1A register to 0
  TCCR1B = 0; //Same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1; //Set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12); //Turn on CTC mode:
  //Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  //Enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei(); //Enable global interrupts
}

/**
  * Disables the transmission interrupt.
  */
void disable_tx_interrupt() {
  cli();
  TIMSK1 &= ~(1 << OCIE1A);
  sei();
}

/**
  * Enables the transmission interrupt.
  */
void enable_tx_interrupt() {
  cli();
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

/**
  * Lets the Status LED (Green) blink.
  * @param blinks Number of blinks
  */
void blinkled(int blinks) {
  for(int i = 0; i <= blinks; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(300);
    digitalWrite(STATUS_LED, LOW);
    delay(300);
  }    
}
