#include "si4463.h"
#include "si4463_config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <math.h>
//API Codes

#define NOP 							0x00 
#define PART_INFO                       0x01
#define FUNC_INFO                       0x10
#define SET_PROPERTY                    0x11 
#define GET_PROPERTY                    0x12 
#define GPIO_PIN_CFG                    0x13
#define GET_ADC_READING                 0x14 
#define FIFO_INFO                       0x15
#define PACKET_INFO                     0x16
#define IRCAL                           0x17 
#define PROTOCOL_CFG                    0x18 
#define GET_INT_STATUS                  0x20
#define GET_PH_STATUS                   0x21
#define GET_MODEM_STATUS                0x22
#define GET_CHIP_STATUS                 0x23
#define START_TX                        0x31 
#define START_RX                        0x32 
#define REQUEST_DEVICE_STAT             0x33
#define CHANGE_STATE                    0x34 
#define READ_CMD_BUFF                   0x44 
#define FRR_A_READ                      0x50
#define FRR_B_READ                      0x51 
#define FRR_C_READ                      0x53 
#define FRR_D_READ                      0x57 
#define WRITE_TX_FIFO                   0x66 
#define READ_RX_FIFO                    0x77 
#define START_MFSK                      0x35 
#define RX_HOP                          0x36 

//config

#define CALLSIGN "D-2"                  //Callsign
#define SEC_CALLSIGN "AF5LI"            //Secondary Callsign which is sent but not included in the package
#define ASCII 7                         //ASCII 7 or 8
#define STOPBITS 2                      //Either 1 or 2
#define TXDELAY 25                      //Transmit-Delay in bits
#define RTTY_BAUD 50                    //Baud rate (600 max)
#define RADIO_FREQUENCY 435.300         //Transmit frequency in MHz (119 - 1050 Mhz)
#define RTTY_SHIFT 440                  //RTTY shift in Hz (varies, differs also on 2m and 70cm)
                                        //490 = 450 Hz @ 434.500 MHz
                                        //440 = 425 Hz @ 145.300 MHz
#define POWER_LEVEL 20

const unsigned char RF_MODEM_CLKGEN_BAND_1_data[] = 		{RF_MODEM_CLKGEN_BAND_1};
const unsigned char RF_FREQ_CONTROL_INTE_8_data[] = 		{RF_FREQ_CONTROL_INTE_8};
const unsigned char RF_POWER_UP_data[] = 			   		{ RF_POWER_UP};
const unsigned char RF_GPIO_PIN_CFG_data[] = 			   	{ RF_GPIO_PIN_CFG}; 
const unsigned char RF_GLOBAL_XO_TUNE_1_data[] = 		   	{ RF_GLOBAL_XO_TUNE_1};
const unsigned char RF_GLOBAL_CONFIG_1_data[] = 		   	{ RF_GLOBAL_CONFIG_1}; 
const unsigned char RF_FRR_CTL_A_MODE_4_data[] = 		   	{ RF_FRR_CTL_A_MODE_4};
const unsigned char RF_PREAMBLE_TX_LENGTH_9_data[] = 		{ RF_PREAMBLE_TX_LENGTH_9};
const unsigned char RF_SYNC_CONFIG_5_data[] = 		 	   	{ RF_SYNC_CONFIG_5};
const unsigned char RF_PKT_CRC_CONFIG_1_data[] = 		   	{ RF_PKT_CRC_CONFIG_1};
const unsigned char RF_PKT_CONFIG1_1_data[] = 			   	{ RF_PKT_CONFIG1_1};
const unsigned char RF_PKT_LEN_3_data[] = 			   		{ RF_PKT_LEN_3};
const unsigned char RF_PKT_FIELD_1_LENGTH_12_8_12_data[] =	{ RF_PKT_FIELD_1_LENGTH_12_8_12};
const unsigned char RF_PKT_FIELD_4_LENGTH_12_8_8_data[] = 	{ RF_PKT_FIELD_4_LENGTH_12_8_8};
const unsigned char RF_MODEM_FREQ_DEV_0_1_data[] = 		   	{ RF_MODEM_FREQ_DEV_0_1};
const unsigned char RF_MODEM_AGC_CONTROL_1_data[] = 		{ RF_MODEM_AGC_CONTROL_1};
const unsigned char RF_MATCH_VALUE_1_12_data[] =            { RF_MATCH_VALUE_1_12};
const unsigned char RF_MODEM_RSSI_COMP_1_data[] = 			{ RF_MODEM_RSSI_COMP_1};
const unsigned char RF_MODEM_MOD_TYPE_12_data[]=			{RF_MODEM_MOD_TYPE_12};
const unsigned char RF_MODEM_TX_RAMP_DELAY_8_data[]=				{RF_MODEM_TX_RAMP_DELAY_8};
const unsigned char RF_MODEM_BCR_OSR_1_9_data[]=					{RF_MODEM_BCR_OSR_1_9};
const unsigned char RF_MODEM_AFC_GEAR_7_data[]=						{RF_MODEM_AFC_GEAR_7};
const unsigned char RF_MODEM_AGC_WINDOW_SIZE_9_data[]=				{RF_MODEM_AGC_WINDOW_SIZE_9};
const unsigned char RF_MODEM_OOK_CNT1_11_data[]=					{RF_MODEM_OOK_CNT1_11};
const unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_data[]=	{RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12};
const unsigned char RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_data[]=	{RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12};
const unsigned char RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_data[]=	{RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12};
const unsigned char RF_SYNTH_PFDCP_CPFF_7_data[]=					{RF_SYNTH_PFDCP_CPFF_7};

const unsigned char tx_test_aa_data[14] = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};  
const unsigned char tx_ph_data[14] = {'s','w','w','x',0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x6d};  

//Pins

int SDI  = 12
int SDO  = 11
int SCK  = 13
int SS   = 10
 
int GPIO = 4
int SDN  = 7
int LED  = 13

void Setup(){
	
	pinMode(LED, OUTPUT);
	pinMode(SDN, OUTPUT);

	digitalWrite(SDN, HIGH);
	setupRadio();
}

void sendCommand(int tx, in rx, const char* data){
	digitalWrite(SS, LOW);
	char answer;

	for(int i = 0; i < tx; i++){
		byte d = data[i];
		SPI.transfer(d);
	}

	digitalWrite(SS, HIGH);

	delayMicroseconds(20);

	digitalWrite(SS, LOW);

	int reply = 0x00;

	while(reply != 0xFF){
		reply = SPI.transfer(0x44);
		if(reply != 0xFF){
			digitalWrite(SS, HIGH);
			delayMicroseconds(20);
			digitalWrite(SS, LOW);
		}
	}

	digitalWrite(SS, HIGH);
	delay(500);
}

void setupRadio(){
	initSPI();
	setFrequency(RADIO_FREQUENCY);
	setShift(RTTY_SHIFT);
	setPowerLevel(POWER_LEVEL);
	radioInit();
}

void initSPI(){
	SPI.begin();
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV2)
}

void setFrequency(unsigned long freq){
	
}

void setShift(){

}

void setPowerLevel(){

}

void radioInit(){
	
	pinMode(GPIO, OUTPUT);
	delay(100);

	digitalWrite(SDN, HIGH);
	delay(600);
	digitalWrite(SDN, LOW);

	//Return Part Info
	const char PART_INFO_COMMAND[] = {0x01};
	sendCommand(1, 9, PART_INFO_COMMAND);

	//Power Up
	sendCommand(7,1,RF_POWER_UP_data)

	//Get Interrupt status
	const char GET_INT_STATUS[] = {0x20, 0x00, 0x00, 0x00};
	sendCommand(4, 9, GET_INT_STATUS)

	//GPIO Config
	sendCommand(7, 6,RF_GPIO_PIN_CFG_data)

}

void startTX(){

}

void stopTX(){

}





