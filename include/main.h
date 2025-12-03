#ifndef __MAIN_H__
#define __MAIN_H__  
#include <SPI.h>
#include "DW1000Ranging.h"
#include <LoRa.h>
#include <EEPROM.h>



/**************** PINS Defination****************/
#define LORA_SS    15
#define LORA_RST   25
#define LORA_DIO0  26    

#define LORA_MISO  12
#define LORA_MOSI  13
#define LORA_SCLK  14

#define PIN_RST     27
#define PIN_IRQ     34
#define PIN_SS      4

/**************** FUNCTION PROTOTYPES ****************/
void newRange();
void newDevice(DW1000Device* device);
void newBlink(DW1000Device* device);
void inactiveDevice(DW1000Device* device);
void init_LoRa();
void lora_RX();
void lora_TX();
void DWM_init_Responder();
void Serial_ID_Input();


#endif /* __MAIN_H__ */