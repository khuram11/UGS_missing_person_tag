
#include "main.h"

#define EEPROM_SIZE 10
#define EEPROM_START_ADDR 0


SPIClass hspi(HSPI);

uint16_t MY_ID = 0x45;

String serialInput = ""; 
unsigned long lastLoraTime = 0;
unsigned long lastPromptTime = 0;

void eeprom_init() 
{
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM");
    return;
  }
  Serial.println("EEPROM initialised");
  if(EEPROM.read(EEPROM_START_ADDR) == 0xAA) 
  {
    MY_ID = EEPROM.readByte(EEPROM_START_ADDR + 1);
    Serial.print("Loaded Device ID from EEPROM: ");
    Serial.println(MY_ID);
  }

}

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  eeprom_init();
  DWM_init_Responder();
  init_LoRa();
}

void loop() 
{
  Serial_ID_Input();
  lora_TX();
  DW1000Ranging.loop();//working as responder
}


void DWM_init_Responder()
{
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  uint8_t dev_id[4];
  DW1000.readBytes(DEV_ID, NO_SUB, dev_id,4);
  Serial.print("[DWM] Device ID: ");
  Serial.printf("0x%02X%02X%02X%02X\n", dev_id[3], dev_id[2], dev_id[1], dev_id[0]);
  if (dev_id[3] != 0xDE || dev_id[2] != 0xCA || dev_id[1] != 0x01 || dev_id[0] != 0x30) 
  {
    Serial.println("[DWM] FAIL (wrong device ID)");
  }else Serial.println("[DWM] OK");

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  char address[8] = {0x00};
  address[0] = MY_ID; // '0' offset for ASCII
  address[1] = 0x00; 
  address[2] = 0xDE; 
  address[3] = 0xCA; 
  address[4] = 0x10; 
  address[5] = 0x11; 
  address[6] = 0x12; 
  address[7] = 0x13;
  DW1000Ranging.startAsAnchor(address,  DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
  Serial.println("DW1000 Anchor Started...");
}

void init_LoRa()
{
  delay(1000);

  /************** INIT LoRa **************/
  LoRa.setPins(LORA_SS, LORA_RST, -1);
  hspi.begin(LORA_SCLK, LORA_MISO, LORA_MOSI);
  LoRa.setSPI(hspi);

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init Failed!");
  } else {
    Serial.println("LoRa init OK");
  }
 
}
void lora_RX()
{
    /************** LORA RX LOOP **************/
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("[LoRa RX] ");

        // Print the received message
        while (LoRa.available()) {
            Serial.print((char)LoRa.read());
        }

        // Print RSSI of the received packet
        Serial.print("  | RSSI: ");
        Serial.println(LoRa.packetRssi());

        Serial.println();
    }
}


void lora_TX()
{
    /**************** LORA TX EVERY 500ms ****************/
  if (millis() - lastLoraTime > 500) {
    lastLoraTime = millis();

    LoRa.beginPacket();
    LoRa.print("ID:");
    LoRa.print(MY_ID);
    LoRa.endPacket();

    Serial.print("[LoRa] Sent -> ID:");
    Serial.println(MY_ID);
  }
}

void DWM_update_id(uint8_t id)
{
  Serial.println("Updating DW1000 ID...");
  char address[8] = {0x00};
  address[0] = id; // '0' offset for ASCII
  address[1] = 0x00; 
  address[2] = 0xDE; 
  address[3] = 0xCA; 
  address[4] = 0x10; 
  address[5] = 0x11; 
  address[6] = 0x12; 
  address[7] = 0x13;
  DW1000Ranging.startAsAnchor(address,  DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}


void Serial_ID_Input()
{
  /**************** PRINT PROMPT EVERY 1 SEC ****************/
  if (millis() - lastPromptTime > 1000) {
    lastPromptTime = millis();
    Serial.println("Enter device ID and press ENTER:");
  }
  /**************** SERIAL INPUT (UPDATE ID) ****************/
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialInput.length() > 0) {
        int newID = serialInput.toInt();

        if (newID >= 0 && newID <= 255) {
          MY_ID = newID;
          Serial.print("Updated Device ID to: ");
          Serial.println(MY_ID);
          Serial.println("Saving to EEPROM...");
          EEPROM.write(EEPROM_START_ADDR, 0xAA); // Validity byte 
          EEPROM.write(EEPROM_START_ADDR + 1, MY_ID);
          EEPROM.commit();
          Serial.println("Saved.");
          DWM_update_id(MY_ID);
        } else {
          Serial.println("Invalid ID! Enter 0–255 only.");
        }

        serialInput = "";
      }
    } 
    else {
      serialInput += c;
    }
  }

}


/**************** DW1000 CALLBACKS ****************/

void newRange() {
  Serial.print("from: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("  Range: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.print(" m   RX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
}

void newDevice(DW1000Device* device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("Inactive device removed: ");
  Serial.println(device->getShortAddress(), HEX);
}
void newBlink(DW1000Device* device) {
  Serial.print("blink; 1 device added -> short: ");
  Serial.println(device->getShortAddress(), HEX);
}
