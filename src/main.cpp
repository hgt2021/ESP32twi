#include "Arduino.h"

#define TWI_DATA 21
#define TWI_CLK 17
#define D_MAXDATABYTES 64
#define D_BITDELAY_MV 999

#define SoftRst6 0xBD
#define SoftRst7 0x01

#define TwiOff6 0x8D
#define TwiOff7 0x02

volatile int interruptCounter;
int fallingEgdeCount = 0;
bool CLK_On = false;
uint8_t state = 0;
 struct dataTwi {
   uint8_t dataBytes[D_MAXDATABYTES];
   int pos;
   int bitCount;
   bool repeat;
 } daten;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}
 
int getNextBit(dataTwi &d) {
  uint8_t bit = 0;
  if (d.pos >= d.bitCount) {
    return -1;
  }
  int posByte = d.pos / 8;
  int posBit = d.pos % 8;
  bit = ((d.dataBytes[posByte] << posBit & 0x80) ? 1 : 0);
 // Serial.printf("posByte: %d  posBit: %d  bit: %d\n", posByte, posBit, bit);
  d.pos++;
  return bit;
}

void SendNRZ(boolean bit) {
  boolean LastPortState = digitalRead(TWI_DATA);
  if (bit == HIGH) {
    // Serial.print("1");
    digitalWrite(TWI_DATA, !LastPortState);
    delayMicroseconds(D_BITDELAY_MV);
    digitalWrite(TWI_DATA, LastPortState);
    delayMicroseconds(D_BITDELAY_MV);
    //readAllButtonsMdelay(D_BITDELAY_MV);
  } else {
    // Serial.print("0");
    digitalWrite(TWI_DATA, !LastPortState);
    delayMicroseconds(D_BITDELAY_MV * 2);
    //readAllButtonsMdelay(D_BITDELAY_MV * 2);
  }
}

void setup() {
  for (int i = 0; i < D_MAXDATABYTES; i++) {
    daten.dataBytes[i] = 0x00;
  }
  
  // TWI RST
  // 4 * 0x00
  daten.dataBytes[4] = 0x8D; // TWI_RST
  // 1 * 0x00
  // Ende TWI RST
  // Soft RST 0xBD01,
  daten.dataBytes[6] = SoftRst6; // SoftRst --> Sleep
  daten.dataBytes[7] = SoftRst7; // SoftRst  --> Sleep
  // Ende Soft RST
  // gesamt bits 8x8 64
  
  daten.bitCount = 64;
  daten.pos = 0;
 

  Serial.begin(115200);
  pinMode(TWI_DATA, OUTPUT); 
  pinMode(TWI_CLK, OUTPUT); 
  digitalWrite(TWI_CLK, HIGH);
  digitalWrite(TWI_DATA, LOW);
  
  //timer = timerBegin(0, 8, true);  // 80 - 5000  = 100Hz  8 - 50
  //timerAttachInterrupt(timer, &onTimer, true);
  //timerAlarmWrite(timer, 1000000, true); // 1hz
  //timerAlarmWrite(timer, 250000, true); // 25 = 20 khz
  //timerAlarmWrite(timer, 250, true);

  //timerAlarmEnable(timer);  
  
  
  timer = timerBegin(0, 8, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 250, true);
  timerAlarmEnable(timer);
  
  CLK_On = false;
  //delay(1000);
//interruptCounter = 0;
//  timerAlarmWrite(timer, 250, true); // 25 = 20 khz
}
 
void sendData() {
  delay(1000);
  return;
    delayMicroseconds(15);// tDD warten
    for (int i = 0; i < 1000; i++)  {
      SendNRZ(HIGH);
      SendNRZ(LOW);
      //Serial.print(".");
    }
    digitalWrite(TWI_DATA, 0);
    digitalWrite(TWI_CLK, 1);
    delayMicroseconds(15);// tDD warten
}
void sendTwi(uint8_t onOff) {
    // aufwecken mit TWI_OFF
    if (onOff == 0) {
      daten.dataBytes[6] = TwiOff6; // send data
      daten.dataBytes[7] = TwiOff7; // send Data
    } else {
      daten.dataBytes[6] = SoftRst6; // SoftRst --> Sleep
      daten.dataBytes[7] = SoftRst7; // SoftRst  --> Sleep
    }
    daten.pos = 0;
    CLK_On = true;
}

void loop() {

  int datenBit = -1; // -1 = nix tun

  if (!CLK_On) {
    if (state == 0) {
      sendTwi(1); // sleep
      delay(1000);
      Serial.print("s");
      delay(1000);
      state = 1;
    } else if (state == 1) {
      //Serial.println("wakeup");
      //sendTwi(0); // wakeup for sending
      state = 2;
    } else if (state == 2) {
     // sendData();
      delay(1000);
      state = 0;
      //Serial.println("data out");
    }
  }
  if (interruptCounter > 1) {
    interruptCounter = 0;
  } else if (interruptCounter > 0) {
    //Serial.println(interruptCounter);
    portENTER_CRITICAL(&timerMux);
    //digitalWrite(TWI_DATA, 0);
    interruptCounter = 0;
    uint8_t clkstate = digitalRead(TWI_CLK);
    if (CLK_On) {
      if (clkstate == 1) {
        fallingEgdeCount++;
      }
     // Serial.print('i');
      digitalWrite(TWI_CLK, !clkstate);

    } else {
      //Serial.print('i');
      digitalWrite(TWI_CLK, 1);
    }
    portEXIT_CRITICAL(&timerMux);
    if (clkstate == 0  && CLK_On) {
      datenBit = getNextBit(daten);
      if (datenBit >= 0) {
        digitalWrite(TWI_DATA, datenBit);
        //Serial.print(datenBit);
      
      } else {
        digitalWrite(TWI_DATA, 0);
        digitalWrite(TWI_CLK, !clkstate);
        CLK_On = false;
        //Serial.println(fallingEgdeCount);
        fallingEgdeCount = 0;
        daten.pos = 0;
      }
    } 
  }
}