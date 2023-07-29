#include <Wire.h>

#define WAIT 10000

void PSGWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x10);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Wire.begin(4, 5);
  Wire.setClock(400000);

}

void loop() {
  // put your main code here, to run repeatedly:

  PSGWrite(0x0f, 0x00);
  PSGWrite(0x06, 0x03);
  PSGWrite(0x07, 0x37);
  PSGWrite(0x08, 0x10);
  PSGWrite(0x0B, 0x00);
  PSGWrite(0x0C, 0x40);
  PSGWrite(0x0D, 0x0E);

  delay(WAIT);

  PSGWrite(0x0f, 0x00);
  PSGWrite(0x06, 0x1f);
  PSGWrite(0x07, 0x37);
  PSGWrite(0x08, 0x10);
  PSGWrite(0x0B, 0x00);
  PSGWrite(0x0C, 0x04);
  PSGWrite(0x0D, 0x0C);

  delay(WAIT);

  PSGWrite(0x0f, 0x00);
  PSGWrite(0x00, 0xff);
  PSGWrite(0x01, 0x0f);
  PSGWrite(0x06, 0x1e);
  PSGWrite(0x07, 0x36);
  PSGWrite(0x08, 0x10);
  PSGWrite(0x0B, 0x00);
  PSGWrite(0x0C, 0x40);
  PSGWrite(0x0D, 0x00);

  delay(WAIT);

}