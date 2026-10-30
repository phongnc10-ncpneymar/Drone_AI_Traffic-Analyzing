/*
  BAI 1: KIEM TRA GIAO TIEP I2C VOI MPU6050
  ------------------------------------------
  Muc tieu:
   - Kiem tra ESP32 co nhan dien duoc MPU6050 tren bus I2C khong (quet dia chi).
   - Doc du lieu tho (raw) tu cam bien va in ra Serial Monitor.

  Phan cung:
   SDA = GPIO23
   SCL = GPIO18
   MPU6050: VCC=3.3V, GND=GND, SDA->GPIO23, SCL->GPIO18

  Cach test:
   1. Nap code, mo Serial Monitor (115200 baud).
   2. Buoc 1 se in ra dia chi I2C tim duoc (phai thay 0x68).
   3. Sau do chuong trinh lien tuc in Acc/Gyro tho. Nghieng board de xem so thay doi.
*/

#include <Wire.h>

#define SDA_PIN 23
#define SCL_PIN 18
#define MPU_ADDR 0x68

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// ----------------------------------------------------------------
// Buoc 1: Quet toan bo dia chi I2C tren bus de kiem tra day noi
// ----------------------------------------------------------------
void scanI2C() {
  Serial.println("Dang quet I2C...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  -> Tim thay thiet bi tai dia chi 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("  !! KHONG tim thay thiet bi nao. Kiem tra lai day SDA/SCL, nguon, dien tro pull-up.");
  } else {
    Serial.printf("Da tim thay %d thiet bi.\n", count);
  }
}

// ----------------------------------------------------------------
// Buoc 2: Danh thuc MPU6050 (thoat che do sleep)
// ----------------------------------------------------------------
void wakeMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // dia chi thanh ghi PWR_MGMT_1
  Wire.write(0x00);   // ghi 0 -> danh thuc cam bien
  byte err = Wire.endTransmission(true);
  if (err == 0) {
    Serial.println("Da danh thuc MPU6050 thanh cong.");
  } else {
    Serial.printf("Loi khi ghi PWR_MGMT_1, ma loi I2C = %d\n", err);
  }
}

// ----------------------------------------------------------------
// Buoc 3: Doc 14 byte du lieu tho (Accel 6 byte, Temp 2 byte, Gyro 6 byte)
// ----------------------------------------------------------------
void readMPU6050raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);               // thanh ghi dau: ACCEL_XOUT_H
  byte err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.printf("Loi beginTransmission khi doc: %d\n", err);
    return;
  }

  uint8_t n = Wire.requestFrom(MPU_ADDR, 14, true);
  if (n != 14) {
    Serial.printf("Canh bao: chi nhan duoc %d/14 byte\n", n);
    return;
  }

  accX  = Wire.read() << 8 | Wire.read();
  accY  = Wire.read() << 8 | Wire.read();
  accZ  = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();        // 2 byte nhiet do, bo qua
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BAI 1: TEST I2C + MPU6050 ===");

  Wire.begin(SDA_PIN, SCL_PIN);   // chi dinh chan I2C tuy chinh cho ESP32
  Wire.setClock(100000);         // 100kHz cho on dinh khi test

  scanI2C();
  wakeMPU6050();
}

void loop() {
  readMPU6050raw();
  Serial.printf("Acc[X=%6d Y=%6d Z=%6d]  Gyro[X=%6d Y=%6d Z=%6d]\n",
                accX, accY, accZ, gyroX, gyroY, gyroZ);
  delay(300);
}
