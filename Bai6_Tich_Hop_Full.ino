/*
  BAI 6: TICH HOP DAY DU - MPU6050 + 4 DONG CO QUA SERIAL
  -----------------------------------------------------------
  Muc tieu:
   - Ghep lai TOAN BO cac bai truoc thanh mot chuong trinh hoan chinh:
       Bai 1 (I2C/MPU6050) + Bai 2-3 (PWM tung dong co) + Bai 4 (throttle)
       + Bai 5 (roll/pitch/yaw) deu co trong file nay.
   - In lien tuc du lieu MPU6050 de nguoi dung quan sat khi dieu khien
     bang Serial, giup lien he giua lenh gui va phan hoi cam bien.

  Phan cung:
   SDA=GPIO23  SCL=GPIO18
   FL=GPIO14  FR=GPIO15  BL=GPIO12  BR=GPIO13

  Phim dieu khien tren Serial Monitor (khong can Enter, bat "No line ending"
  hoac go lien tiep deu duoc vi code doc tung ky tu):
   'w'/'s'  : tang/giam throttle nen
   'a'/'d'  : roll trai/phai (tuc thoi, 1 lan nhan 1 lan nghieng)
   'i'/'k'  : pitch truoc/sau
   'q'/'e'  : yaw CCW/CW
   ' '      : dung khan cap (throttle = 0)
   'h'      : tro ve hover (khong lech, giu throttle hien tai)
*/

#include <Wire.h>

// ---------- Chan I2C ----------
#define SDA_PIN 23
#define SCL_PIN 18
#define MPU_ADDR 0x68

// ---------- Chan dong co ----------
#define MOTOR_FL 14
#define MOTOR_FR 15
#define MOTOR_BL 12
#define MOTOR_BR 13

#define CH_FL 0
#define CH_FR 1
#define CH_BL 2
#define CH_BR 3

#define PWM_FREQ 5000
#define PWM_RES  8

int   throttle = 0;              // 0..200, toc do nen hien tai
const int THROTTLE_STEP = 10;
const int THROTTLE_MAX  = 200;
const int DELTA = 40;            // do lech cho roll/pitch/yaw

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 300; // ms

// ===================================================================
// MPU6050
// ===================================================================
void setupMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX  = Wire.read() << 8 | Wire.read();
  accY  = Wire.read() << 8 | Wire.read();
  accZ  = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// ===================================================================
// Dong co - lop dieu khien tung chan
// ===================================================================
void setupMotor(int pin, int ch) {
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

void setupAllMotors() {
  setupMotor(MOTOR_FL, CH_FL);
  setupMotor(MOTOR_FR, CH_FR);
  setupMotor(MOTOR_BL, CH_BL);
  setupMotor(MOTOR_BR, CH_BR);
}

void writeMotor(int ch, int speed) {
  ledcWrite(ch, constrain(speed, 0, 255));
}

// ===================================================================
// Chuc nang bay - ghep tu lop dieu khien tung chan
// ===================================================================
void applyHover() {
  writeMotor(CH_FL, throttle);
  writeMotor(CH_FR, throttle);
  writeMotor(CH_BL, throttle);
  writeMotor(CH_BR, throttle);
}

void applyRollLeft() {
  writeMotor(CH_FL, throttle - DELTA);
  writeMotor(CH_BL, throttle - DELTA);
  writeMotor(CH_FR, throttle);
  writeMotor(CH_BR, throttle);
}

void applyRollRight() {
  writeMotor(CH_FR, throttle - DELTA);
  writeMotor(CH_BR, throttle - DELTA);
  writeMotor(CH_FL, throttle);
  writeMotor(CH_BL, throttle);
}

void applyPitchForward() {
  writeMotor(CH_FL, throttle - DELTA);
  writeMotor(CH_FR, throttle - DELTA);
  writeMotor(CH_BL, throttle);
  writeMotor(CH_BR, throttle);
}

void applyPitchBackward() {
  writeMotor(CH_BL, throttle - DELTA);
  writeMotor(CH_BR, throttle - DELTA);
  writeMotor(CH_FL, throttle);
  writeMotor(CH_FR, throttle);
}

void applyYawCW() {
  writeMotor(CH_FL, throttle + DELTA);
  writeMotor(CH_BR, throttle + DELTA);
  writeMotor(CH_FR, throttle - DELTA);
  writeMotor(CH_BL, throttle - DELTA);
}

void applyYawCCW() {
  writeMotor(CH_FR, throttle + DELTA);
  writeMotor(CH_BL, throttle + DELTA);
  writeMotor(CH_FL, throttle - DELTA);
  writeMotor(CH_BR, throttle - DELTA);
}

void emergencyStop() {
  throttle = 0;
  writeMotor(CH_FL, 0);
  writeMotor(CH_FR, 0);
  writeMotor(CH_BL, 0);
  writeMotor(CH_BR, 0);
}

// ===================================================================
// Xu ly phim nhan tu Serial
// ===================================================================
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 'w':
      throttle = min(throttle + THROTTLE_STEP, THROTTLE_MAX);
      applyHover();
      Serial.printf(">> Throttle = %d\n", throttle);
      break;
    case 's':
      throttle = max(throttle - THROTTLE_STEP, 0);
      applyHover();
      Serial.printf(">> Throttle = %d\n", throttle);
      break;
    case 'h':
      applyHover();
      Serial.println(">> HOVER");
      break;
    case 'a': applyRollLeft();      Serial.println(">> ROLL TRAI");    break;
    case 'd': applyRollRight();     Serial.println(">> ROLL PHAI");    break;
    case 'i': applyPitchForward();  Serial.println(">> PITCH TRUOC");  break;
    case 'k': applyPitchBackward(); Serial.println(">> PITCH SAU");    break;
    case 'q': applyYawCCW();        Serial.println(">> YAW CCW");      break;
    case 'e': applyYawCW();         Serial.println(">> YAW CW");       break;
    case ' ':
      emergencyStop();
      Serial.println("!! DUNG KHAN CAP !!");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 6: TICH HOP MPU6050 + 4 DONG CO ===");
  Serial.println("w/s throttle | a/d roll | i/k pitch | q/e yaw | h hover | space STOP");

  setupMPU6050();
  setupAllMotors();
}

void loop() {
  handleSerial();

  // In du lieu cam bien dinh ky de doi chieu voi lenh dieu khien
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;
    readMPU6050();
    Serial.printf("[Throttle=%3d] Acc[%6d %6d %6d]  Gyro[%6d %6d %6d]\n",
                  throttle, accX, accY, accZ, gyroX, gyroY, gyroZ);
  }
}
