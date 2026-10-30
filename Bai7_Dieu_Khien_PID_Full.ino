/*
  BAI 7: BO DIEU KHIEN BAY DAY DU (ARM/DISARM + CAN BANG GOC BANG PID)
  ------------------------------------------------------------------------
  Day la ban "day du" hon Bai 6: ngoai doc cam bien + dieu khien tung dong
  co, bai nay THEM:
   1. Loc goc nghieng tu MPU6050 bang BO LOC BU (complementary filter)
      ket hop Accelerometer + Gyroscope (gyro troi theo thoi gian, accel
      on dinh nhung on - ket hop ca hai cho ra goc muot va chinh xac hon).
   2. Vong lap PID giu goc Roll/Pitch ve 0 (tu can bang) chay dinh ky,
      tach rieng khoi toc do doc cam bien.
   3. CO CHE ARM/DISARM va TIMEOUT AN TOAN: dong co CHI quay khi da ARM,
      va se tu DISARM neu qua 1 giay khong nhan lenh tu Serial (mat ket noi).
   4. Hieu chuan (CALIBRATE) gyro luc khoi dong: dung yen board de tinh
      offset, tranh "troi" gia tri khi dung yen.

  Phan cung:
   SDA=GPIO23  SCL=GPIO18
   FL=GPIO14  FR=GPIO15  BL=GPIO12  BR=GPIO13
   (Xem file NOI_DAY.md de biet so do noi day chi tiet tu PIN - driver -
    dong co - MPU6050.)

  Lenh Serial (go lien tiep, khong can Enter):
   'c' = calibrate gyro (giu yen board, lam 1 lan sau khi nap code)
   'r' = ARM (cho phep dong co quay)      'f' = DISARM (dung het, an toan)
   'w'/'s' = tang/giam throttle nen
   'a'/'d' = lenh nghieng trai/phai (target roll)
   'i'/'k' = lenh nghieng truoc/sau (target pitch)
   ' '     = ESTOP (disarm + throttle=0 ngay lap tuc)
*/

#include <Wire.h>

// ---------------- Chan phan cung ----------------
#define SDA_PIN 23
#define SCL_PIN 18
#define MPU_ADDR 0x68

#define MOTOR_FL 14
#define MOTOR_FR 15
#define MOTOR_BL 12
#define MOTOR_BR 13

#define CH_FL 0
#define CH_FR 1
#define CH_BL 2
#define CH_BR 3

#define PWM_FREQ 5000
#define PWM_RES  8        // 0..255

// ---------------- Trang thai bay ----------------
bool armed = false;
int  throttle = 0;                 // 0..200
const int THROTTLE_MAX = 200;
const int THROTTLE_STEP = 10;

float targetRoll  = 0;   // goc muc tieu nguoi dieu khien yeu cau (do)
float targetPitch = 0;
const float TARGET_STEP = 5.0;     // moi lan nhan a/d/i/k doi bao nhieu do
const float TARGET_LIMIT = 25.0;   // gioi han goc an toan khi test ban dau

unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;  // qua 1s khong co lenh -> tu disarm

// ---------------- Cam bien / bo loc ----------------
int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float gyroOffX = 0, gyroOffY = 0, gyroOffZ = 0;

float roll = 0, pitch = 0;         // goc hien tai sau loc (do)
unsigned long lastSensorTime = 0;

const float ACC_SENSITIVITY  = 16384.0; // LSB/g  (thang +-2g, mac dinh)
const float GYRO_SENSITIVITY = 131.0;   // LSB/(deg/s) (thang +-250 deg/s, mac dinh)
const float COMP_ALPHA = 0.98;          // trong so gyro trong bo loc bu

// ---------------- PID can bang goc ----------------
float Kp = 2.2, Ki = 0.0, Kd = 0.8;
float rollErrIntegral = 0, pitchErrIntegral = 0;
float rollErrPrev = 0, pitchErrPrev = 0;
const float PID_OUT_LIMIT = 60;   // gioi han bien do PID cong vao throttle

unsigned long lastPidTime = 0;
const unsigned long PID_PERIOD_MS = 10;   // vong PID ~100Hz

unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_PERIOD_MS = 200;

// ===================================================================
// MPU6050
// ===================================================================
void setupMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);           // danh thuc
  Wire.endTransmission(true);
}

void readMPU6050raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();    // nhiet do, bo qua
  rawGx = Wire.read() << 8 | Wire.read();
  rawGy = Wire.read() << 8 | Wire.read();
  rawGz = Wire.read() << 8 | Wire.read();
}

// Hieu chuan: lay trung binh 500 mau luc dung yen de tinh offset gyro.
// Khong hieu chuan accelerometer trong bai nay (chi can offset gyro la du
// de demo bo loc bu on dinh).
void calibrateGyro() {
  Serial.println("Dang hieu chuan gyro - GIU YEN BOARD, khong rung/chay...");
  long sumX = 0, sumY = 0, sumZ = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    readMPU6050raw();
    sumX += rawGx; sumY += rawGy; sumZ += rawGz;
    delay(3);
  }
  gyroOffX = sumX / (float)N;
  gyroOffY = sumY / (float)N;
  gyroOffZ = sumZ / (float)N;
  Serial.printf("Hieu chuan xong. Offset Gyro X=%.1f Y=%.1f Z=%.1f\n",
                gyroOffX, gyroOffY, gyroOffZ);
}

// Bo loc bu: ket hop goc tinh tu accelerometer (on dinh, nhung co nhieu khi
// rung) va goc tich luy tu gyroscope (muot nhung troi dan theo thoi gian).
void updateAttitude() {
  unsigned long now = micros();
  float dt = (lastSensorTime == 0) ? 0.01 : (now - lastSensorTime) / 1000000.0;
  lastSensorTime = now;

  readMPU6050raw();

  float accRoll  = atan2((float)rawAy, (float)rawAz) * 180.0 / PI;
  float accPitch = atan2(-(float)rawAx,
                          sqrt((float)rawAy * rawAy + (float)rawAz * rawAz)) * 180.0 / PI;

  float gyroRateX = (rawGx - gyroOffX) / GYRO_SENSITIVITY;  // deg/s quanh truc X -> roll
  float gyroRateY = (rawGy - gyroOffY) / GYRO_SENSITIVITY;  // deg/s quanh truc Y -> pitch

  roll  = COMP_ALPHA * (roll  + gyroRateX * dt) + (1 - COMP_ALPHA) * accRoll;
  pitch = COMP_ALPHA * (pitch + gyroRateY * dt) + (1 - COMP_ALPHA) * accPitch;
}

// ===================================================================
// Dong co
// ===================================================================
void setupMotors() {
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FL, CH_FL);
  ledcSetup(CH_FR, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_FR, CH_FR);
  ledcSetup(CH_BL, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BL, CH_BL);
  ledcSetup(CH_BR, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_BR, CH_BR);
  ledcWrite(CH_FL, 0); ledcWrite(CH_FR, 0); ledcWrite(CH_BL, 0); ledcWrite(CH_BR, 0);
}

void writeMotors(float fl, float fr, float bl, float br) {
  ledcWrite(CH_FL, constrain((int)fl, 0, 255));
  ledcWrite(CH_FR, constrain((int)fr, 0, 255));
  ledcWrite(CH_BL, constrain((int)bl, 0, 255));
  ledcWrite(CH_BR, constrain((int)br, 0, 255));
}

void stopAllMotors() {
  writeMotors(0, 0, 0, 0);
}

// ===================================================================
// PID + mixer: tinh ngo ra PID tu sai so (target - goc hien tai), roi
// phoi (mix) vao 4 dong co theo cau hinh X-quad.
// ===================================================================
void runPidAndMix() {
  unsigned long now = millis();
  if (now - lastPidTime < PID_PERIOD_MS) return;
  float dt = (now - lastPidTime) / 1000.0;
  lastPidTime = now;

  float rollErr  = targetRoll  - roll;
  float pitchErr = targetPitch - pitch;

  rollErrIntegral  += rollErr  * dt;
  pitchErrIntegral += pitchErr * dt;
  // Chong "integral windup": gioi han phan tich luy
  rollErrIntegral  = constrain(rollErrIntegral,  -100, 100);
  pitchErrIntegral = constrain(pitchErrIntegral, -100, 100);

  float rollDeriv  = (rollErr  - rollErrPrev)  / dt;
  float pitchDeriv = (pitchErr - pitchErrPrev) / dt;
  rollErrPrev  = rollErr;
  pitchErrPrev = pitchErr;

  float rollOut  = Kp * rollErr  + Ki * rollErrIntegral  + Kd * rollDeriv;
  float pitchOut = Kp * pitchErr + Ki * pitchErrIntegral + Kd * pitchDeriv;
  rollOut  = constrain(rollOut,  -PID_OUT_LIMIT, PID_OUT_LIMIT);
  pitchOut = constrain(pitchOut, -PID_OUT_LIMIT, PID_OUT_LIMIT);

  if (!armed) {
    stopAllMotors();
    return;
  }

  // Mix kieu X-quad: moi dong co = throttle +- rollOut +- pitchOut
  // FL: giam khi roll duong (nghieng phai) va khi pitch duong (cui truoc)
  float fl = throttle - rollOut - pitchOut;
  float fr = throttle + rollOut - pitchOut;
  float bl = throttle - rollOut + pitchOut;
  float br = throttle + rollOut + pitchOut;

  writeMotors(fl, fr, bl, br);
}

// ===================================================================
// Xu ly lenh tu Serial
// ===================================================================
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  lastCommandTime = millis();   // bat ky lenh hop le nao cung reset timeout

  switch (c) {
    case 'c':
      calibrateGyro();
      break;
    case 'r':
      armed = true;
      Serial.println(">> DA ARM - dong co san sang quay theo throttle");
      break;
    case 'f':
      armed = false;
      stopAllMotors();
      Serial.println(">> DA DISARM");
      break;
    case 'w':
      throttle = min(throttle + THROTTLE_STEP, THROTTLE_MAX);
      Serial.printf(">> Throttle = %d\n", throttle);
      break;
    case 's':
      throttle = max(throttle - THROTTLE_STEP, 0);
      Serial.printf(">> Throttle = %d\n", throttle);
      break;
    case 'a':
      targetRoll = constrain(targetRoll - TARGET_STEP, -TARGET_LIMIT, TARGET_LIMIT);
      Serial.printf(">> targetRoll = %.1f\n", targetRoll);
      break;
    case 'd':
      targetRoll = constrain(targetRoll + TARGET_STEP, -TARGET_LIMIT, TARGET_LIMIT);
      Serial.printf(">> targetRoll = %.1f\n", targetRoll);
      break;
    case 'i':
      targetPitch = constrain(targetPitch - TARGET_STEP, -TARGET_LIMIT, TARGET_LIMIT);
      Serial.printf(">> targetPitch = %.1f\n", targetPitch);
      break;
    case 'k':
      targetPitch = constrain(targetPitch + TARGET_STEP, -TARGET_LIMIT, TARGET_LIMIT);
      Serial.printf(">> targetPitch = %.1f\n", targetPitch);
      break;
    case ' ':
      armed = false;
      throttle = 0;
      targetRoll = 0;
      targetPitch = 0;
      stopAllMotors();
      Serial.println("!! ESTOP - DA DISARM VA DUNG HET DONG CO !!");
      break;
    default:
      break;   // ky tu xuong dong, khoang trang khac... bo qua
  }
}

// An toan: neu qua COMMAND_TIMEOUT_MS khong nhan duoc lenh nao (vd mat ket
// noi Serial/USB), tu dong DISARM de tranh dong co quay khong kiem soat.
void checkFailsafe() {
  if (armed && (millis() - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    armed = false;
    stopAllMotors();
    Serial.println("!! FAILSAFE: mat lenh dieu khien qua 1s - DA TU DISARM !!");
  }
}

void printTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetry < TELEMETRY_PERIOD_MS) return;
  lastTelemetry = now;
  Serial.printf("[%s] Throttle=%3d Roll=%6.1f(target %5.1f) Pitch=%6.1f(target %5.1f)\n",
                armed ? "ARMED   " : "DISARMED",
                throttle, roll, targetRoll, pitch, targetPitch);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 7: BO DIEU KHIEN BAY DAY DU (PID + ARM/DISARM) ===");
  Serial.println("Lenh: c=calibrate r=ARM f=DISARM w/s=throttle a/d=roll i/k=pitch space=ESTOP");
  Serial.println("Buoc dau tien: go 'c' va giu yen board de hieu chuan gyro.");

  setupMPU6050();
  setupMotors();
  lastCommandTime = millis();   // tranh failsafe kich hoat ngay luc vua boot
}

void loop() {
  handleSerial();
  updateAttitude();     // doc cam bien + loc goc, chay moi vong loop
  runPidAndMix();        // PID + xuat PWM ra 4 dong co, tu gioi han theo PID_PERIOD_MS
  checkFailsafe();
  printTelemetry();
}
