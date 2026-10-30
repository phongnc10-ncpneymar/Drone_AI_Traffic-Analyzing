/*
  BAI 2: KIEM TRA PWM TREN 1 CHAN DONG CO (FL = GPIO14)
  ------------------------------------------------------
  Muc tieu:
   - Lam quen voi LEDC (bo tao PWM phan cung cua ESP32).
   - Tang/giam toc do dong co FL tu 0 -> max -> 0 de kiem tra day noi,
     driver/MOSFET va dong co truoc khi dong thoi dieu khien 4 dong co.

  Phan cung:
   Dong co FL noi qua driver/MOSFET toi GPIO14.
   LUU Y AN TOAN: KHONG noi dong co truc tiep vao chan GPIO.
   Phai qua mach driver (MOSFET/ESC) chiu duoc dong cua dong co.

  Cach test:
   1. Thao canh quat (propeller) ra truoc khi test, tranh tai nan.
   2. Nap code, mo Serial Monitor 115200 de theo doi gia tri PWM dang gui.
   3. Dong co se tang dan toc do, giu may giay, roi giam dan ve 0, lap lai.
*/

#define MOTOR_FL_PIN   14
#define PWM_CH_FL      0
#define PWM_FREQ       5000   // Hz
#define PWM_RES_BITS   8      // 8-bit -> gia tri 0..255

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 2: TEST PWM MOT DONG CO (FL - GPIO14) ===");
  Serial.println(">>> DA THAO CANH QUAT CHUA? Nhan Enter tren Serial de xac nhan an toan.");

  ledcSetup(PWM_CH_FL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(MOTOR_FL_PIN, PWM_CH_FL);
  ledcWrite(PWM_CH_FL, 0);   // dam bao dong co dung hoan toan luc khoi dong
}

// Tang dan toc do tu 'from' den 'to', buoc 'step', moi buoc cach nhau 'delayMs'
void rampSpeed(int from, int to, int step, int delayMs) {
  if (from <= to) {
    for (int v = from; v <= to; v += step) {
      ledcWrite(PWM_CH_FL, v);
      Serial.printf("PWM FL = %d / 255\n", v);
      delay(delayMs);
    }
  } else {
    for (int v = from; v >= to; v -= step) {
      ledcWrite(PWM_CH_FL, v);
      Serial.printf("PWM FL = %d / 255\n", v);
      delay(delayMs);
    }
  }
}

void loop() {
  Serial.println("--- Tang toc 0 -> 150 ---");
  rampSpeed(0, 150, 10, 150);

  Serial.println("--- Giu toc do 150 trong 2 giay ---");
  delay(2000);

  Serial.println("--- Giam toc 150 -> 0 ---");
  rampSpeed(150, 0, 10, 150);

  Serial.println("--- Nghi 2 giay truoc khi lap lai ---\n");
  delay(2000);
}
