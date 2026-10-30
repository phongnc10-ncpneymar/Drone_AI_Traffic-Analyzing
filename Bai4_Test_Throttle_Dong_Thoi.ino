/*
  BAI 4: DIEU KHIEN DONG THOI 4 DONG CO (THROTTLE)
  --------------------------------------------------
  Muc tieu:
   - Sau khi Bai 3 xac nhan tung dong co hoat dong dung, bai nay cho CA 4
     dong co chay CUNG TOC DO - mo phong dong tac "throttle" (tang/giam
     luc nay dong deu) cua drone thuc te.
   - Nhan ky tu tu Serial de dieu khien throttle thu cong:
       'w' = tang toc, 's' = giam toc, ' ' (space) = dung khan cap

  Phan cung:
   FL=GPIO14  FR=GPIO15  BL=GPIO12  BR=GPIO13
*/

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

int throttle = 0;              // toc do nen hien tai (0..255)
const int THROTTLE_STEP = 10;  // moi lan nhan w/s thay doi bao nhieu
const int THROTTLE_MAX  = 200; // gioi han an toan khi test ban dau

void setupMotor(int pin, int ch) {
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

// Ham dung chung: ghi CUNG mot gia tri toc do ra ca 4 dong co
void setAllMotors(int speed) {
  ledcWrite(CH_FL, speed);
  ledcWrite(CH_FR, speed);
  ledcWrite(CH_BL, speed);
  ledcWrite(CH_BR, speed);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 4: TEST THROTTLE DONG THOI ===");
  Serial.println("Phim dieu khien: 'w' tang toc | 's' giam toc | ' ' dung khan cap");

  setupMotor(MOTOR_FL, CH_FL);
  setupMotor(MOTOR_FR, CH_FR);
  setupMotor(MOTOR_BL, CH_BL);
  setupMotor(MOTOR_BR, CH_BR);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      throttle = min(throttle + THROTTLE_STEP, THROTTLE_MAX);
    } else if (c == 's') {
      throttle = max(throttle - THROTTLE_STEP, 0);
    } else if (c == ' ') {
      throttle = 0;
      Serial.println("!! DUNG KHAN CAP !!");
    }
    setAllMotors(throttle);
    Serial.printf("Throttle hien tai = %d / %d\n", throttle, THROTTLE_MAX);
  }
}
