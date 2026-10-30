/*
  BAI 3: KIEM TRA RIENG LE 4 DONG CO (FL, FR, BL, BR)
  -----------------------------------------------------
  Muc tieu:
   - Mo rong Bai 2 ra ca 4 dong co, moi dong co mot kenh PWM (LEDC) rieng.
   - Chay tung dong co MOT LAN LUOT de xac dinh dung thu tu / dung chieu quay
     truoc khi test dong thoi.

  Phan cung:
   FL = GPIO14   FR = GPIO15   BL = GPIO12   BR = GPIO13
   (Nho thao canh quat truoc khi test!)
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

const int TEST_SPEED = 120;   // toc do dung de test (0..255)
const int TEST_TIME  = 1200;  // thoi gian chay moi dong co (ms)

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

void stopAllMotors() {
  ledcWrite(CH_FL, 0);
  ledcWrite(CH_FR, 0);
  ledcWrite(CH_BL, 0);
  ledcWrite(CH_BR, 0);
}

// Chay mot dong co trong 'ms' mili giay roi dung lai, co in log ro rang
void testOneMotor(const char* name, int channel, int speed, int ms) {
  Serial.printf(">> Dang chay dong co %-3s (speed=%d) trong %d ms\n", name, speed, ms);
  ledcWrite(channel, speed);
  delay(ms);
  ledcWrite(channel, 0);
  Serial.printf(">> Da dung dong co %s\n", name);
  delay(500);   // nghi giua 2 dong co de de quan sat
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 3: TEST RIENG LE 4 DONG CO ===");
  setupAllMotors();
}

void loop() {
  Serial.println("\n----- Vong test moi -----");
  testOneMotor("FL", CH_FL, TEST_SPEED, TEST_TIME);
  testOneMotor("FR", CH_FR, TEST_SPEED, TEST_TIME);
  testOneMotor("BL", CH_BL, TEST_SPEED, TEST_TIME);
  testOneMotor("BR", CH_BR, TEST_SPEED, TEST_TIME);

  stopAllMotors();
  Serial.println("Da test het 4 dong co. Nghi 3 giay...");
  delay(3000);
}
