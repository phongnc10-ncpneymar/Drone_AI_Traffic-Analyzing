/*
  BAI 5: TEST NGHIENG (ROLL), CUI/NGUA (PITCH), XOAY (YAW)
  -----------------------------------------------------------
  Muc tieu:
   - Dung Serial de goi tung dong tac bay co ban: roll trai/phai,
     pitch truoc/sau, yaw thuan/nghich chieu kim dong ho.
   - Moi dong tac chi lech toc do giua CAC CAP dong co, giup quan sat
     ro co che "4 dong co phoi hop" ma khong can bay thuc.

  Phan cung:
      FL (14)        FR (15)
          \\    quadcopter    /
          /                  \\
      BL (12)        BR (13)

  Phim dieu khien tren Serial Monitor:
   'a' = roll trai     'd' = roll phai
   'i' = pitch truoc    'k' = pitch sau
   'q' = yaw CCW        'e' = yaw CW
   's' = tro ve throttle nen (hover) khong lech
   ' ' = dung khan cap
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

const int BASE  = 120;   // toc do nen (hover gia lap)
const int DELTA = 40;    // do lech dung cho roll/pitch/yaw

void setupMotor(int pin, int ch) {
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

void setMotor(int ch, int speed) {
  speed = constrain(speed, 0, 255);
  ledcWrite(ch, speed);
}

void hover() {
  setMotor(CH_FL, BASE);
  setMotor(CH_FR, BASE);
  setMotor(CH_BL, BASE);
  setMotor(CH_BR, BASE);
  Serial.println("-> HOVER (toc do nen, khong lech)");
}

void rollLeft() {
  // Nghieng trai: giam toc 2 dong co ben trai (FL, BL)
  setMotor(CH_FL, BASE - DELTA);
  setMotor(CH_BL, BASE - DELTA);
  setMotor(CH_FR, BASE);
  setMotor(CH_BR, BASE);
  Serial.println("-> ROLL TRAI");
}

void rollRight() {
  // Nghieng phai: giam toc 2 dong co ben phai (FR, BR)
  setMotor(CH_FR, BASE - DELTA);
  setMotor(CH_BR, BASE - DELTA);
  setMotor(CH_FL, BASE);
  setMotor(CH_BL, BASE);
  Serial.println("-> ROLL PHAI");
}

void pitchForward() {
  // Cui dau ra truoc: giam toc 2 dong co truoc (FL, FR)
  setMotor(CH_FL, BASE - DELTA);
  setMotor(CH_FR, BASE - DELTA);
  setMotor(CH_BL, BASE);
  setMotor(CH_BR, BASE);
  Serial.println("-> PITCH FORWARD");
}

void pitchBackward() {
  // Ngua ra sau: giam toc 2 dong co sau (BL, BR)
  setMotor(CH_BL, BASE - DELTA);
  setMotor(CH_BR, BASE - DELTA);
  setMotor(CH_FL, BASE);
  setMotor(CH_FR, BASE);
  Serial.println("-> PITCH BACKWARD");
}

void yawCW() {
  // Xoay thuan chieu kim dong ho: tang cap cheo FL-BR, giam cap cheo FR-BL
  setMotor(CH_FL, BASE + DELTA);
  setMotor(CH_BR, BASE + DELTA);
  setMotor(CH_FR, BASE - DELTA);
  setMotor(CH_BL, BASE - DELTA);
  Serial.println("-> YAW CLOCKWISE");
}

void yawCCW() {
  // Xoay nguoc chieu kim dong ho: nguoc lai voi yawCW
  setMotor(CH_FR, BASE + DELTA);
  setMotor(CH_BL, BASE + DELTA);
  setMotor(CH_FL, BASE - DELTA);
  setMotor(CH_BR, BASE - DELTA);
  Serial.println("-> YAW COUNTER-CLOCKWISE");
}

void emergencyStop() {
  setMotor(CH_FL, 0);
  setMotor(CH_FR, 0);
  setMotor(CH_BL, 0);
  setMotor(CH_BR, 0);
  Serial.println("!! DUNG KHAN CAP !!");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== BAI 5: TEST ROLL / PITCH / YAW ===");
  Serial.println("a=rollTrai d=rollPhai i=pitchTruoc k=pitchSau q=yawCCW e=yawCW s=hover space=STOP");

  setupMotor(MOTOR_FL, CH_FL);
  setupMotor(MOTOR_FR, CH_FR);
  setupMotor(MOTOR_BL, CH_BL);
  setupMotor(MOTOR_BR, CH_BR);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'a': rollLeft(); break;
      case 'd': rollRight(); break;
      case 'i': pitchForward(); break;
      case 'k': pitchBackward(); break;
      case 'q': yawCCW(); break;
      case 'e': yawCW(); break;
      case 's': hover(); break;
      case ' ': emergencyStop(); break;
      default: break;   // bo qua ky tu khong hop le (vd: xuong dong)
    }
  }
}
