// ============================================================
//  Arduino – Raspberry Pi'den seri komut al
//  G / H / I komutuna göre LED3-7 analog voltaj ayarla
//
//  PWM Pinleri:
//    Pin 3  → LED3
//    Pin 5  → LED4
//    Pin 6  → LED5
//    Pin 9  → LED6
//    Pin 10 → LED7
//
//  Voltaj → PWM değeri (5V referans):
//    3.5V  → 178
//    1.49V →  76
//    0V    →   0
//
//  RC Filtre (her PWM pinine):
//    PWM Pin ── 10kΩ ──┬── Çıkış
//                      │
//                    100µF
//                      │
//                     GND
// ============================================================

#define LED3_PIN   3
#define LED4_PIN   5
#define LED5_PIN   6
#define LED6_PIN   9
#define LED7_PIN  10

#define PWM_3V5   178
#define PWM_1V49   76
#define PWM_0V      0

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  pinMode(LED6_PIN, OUTPUT);
  pinMode(LED7_PIN, OUTPUT);

  all_off();
  Serial.println("READY");
}

// ============================================================
//  YARDIMCI FONKSİYONLAR
// ============================================================
void all_off() {
  analogWrite(LED3_PIN, PWM_0V);
  analogWrite(LED4_PIN, PWM_0V);
  analogWrite(LED5_PIN, PWM_0V);
  analogWrite(LED6_PIN, PWM_0V);
  analogWrite(LED7_PIN, PWM_0V);
}

// G: LED3=3.5V  LED4=3.5V  LED5=1.49V  LED6=0V  LED7=1.49V
void cmd_g() {
  analogWrite(LED3_PIN, PWM_3V5);
  analogWrite(LED4_PIN, PWM_3V5);
  analogWrite(LED5_PIN, PWM_1V49);
  analogWrite(LED6_PIN, PWM_0V);
  analogWrite(LED7_PIN, PWM_1V49);
}

// H: LED3=1.49V  LED4=3.5V  LED5=3.5V  LED6=0V  LED7=1.49V
void cmd_h() {
  analogWrite(LED3_PIN, PWM_1V49);
  analogWrite(LED4_PIN, PWM_3V5);
  analogWrite(LED5_PIN, PWM_3V5);
  analogWrite(LED6_PIN, PWM_0V);
  analogWrite(LED7_PIN, PWM_1V49);
}

// I: LED3=1.49V  LED4=0V  LED5=1.49V  LED6=0V  LED7=1.49V
void cmd_i() {
  analogWrite(LED3_PIN, PWM_1V49);
  analogWrite(LED4_PIN, PWM_0V);
  analogWrite(LED5_PIN, PWM_1V49);
  analogWrite(LED6_PIN, PWM_0V);
  analogWrite(LED7_PIN, PWM_1V49);
}

// ============================================================
//  ANA DÖNGÜ
// ============================================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'G':
      case 'g':
        cmd_g();
        Serial.println("OK:G");
        break;

      case 'H':
      case 'h':
        cmd_h();
        Serial.println("OK:H");
        break;

      case 'I':
      case 'i':
        cmd_i();
        Serial.println("OK:I");
        break;

      case 'X':
      case 'x':
        all_off();
        Serial.println("OK:X");
        break;

      default:
        break;
    }
  }
}