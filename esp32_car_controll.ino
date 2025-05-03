/*  UDP-controlled car  —  LEFT / RIGHT / FORWARD / BACK / STOP
    Works with the Python script that sends L, R, F, B, S
*/
#include <WiFi.h>
#include <WiFiUDP.h>

/* ── Wi-Fi & UDP ──────────────────────────────────────────── */
const char* SSID = "ALTEL 5G_82BD";
const char* PASS = "61786318";
const uint16_t PORT = 4210;         // must match Python

WiFiUDP udp;

/* ── L298N pin map (change if you wired differently) ─────── */
const int ENA = 25, IN1 = 26, IN2 = 27;   // left motor
const int ENB = 33, IN3 = 32, IN4 = 23;   // right motor
const uint8_t DUTY = 200;                 // 0-255 speed

inline void duty(uint8_t d){ analogWrite(ENA,d); analogWrite(ENB,d); }

void stopM(){ digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
              digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); duty(0); }

void fwd  (){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
              digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); duty(DUTY); }

void back (){ digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
              digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); duty(DUTY); }

void left (){ digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
              digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); duty(DUTY); }

void right(){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
              digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); duty(DUTY); }

void setup() {
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);

  analogWriteFrequency(ENA, 1000);  // 1 kHz PWM
  analogWriteFrequency(ENB, 1000);
  analogWriteResolution(ENA, 8);    // 0-255
  analogWriteResolution(ENB, 8);
  stopM();

  WiFi.begin(SSID, PASS);
  Serial.print("Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print('.'); delay(500); }
  Serial.printf("\nIP %s  —  listening UDP %u\n",
                WiFi.localIP().toString().c_str(), PORT);

  udp.begin(PORT);
}

void loop() {
  int len = udp.parsePacket();      // ← fixed: no stray parenthesis
  if (len) {
    char cmd = udp.read();          // first byte of the packet
    switch (cmd) {
      case 'L': left();   Serial.println("← LEFT");   break;
      case 'R': right();  Serial.println("→ RIGHT");  break;
      case 'F': fwd();    Serial.println("↑ FORWARD");break;
      case 'B': back();   Serial.println("↓ BACK");   break;
      case 'S':
      default : stopM();  Serial.println("■ STOP");   break;
    }
  }
}
