#include <Arduino.h>

// ===== TB6612FNG =====
#define PWMA 3
#define AIN1 4
#define AIN2 5

#define PWMB 11
#define BIN1 10
#define BIN2 9

// ===== 센서 =====
#define RIGHT_SENSOR 7
#define LEFT_SENSOR 8

// ===== 기본 파라미터 =====
static uint8_t LINE_SPEED = 220;

// TN/RT 기본값(런타임에서 변경됨)
static uint8_t  TN_SPEED   = 240;
static uint16_t TN_TIME_MS = 1500;

static uint8_t  RT_SPEED   = 240;
static uint16_t RT_TIME_MS = 1500;

// ===== KICK(턴 시작 토크 보강) =====
static uint8_t  KICK_SPD = 255;  // 0~255 (런타임 변경 가능)
static uint16_t KICK_MS  = 60;   // 0~20000 (런타임 변경 가능)

// ===== 상태 =====
enum State { STATE_ST, STATE_GO };
State state = STATE_GO;

// ===== 모터 =====
static inline void motorA_fwd(uint8_t spd){ digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW);  analogWrite(PWMA,spd); }
static inline void motorA_rev(uint8_t spd){ digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH); analogWrite(PWMA,spd); }
static inline void motorB_fwd(uint8_t spd){ digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);  analogWrite(PWMB,spd); }
static inline void motorB_rev(uint8_t spd){ digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH); analogWrite(PWMB,spd); }

static inline void motorsStop(){
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ===== 상태 전송 =====
static inline void sendState(){
  if (state == STATE_GO) Serial.println("RX_GO");
  else                   Serial.println("RX_ST");
}

// ===== KICK 헬퍼 =====
static inline void kickTN(){
  // TN: A 역, B 정
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, KICK_SPD);
  analogWrite(PWMB, KICK_SPD);
  delay(KICK_MS);
}
static inline void kickRT(){
  // RT: A 정, B 역
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, KICK_SPD);
  analogWrite(PWMB, KICK_SPD);
  delay(KICK_MS);
}

// ===== 명령 동작 =====
static inline void doST(){
  motorsStop();
  state = STATE_ST;
  sendState();
  Serial.println("D_ST");
}

static inline void doGO(){
  if (state == STATE_GO) return;
  state = STATE_GO;
  sendState();
}

static inline void doTN(){
  if (state != STATE_ST) return;

  kickTN();                 // ★ 매번 TN 시작 시 킥
  motorA_rev(TN_SPEED);
  motorB_fwd(TN_SPEED);

  delay(TN_TIME_MS);
  motorsStop();
  Serial.println("D_TN");
}

static inline void doRT(){
  if (state != STATE_ST) return;

  kickRT();                 // ★ 매번 RT 시작 시 킥
  motorA_fwd(RT_SPEED);
  motorB_rev(RT_SPEED);

  delay(RT_TIME_MS);
  motorsStop();
  Serial.println("D_RT");
}

// ===== "KEY,VALUE" 파서 =====
static inline bool parseKeyValue(const char* s, char* keyOut, size_t keyOutSz, long &valOut){
  const char* comma = strchr(s, ',');
  if (!comma) return false;

  size_t klen = (size_t)(comma - s);
  if (klen == 0 || klen >= keyOutSz) return false;

  memcpy(keyOut, s, klen);
  keyOut[klen] = '\0';

  const char* vstr = comma + 1;
  if (*vstr == '\0') return false;

  valOut = atol(vstr);
  return true;
}

static inline uint8_t clampU8(long v){
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}
static inline uint16_t clampU16(long v, long maxv){
  if (v < 0) return 0;
  if (v > maxv) return (uint16_t)maxv;
  return (uint16_t)v;
}

// ===== UART 수신 =====
static inline void handleSerial(){
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available() > 0){
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n'){
      buf[idx] = '\0';
      idx = 0;

      // 1) KEY,VALUE 명령
      char key[16];
      long val = 0;
      if (parseKeyValue(buf, key, sizeof(key), val)){

        if (strcmp(key, "LS_SPD") == 0){
          LINE_SPEED = clampU8(val);
          Serial.print("ACK_LS_SPD=");
          Serial.println(LINE_SPEED);
          return;
        }

        if (strcmp(key, "RT_SPD") == 0){
          RT_SPEED = clampU8(val);
          Serial.print("ACK_RT_SPD=");
          Serial.println(RT_SPEED);
          return;
        }
        if (strcmp(key, "TN_SPD") == 0){
          TN_SPEED = clampU8(val);
          Serial.print("ACK_TN_SPD=");
          Serial.println(TN_SPEED);
          return;
        }
        if (strcmp(key, "RT_TMS") == 0){
          RT_TIME_MS = clampU16(val, 20000);
          Serial.print("ACK_RT_TMS=");
          Serial.println(RT_TIME_MS);
          return;
        }
        if (strcmp(key, "TN_TMS") == 0){
          TN_TIME_MS = clampU16(val, 20000);
          Serial.print("ACK_TN_TMS=");
          Serial.println(TN_TIME_MS);
          return;
        }

        // ★ 킥 파라미터 런타임 변경
        if (strcmp(key, "KICK_SPD") == 0){
          KICK_SPD = clampU8(val);
          Serial.print("ACK_KICK_SPD=");
          Serial.println(KICK_SPD);
          return;
        }
        if (strcmp(key, "KICK_MS") == 0){
          KICK_MS = clampU16(val, 20000);
          Serial.print("ACK_KICK_MS=");
          Serial.println(KICK_MS);
          return;
        }
      }

      // 2) 단문 명령
      if      (strcmp(buf, "GO") == 0) doGO();
      else if (strcmp(buf, "ST") == 0) doST();
      else if (strcmp(buf, "TN") == 0) doTN();
      else if (strcmp(buf, "RT") == 0) doRT();

      return;
    }

    if (idx < sizeof(buf)-1) buf[idx++] = c;
    else idx = 0;
  }
}

void setup(){
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);

  Serial.begin(9600);
  sendState();
}

void loop(){
  handleSerial();
  if (state == STATE_ST) return;

  bool R = digitalRead(RIGHT_SENSOR);
  bool L = digitalRead(LEFT_SENSOR);

  if (R == 0 && L == 0){
    motorA_fwd(LINE_SPEED);
    motorB_fwd(LINE_SPEED);
  } else if (R == 1 && L == 0){
    motorA_fwd(LINE_SPEED);
    motorB_fwd(0);
  } else if (R == 0 && L == 1){
    motorA_fwd(0);
    motorB_fwd(LINE_SPEED);
  } else {
    motorsStop();
  }
}
