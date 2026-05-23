/*
 * UI Board (COM9) - V7.6 (Throttle Fix)
 * --------------------------------------------------
 * 1. 重大修正：將油門累加運算移回 20ms 計時器內，解決油門瞬間暴衝問題。
 * 2. 顯示保持：標題置中、無油門顯示 (V7.5 設定)。
 * --------------------------------------------------
 */
#include <Wire.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>

// 1. 腳位定義
#define PIN_THR 1
#define PIN_YAW 0
#define PIN_PIT 3
#define PIN_ROL 2

#define PIN_SW_SPD    4  
#define PIN_SW_AUX1   5  
#define PIN_BTN_AUX2  6  
#define PIN_BTN_MODE  7  
#define PIN_CONN_LED  10 

#define OLED_SDA 8
#define OLED_SCL 9
#define UART_TX 21
#define UART_RX 20

// 2. 結構定義
struct __attribute__((packed)) Control_Packet {
  byte throttle; byte yaw; byte pitch; byte roll;
  byte aux1; byte mode; byte aux2; 
  float trim_pitch; float trim_roll; float trim_yaw;
};
Control_Packet txData;

struct __attribute__((packed)) Feedback_Packet {
  float altitude; float voltage; byte ledStatus; 
  byte deviceType; bool nrfStatus;
};
Feedback_Packet rxData;

HardwareSerial LinkSerial(0);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

bool INVERT_THR = false;
const float THR_SPEED[] = {0.3, 1.5}; 
bool valAux1=false, valMode=false, valAux2=false, isHighSpeed=false; 

// 按鍵狀態追蹤
int last_Aux1=HIGH, last_Spd=HIGH, last_Mode=HIGH, last_Aux2=HIGH;
unsigned long db_Aux1=0, db_Spd=0, db_Mode=0, db_Aux2=0;
const int DEBOUNCE_DELAY = 200; 

// 油門控制變數
float lockedThrottle=0;
int center_thr=0, center_yaw=2048, center_pit=2048, center_rol=2048;
const int TRIGGER_THRESHOLD = 1000;

// 調參變數
float trim_pitch = 0.0, trim_roll = 0.0, trim_yaw = 0.0;
bool isTrimMode = false;
int menu_cursor = 0;
unsigned long lastEditTime = 0;
bool isSwitchError = false;
unsigned long lastUartSend = 0;

// === 遙控車專用變數 ===
const int CAR_STEER_MIN[] = {64, 32};
const int CAR_STEER_MAX[] = {191, 223};
int displayN2O = 5000; 
int displayStatus = 0; // 0:Normal, 1:Nitro, 2:Cooling
unsigned long lastValidN2OTime = 0;
unsigned long lastCoolingTime = 0;

// 顯示記憶
byte currentType = 0; // 0:Scanning, 1:Drone, 2:Car
unsigned long lastDraw = 0;
float displayVoltage = 0.0; 

int readStick(int pin, bool invert) {
  int val = analogRead(pin);
  if (invert) val = 4095 - val;
  return val;
}

byte digital_switch(int val, int center) {
    int diff = val - center;
    if (diff > TRIGGER_THRESHOLD) return 255;  
    if (diff < -TRIGGER_THRESHOLD) return 0;   
    return 127;
}

void setup() {
  pinMode(PIN_SW_SPD, INPUT_PULLUP);
  pinMode(PIN_SW_AUX1, INPUT_PULLUP);
  pinMode(PIN_BTN_AUX2, INPUT_PULLUP);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_CONN_LED, OUTPUT); 
  digitalWrite(PIN_CONN_LED, LOW);

  analogSetAttenuation(ADC_11db);
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin(); 
  u8g2.setFont(u8g2_font_6x10_tr);

  // 安全預設值
  txData.throttle = 0;
  txData.yaw = 127; txData.pitch = 127; txData.roll = 127;
  txData.trim_pitch = 0.0; txData.trim_roll = 0.0; txData.trim_yaw = 0.0;

  // 自動校正
  long sum_t=0, sum_y=0, sum_p=0, sum_r=0;
  for(int i=0; i<50; i++) { 
    sum_t += readStick(PIN_THR, INVERT_THR);
    sum_y += readStick(PIN_YAW, false);
    sum_p += readStick(PIN_PIT, false);
    sum_r += readStick(PIN_ROL, false);
    delay(2); 
  }
  center_thr = sum_t/50;
  int temp_yaw = sum_y/50; int temp_pit = sum_p/50; int temp_rol = sum_r/50;
  
  if(temp_yaw > 100 && temp_yaw < 4000) center_yaw = temp_yaw;
  if(temp_pit > 100 && temp_pit < 4000) center_pit = temp_pit;
  if(temp_rol > 100 && temp_rol < 4000) center_rol = temp_rol;
  
  LinkSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
  unsigned long now = millis();
  int rAux1 = digitalRead(PIN_SW_AUX1);
  int rSpd  = digitalRead(PIN_SW_SPD);
  int rMode = digitalRead(PIN_BTN_MODE);
  int rAux2 = digitalRead(PIN_BTN_AUX2);

  // 1. 判斷滑動開關
  bool switchIsOn = (rAux1 == LOW && rSpd == LOW && rMode == LOW && rAux2 == LOW);
  isSwitchError = false; 

  if (switchIsOn) {
      if (currentType == 1) {
          isTrimMode = true;
      } 
      else if (currentType == 2) { 
          isSwitchError = true; 
          valAux1 = false; lockedThrottle = 0; valMode = false; 
      }
      else {
          isTrimMode = false;
          valAux1 = false; lockedThrottle = 0; valMode = false;
      }
      last_Aux1 = LOW; last_Spd = LOW; last_Mode = LOW; last_Aux2 = LOW;
      db_Aux1 = now; db_Spd = now; db_Mode = now; db_Aux2 = now; 
  } 
  else {
      isTrimMode = false;
      if (rxData.nrfStatus) {
            // AUX1
            if (rAux1 != last_Aux1 && (now - db_Aux1 > DEBOUNCE_DELAY)) {
                db_Aux1 = now; last_Aux1 = rAux1;
                if (rAux1 == LOW) { valAux1 = !valAux1; if(!valAux1) lockedThrottle = 0; }
            }
            // SPD
            if (rSpd != last_Spd && (now - db_Spd > DEBOUNCE_DELAY)) {
                db_Spd = now; last_Spd = rSpd;
                if (rSpd == LOW) isHighSpeed = !isHighSpeed;
            }
            // MODE
            if (rMode != last_Mode && (now - db_Mode > DEBOUNCE_DELAY)) {
                db_Mode = now; last_Mode = rMode;
                if (currentType == 2) valMode = (rMode == LOW); 
                else if (rMode == LOW) valMode = !valMode;
            }
            // AUX2
            if (rAux2 != last_Aux2 && (now - db_Aux2 > DEBOUNCE_DELAY)) {
                db_Aux2 = now; last_Aux2 = rAux2;
                if (rAux2 == LOW) valAux2 = !valAux2;
            }
      } else { 
          valAux1 = false; lockedThrottle = 0; valMode = false; 
      }
  }

  // ===============================================
  // 2. 調參模式搖桿邏輯 (不含油門累加)
  // ===============================================
  static bool thr_centered = true; 

  if (isTrimMode) {
      // >>> 調參模式 <<<
      int stickThr = readStick(PIN_THR, INVERT_THR);
      int stickYaw = readStick(PIN_YAW, false);
      int stickPit = readStick(PIN_PIT, false); 
      int stickRol = readStick(PIN_ROL, false); 

      bool is_up   = (stickThr > center_thr + TRIGGER_THRESHOLD);
      bool is_down = (stickThr < center_thr - TRIGGER_THRESHOLD);
      bool is_mid  = (abs(stickThr - center_thr) < TRIGGER_THRESHOLD);

      if (is_mid) { thr_centered = true; } 
      else if (thr_centered) {
          if (is_up) { menu_cursor--; if (menu_cursor < 0) menu_cursor = 0; thr_centered = false; } 
          else if (is_down) { menu_cursor++; if (menu_cursor > 2) menu_cursor = 2; thr_centered = false; }
      }
      if (abs(stickYaw - center_yaw) > TRIGGER_THRESHOLD) {
           if (menu_cursor == 0) trim_pitch = 0.0;
           if (menu_cursor == 1) trim_roll  = 0.0;
           if (menu_cursor == 2) trim_yaw   = 0.0;
      }
      if (now - lastEditTime > 200) {
          float change = 0.0; bool changed = false;
          if (menu_cursor == 0) { 
              if (stickPit > center_pit + TRIGGER_THRESHOLD) { change = 0.05; changed = true; } 
              if (stickPit < center_pit - TRIGGER_THRESHOLD) { change = -0.05; changed = true; } 
              if (changed) trim_pitch += change;
          } else if (menu_cursor == 1) { 
              if (stickRol > center_rol + TRIGGER_THRESHOLD) { change = 0.05; changed = true; } 
              if (stickRol < center_rol - TRIGGER_THRESHOLD) { change = -0.05; changed = true; } 
              if (changed) trim_roll += change;
          } else if (menu_cursor == 2) { 
              if (stickRol > center_rol + TRIGGER_THRESHOLD) { change = 0.05; changed = true; } 
              if (stickRol < center_rol - TRIGGER_THRESHOLD) { change = -0.05; changed = true; } 
              if (changed) trim_yaw += change;
          }
          if (changed) lastEditTime = now;
      }
  }

  // ===============================================
  // 3. 核心運算與發送 (包含油門邏輯) - 每20ms執行一次
  // ===============================================
  if (now - lastUartSend > 20) {
      lastUartSend = now;
      
      // ★★★ [修正] 油門運算邏輯移入此處，恢復 50Hz 累加頻率 ★★★
      if (!isTrimMode) {
          if (currentType == 2) {
              // [車] 直通邏輯
              int rawThr = readStick(PIN_THR, INVERT_THR);
              int carThr = map(rawThr, 0, 4095, 0, 255);
              if (!valAux1) carThr = 127; else { if (carThr > 120 && carThr < 135) carThr = 127; }
              txData.throttle = (byte)carThr;
          } else {
              // [無人機] 累加邏輯 (定速巡航)
              if (valAux1 && !isSwitchError) { 
                 int rawThr = readStick(PIN_THR, INVERT_THR);
                 int spdIdx = isHighSpeed ? 1 : 0;
                 if (rawThr > center_thr + TRIGGER_THRESHOLD) lockedThrottle += THR_SPEED[spdIdx];
                 else if (rawThr < center_thr - TRIGGER_THRESHOLD) lockedThrottle -= THR_SPEED[spdIdx];
                 lockedThrottle = constrain(lockedThrottle, 0, 255);
              } else {
                 if (!valAux1) lockedThrottle = 0;
              }
              txData.throttle = (byte)lockedThrottle;
          }
      } else {
          // 調參模式下鎖定發送的油門值
          txData.throttle = (byte)lockedThrottle;
      }

      // 填入其他數據
      txData.aux1 = valAux1; txData.mode = valMode; txData.aux2 = valAux2;
      txData.trim_pitch = trim_pitch; txData.trim_roll = trim_roll; txData.trim_yaw = trim_yaw;

      if (isTrimMode) {
          txData.yaw = 127; txData.pitch = 127; txData.roll = 127;
      } else {
          if (currentType == 2) {
             int spdIdx = isHighSpeed ? 1 : 0;
             txData.roll = map(readStick(PIN_ROL, false), 0, 4095, CAR_STEER_MIN[spdIdx], CAR_STEER_MAX[spdIdx]);
             txData.yaw = 127; txData.pitch = 127;
          } else {
             txData.yaw   = digital_switch(readStick(PIN_YAW, false), center_yaw);
             txData.pitch = digital_switch(readStick(PIN_PIT, false), center_pit);
             txData.roll  = digital_switch(readStick(PIN_ROL, false), center_rol);
          }
      }
      
      // 發送
      LinkSerial.write(0xA5); LinkSerial.write(0x5A);
      LinkSerial.write((uint8_t*)&txData, sizeof(txData));
  }

  // ===============================================
  // 4. 資料接收
  // ===============================================
  if (LinkSerial.available() >= sizeof(rxData) + 2) {
      if(LinkSerial.read() == 0xA5 && LinkSerial.read() == 0x5A) {
          LinkSerial.readBytes((uint8_t*)&rxData, sizeof(rxData));
          
          if (rxData.deviceType != 0) currentType = rxData.deviceType;
          
          if (rxData.voltage > 1.0) { 
              if (displayVoltage == 0.0 || abs(rxData.voltage - displayVoltage) < 1.0) displayVoltage = rxData.voltage; 
          }
          if (currentType == 2) {
              int rawN2O = (int)rxData.altitude;
              if (rawN2O > 5000) rawN2O = 5000;
              if (abs(rawN2O - displayN2O) > 100 || now - lastValidN2OTime > 500) {
                  displayN2O = rawN2O; lastValidN2OTime = now;
              }
              if (rxData.ledStatus == 2) { displayStatus = 2; lastCoolingTime = now; }
              else if (rxData.ledStatus == 1) { displayStatus = 1; }
              else if (now - lastCoolingTime > 500) { displayStatus = 0; }
          }
      }
  }
  digitalWrite(PIN_CONN_LED, rxData.nrfStatus);

  // ===============================================
  // 5. OLED 顯示
  // ===============================================
  static bool lastNrfStatus = false; static bool firstRun = true;
  bool currentStatus = rxData.nrfStatus;
  bool shouldDraw = false;
  int refreshRate = 100;

  if (firstRun || currentStatus != lastNrfStatus || (currentStatus && now - lastDraw > refreshRate) || isSwitchError || isTrimMode) {
      shouldDraw = true;
  }

  if (shouldDraw) {
    lastDraw = now; lastNrfStatus = currentStatus; firstRun = false;
    u8g2.clearBuffer();
    
    if (isSwitchError) {
        u8g2.drawBox(0, 10, 128, 44); u8g2.setColorIndex(0); u8g2.setFont(u8g2_font_9x15_tr);
        u8g2.setCursor(5, 30); u8g2.print("CAR MODE"); 
        u8g2.setFont(u8g2_font_6x10_tr); u8g2.setCursor(15, 45); u8g2.print("NO PID TRIM");
        u8g2.setColorIndex(1);
    }
    else if (isTrimMode) {
        // [Drone] 調參畫面 (修正：小字體 + 置中)
        u8g2.setFont(u8g2_font_9x15_tr); 
        u8g2.drawStr(10, 12, "* ICM TRIM *"); // X=40 (置中)

        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(20, 32); u8g2.print("P (Pit): "); u8g2.print(trim_pitch, 2);
        u8g2.setCursor(20, 45); u8g2.print("R (Rol): "); u8g2.print(trim_roll, 2);
        u8g2.setCursor(20, 58); u8g2.print("Y (Yaw): "); u8g2.print(trim_yaw, 2);
        
        int boxY = 22 + (menu_cursor * 13); 
        u8g2.drawFrame(5, boxY, 120, 13);
    }
    else {
        // [主畫面]
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(0, 12); 
        if (currentType == 1) u8g2.print("Drone"); 
        else if (currentType == 2) u8g2.print("RC Car"); 
        else u8g2.print("Scan..."); 

        u8g2.setCursor(64, 12); 
        if (currentType == 1) { 
            if(valMode) u8g2.print("HOLD"); 
        }
        if (currentType == 2 && displayStatus == 1) u8g2.print("NITRO!");

        u8g2.setCursor(0, 28); u8g2.print("Spd: "); u8g2.print(isHighSpeed?"Hi":"Lo"); 
        u8g2.setCursor(64, 28); 
        if (valAux1) {
            if(currentType == 2) u8g2.print("Fan:ON"); else u8g2.print("Armed");
        } else {
            u8g2.print("Locked");
        }

        u8g2.setCursor(0, 44); u8g2.print("Thr: "); 
        if(currentType == 2) u8g2.print(txData.throttle); else u8g2.print((int)lockedThrottle);
        u8g2.setCursor(64, 44); u8g2.print("LED: "); u8g2.print(valAux2?"OFF":"ON");

        u8g2.setCursor(0, 60); 
        if (currentType == 2) {
             if(displayStatus == 2) u8g2.print("Cooling...");
             else {
                 int pct = map(displayN2O, 0, 5000, 0, 100);
                 u8g2.print("N2O: "); u8g2.print(pct); u8g2.print("%");
             }
        } else {
             u8g2.print("Alt: "); if(rxData.nrfStatus) u8g2.print(rxData.altitude, 1); else u8g2.print("--");
        }
        u8g2.setCursor(64, 60); u8g2.print("Bat: "); u8g2.print(displayVoltage, 1);
    }
    u8g2.sendBuffer();
  }
}