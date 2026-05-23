/* NRF Relay Board (COM7) - V3.5 (Header Sync Fix) */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <HardwareSerial.h>

struct __attribute__((packed)) Control_Packet {
  byte throttle; byte yaw; byte pitch; byte roll;
  byte aux1; byte mode; byte aux2; 
  float trim_pitch; float trim_roll; float trim_yaw;
};
Control_Packet rxFromUI;

struct __attribute__((packed)) Feedback_Packet {
  float altitude; float voltage; byte ledStatus;
  byte deviceType; bool nrfStatus;
};
Feedback_Packet txToUI;

struct __attribute__((packed)) Drone_Data {
  byte throttle; byte yaw; byte pitch; byte roll;
  byte aux1; byte mode; byte aux2;
  float trim_pitch; float trim_roll; float trim_yaw;
};
Drone_Data droneData;

struct __attribute__((packed)) Drone_Telem {
  float altitude; float voltage; byte ledStatus;
  byte deviceType; bool nrfStatus; 
};
Drone_Telem droneTelem;

#define NRF_SCK 6
#define NRF_MISO 5
#define NRF_MOSI 7
#define NRF_CSN 10
#define NRF_CE 4
#define UART_TX 21
#define UART_RX 20
#define PIN_LED 8

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "00001";
HardwareSerial LinkSerial(0);

unsigned long lastAckTime = 0;
int ackCounter = 0;

void initNRF() {
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  if (!radio.begin()) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }
  radio.openWritingPipe(address);
  radio.enableAckPayload();
  radio.enableDynamicPayloads(); 
  radio.setChannel(108);          
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(5, 3);
  radio.stopListening();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  LinkSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  
  // 安全預設值
  droneData.throttle = 0;
  droneData.yaw   = 127; 
  droneData.pitch = 127; 
  droneData.roll  = 127; 
  droneData.aux1  = 0;
  droneData.mode  = 0;
  droneData.aux2  = 0;
  droneData.trim_pitch = 0.0; droneData.trim_roll  = 0.0; droneData.trim_yaw   = 0.0;

  initNRF();
}

// 在 loop() 上面宣告這個變數，用來記錄上次偷偷重啟的時間
unsigned long lastRebootTime = 0;

void loop() {
  // 檢查 UI 板傳來的資料
  if (LinkSerial.available() >= sizeof(rxFromUI) + 2) {
      if (LinkSerial.read() == 0xA5) {
          if (LinkSerial.read() == 0x5A) {
              
              LinkSerial.readBytes((uint8_t*)&rxFromUI, sizeof(rxFromUI));

              // 搬運資料
              droneData.throttle = rxFromUI.throttle;
              droneData.yaw      = rxFromUI.yaw;
              droneData.pitch    = rxFromUI.pitch;
              droneData.roll     = rxFromUI.roll;
              droneData.aux1     = rxFromUI.aux1;
              droneData.mode     = rxFromUI.mode;
              droneData.aux2     = rxFromUI.aux2;
              droneData.trim_pitch = rxFromUI.trim_pitch;
              droneData.trim_roll  = rxFromUI.trim_roll;
              droneData.trim_yaw   = rxFromUI.trim_yaw;

              // 發送給飛機
              bool success = radio.write(&droneData, sizeof(droneData));
              
              if(success) {
                  // ★ 成功收到 Ack ★
                  lastAckTime = millis(); // 更新連線時間
                  ackCounter = 0;         // 重置失敗計數
                  
                  // 閃爍 LED 代表正常通訊
                  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
                  
                  if(radio.isAckPayloadAvailable()){
                      radio.read(&droneTelem, sizeof(droneTelem));
                      txToUI.altitude = droneTelem.altitude;
                      txToUI.voltage = droneTelem.voltage;
                      txToUI.ledStatus = droneTelem.ledStatus;
                      if(droneTelem.deviceType != 0) txToUI.deviceType = droneTelem.deviceType;
                  }
              } else {
                  radio.flush_tx(); // 清除發送緩存
              }
          }
      }
  }

  unsigned long now = millis();
  unsigned long timeSinceLastAck = now - lastAckTime;

  // ========================================================
  // ★★★ 智慧重連機制 (Auto-Recovery) ★★★
  // ========================================================
  // 此機制旨在快速檢測斷線並通知接收機進入Failsafe
  // ========================================================
  
  // 階段 1: 正常連線 ( < 300ms ) -> 什麼都不用做，nrfStatus = true

  // 階段 2: 黃色警戒 ( 300ms ~ 1500ms ) -> 偷偷重啟 NRF，但不鎖定
  if (timeSinceLastAck > 300 && timeSinceLastAck < 1500) {
      
      // 每 300ms 嘗試重啟一次 NRF，不要每一圈都重啟以免卡死
      if (now - lastRebootTime > 300) {
          lastRebootTime = now;
          
          // 這裡不做 digitalWrite(LED)，以免影響除錯
          // 執行 NRF 初始化，模擬按下 Reset 鍵的效果
          initNRF(); 
      }
      
      // ★ 關鍵：這裡我們還是告訴 UI "連線中 (true)" ★
      // 因為妳說控制是正常的，所以我們先不要觸發飛機的降落鎖定
      txToUI.nrfStatus = true; 
  }
  
  // 階段 3: 紅色警戒 ( > 1500ms ) -> 真的沒救了，宣告斷線
  else if (timeSinceLastAck >= 1500) {
      txToUI.nrfStatus = false;     // ★ 立即告訴接收機斷線了 (接收機會馬上觸發Failsafe)
      txToUI.deviceType = 0;
      digitalWrite(PIN_LED, HIGH);  // LED 恆亮警告
      
      // 持續嘗試重啟
      if (now - lastRebootTime > 1000) {
          lastRebootTime = now;
          initNRF();
      }
  } 
  
  // 階段 1 的狀態設定
  else {
      txToUI.nrfStatus = true;
  }
  
  // ========================================================
  // 除錯: 每 1 秒列印一次連線狀態
  // ========================================================
  static unsigned long lastDebugTime = 0;
  if (now - lastDebugTime >= 1000) {
      lastDebugTime = now;
      Serial.print("[NRF-COM7] ");
      Serial.print("連線時間: "); Serial.print(timeSinceLastAck); Serial.print("ms | ");
      if (txToUI.nrfStatus) {
          Serial.println("✅ 連線正常 -> 通知接收機 nrfStatus=true");
      } else {
          Serial.println("🚨 偵測斷線 -> 立即通知接收機 nrfStatus=false (接收機進入Failsafe)");
      }
  }
  // ========================================================

  // 回傳給 UI 板
  static unsigned long lastUISend = 0;
  if (now - lastUISend > 50) {  
      lastUISend = now;
      LinkSerial.write(0xA5);
      LinkSerial.write(0x5A);
      LinkSerial.write((uint8_t*)&txToUI, sizeof(txToUI));
  }
}