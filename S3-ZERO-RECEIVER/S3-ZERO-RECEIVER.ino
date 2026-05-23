/*
 * =========================================================================
 * 🛸 550 空母中樞大腦 V2.2 (NRF 狀態即時監控 + SIM 卡偵測防卡死)
 * =========================================================================
 */
#include <Adafruit_NeoPixel.h> 
#include <SPI.h>
#include <RF24.h>
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#if __has_include(<TinyGsmClientSecure.h>)
#include <TinyGsmClientSecure.h>
#define HAS_TINY_GSM_SECURE
#endif
#include <WebSocketsClient.h>

#define RGB_PIN 21 
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

#define NRF_CE     1
#define NRF_CSN    2
#define SPI_SCK    3
#define SPI_MOSI   4
#define SPI_MISO   5
#define FC_TX_PIN  7  
#define FC_RX_PIN  8  
#define MODEM_RX_PIN 12 
#define MODEM_TX_PIN 13 

HardwareSerial SerialModem(1);
HardwareSerial SerialFC(2);
RF24 radio(NRF_CE, NRF_CSN);
TinyGsm modem(SerialModem);
#ifdef HAS_TINY_GSM_SECURE
TinyGsmClientSecure secureClient(modem);
#else
TinyGsmClient secureClient(modem); // fallback if secure header not available
#endif
WebSocketsClient webSocket;

const char* wsHost = "five50-drone-telemetry.onrender.com";
const uint16_t wsPort = 443;
const char* wsPath = "/";
const byte address[6] = "00001";

struct __attribute__((packed)) Control_Packet {
  byte throttle; byte yaw; byte pitch; byte roll;
  byte aux1; byte mode; byte aux2;
  float trim_pitch; float trim_roll; float trim_yaw;
} rxData;

struct __attribute__((packed)) Feedback_Packet {
  float altitude; float voltage; byte ledStatus;
  byte deviceType; bool nrfStatus; 
} txTelem;

struct {
  uint8_t  fix; uint8_t  numSat; int32_t  lat; int32_t  lon;             
  float    roll; float    pitch; float    yaw; float    voltage; float    altitude_m;      
  float    vertical_speed;  
} drone_data;

enum MspParseState {
  MSP_SYNC1,
  MSP_SYNC2,
  MSP_DIR,
  MSP_SIZE,
  MSP_CMD,
  MSP_PAYLOAD,
  MSP_CHECKSUM
};

MspParseState msp_parse_state = MSP_SYNC1;
uint8_t msp_rx_size = 0;
uint8_t msp_rx_cmd = 0;
uint8_t msp_rx_checksum = 0;
uint8_t msp_rx_payload[64];
uint8_t msp_rx_index = 0;

// 🕒 時間與狀態變數
unsigned long lastSignalTime = 0; 
unsigned long lastRcTime = 0;
unsigned long lastMspReqTime = 0;
unsigned long lastWebSocketTime = 0;
unsigned long lastNetworkCheckTime = 0; 
unsigned long mspDelayTimer = 0;
unsigned long lastDebugPrintTime = 0; // 👈 新增：除錯列印計時器

uint8_t mspStep = 0; 
uint16_t msp_channels[8] = {1500, 1500, 1500, 1000, 1000, 1000, 1500, 1500};
bool isNrfConnected = false;          // 👈 記錄當前 NRF 連線狀態
bool isInFailsafe = false;            // 👈 記錄是否進入了 Failsafe 且轉舵油門

// CH5 解鎖後延遲觸發 CH8
bool ch5_active = false;
bool ch8_triggered = false;
unsigned long ch8_trigger_time = 0;

// 🔧 核心 1 專用變數
SemaphoreHandle_t modem_mutex = NULL;
SemaphoreHandle_t data_mutex = NULL;
volatile bool webSocketConnectedState = false;
volatile bool gprsConnectedState = false;

void debugLog(const char* tag, const char* msg) {
  unsigned long now = millis();
  Serial.print("["); Serial.print(now); Serial.print("] ");
  Serial.print(tag); Serial.print(" ");
  Serial.println(msg);
}

void debugLog(const char* tag, const char* key, const char* value) {
  unsigned long now = millis();
  Serial.print("["); Serial.print(now); Serial.print("] ");
  Serial.print(tag); Serial.print(" ");
  Serial.print(key); Serial.print(": "); Serial.println(value);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      debugLog("[核心 1]", "WebSocket", "已連線");
      webSocketConnectedState = true;
      break;
    case WStype_DISCONNECTED:
      debugLog("[核心 1]", "WebSocket", "已斷線");
      webSocketConnectedState = false;
      break;
    case WStype_ERROR:
      debugLog("[核心 1]", "WebSocket", "發生錯誤");
      webSocketConnectedState = false;
      break;
    case WStype_TEXT:
      // 目前不需要處理來自伺服器的文字回傳
      break;
    default:
      break;
  }
}

void core1_modem_task(void* pvParameters) {
  // 核心 1：純粹處理 4G / WebSocket，完全不會阻塞核心 0 的 NRF24
  debugLog("[核心 1]", "4G/WebSocket 背景任務已啟動");
  
  unsigned long lastNetworkCheck = 0;
  
  bool lastGprsConnected = false;
  bool lastWebSocketConnected = false;
  unsigned long lastWebSocketLoop = 0;

  while (true) {
    unsigned long now = millis();
    
    // ⏱️ 每 5 秒檢查一次網路狀態
    if (now - lastNetworkCheck > 5000) {
      lastNetworkCheck = now;
      debugLog("[核心 1]", "開始網路檢查");
      
      if (xSemaphoreTake(modem_mutex, pdMS_TO_TICKS(500))) {
        SimStatus sim_status = modem.getSimStatus();
        if (sim_status == SIM_READY) {
          if (!modem.isGprsConnected()) {
            debugLog("[核心 1]", "SIM_READY", "4G 未連接，開始 gprsConnect");
            modem.gprsConnect("internet", "", "");
            if (modem.isGprsConnected()) {
              debugLog("[核心 1]", "gprsConnect", "成功");
            } else {
              debugLog("[核心 1]", "gprsConnect", "失敗");
            }
          } else {
            debugLog("[核心 1]", "4G 狀態", "已連接");
          }
        } else {
          debugLog("[核心 1]", "SIM 卡狀態", "未就緒");
        }
        xSemaphoreGive(modem_mutex);
      } else {
        debugLog("[核心 1]", "modem_mutex", "取鎖失敗");
      }
    }

    // ⏱️ WebSocket 事件與連線維護
    if (now - lastWebSocketLoop > 100) {
      lastWebSocketLoop = now;
      if (xSemaphoreTake(modem_mutex, pdMS_TO_TICKS(500))) {
        if (modem.isGprsConnected()) {
          if (!webSocket.isConnected()) {
            debugLog("[核心 1]", "WebSocket", "嘗試連線");
            webSocket.beginSSL(wsHost, wsPort, wsPath);
          }
          webSocket.loop();
          if (!lastWebSocketConnected && webSocket.isConnected()) {
            debugLog("[核心 1]", "WebSocket", "已連線");
          }
          lastWebSocketConnected = webSocket.isConnected();
          webSocketConnectedState = lastWebSocketConnected;
        } else {
          if (lastWebSocketConnected) {
            debugLog("[核心 1]", "WebSocket", "已斷線");
          }
          lastWebSocketConnected = false;
          webSocketConnectedState = false;
        }
        gprsConnectedState = modem.isGprsConnected();
        xSemaphoreGive(modem_mutex);
      } else {
        debugLog("[核心 1]", "modem_mutex", "取鎖失敗");
      }
    }

    if (now - lastWebSocketTime >= 1000 && drone_data.fix) {
      if (xSemaphoreTake(modem_mutex, pdMS_TO_TICKS(500))) {
        if (webSocket.isConnected()) {
          char payload[256];
          snprintf(payload, sizeof(payload),
                   "{\"lat\":%.7f,\"lon\":%.7f,\"sats\":%d,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"vol\":%.1f,\"alt\":%.1f,\"vspd\":%.2f,\"fs\":%d}",
                   drone_data.lat / 10000000.0, drone_data.lon / 10000000.0,
                   drone_data.numSat, drone_data.roll, drone_data.pitch, drone_data.yaw,
                   drone_data.voltage, drone_data.altitude_m, drone_data.vertical_speed,
                   isInFailsafe ? 1 : 0);
          webSocket.sendTXT(payload);
        }
        xSemaphoreGive(modem_mutex);
      }
      lastWebSocketTime = now;
    }

    if (xSemaphoreTake(modem_mutex, 0)) {
      bool gprsNow = modem.isGprsConnected();
      if (gprsNow != lastGprsConnected) {
        debugLog("[核心 1]", "4G 連線狀態", gprsNow ? "已連接" : "已斷開");
        lastGprsConnected = gprsNow;
      }
      xSemaphoreGive(modem_mutex);
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms 短 yield，不要長時間阻塞
  }
}

void setup() {
  Serial.begin(115200);
  
  // 建立互斥鎖用於保護 Modem/WebSocket 操作
  modem_mutex = xSemaphoreCreateMutex();
  data_mutex = xSemaphoreCreateMutex();
  
  pixels.begin();
  pixels.setBrightness(100); 
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); 
  pixels.show();
  
  SerialModem.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  SerialFC.begin(38400, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (radio.begin()) {
    Serial.println("✅ NRF24L01 啟動成功！");
    radio.setPALevel(RF24_PA_MAX); 
    radio.setDataRate(RF24_250KBPS); 
    radio.setChannel(108);
    radio.openReadingPipe(1, address);
    radio.enableAckPayload(); 
    radio.enableDynamicPayloads(); 
    
    txTelem.deviceType = 1;
    txTelem.nrfStatus = true;
    radio.writeAckPayload(1, &txTelem, sizeof(txTelem));
    radio.startListening();
  } else {
    Serial.println("⚠️ NRF24L01 找不到模組！");
  }

  Serial.println("正在初始化 4G 模組...");
  modem.init();
#ifdef HAS_TINY_GSM_SECURE
  secureClient.setInsecure();
  webSocket.setSocketClient(&secureClient);
#else
  debugLog("[核心 1]","TinyGSM","TinyGsmClientSecure.h not found — using fallback client");
#endif
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  // 🚀 在核心 1 啟動 4G/WebSocket 背景任務
  xTaskCreatePinnedToCore(
    core1_modem_task,      // 函數
    "ModemTask",           // 名稱
    4096,                  // 堆棧大小
    NULL,                  // 參數
    1,                     // 優先級
    NULL,                  // 任務句柄
    1                      // 核心 1
  );
}

void loop() {
  unsigned long now = millis();
  static bool lastNrfConnectedState = false;

  // --- [任務 1] 讀取 NRF24 遙控訊號 ---
  uint8_t pipeNum;
  if (radio.available(&pipeNum)) {
    radio.read(&rxData, sizeof(rxData));
    lastSignalTime = now;
    
    isNrfConnected = true; // 訊號進來了，代表連線中
    if (!lastNrfConnectedState) {
      debugLog("[核心 0]", "NRF", "收到訊號，連線恢復");
      lastNrfConnectedState = true;
    }
    if (isInFailsafe) {
      debugLog("[核心 0]", "Failsafe", "收到訊號，退出 Failsafe");
      isInFailsafe = false;
    }
    
    txTelem.voltage = drone_data.voltage;        
    txTelem.altitude = drone_data.altitude_m;    
    radio.writeAckPayload(1, &txTelem, sizeof(txTelem));

    msp_channels[0] = map(rxData.roll, 0, 255, 1000, 2000);
    msp_channels[1] = map(rxData.pitch, 0, 255, 1000, 2000);
    msp_channels[2] = map(rxData.yaw, 0, 255, 1000, 2000);
    msp_channels[3] = map(rxData.throttle, 0, 255, 1000, 2000);

    bool currentCh5 = (rxData.aux1 == 1);
    if (currentCh5 && !ch5_active) {
      // CH5 解鎖按下後，1 秒後啟動 CH8
      ch8_trigger_time = now + 1000;
      ch8_triggered = false;
    } else if (!currentCh5) {
      ch8_triggered = false;
      ch8_trigger_time = 0;
    }
    ch5_active = currentCh5;
    if (!ch8_triggered && currentCh5 && ch8_trigger_time != 0 && now >= ch8_trigger_time) {
      ch8_triggered = true;
    }

    msp_channels[4] = currentCh5 ? 2000 : 1000; 
    msp_channels[5] = (rxData.mode == 1) ? 2000 : 1000; 
    msp_channels[6] = (rxData.aux2 == 1) ? 2000 : 1000;
    msp_channels[7] = ch8_triggered ? 2000 : 1000;

    if (rxData.roll >= 120 && rxData.roll <= 135)   msp_channels[0] = 1500;
    if (rxData.pitch >= 120 && rxData.pitch <= 135) msp_channels[1] = 1500;
    if (rxData.yaw >= 120 && rxData.yaw <= 135)     msp_channels[2] = 1500;
    // 否則（isInFailsafe）：丟棄第一個 msp_channels 更新，堅決保持 msp_channels[3]=800 一直發送
  }

  // --- [任務 2] 以 50Hz 發送控制訊號與斷訊判斷 ---
  if (now - lastRcTime >= 20) {
    // ★ 接收機自己判斷無線訊號超時 ★
    if (now - lastSignalTime > 1000) {
      // 超過 1 秒沒收到 NRF 訊號 → 判定無線信號丟失
      if (!isInFailsafe) {
        debugLog("[核心 0]", "Failsafe", "無線訊號逾時 (>1s)，立即啟動Failsafe");
        isInFailsafe = true;  // 記錄 Failsafe 狀態
      }
      isNrfConnected = false;
      lastNrfConnectedState = false;
      msp_channels[0] = 1500;  // 橫滾中位
      msp_channels[1] = 1500;  // 俯仰中位
      msp_channels[2] = 1500;  // 偏航中位
      msp_channels[3] = 800;   // 🚨 油門拉到安全值 800 (並持續鎖定此值)
      msp_channels[7] = 1000;  // CH8 延遲觸發歸零
      ch8_triggered = false;
      ch8_trigger_time = 0;
    } else {
      // ★ 信號已恢復，退出 Failsafe ★
      if (isInFailsafe) {
        debugLog("[核心 0]", "Failsafe", "訊號已恢復，退出 Failsafe");
        isInFailsafe = false;
      }
    }
    
    sendVirtualRC();
    lastRcTime = now;
  }

  // --- [任務 3] 完全移除 - 4G/WebSocket 現在在核心 1 獨立執行 ---
  // ❌ 不要在這裡呼叫 websocket.loop()，會導致竞態條件！只在核心 1 呼叫
  
  // ❌ 已移到核心 1，此處保持空白

  // --- [任務 4] 非阻塞式 MSP 請求狀態機 ---
  if (now - lastMspReqTime >= 1000) {
    mspStep = 1; 
    lastMspReqTime = now;
  }
  if (mspStep == 1) {
    sendMspRequest(106); mspStep = 2; mspDelayTimer = now;
  } else if (mspStep == 2 && (now - mspDelayTimer >= 5)) {
    sendMspRequest(108); mspStep = 3; mspDelayTimer = now;
  } else if (mspStep == 3 && (now - mspDelayTimer >= 5)) {
    sendMspRequest(110); mspStep = 4; mspDelayTimer = now;
  } else if (mspStep == 4 && (now - mspDelayTimer >= 5)) {
    sendMspRequest(109); mspStep = 0; 
  }

  handleMspResponse();

  // --- [任務 6] 🔍 新增：每秒在序列埠印出 NRF 連線狀態 + Failsafe狀態 ---
  if (now - lastDebugPrintTime >= 1000) {
    Serial.print("["); Serial.print(now); Serial.print("] ");
    Serial.print("📊 [系統監控] ");
    if (isInFailsafe) {
      Serial.print("🚨 Failsafe 中 (Thr=800) | 無訊號已逾時 ");
      Serial.print((now - lastSignalTime)/1000); Serial.print(" 秒");
    } else {
      Serial.print("✅ 正常 | NRF 間隔 ");
      Serial.print(now - lastSignalTime); Serial.print(" ms");
    }
    Serial.println();
    lastDebugPrintTime = now;
  }

  // --- [任務 7] 🌈 RGB 戰術狀態指示燈 (安全存取狀態) ---
  bool websocket_connected_now = webSocketConnectedState;
  bool modem_connected_now = gprsConnectedState;
  
  if (isInFailsafe) {
    // Failsafe: 紅色快速閃爍 (50ms 週期)
    if ((now / 50) % 2 == 0) pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
    else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  } else if (!isNrfConnected) {
    // 訊號丟失: 紅色快速閃爍
    if ((now / 50) % 2 == 0) pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
    else pixels.setPixelColor(0, pixels.Color(0, 0, 0));   
  } else if (!modem_connected_now || !websocket_connected_now) {
    // 網絡斷線: 黃色緩慢閃爍
    if ((now / 500) % 2 == 0) pixels.setPixelColor(0, pixels.Color(255, 200, 0)); 
    else pixels.setPixelColor(0, pixels.Color(0, 0, 0));     
  } else {
    // 正常: 綠色 (或呼吸火紫)
    if (rxData.aux1 == 1) { 
      int breath = (now / 5) % 510;
      if (breath > 255) breath = 510 - breath;
      pixels.setPixelColor(0, pixels.Color(breath, 0, breath)); 
    } else {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
    }
  }
  pixels.show(); 
}

// ==========================================
// 底層通訊函式區 (維持不變)
// ==========================================
void sendVirtualRC() {
  uint8_t payloadSize = 16;
  uint8_t cmd = 200; 
  SerialFC.write('$'); SerialFC.write('M'); SerialFC.write('<');
  SerialFC.write(payloadSize); SerialFC.write(cmd);
  uint8_t checksum = payloadSize ^ cmd;
  for (int i = 0; i < 8; i++) {
    uint8_t lowByte = msp_channels[i] & 0xFF;
    uint8_t highByte = (msp_channels[i] >> 8) & 0xFF;
    SerialFC.write(lowByte); SerialFC.write(highByte);
    checksum ^= lowByte; checksum ^= highByte;
  }
  SerialFC.write(checksum);
}

void sendMspRequest(uint8_t cmd) {
  uint8_t request[] = {'$', 'M', '<', 0, cmd, cmd};
  SerialFC.write(request, sizeof(request));
}

void handleMspResponse() {
  while (SerialFC.available()) {
    uint8_t c = SerialFC.read();
    switch (msp_parse_state) {
      case MSP_SYNC1:
        if (c == '$') msp_parse_state = MSP_SYNC2;
        break;
      case MSP_SYNC2:
        if (c == 'M') msp_parse_state = MSP_DIR;
        else if (c == '$') msp_parse_state = MSP_SYNC2;
        else msp_parse_state = MSP_SYNC1;
        break;
      case MSP_DIR:
        if (c == '>') msp_parse_state = MSP_SIZE;
        else if (c == '$') msp_parse_state = MSP_SYNC2;
        else msp_parse_state = MSP_SYNC1;
        break;
      case MSP_SIZE:
        msp_rx_size = c;
        msp_rx_checksum = c;
        msp_rx_index = 0;
        if (msp_rx_size > sizeof(msp_rx_payload)) {
          msp_parse_state = MSP_SYNC1;
        } else {
          msp_parse_state = MSP_CMD;
        }
        break;
      case MSP_CMD:
        msp_rx_cmd = c;
        msp_rx_checksum ^= c;
        if (msp_rx_size == 0) {
          msp_parse_state = MSP_CHECKSUM;
        } else {
          msp_parse_state = MSP_PAYLOAD;
        }
        break;
      case MSP_PAYLOAD:
        msp_rx_payload[msp_rx_index++] = c;
        msp_rx_checksum ^= c;
        if (msp_rx_index >= msp_rx_size) {
          msp_parse_state = MSP_CHECKSUM;
        }
        break;
      case MSP_CHECKSUM:
        if (c == msp_rx_checksum) {
          if (msp_rx_cmd == 106 && msp_rx_size >= 14) {
            drone_data.fix    = msp_rx_payload[0];
            drone_data.numSat = msp_rx_payload[1];
            drone_data.lat    = (int32_t)((uint32_t)msp_rx_payload[5] << 24 | (uint32_t)msp_rx_payload[4] << 16 | (uint32_t)msp_rx_payload[3] << 8 | msp_rx_payload[2]);
            drone_data.lon    = (int32_t)((uint32_t)msp_rx_payload[9] << 24 | (uint32_t)msp_rx_payload[8] << 16 | (uint32_t)msp_rx_payload[7] << 8 | msp_rx_payload[6]);
          } else if (msp_rx_cmd == 108 && msp_rx_size == 6) {
            drone_data.roll  = ((int16_t)(msp_rx_payload[1] << 8 | msp_rx_payload[0])) / 10.0;
            drone_data.pitch = ((int16_t)(msp_rx_payload[3] << 8 | msp_rx_payload[2])) / 10.0;
            drone_data.yaw   = ((int16_t)(msp_rx_payload[5] << 8 | msp_rx_payload[4])) / 10.0;
          } else if (msp_rx_cmd == 110 && msp_rx_size >= 7) {
            drone_data.voltage = msp_rx_payload[0] / 10.0;
          } else if (msp_rx_cmd == 109 && msp_rx_size >= 6) {
            int32_t alt_cm = (int32_t)((uint32_t)msp_rx_payload[3] << 24 | (uint32_t)msp_rx_payload[2] << 16 | (uint32_t)msp_rx_payload[1] << 8 | msp_rx_payload[0]);
            int16_t vario_cms = (int16_t)(msp_rx_payload[5] << 8 | msp_rx_payload[4]);
            drone_data.altitude_m = alt_cm / 100.0;
            drone_data.vertical_speed = vario_cms / 100.0;
          }
        }
        msp_parse_state = MSP_SYNC1;
        break;
    }
  }
}
