#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>

// Information about WiFi connection
constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

// Information about ThingsBoard connection
constexpr char TOKEN[] = "vv32bqb4djwi1474yvjr";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Shared attributes/attributes used in ThingsBoard
constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

// General state variables (volatile as they can be changed in the RPC callbacks)
volatile bool attributesChanged = false;    // flag to indicate that attributes were changed and need to be sent to ThingsBoard
volatile int ledMode = 0;                   // led mode
volatile bool ledState = false;             // led's current state

// Blinking interval limits and variable
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U; // default blinking interval in milliseconds

uint32_t previousStateChange;               // save the time of the previous LED state change

constexpr int16_t telemetrySendInterval = 10000U; // telemetry send interval in milliseconds
uint32_t previousDataSend;                        // save the time of the previous telemetry send

// List of shared attributes that device is interested in
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

// initialize WiFi, MQTT and ThingsBoard clients
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// Initialize DHT20 sensor
DHT20 dht20;

/*
  Hàm xử lý RPC từ ThingsBoard:
  - Tham số: data (RPC_Data) chứa payload từ server (ở đây mong là bool)
  - Hành động: bật/tắt LED theo giá trị nhận được, đánh dấu attributesChanged để gửi trạng thái trở lại
  - Trả về: RPC_Response dùng để phản hồi RPC
*/
RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    bool newState = data;
    Serial.print("Switch state change: ");
    Serial.println(newState);
    digitalWrite(LED_PIN, newState);
    attributesChanged = true; // Đánh dấu rằng thuộc tính đã thay đổi để gửi lại trạng thái mới
    // Trả về tên method và giá trị (khớp với tên RPC đã đăng ký)
    return RPC_Response("setValueButtonLED", newState);     
}

// Mảng đăng ký các callback RPC: tên method -> hàm xử lý
const std::array<RPC_Callback, 1U> callbacks = {
  // RPC_Callback{ "setLedSwitchValue", setLedSwitchState }
  RPC_Callback{ "setValueButtonLED", setLedSwitchState }
};

/*
  Xử lý Shared Attributes nhận được từ ThingsBoard:
  - Duyệt các key/value trong data
  - Nếu có blinkingInterval thì cập nhật (nếu nằm trong khoảng hợp lệ)
  - Nếu có ledState thì cập nhật trạng thái LED ngay lập tức
  - Đặt attributesChanged = true để gửi trạng thái hiện tại về server
*/
void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);    // áp dụng trạng thái LED nhận từ server
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true; // Đánh dấu rằng thuộc tính đã thay đổi để gửi lại trạng thái mới
}

// // Wrapper callback cho việc subscribe shared attributes và request ban đầu
const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

/*
  InitWiFi:
  - Bắt đầu kết nối WiFi (blocking): gọi WiFi.begin và chờ đến khi liên kết thành công.
  - Lưu ý: hàm này chặn chương trình nếu không thể kết nối.
*/
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/*
  reconnect:
  - Kiểm tra trạng thái WiFi; nếu chưa kết nối thì gọi InitWiFi()
  - Trả true khi xong (hàm hiện tại giả định InitWiFi thành công)
*/
const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}


/*
  setup:
  - Khởi tạo Serial, cấu hình chân LED, kết nối WiFi, khởi tạo I2C và cảm biến DHT20
*/
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
}

/*
  loop:
  - Kiểm tra kết nối WiFi/MQTT, kết nối tới ThingsBoard nếu cần
  - Khi kết nối thành công: gửi attribute macAddress, subscribe RPC và shared attributes, request giá trị shared attributes ban đầu
  - Nếu có thay đổi thuộc tính (attributesChanged) thì gửi trạng thái LED lên server
  - Định kỳ đọc DHT20 và gửi telemetry (temperature, humidity) cùng một số attribute WiFi
  - Gọi tb.loop() liên tục để xử lý message MQTT / callbacks
*/
void loop() {
  delay(10);

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }

    // Gửi attribute macAddress lên ThingsBoard
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

    // Đăng ký nhận RPC từ server
    Serial.println("Subscribing for RPC...");
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    // Đăng ký nhận cập nhật shared attributes
    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      return;
    }

    Serial.println("Subscribe done");

    // Yêu cầu giá trị shared attributes ban đầu từ server để đồng bộ trạng thái
    if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
      Serial.println("Failed to request for shared attributes");
      return;
    }
  }

  // Nếu có thay đổi thuộc tính (do RPC hoặc shared attributes), gửi trạng thái LED hiện tại
  if (attributesChanged) {
    attributesChanged = false;
    tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
  }

  // if (ledMode == 1 && millis() - previousStateChange > blinkingInterval) {
  //   previousStateChange = millis();
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   Serial.print("LED state changed to: ");
  //   Serial.println(!digitalRead(LED_PIN));
  // }

  // Gửi telemetry định kỳ (nhiệt độ, độ ẩm) và một số attribute WiFi
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();

    dht20.read();
    
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }

    // Gửi thêm các attribute liên quan tới WiFi để tiện giám sát
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
  }

  // Gọi tb.loop() liên tục để xử lý message MQTT / callbacks
  tb.loop();
}
