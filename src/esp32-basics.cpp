// =====================================================================================================================
//
// esp32-basics
//
//    - SPDX-License-Identifier: GPL-2.0-only
//
// =====================================================================================================================
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <BMP280.h>

// baud rate used for arduino ide serial monitor
static const uint32_t SERIAL_MONITOR_BAUD = 115200;

// pin definitions for 'GOOUUU-ESP32-C3'
static const uint8_t PIN_BOOT_BUTTON   = 9;  // boot button right of usb port
static const uint8_t PIN_RGB_LED_RED   = 3;  // rgb led - red   [low active]
static const uint8_t PIN_RGB_LED_GREEN = 4;  // rgb led - green [low active]
static const uint8_t PIN_RGB_LED_BLUE  = 5;  // rgb led -  blue [low active]
static const uint8_t PIN_TX_LED_BLUE   = 21; // blue tx led     [high active]

// mqtt definitions
static const uint32_t MQTT_CONNECT_INTERVAL        = 10000;
static const char*    MQTT_SERVER_ADDRESS          = "test.mosquitto.org";
static const uint16_t MQTT_SERVER_PORT             = 1883;
static const char*    MQTT_PUB_TOPIC_TEMPERATURE   = "/Temperature";
static const char*    MQTT_PUB_TOPIC_PRESSURE      = "/Pressure";
static const char*    MQTT_SUB_TOPIC_RGB_LED_RED   = "/RGB-Led/Red";
static const char*    MQTT_SUB_TOPIC_RGB_LED_GREEN = "/RGB-Led/Green";
static const char*    MQTT_SUB_TOPIC_RGB_LED_BLUE  = "/RGB-Led/Blue";

// loop interval
static const uint32_t LOOP_INTERVAL             = 250; // main loop interval in ms
static const uint32_t LOOP_COUNTER_ACTION_VALUE = 0;   // counter value for serial output and mqtt actions
static const uint32_t LOOP_COUNTER_RESET_VALUE  = 240; // interval of loop actions = LOOP_COUNTER_RESET_VALUE*LOOP_INTERVAL;

// function declarations
void gpio_setup();
void i2c_setup();
void wifi_setup();
void mqtt_setup();
void mqtt_reconnect();
void mqtt_publish(int8_t temperature, uint32_t pressure);
void mqtt_subscribe_callback(char* topic, uint8_t* payload, unsigned int length);

// global variable definitions
static uint64_t loop_counter = 0;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static BMP280 bmp280(0x76); // I2C address 0x76



// =====================================================================================================================
// function: setup()
// =====================================================================================================================
void setup() {

  // setup serial monitor and add setup delay to allow
  // the press of boot button for wifi reset during start
  Serial.begin(SERIAL_MONITOR_BAUD);
  Serial.println();
  Serial.println("################################################################################");
  Serial.println("Starting setup in 3s");
  delay(3000);
  Serial.print("Setting up ");
  Serial.println(ARDUINO_VARIANT);

  // setup pins
  gpio_setup();

  // setup i2c
  i2c_setup();

  // setup wifi
  wifi_setup();

  // setup mqtt
  mqtt_setup();

  Serial.println("Finished setup");
}



// =====================================================================================================================
// function: loop()
// =====================================================================================================================
void loop() {

  if(loop_counter == LOOP_COUNTER_ACTION_VALUE){

    // esp32 internal mcu temperature
    int8_t mcu_temperature = (int8_t)temperatureRead();
    //Get pressure value
    uint32_t bmp280_pressure = bmp280.getPressure();
    int8_t bmp280_temperature = (int8_t)bmp280.getTemperature();

    // send empty line for separation
    Serial.println();

    // send status of wifi connection to serial monitor
    if( WiFi.isConnected() ) {
      Serial.print(WiFi.getHostname());
      Serial.print(" connected to ");
      Serial.print(WiFi.SSID());
      Serial.print(" with ip ");
      Serial.print(WiFi.localIP());
      Serial.print(" (rssi: ");
      Serial.print(WiFi.RSSI());
      Serial.println(")");
    } else {
      Serial.println("WiFi NOT connected");
    }

    // send temperature to serial monitor
    Serial.print("MCU temperature: ");
    Serial.print(mcu_temperature);
    Serial.println("°C");

    Serial.print("BMP280 temperature: ");
    Serial.print(bmp280_temperature);
    Serial.print("°C\tpressure: ");
    Serial.print(bmp280_pressure);
    Serial.println("Pa");

    // reconnect mqtt if needed and publish temperature
    if(!mqttClient.connected()){
      mqtt_reconnect();
    }
    mqtt_publish(bmp280_temperature, bmp280_pressure);
  }

  // run mqtt client loop in high frequency to instantly handle incoming messages
  mqttClient.loop();

  // increment loop counter and reset as specified
  loop_counter++;
  if(loop_counter >= LOOP_COUNTER_RESET_VALUE) loop_counter = 0;
  // wait time according to loop interval
  delay(LOOP_INTERVAL);
}



// =====================================================================================================================
// function: gpio_setup()
// =====================================================================================================================
void gpio_setup(){

  // Setup for ESP32 C3 Dev Board
  if(0==strcmp(ARDUINO_VARIANT, "esp32c3")){

    // setup boot button pin as input using internal pullup
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

    // setup rgb led pins and set initially high to turn led off
    pinMode(PIN_RGB_LED_RED, OUTPUT);
    digitalWrite(PIN_RGB_LED_RED, HIGH);

    pinMode(PIN_RGB_LED_GREEN, OUTPUT);
    digitalWrite(PIN_RGB_LED_GREEN, HIGH);

    pinMode(PIN_RGB_LED_BLUE, OUTPUT);
    digitalWrite(PIN_RGB_LED_BLUE, HIGH);

  // Setup for DOIT ESP32 DevKit V1
  } else if(0==strcmp(ARDUINO_VARIANT, "doitESP32devkitV1")){


  }
}



// =====================================================================================================================
// function: i2c_setup()
// =====================================================================================================================
void i2c_setup(){

  // join i2c bus
  Wire.begin();

  // sensor setup
  bmp280.begin();
}


// =====================================================================================================================
// function: wifi_setup()
// =====================================================================================================================
void wifi_setup(){

  // set station mode - do not act as access point
  WiFi.mode(WIFI_STA);

  // create instance of wifi manager
  WiFiManager wm;

  // reset wifi manager configuration if boot button is pressed on startup
  // to allow this to happen within the right time a bootup delay was added
  // in setup()
  if(LOW == digitalRead(PIN_BOOT_BUTTON)) {
    Serial.println("Resetting wifi manager configuration");
    wm.resetSettings();
  }

  // connect to stored network or launch wifi manager
  if(!wm.autoConnect(WiFi.getHostname(), "config-pw") ){
    Serial.println("Failed to connect to wifi");
  } else {
    Serial.println("WiFi connection established");
  }
}



// =====================================================================================================================
// function: mqtt_setup()
// =====================================================================================================================
void mqtt_setup(){

  mqttClient.setServer(MQTT_SERVER_ADDRESS, MQTT_SERVER_PORT);
  mqttClient.setCallback(mqtt_subscribe_callback);
}



// =====================================================================================================================
// function: mqtt_reconnect()
// =====================================================================================================================
void mqtt_reconnect(){

  do {
    if(!mqttClient.connect(WiFi.getHostname())){
      Serial.println("Failed to connect to MQTT server");
      delay(MQTT_CONNECT_INTERVAL);
    } else {
      Serial.println("MQTT server connection established");

      char szMqttTopic[128];
      const char* aszRGBLedTopics[3] = { MQTT_SUB_TOPIC_RGB_LED_RED, MQTT_SUB_TOPIC_RGB_LED_GREEN, MQTT_SUB_TOPIC_RGB_LED_BLUE };
      for(unsigned int i=0; i<3; i++)
      {
        snprintf(szMqttTopic, sizeof(szMqttTopic), "%s%s", WiFi.getHostname(), aszRGBLedTopics[i]);
        if(!mqttClient.subscribe(szMqttTopic)){
          Serial.print("Failed to subscribe to topic ");
          Serial.println(szMqttTopic);
        }
      }
    }
  } while(!mqttClient.connected());
}



// =====================================================================================================================
// function: mqtt_publish()
// =====================================================================================================================
void mqtt_publish(int8_t temperature, uint32_t pressure){

  char szMqttTopic[128];
  snprintf(szMqttTopic, sizeof(szMqttTopic), "%s%s", WiFi.getHostname(), MQTT_PUB_TOPIC_TEMPERATURE);

  char szTemperature[4];
  snprintf(szTemperature, sizeof(szTemperature), "%hhi", temperature);

  if(!mqttClient.publish(szMqttTopic, szTemperature)) {
    Serial.println("Failed to publish temperature");
  }

  snprintf(szMqttTopic, sizeof(szMqttTopic), "%s%s", WiFi.getHostname(), MQTT_PUB_TOPIC_PRESSURE);

  char szPressure[8];
  snprintf(szPressure, sizeof(szPressure), "%u", pressure);

  if(!mqttClient.publish(szMqttTopic, szPressure)) {
    Serial.println("Failed to publish pressure");
  }
}



// =====================================================================================================================
// function: mqtt_subscribe_callback()
// =====================================================================================================================
void mqtt_subscribe_callback(char* topic, uint8_t* payload, unsigned int length){

  Serial.print(topic);
  Serial.print(" => ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}



// =====================================================================================================================
// EOF - end of file
// =====================================================================================================================
