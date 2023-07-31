#include <ESP8266WiFi.h>
#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <PubSubClient.h>
#include "ThingSpeak.h"

namespace defaults {
const char* board = "ESP8266";

inline namespace mq135 {
const char* type = "MQ-135";
const int voltage_resolution = 5;
const int adc_bit_resolution = 10;
const float ratio_mq135_clean_air = 3.6;  // RS / R0 = 3.6 ppm
}

inline namespace dht22 {
const uint8_t type = DHT22;
}
}

namespace config {
const char* wifi_ssid = "YOUR_WIFI_SSID";
const char* wifi_password = "YOUR_WIFI_PASSWORD";

const char* mqtt_broker = "YOUR_MQTT_BROKER_IP";  // MQTT server IP
const int mqtt_port = 1883; // MQTT server port
const char* mqtt_client_id = "YOUR_MQTT_CLIENT_ID";

const char* thingspeak_api_key = "YOUR_THINGSPEAK_API_KEY"; // Your ThingSpeak api key
const unsigned long thingspeak_channel = -1; // Your ThingSpeak channel id
}

namespace pins {
const int dht22_pin = D4;  // Digital input 4
const int mq135_pin = A0;  // Analog input 0
}

namespace topics {
const char* acetonTopic = "topic/aqi/aceton";
const char* alcoholTopic = "topic/aqi/alcohol";
const char* co2Topic = "topic/aqi/co2";
const char* coTopic = "topic/aqi/co";
const char* nh4Topic = "topic/aqi/nh4";
const char* toluenTopic = "topic/aqi/toluen";
const char* humidityTopic = "topic/aqi/humidity";
const char* temperatureTopic = "topic/aqi/temperature";
}

class SensorManager {
private:
  int mq135Pin;
  MQUnifiedsensor mq135Sensor;

  int dht22Pin;
  DHT dht22Sensor;

public:
  SensorManager(const char* board, int voltageResolution, int adc_bit_resolution, int mq135Pin, const char* mqType, int dht22Pin, uint8_t dhtType)
    : mq135Sensor(board, voltageResolution, adc_bit_resolution, mq135Pin, mqType), dht22Sensor(dht22Pin, dhtType) {}

  void initialize() {
    dht22Sensor.begin();

    mq135Sensor.setRegressionMethod(1);
    mq135Sensor.init();
    calibrateMq135();
  }

  void calibrateMq135() {
    Serial.print("Calibrating MQ-135, please wait...");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++) {
      mq135Sensor.update();
      calcR0 += mq135Sensor.calibrate(defaults::mq135::ratio_mq135_clean_air);
      Serial.print(".");
    }
    mq135Sensor.setR0(calcR0 / 10);
    Serial.println(" Calibration Done!");
  }

  void updateMq135() {
    mq135Sensor.update();
  }

  float readAceton() {
    mq135Sensor.setA(34.668);
    mq135Sensor.setB(-3.369);
    return mq135Sensor.readSensor();
  }

  float readAlcohol() {
    mq135Sensor.setA(77.255);
    mq135Sensor.setB(-3.18);
    return mq135Sensor.readSensor();
  }

  float readCo2() {
    mq135Sensor.setA(110.47);
    mq135Sensor.setB(-2.862);
    return mq135Sensor.readSensor();
  }

  float readCo() {
    mq135Sensor.setA(605.18);
    mq135Sensor.setB(-3.937);
    return mq135Sensor.readSensor();
  }

  float readNh4() {
    mq135Sensor.setA(102.2);
    mq135Sensor.setB(-2.473);
    return mq135Sensor.readSensor();
  }

  float readToluen() {
    mq135Sensor.setA(44.947);
    mq135Sensor.setB(-3.445);
    return mq135Sensor.readSensor();
  }

  float readTemperature() {
    return dht22Sensor.readTemperature();
  }

  float readHumidity() {
    return dht22Sensor.readHumidity();
  }
};

WiFiClient thingSpeakWifiClient;
WiFiClient pubSubWifiClient;
PubSubClient pubSubClient(pubSubWifiClient);

SensorManager sensorManager(defaults::board, defaults::mq135::voltage_resolution, defaults::mq135::adc_bit_resolution, pins::mq135_pin, defaults::mq135::type, pins::dht22_pin, defaults::dht22::type);

void connectWifi(const char* ssid, const char* password) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Connecting to %s \n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Wifi status enum: ");  // https://github.com/esp8266/Arduino/blob/da6ec83b5fdbd5b02f04cf143dcf8e158a8cfd36/cores/esp8266/wl_definitions.h#L50
      Serial.println(WiFi.status());
      delay(1000);
    }
    Serial.print("\nConnected as ");
    Serial.println(WiFi.localIP());
  }
}

void connectPubSubClient() {
  while (!pubSubClient.connected()) {
    Serial.printf("Connecting client %s to MQTT broker\n", config::mqtt_client_id);

    if (pubSubClient.connect(config::mqtt_client_id)) {
      Serial.println("Client connected to MQTT broker");
    } else {
      Serial.print("Failed to connect with MQTT broker with state: ");
      Serial.print(pubSubClient.state());
      Serial.print("\n");
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  connectWifi(config::wifi_ssid, config::wifi_password);

  pubSubClient.setServer(config::mqtt_broker, config::mqtt_port);
  ThingSpeak.begin(thingSpeakWifiClient);

  sensorManager.initialize();
}

void loop() {
  if (pubSubClient.connected()) {
    float aceton = sensorManager.readAceton();
    float alcohol = sensorManager.readAlcohol();
    float co2 = sensorManager.readCo2() + 400;  // 400 Offset for CO2, source: https://github.com/miguel5612/MQSensorsLib/issues/29
    float co = sensorManager.readCo();
    float nh4 = sensorManager.readNh4();
    float toluen = sensorManager.readToluen();
    float humidity = sensorManager.readHumidity();
    float temperature = sensorManager.readTemperature();

    pubSubClient.publish(topics::acetonTopic , String(aceton).c_str());
    pubSubClient.publish(topics::alcoholTopic, String(alcohol).c_str());
    pubSubClient.publish(topics::co2Topic, String(co2).c_str());
    pubSubClient.publish(topics::coTopic, String(co).c_str());
    pubSubClient.publish(topics::nh4Topic, String(nh4).c_str());
    pubSubClient.publish(topics::toluenTopic, String(toluen).c_str());
    pubSubClient.publish(topics::humidityTopic, String(humidity).c_str());
    pubSubClient.publish(topics::temperatureTopic, String(temperature).c_str());
    pubSubClient.loop();

    ThingSpeak.setField(1, aceton);
    ThingSpeak.setField(2, alcohol);
    ThingSpeak.setField(3, co2);
    ThingSpeak.setField(4, co);
    ThingSpeak.setField(5, nh4);
    ThingSpeak.setField(6, toluen);
    ThingSpeak.setField(7, humidity);
    ThingSpeak.setField(8, temperature);
    ThingSpeak.writeFields(config::thingspeak_channel, config::thingspeak_api_key);
  } else {
    connectPubSubClient();
  }

  delay(15000);
}

/*
  Error that i encountered:
  - https://github.com/knolleary/pubsubclient/issues/198
  - https://github.com/esp8266/Arduino/issues/7345
*/
