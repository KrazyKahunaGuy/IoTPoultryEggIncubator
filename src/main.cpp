#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include "DHTesp.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

/**** GPIO Layout ****/
// DHT11
#define DHTpin D2 // Set DHT pin as GPIO16
DHTesp dht;
// Heater
const int heaterRelayPin = D8; // Set heater relay pin as GPIO0
// Heater relay state
bool heaterRelayState = false; // Initialize the heater relay state

// Cooling fan
const int fanRelayPin = D9; // Set heater relay pin as GPIO2
// Cooling fan relay state
bool fanRelayState = false; // Initialize the cooling fan relay state

// Humidifier
const int humidifierPin = D7; // Set the humidifier pin as GPIO13
// Humidifier state
bool humidifierState = false; // Initialize the humidifer state

// Water level sensor
const int waterLevelSensorPin = A0;       // Set the water level sensor pin as A0
const int waterLevelSensorPowerPin = D10; // Set the water level sensor power pin
String waterLevelState = "0";             // Initialize the water level underflow state

// PIR motion sensor
const int motionSensorPin = D6; // Set the motion sensor pin to GPIO12
String motionSensorState = "0"; // Initialize the motion sensor state

// DC motor
const int dcMotorRelayPin = D5; // Set the dc motor relay pin to GPIO14
// DC motor relay state
bool dcMotorRelayState = false; // Initialize the dc motor relay state
/**** GPIO Layout ****/

/**** WiFi connection details ****/
const char *SSID = "ARES I";        // SSID for WiFi
const char *PASSWORD = "yamato111"; // Password for the WiFi
/**** WiFi connection details ****/

/**** MQTT broker connection details ****/
const char *MQTT_SERVER = "90a813dceaa84eec9250eb05ad74c892.s2.eu.hivemq.cloud"; // URL for the MQTT broker
const char *MQTT_USERNAME = "biteater";                                          // MQTT broker username
const char *MQTT_PASSWORD = "password";                                          // MQTT broker password
const int MQTT_PORT = 8883;
/**** MQTT broker connection details ****/ // MQTT broker port number

/**** Secure WiFi connectivity initialisation ****/
WiFiClientSecure espClient;
/**** Secure WiFi connectivity initialisation ****/

/**** MQTT client initialisation using WiFi connection ****/
PubSubClient mqttClient(espClient);
/**** MQTT client initialisation using WiFi connection ****/

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/**** Topic(s) ****/
const char *TOPIC = "IoTEggIncubatorActuatorControl";
/**** Topic(s) ****/

/**** Set root certificate ****/
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";
/**** Set root certificate ****/

/**** Function to handle fan relay state ****/
void setFanRelayState(bool state)
{
  // Set the cooling fan relay state and perform necessary actions
  fanRelayState = state;

  // Fan relay control logic
  if (state)
  {
    digitalWrite(fanRelayPin, LOW);
  }
  else
  {
    digitalWrite(fanRelayPin, HIGH);
  }
}
/**** Function to handle fan relay state ****/

/**** Function to handle heater relay state ****/
void setHeaterRelayState(bool state)
{
  // Set the heater relay state and perform necessary actions
  heaterRelayState = state;

  // Heater relay control logic
  if (state)
  {
    digitalWrite(heaterRelayPin, LOW);
  }
  else
  {
    digitalWrite(heaterRelayPin, HIGH);
  }
}
/**** Function to handle heater relay state ****/

/**** Function to handle humidifier state ****/
void setHumidifierState(bool state)
{
  // Set the humidifier state and perform necessary actions
  humidifierState = state;

  // Humidifier control logic
  digitalWrite(humidifierPin, state);
}
/**** Function to handle humidifier state ****/

/**** Function to handle motion sensor state ****/
void setMotionSensorState(bool state)
{
  // Set the motion sensor state and perform necessary actions
  motionSensorState = state;

  // Motion sensor control logic
  digitalWrite(motionSensorPin, state);
}
/**** Function to handle motion sensor state ****/

/**** Function to handle dc motor relay state ****/
void setMotorRelayState(bool state)
{
  // Set the dc motor relay state and perform necessary actions
  dcMotorRelayState = state;

  // DC motor relay control logic
  if (state)
  {
    digitalWrite(dcMotorRelayPin, LOW);
  }
  else
  {
    digitalWrite(dcMotorRelayPin, HIGH);
  }
}
/**** Function to handle dc motor relay state ****/

/**** Connect to the MQTT broker ****/
void reconnect()
{
  // Loop until we're reconnected to the MQTT broker
  while (!mqttClient.connected())
  {
    Serial.println("Attempting MQTT connection...");

    String clientID = "IoTPoultryEggIncubator-"; // Create a random client ID
    clientID += String(random(0xffff), HEX);
    // Attempt to connect to the MQTT broker
    if (mqttClient.connect(clientID.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("Connected to MQTT broker");

      // Subscribe to the topic(s) here
      mqttClient.subscribe(TOPIC);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" .Retrying in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
/**** Connect to the MQTT broker ****/

/**** Callback method for receiving MQTT messages and switching Heater ****/
void callback(char *topic, byte *payload, unsigned int length)
{
  // Create a buffer and copy the payload data
  char buffer[length + 1];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';

  // Parse the received JSON
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buffer);

  // Check if parsing succeded
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Access JSON values and perform conditionals
  if (doc.containsKey("heaterState"))
  {
    String docHeaterState = doc["heaterState"];

    if (docHeaterState.equals("1") && !heaterRelayState)
    {
      setHeaterRelayState(true);
    }
    else if (docHeaterState.equals("0") && heaterRelayState)
    {
      setHeaterRelayState(false);
    }
  }
  if (doc.containsKey("coolingFanState"))
  {
    String docFanState = doc["coolingFanState"];

    if (docFanState.equals("1") && !fanRelayState)
    {
      setFanRelayState(true);
    }
    else if (docFanState.equals("0") && fanRelayState)
    {
      setFanRelayState(false);
    }
  }
  if (doc.containsKey("humidifierState"))
  {
    String docHumidifierState = doc["humidifierState"];
    if (docHumidifierState.equals("1") && !humidifierState)
    {
      setHumidifierState(true);
    }
    else if (docHumidifierState.equals("0") && humidifierState)
    {
      setHumidifierState(false);
    }
  }
  if (doc.containsKey("motorState"))
  {
    String docMotorState = doc["motorState"];
    if (docMotorState.equals("1") && !dcMotorRelayState)
    {
      setMotorRelayState(true);
    }
    else if (docMotorState.equals("0") && dcMotorRelayState)
    {
      setMotorRelayState(false);
    }
  }
}
/**** Callback method for receiving MQTT messages and switching Heater ****/

/**** Method for publishing MQTT messages ****/
void publishMessage(const char *topic, String payload, boolean retained)
{
  if (mqttClient.publish(topic, payload.c_str(), true))
  {
    Serial.println("Message published [" + String(topic) + "]: " + payload);
  }
}
/**** Method for publishing MQTT messages ****/

/**** Setup ****/
void setup()
{
  // Put your setup code here, to run once:
  Serial.begin(115200);

  // Set up sensors and actuators
  // Set up DHT11 sensor
  dht.setup(DHTpin, DHTesp::DHT11);
  // Set up heater relay module as output
  pinMode(heaterRelayPin, OUTPUT);
  // Set up fan relay module as output
  pinMode(fanRelayPin, OUTPUT);
  // Set up humidifier relay module as output
  pinMode(humidifierPin, OUTPUT);
  // Set up water level sensor as output
  pinMode(waterLevelSensorPowerPin, OUTPUT);
  // Set up DC motor relay module as output
  pinMode(dcMotorRelayPin, OUTPUT);
  // Set up PIR as input
  pinMode(motionSensorPin, INPUT);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(SSID);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("WiFi connected!");

  // Set initial heater relay state
  setHeaterRelayState(false);
  // Set initial fan relay state
  setFanRelayState(false);
  // Set initial humidifier state
  setHumidifierState(false);
  // Set initial DC motor relay state
  setMotorRelayState(false);
  // Set initial state for water level sensor power pin
  digitalWrite(waterLevelSensorPowerPin, LOW);

#ifdef ESP8266
  espClient.setInsecure();
#else
  espClient.setCACert(root_ca); // Enable this line and the "certificate" code for secure connection
#endif

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);
}
/**** Setup ****/

/**** Loop ****/
void loop()
{
  // Put your main code here, to run repeatedly:
  // Check if mqtt client is connected
  if (!mqttClient.connected())
  {
    reconnect();
  }

  // Maintain the MQTT connection and handle incoming messages
  mqttClient.loop();

  // Read DHT11 temperature and humidity reading
  delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  // Read water level sensor
  digitalWrite(waterLevelSensorPowerPin, HIGH);
  delay(10);
  int waterLevelSensorReading = analogRead(waterLevelSensorPin);
  digitalWrite(waterLevelSensorPowerPin, LOW);

  if (waterLevelSensorReading < 300)
  {
    Serial.println("Refill water trough");
    waterLevelState = "1";
  }
  else
  {
    waterLevelState = "0";
  }

  // Read motion sensor state
  if (digitalRead(motionSensorPin))
  {
    motionSensorState = "1";
    Serial.println("Motion detected");
  }
  else if (!digitalRead(motionSensorPin))
  {
    motionSensorState = "0";
  }

  DynamicJsonDocument doc(1024);

  doc["deviceId"] = "IoTEggIncubator";
  doc["humidity"] = humidity;
  doc["temperature"] = temperature;
  doc["motionSensor"] = motionSensorState;
  doc["waterLevel"] = waterLevelState;

  char mqtt_message[128];
  serializeJson(doc, mqtt_message);

  publishMessage("IoTEggIncubatorSensorArray", mqtt_message, true);
}
/**** Loop ****/
