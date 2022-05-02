#include <NTPClient.h>

// Look for all "REPLACEME" before uploading the code.
#include <Stepper.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "REPLACEME"
#define WIFI_PASS "REPLACEME"

#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "REPLACEME"
#define MQTT_PASS "REPLACEME"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800,60000);

//Set up MQTT and WiFi clients
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

//Set up the feed you're subscribing to
Adafruit_MQTT_Subscribe onoff = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/PET FEEDER");
boolean feed = true; // condition for alarm

// stepper
const int steps = 200; //REPLACEME this is the number of steps of the motor for a 360° rotation.
const int stepsPerDose = 50; //REPLACEME as you wish, mine was perfect at about 45-50 steps
Stepper myStepper(steps, D1, D3, D2, D4); // you may want to REPLACEME this based on how you cabled the motor.
int enA = D5;
int enB = D6;
//int motorPower = 990; // legacy.. for using pwm

// ultrasonic
long t;
int trigger = D8;
int echo = D7;
float distance;
float percentageFood;
float max_food = 23.50;  // REPLACEME in cm? seems to be "about" right

// Button
const int buttonPin = 3;     // number of the pushbutton pin (RX, cause no other IO was available)


void setup() {
  // Serial setup
  Serial.begin(115200);
  
  // pins setup
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LEDs  pin as an output
  pinMode(2, OUTPUT); // ^ other led
    
  pinMode(buttonPin, INPUT_PULLUP);  // initialize the pushbutton pin as an input:
  
  // stepper speed
  myStepper.setSpeed(55);

  // OTA setup
  ArduinoOTA.setHostname(SENSORNAME);
  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setPassword((const char *)OTApassword);
  ArduinoOTA.begin();
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // bit more power hungry, but seems stable.
  WiFi.hostname("PetFeeder"); // This will (probably) happear on your router somewhere.
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
/********************************** START CALLBACK*****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  char message[length];
  for (int i = 0; i < length; i++) {
    message[i] = ((char)payload[i]);
    
  }
  message[length] = '\0';
  Serial.print("[");
  Serial.print(message);
  Serial.print("]");
  Serial.println();

  if (strcmp(message,"feed") == 0) {
    Serial.print("Feeding cats...");
    Serial.println();
    feedCats();    
  } else {
    Serial.print("Unknown Message");
    Serial.println();
  }
}

// feeds cats
void feedCats() {
  digitalWrite(2, LOW); // Turn on onboard LED
  digitalWrite(enA, HIGH);  // Enable motors, i dont see the point in pwm with a stepper?
  digitalWrite(enB, HIGH);
  myStepper.step(stepsPerDose);
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  delay(2000); // you may wanna change this based on how many times you press te button continously 
  timeClient.update();   // could this fail?
  String formattedTime = timeClient.getFormattedDate();
  char charBuf[20];
  formattedTime.toCharArray(charBuf, 20);
  client.publish(lastfed_topic, charBuf, true); // Publishing time of feeding to MQTT Sensor retain true
  Serial.print("Fed at: ");
  Serial.print(charBuf);
  Serial.println();
  digitalWrite(2, HIGH); // Turn off onboard LED
  calcRemainingFood();
}

// calc remaining food in %
void calcRemainingFood() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  t = (pulseIn(echo, HIGH) / 2);
  if (t == 0.00) {
    Serial.println("Failed to read from SR02");
    delay(1000);
    return;
  }
  distance = float(t * 0.0343);
  //Serial.println(distance);
  //Serial.println(t);
  percentageFood = (100 - ((100 / max_food) * distance));
  if (percentageFood < 0.00) {
    percentageFood = 0.00;
  }
  Serial.print("Remaining food:\t");
  Serial.print(percentageFood);
  Serial.println(" %");
  char charBuf[6];
  int ret = snprintf(charBuf, sizeof charBuf, "%f", percentageFood);  // Translate float to char before publishing...
  client.publish(remaining_topic, charBuf, true); // Publishing remaining food to MQTT Sensor retain true
  delay(500);
}

void reconnect() {
  // Loop until we're reconnected, i may wanna check for pushbutton here somewhere in case of wifi disaster?
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_username, mqtt_password, willTopic, 1, true, "Offline")) {
      client.publish(willTopic,"Online", true);
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe(feed_topic);;
    }     else {
      Serial.print("MQTT failed");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  // Check for buttonpin push
  if (digitalRead(buttonPin) == LOW) {       
    Serial.println("Button pushed, feeding cats...");
    feedCats();
  }
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  delay(100);
}