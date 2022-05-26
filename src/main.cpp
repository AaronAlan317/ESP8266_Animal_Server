#include <Arduino.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <String>
#include "LittleFS.h"
#include <FS.h>
#include <LittleFS.h>
#include <StackThunk.h>

// ----------------------------------------------------------------------------
// Definition of macros
// ----------------------------------------------------------------------------
#define DHTPIN  D3
#define DHTTYPE DHT11
#define BTNPIN D1

// ----------------------------------------------------------------------------
// Definition of global constants
// ----------------------------------------------------------------------------
const uint8_t DEBOUNCE_DELAY = 10; // in milliseconds

// Network credentials
const char* ssid = "Camp Liberty 2G Cell";
const char* password = "Hernandez";

// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;    // will store last time DHT was updated

// Updates DHT readings every 10 seconds
const long interval = 10000;  

// Stores DHT values
float temp = 0.0;
float humdity = 0.0;

// Tracks state
String taskState = "off";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Creates DHT object
DHT dht(D3, DHT11);

// ----------------------------------------------------------------------------
// Definition of the Button component
// ----------------------------------------------------------------------------
struct Button {
    // state variables
    uint8_t  pin;
    bool     lastReading;
    uint32_t lastDebounceTime;
    uint16_t state;

    // methods determining the logical state of the button
    bool pressed()                { return state == 1; }
    bool released()               { return state == 0xffff; }
    bool held(uint16_t count = 0) { return state > 1 + count && state < 0xffff; }

    // method for reading the physical state of the button
    void read() {
        // reads the voltage on the pin connected to the button
        bool reading = digitalRead(pin);

        // if the logic level has changed since the last reading,
        // we reset the timer which counts down the necessary time
        // beyond which we can consider that the bouncing effect
        // has passed.
        if (reading != lastReading) {
            lastDebounceTime = millis();
        }

        // from the moment we're out of the bouncing phase
        // the actual status of the button can be determined
        if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
            // don't forget that the read pin is pulled-up
            bool pressed = reading == LOW;
            if (pressed) {
                     if (state  < 0xfffe) state++;
                else if (state == 0xfffe) state = 2;
            } else if (state) {
                state = state == 0xffff ? 0 : 0xffff;
            }
        }

        // finally, each new reading is saved
        lastReading = reading;
    }
};

Button button = { BTNPIN, HIGH, 0, 0 };

// ----------------------------------------------------------------------------
// LittleFS initialization
// ----------------------------------------------------------------------------
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("An Error has occurred while mounting LittleFS");
  }
}

// ----------------------------------------------------------------------------
// Connecting to the WiFi network
// ----------------------------------------------------------------------------
void initWiFi() {
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP("AaronESP1");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());
}

// ----------------------------------------------------------------------------
// Web server initialization
// ----------------------------------------------------------------------------
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return String(temp);
  }
  else if(var == "HUMIDITY"){
    return String(humdity);
  }
  else if(var =="STATE"){
    return taskState;
  }
  return String();
}

void initWebServer() {
    // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", String(), false, processor);
  });
  
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/style.css","text/css");
  });

  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/script.js","text/js");
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    String tmp = String(temp);
    request->send_P(200, "text/plain", tmp.c_str(),processor);
  });

  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    String tmp = String(humdity);
    request->send_P(200, "text/plain", tmp.c_str(),processor);
  });

  server.on("/on1", HTTP_GET, [](AsyncWebServerRequest *request){
    taskState = "on";
    request->send(LittleFS, "/index.html", String(),false, processor);
  });

  server.on("/off1", HTTP_GET, [](AsyncWebServerRequest *request){
    taskState = "off";
    request->send(LittleFS, "/index.html", String(),false, processor);
  });

  server.begin();
}

// ----------------------------------------------------------------------------
// WebSocket initialization
// ----------------------------------------------------------------------------
void notifyClients() {
    ws.textAll(taskState ? "on" : "off");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        if (strcmp((char*)data, "reset") == 0) {
            taskState = "off";
            notifyClients();
        }
    }
}

void onEvent(AsyncWebSocket       *server,  //
             AsyncWebSocketClient *client,  //
             AwsEventType          type,    // the signature of this function is defined
             void                 *arg,     // by the `AwsEventHandler` interface
             uint8_t              *data,    //
             size_t                len) { 
  switch (type) {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
  }  
}

void initWebSocket() {
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

// ----------------------------------------------------------------------------
// Utility functions 
// ----------------------------------------------------------------------------
void readSensors(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
    float newT = dht.readTemperature();
    if (isnan(newT)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      temp = newT;
      Serial.println(temp);
    }
    float newH = dht.readHumidity();
    if (isnan(newH)) {
      Serial.println("Failed to read from DHT sensor!");
    }
    else {
      humdity = newH;
      Serial.println(humdity);
    }
  }
}

void readButtons(){
  button.read();

  if (button.pressed()) {
    Serial.print("button pressed");
    notifyClients(); 
    taskState = "on";
  }
}
// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------
void setup(){
  Serial.begin(9600);

  dht.begin();

  pinMode(button.pin, INPUT);

  initLittleFS();
  initWiFi();
  initWebSocket();
  initWebServer();

}
 
// ----------------------------------------------------------------------------
// Main control loop
// ----------------------------------------------------------------------------
void loop(){  
  ws.cleanupClients();

  readButtons();

  readSensors();
  
}