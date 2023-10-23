#include <Arduino.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include "ESPAsyncWebServer.h"
// WiFiAPClass WiFiAP;
AsyncWebServer server(1234);
AsyncWebSocket ws("/");
// Set these to your desired credentials.
const char *ssid = "kun";
const char *password = "0123456789";

uint8_t x=31;
IPAddress local_IP(192, 168, x, 1);
IPAddress gateway(192, 168, x, 1);
IPAddress subnet(255, 255, 255, 0);

// void WiFiEvent(WiFiEvent_t event)
// {
//     switch(event) {
//       case SYSTEM_EVENT_STA_GOT_IP:
//           Serial.println("Station Connected to AP and assigned IP!");
//           break;
//       case SYSTEM_EVENT_STA_DISCONNECTED:
//           Serial.println("Station Disconnected from AP");
//           break;
//       default: break;
//   }
// }
void WiFiEvent(WiFiEvent_t event){
  Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
      case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
          Serial.println("Station connected to AP");
          break;
      case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
          Serial.println("Station disconnected from AP");
          break;
      default: break;
  }
}
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.println("Client connection received");
  } else if(type == WS_EVT_DISCONNECT){
    ws.cleanupClients();
    Serial.println("Client disconnected");
  } else if(type == WS_EVT_DATA){
    // Get the incoming data string
    String msg = "";
    for(int i=0; i<len; i++) {
      msg += (char)data[i];
    }
    // Print it to serial monitor
    Serial.println(msg);
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAPConfig(local_IP,gateway,subnet);
  WiFi.softAP(ssid, password);

  WiFi.onEvent(WiFiEvent);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  // Start server
  server.begin();

  Serial.println("Server started");
}

void loop() {
   // Clean up clients

  // Check if there are any new clients
  if(Serial.available()){
    String text = Serial.readString();
    text.trim(); //remove trailing whitespace
    //broadcast to all connected clients
    ws.textAll(text);
  }
}
