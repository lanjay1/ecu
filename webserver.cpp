//webserver.cpp
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void initWebServer() {
  SPIFFS.begin();

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
      String msg = (char*)data;
      handleWebSocket(msg);  // Parsing JSON → kirim ke STM32
    }
  });

  server.addHandler(&ws);
  server.begin();
}
