//ws_handler.h
#ifndef WS_HANDLER_H
#define WS_HANDLER_H
#include <Arduino.h>
#include <AsyncWebSocket.h>
void handleWebSocketMessage(AsyncWebSocketClient* client, String msg);
void sendRealtimeData();  // Untuk update data real-time
#endif
