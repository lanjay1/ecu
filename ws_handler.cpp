//ws_handler.cpp
#include "ws_handler.h"
#include <ArduinoJson.h>
#include "spi_bridge.h"
#include <AsyncWebSocket.h>
#include "sensor_manager.h"

extern AsyncWebSocket ws;
extern uint16_t currentRPM;
extern uint8_t currentTPS;

void handleWebSocketMessage(AsyncWebSocketClient *client, String msg) {
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (error) {
        Serial.println("[WS] JSON parse failed");
        return;
    }

    const char* cmd = doc["cmd"];
    String response;

    if (strcmp(cmd, "GET_MAPS") == 0) {
        String json = "{";
        json += "\"fuel\": [[12.5,13.0,13.5],[13.2,13.7,14.0],[13.8,14.1,14.5]],";
        json += "\"ignition\": [[10,15,20],[12,18,22],[14,20,24]],";
        json += "\"afr\": [[14.7,14.2,13.8],[13.5,13.2,13.0],[12.8,12.5,12.2]]";
        json += "}";
        client->text(json);
    }
    

    else if (strcmp(cmd, "SET_MAPS") == 0) {
        JsonObject data = doc["data"];
        // Contoh log untuk validasi di serial monitor
        serializeJsonPretty(data, Serial);
        Serial.println("\n[WS] Received map data");
        client->text("{\"status\":\"Maps received\"}");
        // TODO: simpan ke SPI, Flash, atau RAM
    }
    if (strcmp(cmd, "RESET_PROFILE") == 0) {
    uint8_t profile = root["arg"] | 0;
    sendCommandToSTM32("RESET_PROFILE", profile);
}


    else if (strcmp(cmd, "GET_SETTINGS") == 0) {
        spi_send_json("GET_SETTINGS", "{}");
        JsonDocument resDoc;
        resDoc["type"] = "settings";
        resDoc["rpm_limit"] = 8000;
        serializeJson(resDoc, response);
        client->text(response);
    }

    else if (strcmp(cmd, "SET_TPS_MIN") == 0) {
        spi_send_json("SET_TPS_MIN", "{}");
        client->text("{\"status\":\"TPS min saved\"}");
    }

    else if (strcmp(cmd, "SET_TPS_MAX") == 0) {
        spi_send_json("SET_TPS_MAX", "{}");
        client->text("{\"status\":\"TPS max saved\"}");
    }

    else if (strcmp(cmd, "SET_SETTINGS") == 0) {
        String payload;
        serializeJson(doc["data"], payload);
        spi_send_json("SET_SETTINGS", payload);
        client->text("{\"status\":\"OK\"}");
    }
    else if (cmd == "get_config") {
    ECU_Config cfg;
    FlashConfig_Load(&cfg);
    snprintf(resp, sizeof(resp),
        "{\"rpm_limit\":%u,\"blip_duration\":%u,\"vva_mode\":%u}",
        cfg.rpm_limit, cfg.blip_duration, cfg.vva_mode);
    websocket_send(client, resp);
}
else if (cmd == "UPLOAD_FIRMWARE") {
    std::string base64data = json["data"];
    std::vector<uint8_t> firmware = decodeBase64(base64data);
    sendFirmwareToSTM32(firmware); // Fungsi kirim via UART atau SPI
    wsSend(ws, "Firmware dikirim ke STM32.");
}

else if (cmd == "set_config") {
    ECU_Config cfg;
    cfg.rpm_limit = json["rpm_limit"];
    cfg.blip_duration = json["blip_duration"];
    cfg.vva_mode = json["vva_mode"];
    FlashConfig_Save(&cfg);
    websocket_send(client, "{\"status\":\"ok\"}");
}


    else if (strcmp(cmd, "GET_TPS_STATUS") == 0) {
        spi_send_json("GET_TPS_STATUS", "{}");
        JsonDocument resDoc;
        resDoc["type"] = "tps_status";
        resDoc["voltage"] = 0.85;
        resDoc["min"] = 0.40;
        resDoc["max"] = 4.15;
        String res;
        serializeJson(resDoc, res);
        client->text(res);
    }

    else {
        client->text("{\"error\":\"Unknown command\"}");
    }
    else if (strcmp(cmd, "LOAD_PROFILE") == 0) {
    int index = doc["data"];
    ECU_RequestLoadProfile(index);  // panggil SPI/Serial ke STM32
    client->text("{\"status\":\"Load Profile requested\"}");
}
else if (strcmp(cmd, "GET_DIAGNOSTIC") == 0) {
    spi_send_json("GET_DIAGNOSTIC", "{}");
    client->text("{\"status\":\"Diag requested\"}");
}


}

void sendRealtimeData() {
  StaticJsonDocument<256> doc;
  doc["type"] = "realtime";
  doc["rpm"] = currentRPM;
  doc["tps"] = currentTPS;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}
void WebSocket_SendSensorData() {
    SensorData data = SensorManager_GetData();

    char json[512];
    snprintf(json, sizeof(json),
        "{\"rpm\":%u,\"tps\":%u,\"map\":%u,\"iat\":%u,\"ect\":%u,\"gear\":%d,\"speed\":%u,\"vva\":%d,\"lean_cut\":%d}",
        data.rpm,
        data.tps,
        data.map,
        data.iat,
        data.ect,
        data.gear,
        data.speed,
        VVA_getState(),
        LeanAngle_isCutActive()
    );
    websocket_broadcast(json);
}
void sendInitialState() {
    StaticJsonDocument<256> json;
    json["cmd"] = "INIT_STATE";
    json["active_profile"] = ecu_state.active_profile; // ← tambah ini
    wsSend(json);
}


