//main.cpp
#include <Arduino.h>
#include "webserver.h"
#include "spi_bridge.h"
#include "ws_handler.h"

// Variabel global untuk data real-time
volatile uint16_t currentRPM = 0;
volatile uint8_t currentTPS = 0;
// Tambahkan variabel lain sesuai kebutuhan (gear, temperature, dll.)

void setup() {
  Serial.begin(115200);
  delay(100);

  // Inisialisasi SPI dan WebSocket
  spi_setup();
  initWebServer();

  // Konfigurasi interrupt SPI (jika diperlukan)
  // Contoh: attachInterrupt(digitalPinToInterrupt(PIN_MISO), onSPIData, RISING);

  Serial.println("ESP32 ECU Webserver Ready");
}

void loop() {
  static unsigned long lastUpdate = 0;

  // Update data real-time setiap 100ms
  if (millis() - lastUpdate >= 100) {
    sendRealtimeData();  // Kirim data ke semua klien WebSocket
    lastUpdate = millis();
  }
      FuelIgnition_update();
    RPMProtection_Update();
    FanControl_Update();
    CommunicationManager_HandleUARTCommand();


  // Tambahkan logika lain jika diperlukan:
  // - Baca data dari SPI
  // - Handle error/retry
}

// Callback untuk data dari STM32 (contoh)
void onSPIData() {
  uint8_t spiData[32];
  // Baca data dari SPI (implementasi sesuai protokol STM32)
  // Contoh: spi->transfer(spiData, sizeof(spiData));

  // Parsing data (sesuai format protokol)
  if (spiData[0] == 0xAA) {  // Header
    currentRPM = (spiData[1] << 8) | spiData[2];
    currentTPS = spiData[3];
    // Proses data lainnya...
  }
}