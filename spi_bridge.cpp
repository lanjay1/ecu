//spi_bridge.cpp
#define PIN_CS  5
SPIClass *spi = new SPIClass(VSPI);

void spi_setup() {
  spi->begin();  // SCK, MISO, MOSI default
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
}

void spi_send_json(String cmd, String payload) {
  digitalWrite(PIN_CS, LOW);
  spi->transfer(0xAA); // start
  spi->transfer(cmd.length());
  for (char c : cmd) spi->transfer(c);
  spi->transfer16(payload.length());
  for (char c : payload) spi->transfer(c);
  digitalWrite(PIN_CS, HIGH);
}
