/**
 * ESP32-S3 Dual CAN Bridge
 *
 * Receives CAN frames on two independent buses and logs them to Serial.
 *
 * CAN0 (TWAI) — Built-in ESP32 TWAI peripheral, 500 kbps
 *   TX = GPIO 8,  RX = GPIO 9
 *
 * CAN1 (SPI)  — External MCP2517FD via SPI (FSPI), 500 kbps
 *   MOSI = GPIO 11, MISO = GPIO 12, SCK = GPIO 13
 *   CS   = GPIO 10, INT  = GPIO 3
 *
 * Activity LEDs (active-high, brief pulse per frame):
 *   TWAI TX = GPIO 17,  TWAI RX = GPIO 18
 *   SPI  TX = GPIO 38,  SPI  RX = GPIO 37
 */

#include <SPI.h>
#include <ACAN2517FD.h>
#include <ESP32-TWAI-CAN.hpp>

// =====================================================================
//  PIN ASSIGNMENTS
// =====================================================================

// CAN0 — TWAI (built-in peripheral)
static const uint8_t PIN_TWAI_TX = 8;
static const uint8_t PIN_TWAI_RX = 9;

// CAN1 — MCP2517FD via SPI
static const uint8_t PIN_SPI_MOSI = 11;
static const uint8_t PIN_SPI_MISO = 12;
static const uint8_t PIN_SPI_SCK  = 13;
static const uint8_t PIN_CAN_CS   = 10;   // SPI chip-select for MCP2517FD
static const uint8_t PIN_CAN_INT  = 3;    // Interrupt from MCP2517FD
static const int8_t  PIN_SD_CS    = -1;   // SD card CS (unused, -1 = none)

// Activity LEDs
static const uint8_t  PIN_LED_TWAI_TX = 17;
static const uint8_t  PIN_LED_TWAI_RX = 18;
static const uint8_t  PIN_LED_SPI_TX  = 38;
static const uint8_t  PIN_LED_SPI_RX  = 37;
static const uint16_t LED_PULSE_MS    = 35;  // How long each LED stays on per pulse

// =====================================================================
//  CAN BUS OBJECTS & SETTINGS
// =====================================================================

SPIClass   spiBus(FSPI);
ACAN2517FD canSPI(PIN_CAN_CS, spiBus, PIN_CAN_INT);

static const uint32_t CAN_TWAI_BITRATE = 500;     // TWAI lib uses kbps value(CAN 1)
static const uint32_t CAN_SPI_BITRATE  = 500000;  // ACAN lib uses bps value (CAN 2)

// SPI CAN recovery — the MCP2517FD can lose sync; these control auto-retry
static const uint8_t  SPI_CAN_BOOT_INIT_RETRIES = 5;     // Attempts at power-on
static const uint32_t SPI_CAN_RETRY_DELAY_MS    = 500;   // Delay between boot retries
static const uint32_t SPI_CAN_RETRY_INTERVAL_MS = 3000;  // Delay between runtime retries
static const uint32_t SPI_CAN_RX_STALL_MS       = 750;   // INT stuck-low stall threshold

// SPI CAN runtime state
bool     spiCanOnline      = false;
bool     spiCanStarted     = false;
uint32_t nextSpiCanRetryMs = 0;
uint32_t lastSpiRxMs       = 0;       // Timestamp of last received SPI frame
uint32_t spiIntLowSinceMs  = 0;       // Tracks how long INT has been held low

// =====================================================================
//  LED ACTIVITY INDICATORS
// =====================================================================

// Timestamps for turning each LED off (0 = already off)
uint32_t twaiTxLedOffAt = 0;
uint32_t twaiRxLedOffAt = 0;
uint32_t spiTxLedOffAt  = 0;
uint32_t spiRxLedOffAt  = 0;

// Turn an LED on and schedule it to turn off after LED_PULSE_MS
void pulseLed(uint8_t pin, uint32_t& offAt) {
  digitalWrite(pin, HIGH);
  offAt = millis() + LED_PULSE_MS;
}

// Turn off any LED whose pulse duration has elapsed
void serviceLedIndicators() {
  uint32_t now = millis();
  if (twaiTxLedOffAt && (int32_t)(now - twaiTxLedOffAt) >= 0) { digitalWrite(PIN_LED_TWAI_TX, LOW); twaiTxLedOffAt = 0; }
  if (twaiRxLedOffAt && (int32_t)(now - twaiRxLedOffAt) >= 0) { digitalWrite(PIN_LED_TWAI_RX, LOW); twaiRxLedOffAt = 0; }
  if (spiTxLedOffAt  && (int32_t)(now - spiTxLedOffAt)  >= 0) { digitalWrite(PIN_LED_SPI_TX,  LOW); spiTxLedOffAt  = 0; }
  if (spiRxLedOffAt  && (int32_t)(now - spiRxLedOffAt)  >= 0) { digitalWrite(PIN_LED_SPI_RX,  LOW); spiRxLedOffAt  = 0; }
}

// =====================================================================
//  CAN0 — TWAI SETUP
// =====================================================================

// Initialize the built-in TWAI peripheral. Halts on failure.
static void setupTWAI() {
  ESP32Can.setPins(PIN_TWAI_TX, PIN_TWAI_RX);
  if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_TWAI_BITRATE))) {
    Serial.println("TWAI started @ 500k (TX=GPIO8 RX=GPIO9)");
  } else {
    Serial.println("TWAI FAILED — halting");
    while (true) delay(1000);
  }
}

// =====================================================================
//  CAN1 — SPI MCP2517FD SETUP & HEALTH
// =====================================================================

// Attempt a single SPI CAN init (or re-init if previously started)
static bool setupSPICANOnce() {
  if (spiCanStarted) {
    canSPI.end();
    delay(2);
  }

  // Deselect SD card if present to avoid SPI bus contention
  if (PIN_SD_CS >= 0) {
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
  }

  // Configure SPI bus pins
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);

  spiBus.end();
  delay(5);
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);
  delay(10);

  // Configure MCP2517FD: 40 MHz oscillator, 500 kbps, classic CAN (Normal20B)
  ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_40MHz,
    CAN_SPI_BITRATE,
    ACAN2517FDSettings::DATA_BITRATE_x1,
    256
  );
  settings.mDriverReceiveFIFOSize = 200;
  settings.mRequestedMode = ACAN2517FDSettings::Normal20B;

  const uint32_t errorCode = canSPI.begin(settings, [] { canSPI.isr(); });
  if (errorCode == 0) {
    Serial.printf("SPI MCP2517FD started @ %lu\n", (unsigned long)CAN_SPI_BITRATE);
    spiCanStarted    = true;
    lastSpiRxMs      = millis();
    spiIntLowSinceMs = 0;
    return true;
  }

  Serial.print("SPI MCP2517FD FAILED, error=0x");
  Serial.println(errorCode, HEX);
  return false;
}

// Try up to SPI_CAN_BOOT_INIT_RETRIES at startup
static bool setupSPICANWithRetry() {
  for (uint8_t attempt = 1; attempt <= SPI_CAN_BOOT_INIT_RETRIES; attempt++) {
    Serial.printf("SPI MCP2517FD init attempt %u/%u...\n", attempt, SPI_CAN_BOOT_INIT_RETRIES);
    if (setupSPICANOnce()) {
      spiCanOnline = true;
      return true;
    }
    if (attempt < SPI_CAN_BOOT_INIT_RETRIES) delay(SPI_CAN_RETRY_DELAY_MS);
  }
  spiCanOnline = false;
  return false;
}

// Periodically re-attempt init when the SPI CAN is offline
static void serviceSPICANRetry() {
  if (spiCanOnline) return;
  const uint32_t now = millis();
  if ((int32_t)(now - nextSpiCanRetryMs) < 0) return;

  Serial.println("SPI MCP2517FD offline — retrying init...");
  if (setupSPICANOnce()) {
    spiCanOnline = true;
    Serial.println("SPI MCP2517FD recovered.");
  } else {
    nextSpiCanRetryMs = now + SPI_CAN_RETRY_INTERVAL_MS;
  }
}

// Detect RX stalls: if INT is stuck low and no frames are being drained,
// assume the MCP2517FD is wedged and force a reinit.
static void serviceSPICANHealth() {
  if (!spiCanOnline) return;

  canSPI.poll();  // Let the driver process any pending SPI transactions

  const uint32_t now      = millis();
  const bool     intIsLow = (digitalRead(PIN_CAN_INT) == LOW);

  if (intIsLow) {
    if (spiIntLowSinceMs == 0) {
      spiIntLowSinceMs = now;  // Start tracking
    } else if ((int32_t)(now - spiIntLowSinceMs) >= (int32_t)SPI_CAN_RX_STALL_MS &&
               (int32_t)(now - lastSpiRxMs)      >= (int32_t)SPI_CAN_RX_STALL_MS) {
      Serial.println("SPI CAN RX stall detected — forcing reinit");
      spiCanOnline      = false;
      nextSpiCanRetryMs = now;
      spiIntLowSinceMs  = 0;
    }
  } else {
    spiIntLowSinceMs = 0;
  }
}

// =====================================================================
//  SPI -> TWAI TRIGGER (0x140810FF bit 6 -> 0x062 pulse)
// =====================================================================

// When SPI CAN receives extended ID 0x140810FF with bit 7 of byte 0 set,
// send a single-pulse on TWAI: ID 0x062, byte 0 = 0x01, then 0x00 after 10 ms.
static const uint32_t TRIGGER_SPI_ID       = 0x140810FF;  // 29-bit extended ID to watch
static const uint8_t  TRIGGER_BIT_MASK     = (1 << 7);    // Bit 7 (MSB) of byte 0
static const uint32_t TRIGGER_TWAI_ID      = 0x062;       // 11-bit standard ID to send
static const uint32_t TRIGGER_RESET_MS     = 10;          // Time before sending the clear frame

bool     triggerArmed    = true;   // Re-arms when the source bit goes back to 0
uint32_t triggerResetAt  = 0;      // 0 = no pending reset; non-zero = millis to send clear

// Send the one-shot "on" frame and schedule the "off" frame
static void fireTrigger() {
  CanFrame frame = {};
  frame.identifier      = TRIGGER_TWAI_ID;
  frame.extd            = 0;            // Standard 11-bit
  frame.data_length_code = 8;
  memset(frame.data, 0, 8);
  frame.data[0]         = 0x01;         // Bit 0 = 1

  const bool ok = ESP32Can.writeFrame(frame, 20);
  if (ok) pulseLed(PIN_LED_TWAI_TX, twaiTxLedOffAt);
  Serial.printf("TRIGGER -> TWAI 0x%03X [01] %s\n", TRIGGER_TWAI_ID, ok ? "ok" : "FAIL");

  triggerResetAt = millis() + TRIGGER_RESET_MS;
}

// Called from loop() — sends the clear frame once the 10 ms window expires
static void serviceTriggerReset() {
  if (triggerResetAt == 0) return;
  if ((int32_t)(millis() - triggerResetAt) < 0) return;

  CanFrame frame = {};
  frame.identifier      = TRIGGER_TWAI_ID;
  frame.extd            = 0;
  frame.data_length_code = 8;
  memset(frame.data, 0, 8);             // Bit 0 = 0

  const bool ok = ESP32Can.writeFrame(frame, 20);
  if (ok) pulseLed(PIN_LED_TWAI_TX, twaiTxLedOffAt);
  Serial.printf("TRIGGER -> TWAI 0x%03X [00] reset %s\n", TRIGGER_TWAI_ID, ok ? "ok" : "FAIL");

  triggerResetAt = 0;
}

// =====================================================================
//  CAN FRAME RECEIVE & LOGGING
// =====================================================================

// Drain up to 20 frames from TWAI and print each to Serial
void receiveCanTWAI() {
  CanFrame frame = {};
  uint8_t count = 0;
  while (count < 20 && ESP32Can.readFrame(frame, 0)) {
    pulseLed(PIN_LED_TWAI_RX, twaiRxLedOffAt);
    Serial.printf("TWAI 0x%03X ", frame.identifier);
    for (uint8_t i = 0; i < frame.data_length_code; i++) {
      Serial.printf("%02X", frame.data[i]);
      if (i + 1 < frame.data_length_code) Serial.print(" ");
    }
    Serial.println();
    count++;
  }
}

// Drain up to 64 frames from SPI CAN and print each to Serial
void receiveCanSPI() {
  if (!spiCanOnline) return;

  CANFDMessage frame;
  uint8_t count = 0;
  while (count < 64 && canSPI.receive(frame)) {
    lastSpiRxMs      = millis();
    spiIntLowSinceMs = 0;
    pulseLed(PIN_LED_SPI_RX, spiRxLedOffAt);

    // Check for trigger frame (29-bit 0x140810FF, bit 6 of byte 0)
    if (frame.ext && frame.id == TRIGGER_SPI_ID && frame.len >= 1) {
      const bool bitSet = (frame.data[0] & TRIGGER_BIT_MASK) != 0;
      if (bitSet && triggerArmed && triggerResetAt == 0) {
        fireTrigger();
        triggerArmed = false;   // Don't re-fire until bit clears
      } else if (!bitSet) {
        triggerArmed = true;    // Re-arm when bit goes back to 0
      }
    }

    const uint8_t len = (frame.len > 8) ? 8 : frame.len;
    Serial.printf("SPI  0x%03X ", frame.id);
    for (uint8_t i = 0; i < len; i++) {
      Serial.printf("%02X", frame.data[i]);
      if (i + 1 < len) Serial.print(" ");
    }
    Serial.println();
    count++;
  }
}

// =====================================================================
//  SETUP & LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize activity LEDs (all off)
  pinMode(PIN_LED_TWAI_TX, OUTPUT);  digitalWrite(PIN_LED_TWAI_TX, LOW);
  pinMode(PIN_LED_TWAI_RX, OUTPUT);  digitalWrite(PIN_LED_TWAI_RX, LOW);
  pinMode(PIN_LED_SPI_TX,  OUTPUT);  digitalWrite(PIN_LED_SPI_TX,  LOW);
  pinMode(PIN_LED_SPI_RX,  OUTPUT);  digitalWrite(PIN_LED_SPI_RX,  LOW);

  Serial.println("\nESP32-S3 Dual CAN Bridge");
  Serial.println("Starting CAN buses...");

  // CAN0 — TWAI (halts on failure)
  setupTWAI();

  // CAN1 — SPI MCP2517FD (retries in loop() if boot-time init fails)
  if (!setupSPICANWithRetry()) {
    Serial.println("SPI MCP2517FD failed at boot — will retry in loop.");
    nextSpiCanRetryMs = millis() + SPI_CAN_RETRY_INTERVAL_MS;
  }

  Serial.println("Ready.");
}

void loop() {
  serviceLedIndicators();   // Turn off LEDs whose pulse has expired
  serviceSPICANRetry();     // Re-attempt SPI CAN init if offline
  serviceSPICANHealth();    // Detect SPI CAN RX stalls
  receiveCanTWAI();         // Drain & log TWAI frames
  receiveCanSPI();          // Drain & log SPI CAN frames
  serviceTriggerReset();    // Send trigger clear frame after 10 ms
  delay(1);                 // Yield to RTOS
}
