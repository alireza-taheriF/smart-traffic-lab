// EVP-12 Vehicle Transmitter Unit (VTU) — per-vehicle settings
#pragma once

// ---- identity ---------------------------------------------------------------
#define VEHICLE_ID      0x0A34      // unique per vehicle (16 bit). IR carries the low 8 bits.
#define VEHICLE_PRIO    2           // 0..3  (3 = fire/rescue, 2 = ambulance, 1 = police, 0 = test)
#define SYSTEM_CODE     0x5A        // must match the intersections (rejects other systems' IR)

// 32-byte fleet key, shared with every intersection controller. CHANGE IT.
// Generate:  python3 -c "import os;print(', '.join(f'0x{b:02X}' for b in os.urandom(32)))"
#define FLEET_KEY { \
  0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10, \
  0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20 }

// ---- radio --------------------------------------------------------------------
#define LORA_FREQ_HZ    433E6       // check the band and power allowed by CRA before road use
#define LORA_TX_DBM     14          // 2..20; use the lowest that reaches ~1 km
#define LORA_PERIOD_MS  1000

// ---- pins (ESP32-WROOM-32E DevKitC) -----------------------------------------
#define PIN_IR_DATA     25          // RMT out, 38 kHz modulated envelope -> 74HCT08 inputs
#define PIN_EN_A        26          // gate group A: 6 strings (HIGH level only)
#define PIN_EN_B        27          // gate group B: 2 strings (HIGH and MID)
#define PIN_EN_C        14          // gate group C: 1 string via 47R (LOW level)
#define PIN_LORA_NSS    5
#define PIN_LORA_RST    22
#define PIN_LORA_DIO0   21
#define PIN_GPS_RX      16          // ESP32 RX  <- GPS TX
#define PIN_GPS_TX      17          // ESP32 TX  -> GPS RX
#define GPS_BAUD        9600
#define PIN_ENABLE_IN   34          // PC817 collector, LOW = lights/siren ON (external 10k pull-up)
#define PIN_STATUS_LED  13          // cab indicator through NPN driver
#define ENABLE_ALWAYS   0           // 1 = ignore ENABLE_IN (bench testing only)
