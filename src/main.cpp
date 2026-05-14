#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// ============================================================================
// ESP-NOW Configuration
// ============================================================================
// REPLACE WITH THE ACTUAL MAC ADDRESS OF YOUR ESP32-C6
uint8_t receiverAddress[] = {0x98, 0x88, 0xE0, 0x76, 0x93, 0xEC};

typedef struct __attribute__((packed))
{
    uint8_t type; // 'L' (0x4C)
    uint8_t on;   // 0 = off, 1 = on
} LightPacket;

// ============================================================================
// BEAN Protocol Configuration
// ============================================================================
const int BEAN_RX_PIN = 5;       // GPIO5 (D1 on NodeMCU/Wemos)
const bool INVERT_SIGNAL = true; // Set to true if your LM393 circuit inverts

// Message filters (from your ESP32-C6 settings)
const uint8_t TARGET_ECU_DID = 0xFE;
const uint8_t TARGET_ECU_SID = 0x7F;
const uint8_t LIGHT_STATUS_BITMASK = 0x08;
const uint8_t LIGHT_PAYLOAD_BYTE = 0;

// ============================================================================
// Hardware Interrupt Tracking Buffers
// ============================================================================
#define MAX_PULSES 512

typedef struct
{
    uint32_t dur;
    uint8_t lvl;
} pulse_t;

volatile pulse_t isr_pulses[MAX_PULSES];
volatile int pulse_count = 0;
volatile uint32_t last_edge_time = 0;
volatile uint8_t last_level = 0;
volatile bool pending_light_update = false;
volatile bool current_light_state = false;

bool last_light_status = false;
bool first_light_run = true;

// ============================================================================
// Software Processing Buffers (Statically allocated to save stack space)
// ============================================================================
pulse_t raw_pulses[512];
pulse_t clean_pulses[512];
uint8_t raw_bits[256];

// Standard Toyota BEAN Protocol CRC8 table
static const uint8_t crc8_table[256] = {
    0x00, 0x13, 0x26, 0x35, 0x4C, 0x5F, 0x6A, 0x79, 0x98, 0x8B, 0xBE, 0xAD, 0xD4, 0xC7, 0xF2, 0xE1,
    0x23, 0x30, 0x05, 0x16, 0x6F, 0x7C, 0x49, 0x5A, 0xBB, 0xA8, 0x9D, 0x8E, 0xF7, 0xE4, 0xD1, 0xC2,
    0x46, 0x55, 0x60, 0x73, 0x0A, 0x19, 0x2C, 0x3F, 0xDE, 0xCD, 0xF8, 0xEB, 0x92, 0x81, 0xB4, 0xA7,
    0x65, 0x76, 0x43, 0x50, 0x29, 0x3A, 0x0F, 0x1C, 0xFD, 0xEE, 0xDB, 0xC8, 0xB1, 0xA2, 0x97, 0x84,
    0x8C, 0x9F, 0xAA, 0xB9, 0xC0, 0xD3, 0xE6, 0xF5, 0x14, 0x07, 0x32, 0x21, 0x58, 0x4B, 0x7E, 0x6D,
    0xAF, 0xBC, 0x89, 0x9A, 0xE3, 0xF0, 0xC5, 0xD6, 0x37, 0x24, 0x11, 0x02, 0x7B, 0x68, 0x5D, 0x4E,
    0xCA, 0xD9, 0xEC, 0xFF, 0x86, 0x95, 0xA0, 0xB3, 0x52, 0x41, 0x74, 0x67, 0x1E, 0x0D, 0x38, 0x2B,
    0xE9, 0xFA, 0xCF, 0xDC, 0xA5, 0xB6, 0x83, 0x90, 0x71, 0x62, 0x57, 0x44, 0x3D, 0x2E, 0x1B, 0x08,
    0x0B, 0x18, 0x2D, 0x3E, 0x47, 0x54, 0x61, 0x72, 0x93, 0x80, 0xB5, 0xA6, 0xDF, 0xCC, 0xF9, 0xEA,
    0x28, 0x3B, 0x0E, 0x1D, 0x64, 0x77, 0x42, 0x51, 0xB0, 0xA3, 0x96, 0x85, 0xFC, 0xEF, 0xDA, 0xC9,
    0x4D, 0x5E, 0x6B, 0x78, 0x01, 0x12, 0x27, 0x34, 0xD5, 0xC6, 0xF3, 0xE0, 0x99, 0x8A, 0xBF, 0xAC,
    0x6E, 0x7D, 0x48, 0x5B, 0x22, 0x31, 0x04, 0x17, 0xF6, 0xE5, 0xD0, 0xC3, 0xBA, 0xA9, 0x9C, 0x8F,
    0x87, 0x94, 0xA1, 0xB2, 0xCB, 0xD8, 0xED, 0xFE, 0x1F, 0x0C, 0x39, 0x2A, 0x53, 0x40, 0x75, 0x66,
    0xA4, 0xB7, 0x82, 0x91, 0xE8, 0xFB, 0xCE, 0xDD, 0x3C, 0x2F, 0x1A, 0x09, 0x70, 0x63, 0x56, 0x45,
    0xC1, 0xD2, 0xE7, 0xF4, 0x8D, 0x9E, 0xAB, 0xB8, 0x59, 0x4A, 0x7F, 0x6C, 0x15, 0x06, 0x33, 0x20,
    0xE2, 0xF1, 0xC4, 0xD7, 0xAE, 0xBD, 0x88, 0x9B, 0x7A, 0x69, 0x5C, 0x4F, 0x36, 0x25, 0x10, 0x03};

uint8_t calc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc = crc8_table[crc ^ data[i]];
        // Serial.printf("Byte %d: 0x%02X, Intermediate CRC: 0x%02X\n", i, data[i], crc);
    }
    return crc;
}

// ============================================================================
// Logging Mode Configuration
// ============================================================================
bool LOGGING_MODE_ENABLED = false; // Enable with serial command "log on"

// ============================================================================
// Test Mode Configuration
// ============================================================================
bool TEST_MODE_ENABLED = false;            // Set to false to disable test mode
const uint32_t TEST_TX_INTERVAL = 500;     // Send message every 500ms
const uint32_t TEST_CYCLE_DURATION = 5000; // 5 seconds per state (on/off)

// Test mode state tracking
uint32_t test_last_tx_time = 0;
uint32_t test_cycle_start_time = 0;
bool test_lights_state = true; // Start with lights ON

uint32_t last_sync_time = 0;

static uint32_t s_last_send_ok_ms = 0;
static const uint32_t RADIO_WATCHDOG_MS = 10000;

// ============================================================================
// Hardware ISR (Pin Change Interrupt)
// ============================================================================
void sendTestMessage(bool lights_on)
{
    LightPacket pkt;
    pkt.type = 'L';
    pkt.on = lights_on ? 1 : 0;
    esp_now_send(receiverAddress, (uint8_t *)&pkt, sizeof(pkt));
    if (LOGGING_MODE_ENABLED)
    {
        Serial.printf("TEST MODE: Transmitting lights %s\n", lights_on ? "ON" : "OFF");
    }
}

bool found_intra_message = false;
void ICACHE_RAM_ATTR handleEdge()
{
    uint32_t now = micros();
    uint32_t duration = now - last_edge_time;
    last_edge_time = now;

    // Evaluate the logical state of the pulse that just COMPLETED
    uint8_t logical_level = INVERT_SIGNAL ? (1 - last_level) : last_level;

    // If the bus was resting at logical 0 (idle) for > 600us, this is a new message
    if (duration > 600 && logical_level == 0)
    {
        pulse_count = 0; // Start of new message, reset pulse count
    }
    else
    {
        if (pulse_count < MAX_PULSES)
        {
            isr_pulses[pulse_count].dur = duration;
            isr_pulses[pulse_count].lvl = logical_level;
            pulse_count++;
        }
    }

    last_level = digitalRead(BEAN_RX_PIN);
}

// ============================================================================
// ESP-NOW Callback
// ============================================================================
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
    if (sendStatus == 0)
        s_last_send_ok_ms = millis();
    if (LOGGING_MODE_ENABLED)
        Serial.println(sendStatus == 0 ? "ESP-NOW Delivery Success" : "ESP-NOW Delivery Fail");
}

// ============================================================================
// Initialization
// ============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nToyota BEAN ESP8266 Reader");

    // Init Wi-Fi as Station mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != 0)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Configure ESP-NOW Controller Role
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onDataSent);

    // Add the ESP32-C6 as our peer
    esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    // Initialize the bus reader pin
    pinMode(BEAN_RX_PIN, INPUT_PULLUP);
    last_level = digitalRead(BEAN_RX_PIN);
    last_edge_time = micros();

    // Arm the hardware interrupt tracker
    attachInterrupt(digitalPinToInterrupt(BEAN_RX_PIN), handleEdge, CHANGE);

    s_last_send_ok_ms = millis();
}

int message_count = 0;
// ============================================================================
// Main Loop & Decoder
// ============================================================================
void decode_bean_message(int count)
{
    message_count++;
    // 1. Hardware Noise Filter & Merge logic (Absorb jitter <= 10us)
    int clean_cnt = 0;
    for (int i = 0; i < count; i++)
    {
        if (raw_pulses[i].dur <= 10 && clean_cnt > 0)
        {
            if (LOGGING_MODE_ENABLED)
            {
                // Serial.printf("Merging pulse at index %d with duration %d us into previous pulse due to noise filter\n", i, raw_pulses[i].dur);
            }
            clean_pulses[clean_cnt - 1].dur += raw_pulses[i].dur;
            while (i + 1 < count && raw_pulses[i + 1].lvl == clean_pulses[clean_cnt - 1].lvl)
            {
                clean_pulses[clean_cnt - 1].dur += raw_pulses[i + 1].dur;
                i++;
            }
        }
        else
        {
            if (clean_cnt > 0 && clean_pulses[clean_cnt - 1].lvl == raw_pulses[i].lvl)
            {
                clean_pulses[clean_cnt - 1].dur += raw_pulses[i].dur;
            }
            else
            {
                clean_pulses[clean_cnt] = raw_pulses[i];
                clean_cnt++;
            }
        }

        if (LOGGING_MODE_ENABLED && clean_cnt > 0)
        {
            // Serial.printf("%d: %d\n", clean_pulses[clean_cnt - 1].lvl, clean_pulses[clean_cnt - 1].dur);
        }
    }

    // 2. Find Start of Frame (Wakeup HIGH >= 60us)
    int i = 0;
    while (i < clean_cnt)
    {
        if (clean_pulses[i].lvl == 1 && clean_pulses[i].dur >= 60)
            break;
        i++;
    }
    if (i >= clean_cnt)
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.println("No valid wakeup signal found. Discarding message.");
        }
        return; // No valid wakeup
    }
    if (LOGGING_MODE_ENABLED)
    {
        // Serial.printf("Wakeup signal found at pulse index %d with duration %d us\n", i, clean_pulses[i].dur);
    }

    int bit_cnt = 0;
    i++; // SOM pulse marks the frame boundary but contributes no data bits

    // 3. Validate first data pulse is LOW (LENGTH MSB is always 0)
    if (i >= clean_cnt || clean_pulses[i].lvl != 0)
    {
        if (LOGGING_MODE_ENABLED)
            Serial.println("Expected LOW after SOM, discarding.");
        return;
    }
    // Don't consume — step 4 decodes all bits from this pulse onward

    // 4. Decode remainder of message & Destuff Bits
    while (i < clean_cnt)
    {
        uint8_t lvl = clean_pulses[i].lvl;
        uint32_t dur = clean_pulses[i].dur;

        if (dur < 70)
        {
            if (LOGGING_MODE_ENABLED)
            {
                Serial.printf("Skipping pulse at index %d with duration %d us as too short to be valid\n", i, dur);
            }
            i++;
            continue;
        }

        if (dur >= 600)
            break; // EOM LOW (idle timeout) or EOM HIGH (EOF marker)

        int b_count = (dur + 50) / 100;
        if (b_count < 1)
            b_count = 1;
        for (int b = 0; b < b_count && bit_cnt < 256; b++)
        {
            raw_bits[bit_cnt++] = lvl;

            // Optimization: Abort early if the message is not for our target ECU
            if (bit_cnt == 24)
            {
                uint8_t check_did = 0, check_sid = 0;
                for (int bit = 0; bit < 8; bit++)
                {
                    check_did = (check_did << 1) | raw_bits[8 + bit];
                    check_sid = (check_sid << 1) | raw_bits[16 + bit];
                }
                if (check_did != TARGET_ECU_DID || check_sid != TARGET_ECU_SID)
                {
                    return; // Kills processing immediately, saving CPU time!
                }
            }
        }

        i++;

        // Pack bit check: Opposite value stuffed after a run of exactly 5 identical bits
        if (b_count > 5)
        {
            if (LOGGING_MODE_ENABLED)
            {
                Serial.println("Bit stuffing violation detected. Duration: " + String(dur) + " us, Count: " + String(b_count));
                Serial.println("Discarding message.");
            }
            return; // Protocol violation - too much consecutive bits without a stuff
        }
        if (b_count == 5)
        {
            if (i >= clean_cnt)
                break;

            if (clean_pulses[i].dur >= 600)
                break; // Stuffed bit gracefully merged with the EOF marker

            if (clean_pulses[i].lvl == lvl)
            {
                if (LOGGING_MODE_ENABLED)
                {
                    Serial.println("Bit stuffing violation: Expected opposite bit after 5 consecutive bits. Discarding message.");
                }
                return; // Protocol violation - expected opposite bit after 5 consecutive bits
            }
            int bits = (clean_pulses[i].dur + 50) / 100;
            if (bits > 1)
            // If the stuff bit is longer than 100,
            // it's actually the stuff bit + valid data,
            // so we need to save that data. We'll chop off the stuff bit
            // and process the rest of the pulse as normal in the next
            // iteration.
            {
                // Serial.println("Long stuff bit detected. Adjusting duration.");
                clean_pulses[i].dur -= 100;
                // Serial.println("New duration is " + String(clean_pulses[i].dur) + " us, which should yield " + String(clean_pulses[i].dur / 100) + " bits on the next loop.");
            }
            else // Just skip the stuff bit
            {
                i++;
            }
        }
    }

    if (bit_cnt < 8)
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.println("Not enough bits received. Discarding message.");
        }
        return;
    }

    // 5. Pack raw bits into proper bytes (MSB first)
    uint8_t bytes[32] = {0};
    if (LOGGING_MODE_ENABLED)
    {
        // Serial.printf("Raw bits count: %d\n", bit_cnt);
    }
    int byte_cnt = bit_cnt / 8;
    if (byte_cnt > 32)
        byte_cnt = 32;

    for (int b = 0; b < byte_cnt; b++)
    {
        uint8_t val = 0;
        for (int bit = 0; bit < 8; bit++)
        {
            if (LOGGING_MODE_ENABLED)
            {
                // Serial.printf("%d", raw_bits[b * 8 + bit]);
            }
            val = (val << 1) | raw_bits[b * 8 + bit];
        }
        bytes[b] = val;
        if (LOGGING_MODE_ENABLED)
        {
            // Serial.printf("\n");
            // Serial.printf("Byte %d: 0x%02X\n", b, bytes[b]);
        }
    }

    int extra_bits = bit_cnt - byte_cnt * 8;
    if (extra_bits > 0)
    {
        if (extra_bits == 1)
        {
            // 1 extra bit is expected in BEAN. It is the ACK slot!
            if (LOGGING_MODE_ENABLED)
            {
                // Serial.printf("ACK bit received: %d\n", raw_bits[byte_cnt * 8]);
            }
        }
        else if (LOGGING_MODE_ENABLED)
        {
            Serial.printf("Warning: %d bits received but only %d full bytes could be formed. %d extra bits will be discarded.\n", bit_cnt, byte_cnt, extra_bits);
            for (int b = 0; b < extra_bits; b++)
            {
                Serial.printf("Discarding extra bit: %d\n", raw_bits[byte_cnt * 8 + b]);
            }
        }
    }

    if (byte_cnt < 4)
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.println("Not enough bytes received. Discarding message.");
        }
        return;
    }
    if (LOGGING_MODE_ENABLED)
    {
        Serial.println("byte_cnt: " + String(byte_cnt));
    }

    uint8_t length = bytes[0] & 0x0F;
    if (byte_cnt != length + 2) // +2 for the length byte and CRC byte
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.println("Invalid message length. Discarding message.");
            Serial.printf("Expected length: %d, Bytes received: %d\n", length, byte_cnt);
        }
        return;
    }

    for (int i = 0; i < byte_cnt; i++)
    {
        if (LOGGING_MODE_ENABLED)
        {
            // Serial.printf("Byte %d: 0x%02X\n", i, bytes[i]);
        }
    }

    // 6. Validate the Checksum
    uint8_t crc_rx = bytes[byte_cnt - 1];
    if (LOGGING_MODE_ENABLED)
    {
        // Serial.printf("Received CRC: 0x%02X, Calculating CRC over bytes[0..%d]\n", crc_rx, length);
    }
    uint8_t crc_calc = calc_crc8(bytes, length + 1);
    if (crc_rx != crc_calc)
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.printf("CRC mismatch: received 0x%02X, calculated 0x%02X. Discarding message.\n", crc_rx, crc_calc);
        }
        return; // Protocol mismatch or noisy packet dropped
    }

    // 7. Inspect Application Layer Data
    uint8_t did = bytes[1];
    uint8_t sid = bytes[2];

    if (LOGGING_MODE_ENABLED)
    {
        Serial.printf("DID: 0x%02X, SID: 0x%02X\n", did, sid);
        for (int i = 3; i < byte_cnt - 1; i++)
        {
            Serial.printf("Payload Byte %d: 0x%02X\n", i - 3, bytes[i]);
        }
        Serial.printf("Got the right message\n");
    }

    int payload_len = length - 3;
    // if (LOGGING_MODE_ENABLED)
    // {
    //     Serial.printf("Payload length: %d\n", payload_len);
    // }
    if (payload_len > LIGHT_PAYLOAD_BYTE)
    {
        if (LOGGING_MODE_ENABLED)
        {
            Serial.printf("Light data: 0x%02X\n", bytes[4 + LIGHT_PAYLOAD_BYTE]);
        }

        uint8_t light_data = bytes[4 + LIGHT_PAYLOAD_BYTE];
        bool lights_on = (light_data & LIGHT_STATUS_BITMASK) != 0;
        if (LOGGING_MODE_ENABLED)
        {
            Serial.printf("Lights status: %s\n", lights_on ? "ON" : "OFF");
        }

        if (first_light_run || lights_on != last_light_status)
        {
            last_light_status = lights_on;
            first_light_run = false;

            Serial.printf("Lights updated: %s\n", lights_on ? "ON" : "OFF");

            // Queue the update instead of sending it immediately
            current_light_state = lights_on;
        }
        // Always send the message, just like BEAN does,
        // even if the state didn't change, to keep the
        // speedometer in sync and test reliability
        pending_light_update = true;
    }
}

bool pulse_started = false;

void loop()
{
    // ============================================================================
    // Test Mode Handler
    // ============================================================================
    if (TEST_MODE_ENABLED)
    {
        uint32_t now_ms = millis();

        // Initialize timing on first run
        if (test_cycle_start_time == 0)
        {
            test_cycle_start_time = now_ms;
            test_last_tx_time = now_ms;
        }

        uint32_t elapsed_in_cycle = now_ms - test_cycle_start_time;

        // Check if it's time to switch states (5 second cycle)
        if (elapsed_in_cycle >= TEST_CYCLE_DURATION)
        {
            test_lights_state = !test_lights_state; // Toggle between ON and OFF
            test_cycle_start_time = now_ms;
            test_last_tx_time = now_ms;
            sendTestMessage(test_lights_state);
        }
        // Check if it's time to transmit (every 500ms)
        else if (now_ms - test_last_tx_time >= TEST_TX_INTERVAL)
        {
            test_last_tx_time = now_ms;
            sendTestMessage(test_lights_state);
        }

        // -------------------------------------------------------------------
        // Dispatch queued messages OUTSIDE the decoding and interrupt blocks
        // -------------------------------------------------------------------
        if (pending_light_update)
        {
            pending_light_update = false;
            LightPacket pkt;
            pkt.type = 'L';
            pkt.on = current_light_state ? 1 : 0;
            esp_now_send(receiverAddress, (uint8_t *)&pkt, sizeof(pkt));
        }

        delay(10); // Minimal delay to prevent watchdog issues
        return;    // Skip normal BEAN message processing when in test mode
    }

    // ============================================================================
    // Normal BEAN Message Processing
    // ============================================================================
    noInterrupts();
    uint32_t now = micros();
    uint32_t time_since_last_edge = now - last_edge_time;
    bool message_ready = (pulse_count > 0 && time_since_last_edge > 600);

    if (message_ready)
    {
        int count = pulse_count;

        // Copy the snapshot safely out of the ISR buffers instantly
        memcpy(raw_pulses, (const void *)isr_pulses, count * sizeof(pulse_t));

        // Finalize with the trailing End-Of-Message condition so logic can break out
        if (count < 512)
        {
            raw_pulses[count].dur = time_since_last_edge;
            raw_pulses[count].lvl = 0; // A timeout means the bus is resting at logical 0
            count++;
        }

        pulse_count = 0; // Arm tracker for next message
        interrupts();

        // Decode it
        decode_bean_message(count);
    }
    else
    {
        interrupts();
        // Using yield() instead of delay(1) prevents the loop from sleeping
        // and missing the critical >600us idle gap before the next message begins.
        yield();
    }

    uint32_t current_ms = millis();
    if (current_ms - last_sync_time >= 500)
    {
        pending_light_update = false;
        last_sync_time = current_ms;
        LightPacket pkt;
        pkt.type = 'L';
        pkt.on = current_light_state ? 1 : 0;
        esp_now_send(receiverAddress, (uint8_t *)&pkt, sizeof(pkt));
    }

    if (millis() - s_last_send_ok_ms > RADIO_WATCHDOG_MS)
    {
        Serial.println("Watchdog: radio hang detected, rebooting.");
        delay(100);
        ESP.restart();
    }
}
