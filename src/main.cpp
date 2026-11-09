#include <Arduino.h>
#include "uart_switch_protocol.h"
#include "uart_update_protocol.h"
#include <esp_ota_ops.h>
#include <driver/gpio.h>
#include <mbedtls/sha256.h>

static HardwareSerial uart_link(1);
static switch_parser_t parser = {};
static constexpr uint8_t outputs[] = {1, 2, 3, 4};
static uint8_t state;
static uint32_t last_command;
static bool armed;
static update_parser_t update_parser = {};
static esp_ota_handle_t ota_handle;
static const esp_partition_t *ota_partition;
static mbedtls_sha256_context ota_sha;
static uint8_t expected_digest[32];
static uint32_t image_size;
static uint32_t image_offset;
static uint32_t last_update_byte;
static uint32_t last_block_offset;
static uint16_t last_block_length;
static uint16_t last_block_crc;
static bool updating;

static void set_outputs(uint8_t mask)
{
    state = mask & 0x0F;
    for(unsigned i = 0; i < 4; i++)
        digitalWrite(outputs[i], state & (1U << i) ? HIGH : LOW);
}

static void abort_update()
{
    if(updating) {
        esp_ota_abort(ota_handle);
        mbedtls_sha256_free(&ota_sha);
        updating = false;
        Serial.println("Mainboard OTA: update aborted");
    }
}

static void reply_update(uint8_t command, uint32_t offset, bool success)
{
    const uint8_t response[] = {command, (uint8_t)(success ? 0 : 1)};
    uint8_t frame[UPDATE_FRAME_MAX];
    size_t size = update_encode(frame, UPDATE_REPLY, offset, response, sizeof(response));
    uart_link.write(frame, size);
}

static void handle_update_frame()
{
    const uint8_t *frame = update_parser.bytes;
    const uint8_t command = frame[2];
    const uint32_t offset = update_get_u32(frame + 3);
    const uint16_t length = ((uint16_t)frame[7] << 8) | frame[8];
    const uint8_t *data = frame + UPDATE_HEADER_SIZE;
    bool success = false;

    if(command == UPDATE_BEGIN && offset == 0 && length == 36) {
        set_outputs(0);
        armed = false;
        const uint32_t size = update_get_u32(data);
        if(updating && image_size == size && memcmp(expected_digest, data + 4, 32) == 0) {
            success = image_offset == 0;
        } else if(!updating) {
            ota_partition = esp_ota_get_next_update_partition(nullptr);
            if(ota_partition && size > 0 && size <= ota_partition->size &&
               esp_ota_begin(ota_partition, size, &ota_handle) == ESP_OK) {
                mbedtls_sha256_init(&ota_sha);
                if(mbedtls_sha256_starts(&ota_sha, 0) == 0) {
                    image_size = size;
                    image_offset = 0;
                    last_block_length = 0;
                    memcpy(expected_digest, data + 4, sizeof(expected_digest));
                    updating = true;
                    success = true;
                } else {
                    mbedtls_sha256_free(&ota_sha);
                    esp_ota_abort(ota_handle);
                }
            }
        }
    } else if(command == UPDATE_DATA && updating && length > 0) {
        const uint16_t crc = switch_crc16(data, length);
        if(offset == last_block_offset && length == last_block_length &&
           crc == last_block_crc && offset + length == image_offset) {
            success = true;
        } else if(offset == image_offset && length <= image_size - image_offset) {
            if(esp_ota_write(ota_handle, data, length) == ESP_OK &&
               mbedtls_sha256_update(&ota_sha, data, length) == 0) {
                last_block_offset = offset;
                last_block_length = length;
                last_block_crc = crc;
                image_offset += length;
                success = true;
            } else {
                abort_update();
            }
        }
    } else if(command == UPDATE_FINISH && updating && length == 0 &&
              offset == image_size && image_offset == image_size) {
        uint8_t digest[32];
        success = mbedtls_sha256_finish(&ota_sha, digest) == 0 &&
                  memcmp(digest, expected_digest, sizeof(digest)) == 0;
        mbedtls_sha256_free(&ota_sha);
        if(success) {
            success = esp_ota_end(ota_handle) == ESP_OK;
            updating = false;
            if(success) success = esp_ota_set_boot_partition(ota_partition) == ESP_OK;
        } else {
            esp_ota_abort(ota_handle);
            updating = false;
        }
        reply_update(command, offset, success);
        if(success) {
            uart_link.flush();
            delay(100);
            ESP.restart();
        }
        return;
    } else if(command == UPDATE_ABORT && length == 0) {
        abort_update();
        set_outputs(0);
        armed = false;
        success = true;
    }
    reply_update(command, offset, success);
}

void setup()
{
    for(uint8_t pin : outputs) {
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        pinMode(pin, OUTPUT);
    }
    uart_link.begin(115200, SERIAL_8N1, 44, 43);
}

void loop()
{
    const uint32_t now = millis();
    if(armed && now - last_command >= 2000) {
        set_outputs(0);
        armed = false;
    }
    if(update_parser.used && now - last_update_byte >= 500) update_parser.used = 0;
    if(updating && now - last_update_byte >= 15000) {
        abort_update();
        set_outputs(0);
        armed = false;
    }
    while(uart_link.available()) {
        const uint8_t value = (uint8_t)uart_link.read();
        if(update_parser.used || (parser.used == 0 && value == 0xC3)) {
            last_update_byte = millis();
            if(update_parse(&update_parser, value)) {
                handle_update_frame();
                last_update_byte = millis();
            }
            continue;
        }
        if(updating) continue;
        if(!switch_parse(&parser, value)) continue;
        if(parser.data[2] != SWITCH_SET || (parser.data[4] & 0xF0)) continue;
        if(!armed && parser.data[4] == 0) armed = true;
        if(armed) {
            set_outputs(parser.data[4]);
            last_command = now;
        }
        uint8_t reply[SWITCH_FRAME_SIZE];
        switch_encode(reply, SWITCH_ACK, parser.data[3], state);
        uart_link.write(reply, sizeof(reply));
    }
    delay(1);
}
