#include <Arduino.h>
#include "uart_switch_protocol.h"
#include "uart_update_protocol.h"
#include "mainboard_version.h"
#include <esp_ota_ops.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
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
static bool pressure_streaming;
static uint32_t last_pressure_subscription;
static uint32_t last_pressure_sent;
static adc_oneshot_unit_handle_t adc_handle;
static bool adc_read_failed;
static constexpr uint8_t PRESSURE_PIN = 5;
static constexpr uint8_t AUX_ANALOG_PIN = 6;
static constexpr uint32_t ADC_MAX = 4095;
static constexpr unsigned PRESSURE_SAMPLES = 4;
static constexpr unsigned PRESSURE_AVERAGE_WINDOW = 8;
static constexpr uint32_t PRESSURE_INTERVAL_MS = 16;
static constexpr uint32_t PRESSURE_SUBSCRIPTION_TIMEOUT_MS = 750;
static uint16_t pressure_samples[PRESSURE_AVERAGE_WINDOW];
static uint32_t pressure_sum;
static unsigned pressure_count;
static unsigned pressure_index;

static void reset_pressure_average()
{
    pressure_sum = 0;
    pressure_count = 0;
    pressure_index = 0;
}

static bool init_analog_inputs()
{
    adc_oneshot_unit_init_cfg_t unit_config = {};
    unit_config.unit_id = ADC_UNIT_1;
    unit_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if(err != ESP_OK) {
        Serial.printf("ADC: unit initialization failed (%d)\n", err);
        return false;
    }
    const adc_oneshot_chan_cfg_t channel_config = {ADC_ATTEN_DB_12, ADC_BITWIDTH_12};
    for(uint8_t pin : {PRESSURE_PIN, AUX_ANALOG_PIN}) {
        adc_unit_t unit;
        adc_channel_t channel;
        err = adc_oneshot_io_to_channel(pin, &unit, &channel);
        if(err == ESP_OK && unit == ADC_UNIT_1)
            err = adc_oneshot_config_channel(adc_handle, channel, &channel_config);
        else if(err == ESP_OK)
            err = ESP_ERR_INVALID_ARG;
        if(err != ESP_OK) {
            Serial.printf("ADC: GPIO%u configuration failed (%d)\n", pin, err);
            adc_oneshot_del_unit(adc_handle);
            adc_handle = nullptr;
            return false;
        }
    }
    return true;
}

static bool read_pressure(uint16_t *pressure)
{
    if(!adc_handle) return false;
    uint32_t sum = 0;
    for(unsigned i = 0; i < PRESSURE_SAMPLES; i++) {
        int raw;
        const esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &raw);
        if(err != ESP_OK) {
            if(!adc_read_failed) Serial.printf("ADC: GPIO5 read failed (%d)\n", err);
            adc_read_failed = true;
            reset_pressure_average();
            return false;
        }
        sum += static_cast<uint32_t>(raw);
    }
    if(adc_read_failed) Serial.println("ADC: GPIO5 read recovered");
    adc_read_failed = false;
    if(pressure_count == PRESSURE_AVERAGE_WINDOW)
        pressure_sum -= pressure_samples[pressure_index];
    else
        pressure_count++;
    pressure_samples[pressure_index] = sum;
    pressure_sum += sum;
    pressure_index = (pressure_index + 1) % PRESSURE_AVERAGE_WINDOW;
    const uint32_t divisor = ADC_MAX * PRESSURE_SAMPLES * pressure_count;
    *pressure = (pressure_sum * SWITCH_PRESSURE_MAX_CENTIBAR + divisor / 2) / divisor;
    return true;
}

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
    Serial.begin(115200);
    for(uint8_t pin : outputs) {
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        pinMode(pin, OUTPUT);
    }
    init_analog_inputs();
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
            if(pressure_streaming) reset_pressure_average();
            pressure_streaming = false;
            last_update_byte = millis();
            if(update_parse(&update_parser, value)) {
                handle_update_frame();
                last_update_byte = millis();
            }
            continue;
        }
        if(updating) continue;
        if(!switch_parse(&parser, value)) continue;
        if(parser.data[2] == SWITCH_PRESSURE_STREAM && parser.data[4] <= 1) {
            if(pressure_streaming != (parser.data[4] == 1))
                reset_pressure_average();
            pressure_streaming = parser.data[4] == 1;
            last_pressure_subscription = millis();
            continue;
        }
        if(parser.data[2] == SWITCH_VERSION_REQUEST && parser.data[4] == 0) {
            const uint8_t version[] = {MAINBOARD_VERSION_MAJOR, MAINBOARD_VERSION_MINOR,
                                       MAINBOARD_VERSION_PATCH};
            for(unsigned i = 0; i < sizeof(version); i++) {
                uint8_t reply[SWITCH_FRAME_SIZE];
                switch_encode(reply, SWITCH_VERSION_MAJOR + i, parser.data[3], version[i]);
                uart_link.write(reply, sizeof(reply));
            }
            continue;
        }
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
    const uint32_t pressure_now = millis();
    if(pressure_streaming && pressure_now - last_pressure_subscription >= PRESSURE_SUBSCRIPTION_TIMEOUT_MS) {
        pressure_streaming = false;
        reset_pressure_average();
    }
    if(pressure_streaming && !updating && pressure_now - last_pressure_sent >= PRESSURE_INTERVAL_MS &&
       uart_link.availableForWrite() >= SWITCH_FRAME_SIZE) {
        uint16_t pressure;
        if(read_pressure(&pressure)) {
            uint8_t frame[SWITCH_FRAME_SIZE];
            switch_encode(frame, SWITCH_PRESSURE, (uint8_t)(pressure >> 8), (uint8_t)pressure);
            uart_link.write(frame, sizeof(frame));
        }
        last_pressure_sent = pressure_now;
    }
    delay(1);
}
