# Master (STA) - Bridge to Router (open network)
# Connects to router in STA mode, captures raw 802.11 frames, sends to slave via SPI,
# and injects uplink frames received from slave into router.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

#define TAG "BRIDGE_MASTER"

/* Router (open) */
#define ROUTER_SSID  "YOUR_ROUTER_SSID"

/* SPI pins (adjust to your wiring) */
#define GPIO_MOSI      23
#define GPIO_MISO      19
#define GPIO_SCLK      18
#define GPIO_CS        5

/* Slave -> Master ready line (input) */
#define GPIO_SLAVE_READY 4

#define MAX_FRAME 1536
#define HEADER_SIZE 8
#define BUFFER_SIZE (HEADER_SIZE + MAX_FRAME)

static spi_device_handle_t spi_handle;
static RingbufHandle_t tx_ringbuf; // stores pointers to allocated blocks that start with 2-byte length
static uint8_t *dma_tx_buf;
static uint8_t *dma_rx_buf;

typedef struct __attribute__((packed)) {
    uint16_t magic;   // 0xA5A5
    uint8_t version;
    uint8_t flags;
    uint16_t len_m2s; // big endian
    uint16_t len_s2m; // big endian
} spi_hdr_t;

static inline uint16_t be16(uint16_t v) { return (v >> 8) | (v << 8); }

// promiscuous callback: copy payload into allocated block: [len_hi][len_lo][payload...]
static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t len = pkt->rx_ctrl.sig_len;
    if (len == 0 || len > MAX_FRAME) {
        esp_wifi_internal_free_rx_buffer(pkt);
        return;
    }
    // allocate len + 2 bytes to store length
    uint8_t *block = heap_caps_malloc(len + 2, MALLOC_CAP_8BIT);
    if (!block) {
        ESP_LOGW(TAG, "malloc fail - drop frame len=%d", len);
        esp_wifi_internal_free_rx_buffer(pkt);
        return;
    }
    block[0] = (len >> 8) & 0xFF;
    block[1] = len & 0xFF;
    memcpy(block + 2, pkt->payload, len);

    if (xRingbufferSend(tx_ringbuf, block, len + 2, 0) != pdTRUE) {
        ESP_LOGW(TAG, "tx ring full - drop");
        free(block);
    }
    esp_wifi_internal_free_rx_buffer(pkt);
}

static void init_spi_master(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_SLAVE_READY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_MISO,
        .mosi_io_num = GPIO_MOSI,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000, // 20MHz for higher throughput; change if unstable
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle));

    dma_tx_buf = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    dma_rx_buf = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!dma_tx_buf || !dma_rx_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers");
        abort();
    }
}

static void init_wifi_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, ROUTER_SSID, sizeof(wifi_config.sta.ssid));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_rx_cb);
    ESP_LOGI(TAG, "WiFi STA started (promiscuous)");
}

static void spi_master_task(void *pv)
{
    size_t item_size;
    uint8_t *item;
    spi_transaction_t t;

    while (1) {
        // prepare header default
        spi_hdr_t hdr = { .magic = 0xA5A5, .version = 1, .flags = 0, .len_m2s = 0, .len_s2m = 0 };
        uint16_t len_m2s = 0;
        uint16_t expected_s2m = 0;

        // check slave-ready pin (slave indicates it has uplink)
        if (gpio_get_level(GPIO_SLAVE_READY)) {
            // Ask slave to provide up to MAX_FRAME by setting len_s2m = 0xFFFF (slave will place actual value)
            hdr.len_s2m = be16(0xFFFF);
            expected_s2m = MAX_FRAME;
        }

        item = (uint8_t *) xRingbufferReceive(tx_ringbuf, &item_size, 0);
        if (item != NULL) {
            // item[0..1] = len, payload starts at item+2
            uint16_t payload_len = (item[0] << 8) | item[1];
            if (payload_len > MAX_FRAME) payload_len = MAX_FRAME;
            len_m2s = payload_len;
            hdr.len_m2s = be16(len_m2s);
            memset(dma_tx_buf, 0, BUFFER_SIZE);
            memcpy(dma_tx_buf, &hdr, HEADER_SIZE);
            memcpy(dma_tx_buf + HEADER_SIZE, item + 2, len_m2s);
            vRingbufferReturnItem(tx_ringbuf, (void *)item);
        } else {
            // no downlink payload: send only header (may still ask for uplink)
            memset(dma_tx_buf, 0, BUFFER_SIZE);
            memcpy(dma_tx_buf, &hdr, HEADER_SIZE);
        }

        uint16_t transfer_payload = (len_m2s > expected_s2m) ? len_m2s : expected_s2m;
        uint32_t bits = (HEADER_SIZE + transfer_payload) * 8;

        memset(&t, 0, sizeof(t));
        t.length = bits;
        t.tx_buffer = dma_tx_buf;
        t.rx_buffer = dma_rx_buf;

        esp_err_t ret = spi_device_transmit(spi_handle, &t);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "spi transmit failed %d", ret);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // parse rx header
        spi_hdr_t rx_hdr;
        memcpy(&rx_hdr, dma_rx_buf, HEADER_SIZE);
        if (rx_hdr.magic != 0xA5A5) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        uint16_t rx_len = be16(rx_hdr.len_s2m);
        if (rx_len > 0 && rx_len <= MAX_FRAME) {
            // inject into router
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, dma_rx_buf + HEADER_SIZE, rx_len, false);
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "esp_wifi_80211_tx failed: %d", e);
            }
        }

        // small yield to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Bridge Master starting...");
    tx_ringbuf = xRingbufferCreate(64 * 1024, RINGBUF_TYPE_NOSPLIT);
    if (!tx_ringbuf) {
        ESP_LOGE(TAG, "ringbuf create failed");
        return;
    }
    init_spi_master();
    init_wifi_sta();
    xTaskCreatePinnedToCore(spi_master_task, "spi_master_task", 8 * 1024, NULL, 5, NULL, 1);
}
