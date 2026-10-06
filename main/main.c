// Slave (AP) - hosts clients and exchanges raw frames with Master over SPI
// This implementation stores each captured frame in a queue as a single allocated block
// with the first two bytes holding the big-endian length, followed by payload.
#include "esp_private/wifi.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"

#define TAG "BRIDGE_SLAVE"

#define AP_SSID "Ahmed22"
#define AP_CHANNEL 1

/* SPI pins (match master wiring) */
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   5

/* Ready line: slave -> master (output) */
#define GPIO_SLAVE_READY 4

#define MAX_FRAME 1536
#define HEADER_SIZE 8
#define BUFFER_SIZE (HEADER_SIZE + MAX_FRAME)

static QueueHandle_t uplink_q; // stores pointers to allocated blocks (first 2 bytes = len)
static uint8_t *dma_tx_buf;
static uint8_t *dma_rx_buf;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t len_m2s;
    uint16_t len_s2m;
} spi_hdr_t;

static inline uint16_t be16(uint16_t v) { return (v >> 8) | (v << 8); }

static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t len = pkt->rx_ctrl.sig_len;
    if (len == 0 || len > MAX_FRAME) {
        esp_wifi_internal_free_rx_buffer(pkt);
        return;
    }
    // allocate len + 2
    uint8_t *block = heap_caps_malloc(len + 2, MALLOC_CAP_8BIT);
    if (!block) {
        esp_wifi_internal_free_rx_buffer(pkt);
        return;
    }
    block[0] = (len >> 8) & 0xFF;
    block[1] = len & 0xFF;
    memcpy(block + 2, pkt->payload, len);

    if (xQueueSend(uplink_q, &block, 0) != pdTRUE) {
        free(block);
    } else {
        // signal master that uplink exists
        gpio_set_level(GPIO_SLAVE_READY, 1);
    }

    esp_wifi_internal_free_rx_buffer(pkt);
}

static void init_wifi_ap(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = { 0 };
    strcpy((char*)ap_cfg.ap.ssid, AP_SSID);
    ap_cfg.ap.ssid_len = strlen(AP_SSID);
    ap_cfg.ap.channel = AP_CHANNEL;
    ap_cfg.ap.max_connection = 8;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous_rx_cb);
    ESP_LOGI(TAG, "AP started (promiscuous enabled)");
}

static void init_spi_slave(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_SLAVE_READY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_SLAVE_READY, 0);

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
    };

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    dma_tx_buf = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    dma_rx_buf = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!dma_tx_buf || !dma_rx_buf) {
        ESP_LOGE(TAG, "dma alloc failed");
        abort();
    }
}

static void spi_slave_task(void *pv)
{
    while (1) {
        // prepare default response header (no uplink)
        spi_hdr_t hdr = { .magic = 0xA5A5, .version = 1, .flags = 0, .len_m2s = 0, .len_s2m = 0 };
        memset(dma_tx_buf, 0, BUFFER_SIZE);
        memcpy(dma_tx_buf, &hdr, HEADER_SIZE);

        // if uplink available, dequeue and prepare payload
        uint8_t *block = NULL;
        uint16_t uplen = 0;
        if (xQueueReceive(uplink_q, &block, 0) == pdTRUE) {
            uplen = (block[0] << 8) | block[1];
            if (uplen > MAX_FRAME) uplen = MAX_FRAME;
            hdr.len_s2m = be16(uplen);
            memcpy(dma_tx_buf, &hdr, HEADER_SIZE);
            memcpy(dma_tx_buf + HEADER_SIZE, block + 2, uplen);
            // keep READY high until transaction completes (master will read it at next loop)
            gpio_set_level(GPIO_SLAVE_READY, 1);
        }

        spi_slave_transaction_t t;
        memset(&t, 0, sizeof(t));
        // allow master to read up to BUFFER_SIZE; actual meaningful bytes controlled by header
        t.length = (HEADER_SIZE + MAX_FRAME) * 8;
        t.tx_buffer = dma_tx_buf;
        t.rx_buffer = dma_rx_buf;

        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "spi_slave_transmit failed %d", ret);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // parse received downlink header
        spi_hdr_t rx_hdr;
        memcpy(&rx_hdr, dma_rx_buf, HEADER_SIZE);
        if (rx_hdr.magic == 0xA5A5) {
            uint16_t down_len = be16(rx_hdr.len_m2s);
            if (down_len > 0 && down_len <= MAX_FRAME) {
                // inject into AP interface for local clients
                esp_err_t e = esp_wifi_80211_tx(WIFI_IF_AP, dma_rx_buf + HEADER_SIZE, down_len, false);
                if (e != ESP_OK) {
                    ESP_LOGW(TAG, "AP inject failed %d", e);
                }
            }
        }

        // transaction done: lower READY and free uplink block if used
        gpio_set_level(GPIO_SLAVE_READY, 0);
        if (block) {
            free(block);
            block = NULL;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    uplink_q = xQueueCreate(128, sizeof(void*));
    if (!uplink_q) {
        ESP_LOGE(TAG, "uplink queue create failed");
        return;
    }
    init_spi_slave();
    init_wifi_ap();
    xTaskCreatePinnedToCore(spi_slave_task, "spi_slave_task", 8 * 1024, NULL, 5, NULL, 1);
}
