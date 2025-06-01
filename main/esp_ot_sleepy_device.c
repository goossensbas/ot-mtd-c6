/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_ot_sleepy_device_config.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "openthread/logging.h"
#include "openthread/thread.h"
#include "openthread/coap.h"


#include "driver/gpio.h"
#include "bme280.h"
// I2C configuration  found in bme280.h

#define I2C_TAG "I2C"
#define BME280_TAG "BME280"
#define COAP_TAG "CoAP"
#define COAP_URI_PATH "test"
#define COAP_SERVER "fd3f:4f29:5339:1ff9:ffc:7570:df42:e75f"
#define OT_COAP_PORT 5683

#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_private/esp_pmu.h"
#include "esp_private/esp_sleep_internal.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#if !SOC_IEEE802154_SUPPORTED
#error "Openthread sleepy device is only supported for the SoCs which have IEEE 802.15.4 module"
#endif

#define TAG "ot_esp_power_save"

static esp_pm_lock_handle_t s_cli_pm_lock = NULL;
static esp_pm_lock_handle_t s_i2c_pm_lock = NULL;

#if CONFIG_OPENTHREAD_AUTO_START
static void create_config_network(otInstance *instance)
{
    otLinkModeConfig linkMode = { 0 };

    linkMode.mRxOnWhenIdle = false;
    linkMode.mDeviceType = false;
    linkMode.mNetworkData = false;

    if (otLinkSetPollPeriod(instance, CONFIG_OPENTHREAD_NETWORK_POLLPERIOD_TIME) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set OpenThread pollperiod.");
        abort();
    }

    if (otThreadSetLinkMode(instance, linkMode) != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set OpenThread linkmode.");
        abort();
    }

    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "No active dataset found!");
    }
    else {
        ESP_LOGE(TAG, "Active dataset found!");
    }

}
#endif // CONFIG_OPENTHREAD_AUTO_START

static esp_err_t esp_openthread_sleep_device_init(void)
{
    esp_err_t ret = ESP_OK;

    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "otcli", &s_cli_pm_lock);
    if (ret == ESP_OK) {
        esp_pm_lock_acquire(s_cli_pm_lock);
        ESP_LOGI(TAG, "Successfully created CLI pm lock");
    } else {
        if (s_cli_pm_lock != NULL) {
            esp_pm_lock_delete(s_cli_pm_lock);
            s_cli_pm_lock = NULL;
        }
        ESP_LOGI(TAG, " Failed to create CLI pm lock");
    }
    return ret;
}
static esp_err_t esp_i2c_sleep_device_init(void)
{
    esp_err_t ret = ESP_OK;

    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "i2c", &s_i2c_pm_lock);
    if (ret == ESP_OK) {
        esp_pm_lock_acquire(s_i2c_pm_lock);
        ESP_LOGI(TAG, "Successfully created CLI pm lock");
    } else {
        if (s_i2c_pm_lock != NULL) {
            esp_pm_lock_delete(s_i2c_pm_lock);
            s_i2c_pm_lock = NULL;
        }
        ESP_LOGI(TAG, " Failed to create CLI pm lock");
    }
    return ret;
}

static void process_state_change(otChangedFlags flags, void* context)
{
    otDeviceRole ot_device_role = otThreadGetDeviceRole(esp_openthread_get_instance());
    if(ot_device_role == OT_DEVICE_ROLE_CHILD) {
        if (s_cli_pm_lock != NULL) {
            esp_pm_lock_release(s_cli_pm_lock);
            esp_pm_lock_delete(s_cli_pm_lock);
            s_cli_pm_lock = NULL;
        }
    }
}

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}

#if CONFIG_ESP_SLEEP_DEBUG
static esp_sleep_context_t s_sleep_ctx;

static void print_sleep_flag(void *arg)
{
    ESP_LOGD(TAG, "sleep_flags %lu", s_sleep_ctx.sleep_flags);
    ESP_LOGD(TAG, "PMU_SLEEP_PD_TOP: %s", (s_sleep_ctx.sleep_flags & PMU_SLEEP_PD_TOP) ? "True":"False");
    ESP_LOGD(TAG, "PMU_SLEEP_PD_MODEM: %s", (s_sleep_ctx.sleep_flags & PMU_SLEEP_PD_MODEM) ? "True":"False");
}
#endif

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_dev_handle;

void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    for (uint8_t addr = 1; addr < 127; addr++) {  // I2C address range: 0x01 to 0x7F
        esp_err_t err = i2c_master_probe(i2c_bus_handle, addr, -1);
        if (err == ESP_OK) {
            ESP_LOGI(I2C_TAG, "Found device at address 0x%02X", addr);
        } 
    }

    ESP_LOGI(TAG, "I2C scan complete.");
}



// Function to initialize the I2C bus
static esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,   // SDA GPIO
        .scl_io_num = I2C_MASTER_SCL_IO,   // SCL GPIO
        .flags.enable_internal_pullup = true
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to initialize I2C master: %d", err);
    }

    // **Register BME280 as an I2C device**
    i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BME280_SENSOR_ADDR,
    .scl_speed_hz = 100000,
};

    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg , &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME280 device: %d", err);
    }

    return err;  // Ensure the function returns a success/fail result
}

//CoAP response function
void handle_coap_response(void *context, otMessage *message, const otMessageInfo *messageInfo, otError result) {
    if (result == OT_ERROR_NONE) {
        ESP_LOGI(COAP_TAG, "CoAP response received");
    } else {
        ESP_LOGE(COAP_TAG, "Failed to receive CoAP response: %d", result);
    }
}

//send CoAP message
void send_coap_message(void) {

    uint8_t data[8];
    int32_t t_fine;
    
    // Initialize the sensor and read calibration data
    bme280_read_calibration_data();

    bme280_read(0xF7, data, 8);  // Read pressure (3 bytes), temperature (3 bytes), humidity (2 bytes)

    int32_t adc_P = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t adc_T = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    int32_t adc_H = (int32_t)((data[6] << 8) | data[7]);

    int32_t temperature = bme280_compensate_T(adc_T, &t_fine);
    uint32_t pressure = bme280_compensate_P(adc_P, t_fine);
    uint32_t humidity = bme280_compensate_H(adc_H);
    
    // Log the results
    ESP_LOGI(BME280_TAG, "Temperature: %.2f�C", (float)temperature / 100.0);
    ESP_LOGI(BME280_TAG, "Pressure: %.2f hPa", (float)pressure / 100.0);
    ESP_LOGI(BME280_TAG, "Humidity: %.2f%%", (float)humidity / 1024.0);

    otError error;
    otMessage *message;
    otMessageInfo messageInfo;
    otMessageSettings messageSettings = { true, OT_COAP_PORT };
    
    otInstance *instance = esp_openthread_get_instance();
    if (instance == NULL) {
        ESP_LOGE(COAP_TAG, "OpenThread instance is NULL. Cannot send CoAP message.");
        return;
    }
    otCoapStart(instance, OT_DEFAULT_COAP_PORT);
    ESP_LOGI(COAP_TAG, "CoAP started on port %d", OT_DEFAULT_COAP_PORT);
    
    // Create a new CoAP message
    message = otCoapNewMessage(instance, NULL);
    if (message == NULL) {
        ESP_LOGE(COAP_TAG, "Failed to allocate CoAP message");
        return;
    }

    // Set CoAP message type and code (POST request)
    otCoapMessageInit(message, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
    otCoapMessageAppendUriPathOptions(message, COAP_URI_PATH);
    
    //set the message type to JSON
    otCoapMessageAppendContentFormatOption(message, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
    // Build JSON payload with sensor data
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"temperature\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f}",
             (float)temperature / 100.0, (float)pressure / 100.0, (float)humidity / 1024.0);
    
    //set a payload marker         
    otCoapMessageSetPayloadMarker(message);
    
    // Add the payload to the CoAP message
    otMessageAppend(message, payload, strlen(payload));

    // Set the destination address (replace with actual CoAP server address)
    memset(&messageInfo, 0, sizeof(messageInfo));
    otIp6AddressFromString(COAP_SERVER, &messageInfo.mPeerAddr); // Multicast or specific server address
    messageInfo.mPeerPort = OT_COAP_PORT;

    // Send the CoAP message
    error = otCoapSendRequest(instance, message, &messageInfo, handle_coap_response, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(COAP_TAG, "Failed to send CoAP message: %d", error);
        otMessageFree(message);
    } else {
        ESP_LOGI(COAP_TAG, "CoAP message sent successfully");
    }
}

static void ot_task_worker(void *aContext)
{
    ESP_LOGI(TAG, "Starting OpenThread task...");

    otError ret;
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Initialize the OpenThread stack
    ESP_LOGI(TAG, "Initializing OpenThread stack...");
    ESP_ERROR_CHECK(esp_openthread_init(&config));
     ESP_LOGI(TAG, "OpenThread stack initialized.");

    esp_openthread_lock_acquire(portMAX_DELAY);
    ret = otSetStateChangedCallback(esp_openthread_get_instance(), process_state_change, esp_openthread_get_instance());
    esp_openthread_lock_release();
    if(ret != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set state changed callback");
    }else {
        ESP_LOGI(TAG, "State change callback set successfully.");
    }
#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // The OpenThread log level directly matches ESP log level
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif

    // Initialize the OpenThread cli
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif
    esp_netif_t *openthread_netif;

    // Initialize the esp_netif bindings
    ESP_LOGI(TAG, "Attaching OpenThread to network interface...");
    openthread_netif = init_openthread_netif(&config);
    ESP_LOGI(TAG, "OpenThread network interface attached.");
    ESP_LOGI(TAG, "Setting default network interface...");
    esp_netif_set_default_netif(openthread_netif);
    ESP_LOGI(TAG, "Default network interface set.");
#if CONFIG_OPENTHREAD_AUTO_START
    create_config_network(esp_openthread_get_instance());
#endif // CONFIG_OPENTHREAD_AUTO_START

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif



#if CONFIG_ESP_SLEEP_DEBUG
    esp_sleep_set_sleep_context(&s_sleep_ctx);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    // create a timer to print the status of sleepy device
    int periods = 2000;
    const esp_timer_create_args_t timer_args = {
            .name = "print_sleep_flag",
            .arg  = NULL,
            .callback = &print_sleep_flag,
            .skip_unhandled_events = true,
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, periods * 1000));
#endif

    // Run the main loop
    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

static esp_err_t ot_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };

    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

void send_data_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Acquiring PM lock, preventing sleep...");
        if (s_i2c_pm_lock) {
            esp_pm_lock_acquire(s_i2c_pm_lock);  // Prevent sleep during processing
        }
        //ESP_ERROR_CHECK(i2c_master_bus_reset(i2c_bus_handle));
        ESP_LOGI(TAG, "ESP woke up, checking I2C bus...");
        if (i2c_bus_handle == NULL) {
            ESP_LOGW(TAG, "I2C bus handle is NULL, reinitializing...");
            ESP_ERROR_CHECK(i2c_master_init());
            bme280_init();
        }
        ESP_LOGI(TAG, "Sending BME280 data via CoAP...");
        send_coap_message();  // Send sensor data
        ESP_LOGI(TAG, "Releasing PM lock, allowing sleep...");

        if (s_i2c_pm_lock) {
            esp_pm_lock_release(s_i2c_pm_lock);  // Allow sleep after processing
        }

        ESP_LOGI(TAG, "Task sleeping before next send...");
        vTaskDelay(pdMS_TO_TICKS(30 * 1000));
        // esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);  // Wake up after 30s
        // ESP_LOGI(TAG, "Entering light sleep...");
        // esp_light_sleep_start();
        // ESP_LOGI(TAG, "Woke up from light sleep!");
    }
}

void app_main(void)
{
    // Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    ESP_ERROR_CHECK(ot_power_save_init());
    ESP_ERROR_CHECK(esp_openthread_sleep_device_init());
    //init the I2C master and the BME280
    ESP_ERROR_CHECK(i2c_master_init());  // Initialize I2C master
    ESP_LOGI(I2C_TAG, "I2C initialized successfully");
    i2c_scanner();
    bme280_init();

    xTaskCreate(send_data_task, "send_data_task", 4096, NULL, 10, NULL);
    xTaskCreate(ot_task_worker, "ot_power_save_main", 4096, NULL, 5, NULL);
    
// Light sleep is handled automatically by IEEE 802.15.4
    ESP_LOGI(TAG, "Waiting for light sleep wake-up...");
}
