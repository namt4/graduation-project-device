/* MQTT over Websockets Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include <cJSON.h>
#include "ws2812b.h"
#include "bme680.h"
#include <math.h>
#include "wifi_manager.h"
static const char *TAG = "MQTTWS_EXAMPLE";
/* bme680 */
#define PORT 0
#define ADDR BME680_I2C_ADDR_1
#define CONFIG_EXAMPLE_I2C_MASTER_SCL 22
#define CONFIG_EXAMPLE_I2C_MASTER_SDA 21
char json_data[] = "{\"Device\":\"BME680\",\"temperature\": 29,\"humidity\": 90,\"pressure\": 875,\"altitude\": 26}";
uint32_t led_state = 0;
typedef struct {
    char *state;
    uint8_t brightness;
    uint8_t color[3];
    char *effect;
} rgb_light;

typedef enum {
    OFF,
    COLOR,
    EFFECT_RAINBOW,
    EFFECT_WAVEFORM,
    EFFECT_FLASMA_WAVE,
    WIFI_DISCONNECT
} effect_t;
//char *updated_json_data = "{\"state\": \"on\", \"brightness\": 255, \"color\": [255, 255, 255], \"effect\": \"none\"}";
esp_mqtt_client_handle_t client;
effect_t currentEffect = COLOR;
effect_t saveEffect = COLOR;

rgb_light light_data = {
    .state = "on",
    .brightness = 255,
    .color = {255, 255, 255},
    .effect = "color"
};

#define PLASMA_WIDTH 8
#define PLASMA_HEIGHT 1
#define PLASMA_SCALE 20.0

// Hàm tính giá trị plasma cho mỗi pixel
float plasma_value(int x, int y, float time)
{
    float cx = x / PLASMA_SCALE;
    float cy = y / PLASMA_SCALE;
    float value = sinf(cx + time) + sinf(cy + time) + sinf(sqrtf(cx * cx + cy * cy) + time);
    return value;
}

// Hàm tạo hiệu ứng plasma_waves
void plasma_waves_effect(float time)
{
    for (int i = 0; i < 8; i++) {
        // Tính toán giá trị plasma cho từng pixel
        float value = plasma_value(i, 0, time);

        // Chuyển đổi giá trị plasma thành màu RGB
        uint32_t r, g, b;
        led_strip_hsv2rgb((int)((value + 1.0) * 180), 100, 100, &r, &g, &b);

        // Ghi giá trị màu lên LED
        ws2812_write_led(i, r * light_data.brightness / 255, g * light_data.brightness / 255, b * light_data.brightness / 255);
    }

    // Cập nhật hiển thị
    ws2812_set_refresh();
}


void parse_rgb_light_data(rgb_light* light, char* json_data);
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void parse_rgb_light_data(rgb_light* light, char* json_data)
{
    cJSON* root = cJSON_Parse(json_data);
    if (root == NULL) {
        printf("JSON parse error.\n");
        return;
    }

    // Truy xuất giá trị của các đối tượng trong chuỗi JSON
    cJSON* state      = cJSON_GetObjectItemCaseSensitive(root, "state");
    cJSON* brightness = cJSON_GetObjectItemCaseSensitive(root, "brightness");
    cJSON* color      = cJSON_GetObjectItemCaseSensitive(root, "color");
    cJSON* effect     = cJSON_GetObjectItemCaseSensitive(root, "effect");

    // Kiểm tra và truy xuất giá trị của mỗi đối tượng
    if (cJSON_IsString(state) && (state->valuestring != NULL)) {
        light->state = strdup(state->valuestring);
        printf("State: %s\n", light->state);
    }

    if (cJSON_IsNumber(brightness)) {
        light->brightness = (uint8_t)brightness->valueint;
        printf("Brightness: %d\n", light->brightness);
    }

    if (cJSON_IsArray(color)) {
        if (cJSON_GetArraySize(color) >= 3) {
            cJSON* r = cJSON_GetArrayItem(color, 0);
            cJSON* g = cJSON_GetArrayItem(color, 1);
            cJSON* b = cJSON_GetArrayItem(color, 2);
            if (cJSON_IsNumber(r) && cJSON_IsNumber(g) && cJSON_IsNumber(b)) {
                light->color[0] = (uint8_t)r->valueint;
                light->color[1] = (uint8_t)g->valueint;
                light->color[2] = (uint8_t)b->valueint;
                printf("Color: [%d, %d, %d]\n", light->color[0], light->color[1], light->color[2]);
            }
        }
    }

    if (cJSON_IsString(effect) && (effect->valuestring != NULL)) {
        light->effect = strdup(effect->valuestring);
        printf("Effect: %s\n", light->effect);
    }

    // Giải phóng bộ nhớ
    cJSON_Delete(root);
}

void update_json_value(rgb_light data, char *json_data_update)
{
     cJSON* root = cJSON_Parse(json_data_update);
    if (root == NULL) {
        printf("Failed to parse JSON data.\n");
    }
    cJSON_ReplaceItemInObject(root, "state", cJSON_CreateString(data.state));
    cJSON_ReplaceItemInObject(root, "brightness", cJSON_CreateNumber(data.brightness));

    cJSON* color = cJSON_GetObjectItemCaseSensitive(root, "color");
    cJSON_SetIntValue(cJSON_GetArrayItem(color, 0), data.color[0]);
    cJSON_SetIntValue(cJSON_GetArrayItem(color, 1), data.color[1]);
    cJSON_SetIntValue(cJSON_GetArrayItem(color, 2), data.color[2]);

    cJSON_ReplaceItemInObject(root, "effect", cJSON_CreateString(data.effect));

    // Chuyển đổi chuỗi JSON đã cập nhật thành chuỗi mới
    char *updated_json_data = cJSON_PrintUnformatted(root);
    //strcpy(json_data_update, updated_json_data);
    // In ra chuỗi JSON đã cập nhật
    printf("Updated JSON: %s\n", updated_json_data);
    esp_mqtt_client_publish(client, "home/rgb1/status", updated_json_data, 0, 0, 0);
    // Giải phóng bộ nhớ
    cJSON_Delete(root);
    free(updated_json_data);
}

void mqtt_handle_data(char* json_data)
{
    /* khởi tạo các giá trị cần lưu trũ để truyền con trỏ vào hàm tách */

    /* tách các thuộc tính bằng 1 hàm truyền vào đó các con trỏ chứa giá trị, chuỗi json */
    
    parse_rgb_light_data(&light_data, json_data);
    printf("parse_rgb_light_data finish\n");
    /* gửi giá trị dữ liệu tới hàm điều khiển led tương ứng với các effect */
    /*     printf("State: %s\n", light_data.state);
        printf("Brightness: %d\n", light_data.brightness);
        printf("Color: [%d, %d, %d]\n", light_data.color[0], light_data.color[1], light_data.color[2]);
        printf("Effect: %s\n", light_data.effect); */
    if (strncmp(light_data.state, "off", 3) == 0) {
        currentEffect = OFF;
        esp_mqtt_client_publish(client, "home/rgb1/status", "{\"state\": \"off\"}", 0, 0, 0);
    } else if (strncmp(light_data.effect, "rainbow", 7) == 0) {
        currentEffect = EFFECT_RAINBOW;

        update_json_value(light_data, json_data); //vì trên homeassistant gửi xuống gì thì gửi lại y hệt không cần update gì cả
        // printf("chuoi json: %s\n", json_data);

        // esp_mqtt_client_publish(client, "home/rgb1/status", json_data, 0, 0, 0);

    } else if (strncmp(light_data.effect, "flasma wave", 11) == 0) {
        currentEffect = EFFECT_FLASMA_WAVE;

        update_json_value(light_data, json_data);
        // esp_mqtt_client_publish(client, "home/rgb1/status", json_data, 0, 0, 0);
    } else if (strncmp(light_data.effect, "color", 5) == 0) {
        currentEffect = COLOR;
        update_json_value(light_data, json_data);
        // esp_mqtt_client_publish(client, "home/rgb1/status", json_data, 0, 0, 0);
    } else if (strncmp(light_data.effect, "waveform", 8) == 0) {
        currentEffect = EFFECT_WAVEFORM;
        update_json_value(light_data, json_data);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "home/rgb1/status", "{\"state\": \"on\", \"brightness\": 255, \"color\": [255, 255, 255], \"effect\": \"color\"}", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "home/rgb1/set", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        /* char *again_json = NULL;
        strcpy(again_json, event->data);
        printf("again_json: %s\n", again_json); */
        //esp_mqtt_client_publish(client, "home/rgb1/status", event->data, 0, 0, 0);
        mqtt_handle_data(event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .client_id = "my_esp_idf_device",
        .username = "mqttuser",
        .password = "mqttpassword",
        .uri = "mqtt://homeassistant.local/:1883"
        /* .protocol = "ws" */
        /* .uri = "mqtt://192.168.1.100:1883" */
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void ws2812_task(void* pvParameters)
{
    int count          = 0;
    uint32_t red       = 0;
    uint32_t green     = 0;
    uint32_t blue      = 0;
    uint16_t hue       = 0;
    uint16_t start_rgb = 0;

    float phase = 0.0; // Biến phase để điều khiển hình dạng sóng
    while (1) {
        // Kiểm tra trạng thái hiệu ứng và thực hiện các hiệu ứng tương ứng
        switch (currentEffect) {
        case COLOR:
            for (int i = 0; i < 8; i++) {
                ws2812_set_pixel(i, light_data.color[0] * light_data.brightness / 255, light_data.color[1] * light_data.brightness / 255, light_data.color[2] * light_data.brightness / 255);
            }
            ws2812_set_refresh();
            if (count % 100 == 0) {
                printf("COLOR\n");
                // printf("RGB value: %x\n", new_state.leds[0]);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            break;
        case EFFECT_RAINBOW:
            if (count % 100 == 0) {
                printf("EFFECT_RAINBOW\n");
            }
            for (int i = 0; i < 3; i++) {
                for (int j = i; j < 8; j += 3) {
                    hue = j * 360 / 8 + start_rgb;
                    led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                    ws2812_set_pixel(j, red * light_data.brightness / 255, green * light_data.brightness / 255, blue * light_data.brightness / 255);
                }
                ws2812_set_refresh();
                vTaskDelay(pdMS_TO_TICKS(10));
                ws2812_clear_led();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            start_rgb += 60;
            break;
        case EFFECT_WAVEFORM:
            if (count % 100 == 0) {
                printf("EFFECT_WAVEFORM\n");
            }
            // Tính toán giá trị màu sắc dựa trên phase
            for (int i = 0; i < 8; i++) {
                float value = sinf(phase + i * 0.1) * 5;

                // Chuyển đổi giá trị HSV sang RGB
                uint32_t r, g, b;
                led_strip_hsv2rgb((uint32_t)value * 60, 100, 100, &r, &g, &b);

                ws2812_set_pixel(i, (uint8_t)r * light_data.brightness / 255, (uint8_t)g * light_data.brightness / 255, (uint8_t)b * light_data.brightness / 255);
            }

            // Cập nhật hiển thị
            ws2812_set_refresh();

            // Tăng giá trị phase để tạo hiệu ứng chuyển động
            phase += 0.1;

            // Delay để tạo hiệu ứng
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        case EFFECT_FLASMA_WAVE:
            if (count % 100 == 0) {
                printf("EFFECT_FLASMA_WAVE\n");
            }
            // Lấy thời gian hiện tại (đơn vị ms)
            uint32_t current_time = esp_timer_get_time() / 1000;

            // Gọi hiệu ứng plasma_waves với thời gian hiện tại
            plasma_waves_effect(current_time);

            // Delay để cập nhật hiệu ứng mỗi 10ms
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        case OFF:
            if (count % 100 == 0) {
                printf("OFF\n");
                // printf("RGB value: %x\n", new_state.leds[0]);
            }
            ws2812_clear_led();
            break;
        case WIFI_DISCONNECT:
            if (count % 100 == 0) {
                printf("WIFI_DISCONNECT\n");
                // printf("RGB value: %x\n", new_state.leds[0]);
            }
            for (int i = 0; i < 8; i++){
                ws2812_write_led(i, 255, 255, 255);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            ws2812_clear_led();
            break;
        default:
            break;
        }

        // Tiếp tục các hoạt động khác
        // ...
        count++;
    }

}
void bme680_data_update(float temperature, float humidity, float pressure, float altitude)
{
    cJSON* root       = cJSON_Parse(json_data);
    if (root == NULL) {
        printf("Failed to parse JSON data.\n");
    }

    // Tiếp theo, bạn có thể truy cập và cập nhật các giá trị trong chuỗi JSON bằng cách sử dụng các hàm cJSON_GetObjectItem() và cJSON_SetNumberValue():

    // Cập nhật giá trị "temperature"
    cJSON* temperature_item = cJSON_GetObjectItem(root, "temperature");
    if (temperature_item != NULL) {
        cJSON_SetNumberValue(temperature_item, roundf(temperature * 100) / 100); // Giá trị mới của temperature
    }

    // Cập nhật giá trị "humidity"
    cJSON* humidity_item = cJSON_GetObjectItem(root, "humidity");
    if (humidity_item != NULL) {
        cJSON_SetNumberValue(humidity_item, roundf(humidity * 100) / 100); // Giá trị mới của humidity
    }

    // Cập nhật giá trị "pressure"
    cJSON* pressure_item = cJSON_GetObjectItem(root, "pressure");
    if (pressure_item != NULL) {
        cJSON_SetNumberValue(pressure_item, roundf(pressure * 100) / 100); // Giá trị mới của pressure
    }

    // Cập nhật giá trị "altitude"
    cJSON* altitude_item = cJSON_GetObjectItem(root, "altitude");
    if (altitude_item != NULL) {
        cJSON_SetNumberValue(altitude_item, roundf(altitude * 100) / 100); // Giá trị mới của altitude
    }

    // Chuyển đổi lại chuỗi JSON đã cập nhật
    char* updated_json_data = cJSON_PrintUnformatted(root);

    if (updated_json_data == NULL) {
        printf("Failed to convert JSON data to string.\n");
        cJSON_Delete(root);
    }

    // In chuỗi JSON đã cập nhật
    printf("Updated JSON data: %s\n", updated_json_data);
    esp_mqtt_client_publish(client, "office/sensor", updated_json_data, 0, 0, 0);
    // Giải phóng bộ nhớ đã cấp phát
    cJSON_Delete(root);
    free(updated_json_data);
}
void bme680_task(void *pvParameters)
{
    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_4X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 25);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    //TickType_t last_wakeup = xTaskGetTickCount();

    bme680_values_float_t bme680_data;
    while (1)
    {

        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(duration);

            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &bme680_data) == ESP_OK) {
                printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       bme680_data.temperature, bme680_data.humidity,
                       bme680_data.pressure, bme680_data.gas_resistance);
                bme680_data_update(bme680_data.temperature, bme680_data.humidity,
                       bme680_data.pressure, bme680_data.gas_resistance);
            }
                
            
        }
        // passive waiting until 1 second is over
        /* BaseType_t xStatus = xQueueReceive(uv_queue, &sensor_messages_data.uv_values, portMAX_DELAY);
        if ( xStatus == pdPASS ) {
            xQueueSend( sensor_queue, ( void * )&sensor_messages_data, portMAX_DELAY);
        } */
        //xQueueSend(bme680_queue, (void *)&bme680_data, portMAX_DELAY);
        /* vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000)); */
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief this is an exemple of a callback that you can setup in your own app to get notified of wifi manager event.
 */
void cb_connection_ok(void *pvParameter){
	ip_event_got_ip_t* param = (ip_event_got_ip_t*)pvParameter;

	/* transform IP to human readable string */
	char str_ip[16];
	esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

	ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

void wifi_event_handler(esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		printf("No_Led_state\n");
        led_state = 0;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        led_state = 1;
		printf("Led_state\n");
    }

}

/* button */
void IRAM_ATTR gpio_isr_handler(void* arg)
{
    static TickType_t start_time = 0;
    TickType_t end_time          = xTaskGetTickCount();

    if (gpio_get_level(GPIO_NUM_0) == 0) {
        start_time = end_time;
    } else {
        TickType_t press_duration = end_time - start_time;
        if (press_duration >= pdMS_TO_TICKS(3000)) {
            gpio_set_level(2, 0);
            change_state_wifi();
        } else {
			gpio_set_level(2, 1);
		}
        start_time = 0;
    }
}

void GPIO_intr_init(gpio_num_t gpio_pin, gpio_int_type_t intr_type)
{

    // Cấu hình GPIO làm input
    gpio_pad_select_gpio(gpio_pin);
    gpio_set_direction(gpio_pin, GPIO_MODE_INPUT);
    // Cấu hình ngắt GPIO
    gpio_set_intr_type(gpio_pin, intr_type); // Ngắt khi cạnh lên, xuống
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_pin, gpio_isr_handler, (void*)gpio_pin);
}

void wifi_led_task(void* pvParameters)
{
	while(1)
	{
		if(led_state == 0) {
		gpio_set_level(2, 1);  // Bật đèn
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(2, 0);  // Tắt đèn
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		} else {
			gpio_set_level(2, 1);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		//vTaskDelay(3000 / portTICK_PERIOD_MS);
	}

}


void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_WS", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_netif_init());
   // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
    /* start the wifi manager */
	wifi_manager_start();

	/* register a callback as an example to how you can integrate your code with the wifi manager */
	wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);
    wifi_handler_set_callback(wifi_event_handler);

	GPIO_intr_init(GPIO_NUM_0, GPIO_INTR_ANYEDGE);
    gpio_pad_select_gpio(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
	gpio_set_level(2, 1);

    ws2812_init(18, 8);
    mqtt_app_start();

    xTaskCreate(wifi_led_task, "wifi_led_task", 2048, NULL, 1, NULL);
    xTaskCreate(ws2812_task, "ws2810_task", 2048, NULL, 1, NULL);
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreatePinnedToCore(bme680_task, "bme680_task", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL, 0);
    //memset(&light_data, 0, sizeof(rgb_light));
    
}
