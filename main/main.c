#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define TAG "CONFIG"

// GPIO pins for ultrasonic sensors
#define TRIG_PIN_1 GPIO_NUM_15
#define ECHO_PIN_1 GPIO_NUM_4
#define TRIG_PIN_2 GPIO_NUM_16
#define ECHO_PIN_2 GPIO_NUM_17
#define TRIG_PIN_3 GPIO_NUM_18
#define ECHO_PIN_3 GPIO_NUM_19
#define TRIG_PIN_4 GPIO_NUM_21
#define ECHO_PIN_4 GPIO_NUM_22
#define TRIG_PIN_5 GPIO_NUM_23
#define ECHO_PIN_5 GPIO_NUM_25

// Distance threshold to consider a space occupied (in cm)
#define DISTANCE_THRESHOLD 20

// Semaphore and Mutex
SemaphoreHandle_t display_semaphore;
SemaphoreHandle_t space_mutex;

// Global counter for available spaces
int available_spaces;

void init_ultrasonic_sensor(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
}

int read_distance(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    int distance = 0;
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10); // Send 10us pulse
    gpio_set_level(trig_pin, 0);

    // Measure echo pulse width
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) { // Wait for pulse to start
        if ((esp_timer_get_time() - start_time) > 1000) {
            return -1; // Timeout
        }
    }

    start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) { // Wait for pulse to end
        if ((esp_timer_get_time() - start_time) > 30000) {
            return -1; // Timeout
        }
    }
    uint32_t pulse_duration = esp_timer_get_time() - start_time;

    // Calculate distance in cm
    distance = pulse_duration / 58;
    return distance;
}

void sensor_task(void *param) {
    gpio_num_t trig_pin = ((gpio_num_t *)param)[0];
    gpio_num_t echo_pin = ((gpio_num_t *)param)[1];
    int sensor_space = ((int *)param)[2];

    while (1) {
        int distance = read_distance(trig_pin, echo_pin);
        bool is_occupied = (distance >= 0 && distance < DISTANCE_THRESHOLD);

        // Lock the mutex to update shared variable
        xSemaphoreTake(space_mutex, portMAX_DELAY);
        if (is_occupied) {
            available_spaces--;
        } else {
            available_spaces++;
        }
        xSemaphoreGive(space_mutex);

        // Use the semaphore to safely print the data
        if (xSemaphoreTake(display_semaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Space %d: %s", sensor_space, is_occupied ? "Occupied" : "Available");
            ESP_LOGI(TAG, "Currently available spaces: %d", available_spaces);
            xSemaphoreGive(display_semaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
#ifdef CONFIG_SPACE_OPEN
    ESP_LOGI("Parking space", "parking space is operational.");
    int max_car = CONFIG_MAX_CAR;
    ESP_LOGI("Total space", "Max car parking available: %d", max_car);
#endif

    const char *PARKING_MESSAGE = CONFIG_MY_MESSAGE;
    ESP_LOGI("CAUTION", "usage message: %s", PARKING_MESSAGE);

  
    available_spaces = CONFIG_MAX_CAR;

    display_semaphore = xSemaphoreCreateBinary();
    space_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(display_semaphore);

    
    gpio_num_t sensor_pins[5][3] = {
        {TRIG_PIN_1, ECHO_PIN_1, 1},
        {TRIG_PIN_2, ECHO_PIN_2, 2},
        {TRIG_PIN_3, ECHO_PIN_3, 3},
        {TRIG_PIN_4, ECHO_PIN_4, 4},
        {TRIG_PIN_5, ECHO_PIN_5, 5}
    };

    for (int i = 0; i < 5; i++) {
        init_ultrasonic_sensor(sensor_pins[i][0], sensor_pins[i][1]);
        xTaskCreate(sensor_task, "sensor_task", 2048, sensor_pins[i], 5, NULL);
    }
}
