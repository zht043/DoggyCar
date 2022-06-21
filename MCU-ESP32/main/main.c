#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"

#define ON 1
#define OFF 0
#define ESP_LED_PIN 2

#define BUTTON_0_PIN 27 // the white Button 

#define SERVER_CONNECT_REQUEST_BYTE 0x66
#define SERVER_CONNECT_RESPONSE_BYTE 0x77
#define SERVER_CONNECT_VALIDATION_BYTE 0x88
#define SERVER_UART_RX_TIMEOUT 500 // ms, if cmd sent over uart lags more than this amount of time or other error occurs, invoke SOS sequence to stop the robot
#define SERVER_UART_TX_FREQ 50 //Hz
#define SERVER_UART_BAUDRATE 115200
#define SERVER_UART_NUM UART_NUM_0
#define UART_RX_BUF_SIZE (1024)
#define UART_TX_BUF_SIZE (128)

#define delay(MILLIS) vTaskDelay(MILLIS / portTICK_PERIOD_MS);

//==============================================================================//
/* Data to be published back to the SoC layer (via USB<->CH340<->UART) */ 
SemaphoreHandle_t steer_angle_mutex = NULL;
float steer_angle = 0.00f; // (-90~90) [degrees]

SemaphoreHandle_t ambient_light_mutex = NULL;
float ambient_light = 0.00f; // (0~100) [% of max adc val]

SemaphoreHandle_t tofs_mutex = NULL;
float left_front_ToF = 0.00f; // (0~???) [cm]
float right_front_ToF = 0.00f; // (0~???) [cm]

SemaphoreHandle_t ultrasonic_mutex = NULL;
float reer_ultrasonic = 0.00f; // (0~???) [cm]

uint8_t buttons_bits = 0x00; // (0x00~0x7F) LSB-Button0, MSB-Button7

static void init_mutexes() {
    steer_angle_mutex = xSemaphoreCreateMutex();
    ambient_light_mutex = xSemaphoreCreateMutex();
    tofs_mutex = xSemaphoreCreateMutex();
    ultrasonic_mutex = xSemaphoreCreateMutex();
}
//==============================================================================//




static void init_server_uart();

static void serial_server_tx_task();
static void process_cmd(uint8_t* cmd_bytes, uint32_t num_bytes);
static void invoke_SOS();


// QueueHandle_t gpio_evt_queue = NULL;
// static void IRAM_ATTR gpio_isr_handler(void* arg) {
//     uint32_t gpio_num = (uint32_t) arg;
//     xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }


static void mock_test_task() {
    float thetas[7] = {-88.66, -45, -30.123, 0, 30.321, 45, 66.88};
    int i = 0;
    while(1) {
        if(gpio_get_level(BUTTON_0_PIN)) {
            gpio_set_level(ESP_LED_PIN, ON);
            i = (i >= 6) ? 0 : (i + 1); 
            xSemaphoreTake(steer_angle_mutex, portMAX_DELAY );
            steer_angle = thetas[i];
            xSemaphoreGive(steer_angle_mutex);
        } else {
            gpio_set_level(ESP_LED_PIN, OFF);
        }
        delay(50);
    }
}

static void dummy_test() {
    // while(1) {
    //     if(gpio_get_level(BUTTON_0_PIN)) {
    //         gpio_set_level(ESP_LED_PIN, ON);
    //     } else {
    //         gpio_set_level(ESP_LED_PIN, OFF);
    //     }
    //     delay(10);
    // }
    // uint32_t io_num;
    // uint8_t b0_state = 0;
    // while(1) {
    //     if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
    //         if(io_num == BUTTON_0_PIN) {
    //             b0_state = b0_state ? 0 : 1;
    //         }
    //     }

    //     gpio_set_level(ESP_LED_PIN, b0_state);
    // }
    gpio_reset_pin(26);
    gpio_set_direction(26, GPIO_MODE_OUTPUT);
    while(1) {
        gpio_set_level(26, 0);
        delay(20);
        gpio_set_level(26, 1);
        delay(20);
    }
}

static void wait_for_connect_seq() {
    gpio_set_level(ESP_LED_PIN, ON);
    uint8_t in_byte = 0;
    uint8_t connection_established = 0;
    while(!connection_established) {
        uart_read_bytes(SERVER_UART_NUM, &in_byte, 1, SERVER_UART_RX_TIMEOUT);
        if(in_byte == SERVER_CONNECT_REQUEST_BYTE) {
            uint8_t response_byte = SERVER_CONNECT_RESPONSE_BYTE;
            uart_write_bytes(SERVER_UART_NUM, &response_byte, 1);
            uart_read_bytes(SERVER_UART_NUM, &in_byte, 1, SERVER_UART_RX_TIMEOUT);
            if(in_byte == SERVER_CONNECT_VALIDATION_BYTE) {
                connection_established = 1;
            }
        }
    }
    gpio_set_level(ESP_LED_PIN, OFF);
}

static void setup() {
    init_mutexes();

    /* initialize the built-in LED on the ESP32 board */
    gpio_reset_pin(ESP_LED_PIN);
    gpio_set_direction(ESP_LED_PIN, GPIO_MODE_OUTPUT);

    /* initialize the buttons */
    // gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    gpio_reset_pin(BUTTON_0_PIN);
    gpio_set_direction(BUTTON_0_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_0_PIN, GPIO_PULLUP_ENABLE);
    // gpio_set_intr_type(BUTTON_0_PIN, GPIO_INTR_ANYEDGE);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(BUTTON_0_PIN, gpio_isr_handler, (void*)BUTTON_0_PIN);


    /* initialize this MCU Server node, which communicates with the SoC node via UART */
    init_server_uart();
}


void app_main(void)
{
    setup();

    //dummy_test();

    wait_for_connect_seq();

    //xTaskCreate(serial_server_tx_task, "serial_server_tx_task", 2*1024, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(mock_test_task, "mock_test_task", 1024, NULL, configMAX_PRIORITIES, NULL);

    uint8_t packet_frame_head;
    uint8_t* packet_data = (uint8_t*) malloc(UART_TX_BUF_SIZE + 1);
    uint8_t packet_frame_tail;
    while(1) {
        /* 
         * [#][<packet size>][cmd_bytes][\n]
         * i.e. 
         * first byte is the start symbol: #
         * second byte specify the packet size (<255)
         * last byte is the end symbol: "\n" (i.e. the newline character)
         * the bytes in the middle are the content of this packet
         */

        // read one byte from UART
        if(uart_read_bytes(SERVER_UART_NUM, &packet_frame_head, 1, SERVER_UART_RX_TIMEOUT) <= 0) {
            // Error or Timeout occurred
            invoke_SOS();
        }
        if((char)packet_frame_head == '#') {
            uint8_t packet_size = 0;
            if(uart_read_bytes(SERVER_UART_NUM, &packet_size, 1, SERVER_UART_RX_TIMEOUT) > 0) {
                //printf("%x %x\n", packet_frame_head, packet_size);
                //uart_write_bytes(SERVER_UART_NUM, &packet_size, 1);
                if(uart_read_bytes(SERVER_UART_NUM, packet_data, (uint32_t)packet_size, SERVER_UART_RX_TIMEOUT) > 0) {
                    if(uart_read_bytes(SERVER_UART_NUM, &packet_frame_tail, 1, SERVER_UART_RX_TIMEOUT) > 0) {
                        if((char)packet_frame_tail == '\n') {
                            //gpio_set_level(ESP_LED_PIN, ON);
                            process_cmd(packet_data, (uint32_t)packet_size);
                        }
                    }
                }
            }
        } 
    }
    invoke_SOS();
    free(packet_frame_head);
    free(packet_data);
}


static void init_server_uart() {
    const uart_config_t uart_config = {
        .baud_rate = SERVER_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,   // ODD Parity!
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(SERVER_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(SERVER_UART_NUM, &uart_config);
}

/* emergency stop & shutdown */
static void invoke_SOS() {
    while(1) {
        for(int i = 0; i < 6; i++) {
            gpio_set_level(ESP_LED_PIN, ON);
            delay(50);
            gpio_set_level(ESP_LED_PIN, OFF);
            delay(50);
        }
        delay(1000);
    }
    esp_restart();
}

static void process_cmd(uint8_t* cmd_bytes, uint32_t num_bytes) {
    uart_write_bytes(SERVER_UART_NUM, cmd_bytes, num_bytes);
}


static void serial_server_tx_task() {
    uint8_t* data_bytes = (uint8_t*) malloc(UART_TX_BUF_SIZE + 1);
    while(1) {
        /* set packet head */
        data_bytes[0] = (uint8_t)'#';
        data_bytes[1] = 0x00;  // 0x00 is a dummy value; will bet set below
        uint8_t num_bytes = 2; // num < 256
        //================================================================================//
    
        xSemaphoreTake(steer_angle_mutex, portMAX_DELAY);
        memcpy(data_bytes + num_bytes, (uint8_t*)(&steer_angle), 4);
        xSemaphoreGive(steer_angle_mutex);
        num_bytes += 4;

        xSemaphoreTake(ambient_light_mutex, portMAX_DELAY);
        memcpy(data_bytes + num_bytes, (uint8_t*)(&ambient_light), 4);
        xSemaphoreGive(ambient_light_mutex);
        num_bytes += 4;

        xSemaphoreTake(tofs_mutex, portMAX_DELAY);
        memcpy(data_bytes + num_bytes, (uint8_t*)(&left_front_ToF), 4);
        memcpy(data_bytes + num_bytes, (uint8_t*)(&right_front_ToF), 4);
        xSemaphoreGive(tofs_mutex);
        num_bytes += 8;

        xSemaphoreTake(ultrasonic_mutex, portMAX_DELAY);
        memcpy(data_bytes + num_bytes, (uint8_t*)(&reer_ultrasonic), 4);
        xSemaphoreGive(ultrasonic_mutex);
        num_bytes += 4;
        

        //================================================================================//
        /* set packet tail */
        data_bytes[1] = num_bytes - 2; 
        data_bytes[++num_bytes - 1] = (uint8_t)'\n';
        
        // send data packet back to SoC layer
        // printf("%d: ", num_bytes);
        // for(int i = 0; i < num_bytes; i++) printf("%hhx ", data_bytes[i]);
        // printf("\n");
        uart_write_bytes(SERVER_UART_NUM, data_bytes, num_bytes);

        // periodically sending in a certain frequency
        delay(1000 / SERVER_UART_TX_FREQ);
    }
    free(data_bytes);
}



