#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h" 
#include "driver/gptimer.h" 
#include "driver/ledc.h" 

#define GPIO_OUTPUT_IO_0     2
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0     21
#define GPIO_INPUT_IO_1     22
#define GPIO_INPUT_IO_2     23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))

#define ESP_INTR_FLAG_DEFAULT 0

#define LEDC_TIMER_01           LEDC_TIMER_0
#define LEDC_TIMER_02           LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_1        (16) 
#define LEDC_OUTPUT_IO_2        (33) 
#define LEDC_CHANNEL_01         LEDC_CHANNEL_0 
#define LEDC_CHANNEL_02         LEDC_CHANNEL_1 
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY          (6000) 

// Declaração da estrutura para o timer do relogio
typedef struct {
    uint64_t event_count;
    uint64_t alarm_count;
} struct_queue_element_t;

// Estrutura do relógio
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
} struct_watch_t;

// Estrutura para passar informações entre as tasks do PWM e IOs
typedef struct {
    bool automatic_mode;
    int16_t duty_cycle;
} PWM_elements_t;

// Declaração das filas
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t timer_evt_queue = NULL;
static QueueHandle_t pwm_evt_queue = NULL;

// Declaração do semaphoro
static SemaphoreHandle_t semaphore_pwm = NULL; 

// Declaração das TAGs
static const char* TAG_INFO = "System Info"; // O asterístico é pra declarar como ponteiro? Sim
static const char* TAG_GPIO_INFO = "GPIOs Info"; 
static const char* TAG_TIMER_INFO = "Timers Info";
static const char* TAG_WATCH_INFO = "Watch Info";
static const char* TAG_PWM_INFO = "PWM Info";

// Variável auxiliar para salvar o estado lógico do LED
int stateOfOutput = 0; 

// Interrupção para os IOs
static void IRAM_ATTR gpio_isr_handler(void *arg){ 
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); 
}

// Interrupção para o timer
static bool IRAM_ATTR timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    BaseType_t high_task_awoken = pdFALSE;
    // Retrieve count value and send to queue
    struct_queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(timer_evt_queue, &ele, &high_task_awoken);
    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 100000, // alarm in next 0,1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    return (high_task_awoken == pdTRUE);
}

// Task para os IOs - Será executada quando for gerada uma interrupção para os IOs
static void gpio_task(void* arg){ // É uma tarefa (task) -> Tem "cara" de função e tem loop infinito
    uint32_t io_num;
    PWM_elements_t pwm;
    for (;;) {
        // Verifica se a interrupção foi chamada devido ao acionamento de algum botão
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            ESP_LOGI(TAG_GPIO_INFO, "Botão acionado: GPIO[%"PRIu32"] - Estado: %d\n", io_num, gpio_get_level(io_num)); 

            if (io_num == GPIO_INPUT_IO_0) { // Se o botão 1 é acionado , o led acende (1)
                gpio_set_level(GPIO_OUTPUT_IO_0, 1); // Seta o nível lógico do led para "1"
                stateOfOutput = 1; // Atualiza o valor da variável auxiliar

                pwm.automatic_mode = true;
                xQueueSend(pwm_evt_queue, &pwm, NULL);

            } else if (io_num == GPIO_INPUT_IO_1) { // Se o botão 2 é acionado , o led apaga (0)
                gpio_set_level(GPIO_OUTPUT_IO_0, 0); // Seta o nível lógico do led para "0"
                stateOfOutput = 0; // Atualiza o valor da variável auxiliar

                pwm.automatic_mode = false;
                xQueueSend(pwm_evt_queue, &pwm, NULL);

            } else if (io_num == GPIO_INPUT_IO_2) { // Se o botão 3 é acionado , o led altera seu valor lógico
                if(stateOfOutput==1){ 
                    gpio_set_level(GPIO_OUTPUT_IO_0, 0); // Seta o nível lógico do led para "0"
                    stateOfOutput=0; // Atualiza o valor da variável auxiliar
                } else {
                    gpio_set_level(GPIO_OUTPUT_IO_0, 1); // Seta o nível lógico do led para "1"
                    stateOfOutput=1; // Atualiza o valor da variável auxiliar
                }

                pwm.automatic_mode = false;
                pwm.duty_cycle = 100 + pwm.duty_cycle;
                if (pwm.duty_cycle >= 8192){
                    pwm.duty_cycle = 2;
                }
                ESP_LOGI(TAG_PWM_INFO, " Duty cycle: %u", pwm.duty_cycle);
                xQueueSend(pwm_evt_queue, &pwm, NULL);
            } 
        }
    }
}

// Task para o timer - Será executada quando for gerada uma interrupção para o timer
static void timer_task(void* arg){

    // Criando e configurando o timer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); 
    
    // Criando e configurando o callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // Habilitando o timer
    ESP_LOGI(TAG_TIMER_INFO, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG_TIMER_INFO, "Start timer, stop it at alarm event");

    // Configurando o alarme para o timer (Define quando a interrupção será gerada)
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // period = 0,1s = 100ms
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    // Iniciando o timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Instanciando um objeto do tipo "watch" para representar o relógio
    struct_watch_t watch = {
        .seconds = 0,
        .minutes = 0,
        .hours = 0,
    };

    // Criando uma fila para as ações do timer
    struct_queue_element_t ele;
    int i = 0;

    while(1){
        if (xQueueReceive(timer_evt_queue, &ele, pdMS_TO_TICKS(2000))) {
            xSemaphoreGive(semaphore_pwm);
            i++;
            if(i == 10){
                watch.seconds += 1;
                if(watch.seconds == 60){
                    watch.seconds = 0;
                    watch.minutes += 1;

                    if(watch.minutes == 60){
                        watch.minutes = 0;
                        watch.hours += 1;

                            if(watch.hours == 24){
                                watch.hours = 0 ;
                            }
                    }
                }
                i=0;
                ESP_LOGI(TAG_WATCH_INFO, "Watch: %u h %u m %u s", watch.hours, watch.minutes, watch.seconds);
            }
        } else {
            ESP_LOGW(TAG_TIMER_INFO, "Missed one count event");
        }
    }
}

// Função que cria e configura o PWM
static void pwm_task(void* arg){
    // Configurações do timer para o LED [PWM]
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_01,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    // Configurações do canal para o LED [PWM]
    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_01,
        .timer_sel      = LEDC_TIMER_01,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_1,
        .duty           = 128, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    // Configurações do timer para o OSCILOSCOPIO [PWM]
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_02,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    // Configurações do canal para o OSCILOSCOPIO [PWM]
    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_02,
        .timer_sel      = LEDC_TIMER_02,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_2,
        .duty           = 128, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    PWM_elements_t pwm_control = {
        .automatic_mode = true,
        .duty_cycle = 2, 
    };

    while(1){
        if (xSemaphoreTake(semaphore_pwm, portMAX_DELAY)) {
            if (pwm_control.automatic_mode) {
                pwm_control.duty_cycle = 100 + pwm_control.duty_cycle;
                if( pwm_control.duty_cycle == 8192){
                    pwm_control.duty_cycle = 2;
                }
            }
            xQueueReceive(pwm_evt_queue, &pwm_control, pdMS_TO_TICKS(20));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_01, pwm_control.duty_cycle));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_01));
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_02, pwm_control.duty_cycle));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_02));
        }
    }
}

void app_main(void){
 
    esp_log_level_set(TAG_INFO, ESP_LOG_ERROR); 
    esp_log_level_set(TAG_GPIO_INFO, ESP_LOG_ERROR);
    esp_log_level_set(TAG_TIMER_INFO, ESP_LOG_ERROR);

// -------------------------------------- AULA 1 - 22/03/2024 ---------------------------------------

    esp_chip_info_t chip_info; 
    uint32_t flash_size; 
    esp_chip_info(&chip_info);
    // Exibindo informações do chip
    ESP_LOGI(TAG_INFO, "This is %s chip with %d CPU core(s), %s%s%s%s, ",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG_INFO, "silicon revision v%d.%d, ", major_rev, minor_rev);

// -------------------------------------- AULA 2 - 04/05/2024 --------------------------------------- 

    // Configurando as saídas (outputs)
    gpio_config_t io_conf = {}; 
    io_conf.intr_type = GPIO_INTR_DISABLE;  
    io_conf.mode = GPIO_MODE_OUTPUT; 
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; 
    io_conf.pull_down_en = 0; 
    io_conf.pull_up_en = 0; 
    gpio_config(&io_conf); 

    // Configurando as entradas (inputs)
    io_conf.intr_type = GPIO_INTR_NEGEDGE; 
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; 
    io_conf.mode = GPIO_MODE_INPUT; 
    io_conf.pull_up_en = 1; 
    gpio_config(&io_conf);

    // Criando a fila dos IOs
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // 10 posicoes de 32 bits 

    // Criando a task dos IOs
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // Configurando a interrupção
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //install gpio isr service
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0); 
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1); 
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2); 

// -------------------------------------- AULA 3 - 12/07/2024 ---------------------------------------

    // Criando a fila do timer
    timer_evt_queue = xQueueCreate(10, sizeof(struct_queue_element_t));
    // Criando a task do timer
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 10, NULL);

// -------------------------------------- AULA 4 - 19/07/2024 --------------------------------------- 
    
    // Criando o semaphoro
    semaphore_pwm = xSemaphoreCreateBinary();
    xSemaphoreTake(semaphore_pwm, portMAX_DELAY);
    // Criando a fila do PWM - GPIO
    pwm_evt_queue = xQueueCreate(10, sizeof(PWM_elements_t));
    // Criando a task do LED PWM
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 10, NULL);

// ---------------------------------------------------------------------------------------------------
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } 
}