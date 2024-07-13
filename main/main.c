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
#include "esp_log.h" // Adicionado para utilizar a função de registro de log
#include "driver/gptimer.h" // Adicionando para utilizar as funções e constantes do timer

#define GPIO_OUTPUT_IO_0     2
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#define GPIO_INPUT_IO_0     21
#define GPIO_INPUT_IO_1     22
#define GPIO_INPUT_IO_2     23
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))

#define ESP_INTR_FLAG_DEFAULT 0

// Declaração das estruturas
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

// Declaração das filas
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t timer_evt_queue = NULL;

// Declaração das TAGs
static const char* TAG_INFO = "System Info"; // O asterístico é pra declarar como ponteiro? Sim
static const char* TAG_GPIO_INFO = "GPIOs Info"; // O asterístico é pra declarar como ponteiro? Sim
static const char* TAG_TIMER_INFO = "Timers Info";
static const char* TAG_WATCH_INFO = "Watch Info";

int stateOfOutput = 0; // Variável auxiliar para salvar o estado lógico do LED

static bool IRAM_ATTR timer_on_alarm (gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    
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

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

// Configuração da interrupção para os IOs
static void IRAM_ATTR gpio_isr_handler(void *arg) // Quando tiver IRAM é uma interrupção
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); // ISR = Interrupt service routine
}

// Envio de task (tarefa) para a fila de execução
static void gpio_task_auxiliar(void* arg) // É uma tarefa (task) -> Tem "cara" de função e tem loop infinito
{   
    uint32_t io_num;
    for (;;) {
        // Verifica se a interrupção foi chamada devido ao acionamento de algum botão
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            // ESP_LOGI informa na tela os dados requeridos
            ESP_LOGI(TAG_GPIO_INFO, "Botão acionado: GPIO[%"PRIu32"] - Estado: %d\n", io_num, gpio_get_level(io_num));
            
            if(io_num == GPIO_INPUT_IO_0){ // Se o botão 1 é acionado , o led acende (1)
                gpio_set_level(GPIO_OUTPUT_IO_0, 1); // Seta o nível lógico do led para "1"
                stateOfOutput = 1; // Atualiza o valor da variável auxiliar
            } else if(io_num == GPIO_INPUT_IO_1){ // Se o botão 2 é acionado , o led apaga (0)
                gpio_set_level(GPIO_OUTPUT_IO_0, 0); // Seta o nível lógico do led para "0"
                stateOfOutput = 0; // Atualiza o valor da variável auxiliar
            } else if (io_num == GPIO_INPUT_IO_2){ // Se o botão 3 é acionado , o led altera seu valor lógico
                if(stateOfOutput==1){ 
                    gpio_set_level(GPIO_OUTPUT_IO_0, 0); // Seta o nível lógico do led para "0"
                    stateOfOutput=0; // Atualiza o valor da variável auxiliar
                } else {
                    gpio_set_level(GPIO_OUTPUT_IO_0, 1); // Seta o nível lógico do led para "1"
                    stateOfOutput=1; // Atualiza o valor da variável auxiliar
                }
            } 
        }
    }
}

// Criando uma task para o timer
static void timer_task_auxiliar(void* arg){

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); 

        gptimer_event_callbacks_t cbs = {{}
        .on_alarm = timer_on_alarm
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_LOGI(TAG_TIMER_INFO, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG_TIMER_INFO, "Start timer, stop it at alarm event");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // period = 0,1s = 100ms
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    struct_watch_t watch = {
        .seconds = 0,
        .minutes = 0,
        .hours = 0,
    };

    struct_queue_element_t ele;
    int i = 0;

    while(1){
        if (xQueueReceive(timer_evt_queue, &ele, pdMS_TO_TICKS(2000))) {
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
            
            // ESP_LOGI(TAG_TIMER_INFO, " -------- %llu", ele.event_count);
            
        } else {
            ESP_LOGW(TAG_TIMER_INFO, "Missed one count event");
        }
    }

}

void app_main(void){

    esp_log_level_set(TAG_GPIO_INFO, ESP_LOG_ERROR); 
/*
    // -------------------------------------- AULA 1 - 22/03/2024 ---------------------------------------
    esp_log_level_set(TAG_INFO, ESP_LOG_INFO); // Essa função e o ESP_LOG_INFO estão dentro de uma biblioteca

    // Print chip information 
    esp_chip_info_t chip_info; // Declaração de um tipo de variável usando a estrutura esp_chip_info_t
    uint32_t flash_size; 
    esp_chip_info(&chip_info);

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
    // -------------------------------------- FIM AULA 1 - 22/03/2024 --------------------------------------- 
    
    // -------------------------------------- AULA 2 - 04/05/2024 --------------------------------------- 

    // CONFIGURANDO A SAÍDA
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // CONFIGURANDO AS ENTRADAS
    //interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // CONFIGURANDO E CRIANDO A FILA
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // 10 posicoes de 32 bits
    //start gpio task
    xTaskCreate(gpio_task_auxiliar, "gpio_task_auxiliar", 2048, NULL, 10, NULL);

    // CONFIGURANDO A INTERRUPÇÃO
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
*/
   //  -------------------------------------- AULA 3 - 12/07/2024 ---------------------------------------

    timer_evt_queue = xQueueCreate(10, sizeof(struct_queue_element_t));
    if (!timer_evt_queue) {
        ESP_LOGE(TAG_TIMER_INFO, "Creating queue failed");
        return;
    }
    ESP_LOGI(TAG_TIMER_INFO, "Create timer handle");
    xTaskCreate(timer_task_auxiliar, "timer_task_auxiliar", 2048, NULL, 10, NULL);

    // Sempre será o último trecho do script
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}