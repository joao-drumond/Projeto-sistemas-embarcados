#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h" 
#include "driver/gptimer.h" 
#include "driver/ledc.h" 
#include "esp_adc/adc_oneshot.h"      
#include "esp_adc/adc_cali.h"         
#include "esp_adc/adc_cali_scheme.h"  
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "protocol_examples_common.h"
#include "nvs_flash.h"

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
#define LEDC_OUTPUT_IO_3        (26) 
#define LEDC_OUTPUT_IO_4        (17)
#define LEDC_CHANNEL_01         LEDC_CHANNEL_0 
#define LEDC_CHANNEL_02         LEDC_CHANNEL_1 
#define LEDC_CHANNEL_03         LEDC_CHANNEL_2 
#define LEDC_CHANNEL_04         LEDC_CHANNEL_3 
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT 
#define LEDC_FREQUENCY          (6000) 

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11
#define EXAMPLE_ADC_BITWIDTH        ADC_BITWIDTH_10

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

// Estrutura para passar informações entre as tasks do ADC e Timer
typedef struct {
    int16_t raw;
    int16_t voltage;
} ADC_elements_t;

// Declaração das filas
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t timer_evt_queue = NULL;
static QueueHandle_t pwm_evt_queue = NULL;
static QueueHandle_t adc_evt_queue = NULL;

// Declaração dos semaphoros
static SemaphoreHandle_t pwm_semaphore = NULL; 
static SemaphoreHandle_t adc_semaphore = NULL; 

// Declaração das TAGs
static const char* TAG_INFO = "System Info"; // O asterístico é pra declarar como ponteiro? Sim
static const char* TAG_GPIO_INFO = "GPIO Info"; 
static const char* TAG_TIMER_INFO = "Timer Info";
static const char* TAG_WATCH_INFO = "Watch Info";
static const char* TAG_PWM_INFO = "PWM Info";
static const char* TAG_ADC_INFO = "ADC Info";
static const char* TAG_MQTT_INFO = "MQTT Info";

// Variável auxiliar para salvar o estado lógico do LED
int stateOfOutput = 0; 

// Declaração das funções de calibração do ADC
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

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

    ADC_elements_t adc = {
        .raw = 0,
        .voltage = 0,
    };
   
    while(1){
        if (xQueueReceive(timer_evt_queue, &ele, pdMS_TO_TICKS(2000))) {
            xSemaphoreGive(pwm_semaphore);
            xSemaphoreGive(adc_semaphore);
            i++;
            if(i == 10){
                watch.seconds += 1;
                if(xQueueReceive(adc_evt_queue, &adc, NULL)){
                    ESP_LOGI(TAG_ADC_INFO, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc.raw); // Loga o valor bruto lido
                    ESP_LOGI(TAG_ADC_INFO, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc.voltage); // Loga o valor calibrado
                }
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
        if (xSemaphoreTake(pwm_semaphore, portMAX_DELAY)) {
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

// Função que cria e configura os pwms para o mqtt
static void pwm_mqtt(void* arg){
    // Configurações do timer para o LED [PWM]
    ledc_timer_config_t ledc_timer3 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_02,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer3));

    // Configurações do canal para o LED [PWM]
    ledc_channel_config_t ledc_channel3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_03,
        .timer_sel      = LEDC_TIMER_02,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_3,
        .duty           = 128, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel3));

    // Configurações do timer para o LED [PWM]
    ledc_timer_config_t ledc_timer4 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_02,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer4));

    // Configurações do canal para o LED [PWM]
    ledc_channel_config_t ledc_channel4 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_04,
        .timer_sel      = LEDC_TIMER_02,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_4,
        .duty           = 128, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel4));
}

static void adc_task(void *arg){
    //-------------Inicialização do ADC1---------------//
    adc_oneshot_unit_handle_t adc1_handle; // Handle para a unidade ADC1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1, // Define que o ADC1 será utilizado
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Inicializa o ADC1

    //-------------Configuração do ADC1---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = EXAMPLE_ADC_BITWIDTH, // Configura a resolução do ADC para 13 bits
        .atten = EXAMPLE_ADC_ATTEN,       // Configura a atenuação para 12 dB
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config)); // Configura o canal 3 do ADC1

    //-------------Inicialização da Calibração do ADC1---------------//
    adc_cali_handle_t adc1_cali_handle = NULL; // Handle para a calibração do ADC1
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC_ATTEN, &adc1_cali_handle); // Inicializa a calibração

    ADC_elements_t adc;

    // Loop infinito que realiza leituras periódicas do ADC
    while (1) {
        if (xSemaphoreTake(adc_semaphore, portMAX_DELAY)) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc.raw)); // Lê o valor bruto do ADC
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc.raw, &adc.voltage)); // Converte o valor bruto para tensao calibrada 
            if(do_calibration1){
                xQueueSend(adc_evt_queue, &adc , NULL);
            }  
        }    
        vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de realizar a próxima leitura
    }

    // Libera a unidade ADC1 e a calibração (isso nunca será alcançado devido ao loop infinito)
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle)); // Deleta a unidade ADC1
    if (do_calibration1) {
        example_adc_calibration_deinit(adc1_cali_handle); // Deleta o handle de calibração do ADC1
    }

}

// Função para inicializar a calibração do ADC
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL; // Handle para a calibração
    esp_err_t ret = ESP_FAIL; // Variável para armazenar o status da operação
    bool calibrated = false; // Flag para indicar se a calibração foi bem-sucedida

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG_ADC_INFO, "calibration scheme version is %s", "Curve Fitting"); // Loga o esquema de calibração sendo usado
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,        // Define a unidade do ADC
            .atten = atten,         // Define a atenuação do ADC
            .bitwidth = EXAMPLE_ADC_BITWIDTH, // Define a resolução do ADC
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle); // Cria o esquema de calibração "Curve Fitting"
        if (ret == ESP_OK) {
            calibrated = true; // Marca como calibrado se a operação foi bem-sucedida
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG_ADC_INFO, "calibration scheme version is %s", "Line Fitting"); // Loga o esquema de calibração sendo usado
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,        // Define a unidade do ADC
            .atten = atten,         // Define a atenuação do ADC
            .bitwidth = EXAMPLE_ADC_BITWIDTH, // Define a resolução do ADC
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle); // Cria o esquema de calibração "Line Fitting"
        if (ret == ESP_OK) {
            calibrated = true; // Marca como calibrado se a operação foi bem-sucedida
        }
    }
#endif

    *out_handle = handle; // Atribui o handle de calibração para a variável externa
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_ADC_INFO, "Calibration Success"); // Loga sucesso na calibração
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG_ADC_INFO, "eFuse not burnt, skip software calibration"); // Loga aviso se a calibração não foi possível
    } else {
        ESP_LOGE(TAG_ADC_INFO, "Invalid arg or no memory"); // Loga erro se houve falha na calibração
    }

    return calibrated; // Retorna o status da calibração
}

// Função para desinicializar a calibração do ADC
static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG_ADC_INFO, "deregister %s calibration scheme", "Curve Fitting"); // Loga o esquema de calibração sendo desregistrado
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle)); // Deleta o esquema de calibração "Curve Fitting"

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG_ADC_INFO, "deregister %s calibration scheme", "Line Fitting"); // Loga o esquema de calibração sendo desregistrado
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle)); // Deleta o esquema de calibração "Line Fitting"
#endif
}

// MQTT
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG_MQTT_INFO, "Last error %s: 0x%x", message, error_code);
    }
}

// MQTT
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG_MQTT_INFO, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG_MQTT_INFO, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG_MQTT_INFO, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "teste", 1);
        ESP_LOGI(TAG_MQTT_INFO, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "led_vermelho", 1);
        ESP_LOGI(TAG_MQTT_INFO, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG_MQTT_INFO, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_DATA");

    // Comparando exatamente o número de caracteres da string "/topic/qos0"
    if(strncmp(event->topic, "/topic/qos0", strlen("/topic/qos0")) == 0){
        printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
            printf("DUTY CYCLE = %.*s\r\n", event->data_len, event->data);

        // Converte os dados recebidos em string para inteiro
        char dutyCycleStr[event->data_len + 1];
        strncpy(dutyCycleStr, event->data, event->data_len);
        dutyCycleStr[event->data_len] = '\0';  // Adiciona o caractere nulo ao final da string

        int dutyCycle = atoi(dutyCycleStr);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_03, dutyCycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_03));

    } else if (strncmp(event->topic, "led_vermelho", strlen("led_vermelho")) == 0){
        printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
        printf("DUTY CYCLE = %.*s\r\n", event->data_len, event->data);

        // Converte os dados recebidos em string para inteiro
        char dutyCycleStr[event->data_len + 1];
        strncpy(dutyCycleStr, event->data, event->data_len);
        dutyCycleStr[event->data_len] = '\0';  // Adiciona o caractere nulo ao final da string

        int dutyCycle = atoi(dutyCycleStr);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_04, dutyCycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_04));

    } else {
        printf("TOPIC = %.*s\r\n", event->topic_len, event->topic);
        printf("Topico inadequado para controle de duty cycle\r\n");  
    }
    
    break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT_INFO, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG_MQTT_INFO, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG_MQTT_INFO, "Other event id:%d", event->event_id);
        break;
    }
}

// MQTT
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://device_1:device_1@node02.myqtthub.com:1883",
        .credentials.client_id = "device_1",
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
                } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void){
 
    esp_log_level_set(TAG_INFO, ESP_LOG_ERROR); 
    esp_log_level_set(TAG_GPIO_INFO, ESP_LOG_ERROR);
    esp_log_level_set(TAG_TIMER_INFO, ESP_LOG_ERROR);
    esp_log_level_set(TAG_WATCH_INFO, ESP_LOG_ERROR);
    esp_log_level_set(TAG_PWM_INFO, ESP_LOG_ERROR);
    esp_log_level_set(TAG_ADC_INFO, ESP_LOG_ERROR);

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
    
    // Criando o semaphoro para o ADC
    adc_semaphore = xSemaphoreCreateBinary();
    // Criando a fila do ADC - Timer
    adc_evt_queue = xQueueCreate(1, sizeof(ADC_elements_t));
    // Criando a fila do timer
    timer_evt_queue = xQueueCreate(10, sizeof(struct_queue_element_t));
    // Criando a task do timer
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 10, NULL);

// -------------------------------------- AULA 4 - 19/07/2024 --------------------------------------- 
    
    // Criando o semaphoro para o PWM
    pwm_semaphore = xSemaphoreCreateBinary();
    // Criando a fila do PWM - GPIO
    pwm_evt_queue = xQueueCreate(10, sizeof(PWM_elements_t));
    // Criando a task do LED PWM
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 10, NULL);

    pwm_mqtt(NULL);

// -------------------------------------- AULA 5 - 02/08/2024 ---------------------------------------
    
    // Criando a task do ADC
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 10, NULL);

// --------------------------------------- AULA 6 - 30/08/2024 --------------------------------------

    // Protocolo MQTT
    ESP_LOGI(TAG_MQTT_INFO, "[APP] Startup..");
    ESP_LOGI(TAG_MQTT_INFO, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG_MQTT_INFO, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();

// --------------------------------------------------------------------------------------------------
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } 
}
