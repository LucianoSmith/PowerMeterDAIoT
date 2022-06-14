#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "math.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "smart_config.h"
#include "sntp_time.h"
#include "esp_sntp.h"
#include "mqtt_client.h"
#include "jwt_token_gcp.h"
#include "pemkey.h"
#include "gcp_min_ca.h"
#include "esp_wifi.h"

extern bool time_sinc_ok;

#define LED_R           26
#define LED_Y           27 // muy bajo cambiar resistencia
#define LED_G           14 // un poco bajo cambiar resistencia
#define RELAY           12

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RELAY) | (1ULL<<LED_R) | (1ULL<<LED_Y) | (1ULL<<LED_G))
#define ON 1
#define OFF 0

#define I2C_SLAVE_ADDR                      0x48
#define TIMEOUT_MS                          5
#define DELAY_MS                            5
#define EXAMPLE_I2C_ACK_CHECK_EN			0x1
#define voltage_coeficient                  1
#define current_factor                      0.006626

i2c_config_t i2c_conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,	    
    .master.clk_speed = 300400,
};		

esp_err_t i2c_ret = ESP_OK;
uint8_t data_h;
uint8_t data_l;

char rele_status[15] = "RELE: OFF";
char led_g_status[15] = "LED 3: OFF";
char led_y_status[15] = "LED 2: OFF";
char led_r_status[15] = "LED 1: OFF";



//**** MQTT

#define BROKER_URI				"mqtts://mqtt.2030.ltsapis.goog"

#define IOTCORE_USERNAME		"unused"
#define IOTCORE_PROJECTID		"daiot-pm"
#define IOTCORE_DEVICEID		"pm_ven5290" 
#define IOTCORE_REGION			"us-central1"
#define IOTCORE_REGISTRY		"pm_reg"
#define IOTCORE_CLIENTID		"projects/"IOTCORE_PROJECTID"/locations/"IOTCORE_REGION"/registries/"IOTCORE_REGISTRY"/devices/"IOTCORE_DEVICEID

#define IOTCORE_TOKEN_EXPIRATION_TIME_MINUTES		60 * 24

#define TEMP_PUBLISH_INTERVAL_SECONDS       1

#define ERROR_CODE_RESET	1
#define ERROR_CODE_JWT		2
#define ERROR_CODE_SNTP		4
#define ERROR_CODE_MQTT		8
#define ERROR_CODE_TIMEOUT  16
#define ERROR_CODE_WIFI  	32
#define ERROR_CODE_IP		64

extern int last_error_count;
extern int last_error_code;
extern unsigned int tph_on_time;
extern unsigned int last_on_time_seconds;
extern int last_sntp_response_time_seconds;

static const char *TAG = "MQTT MODULE: ";

esp_mqtt_client_handle_t cliente = NULL;

int RTC_DATA_ATTR last_error_count = 0;
int RTC_DATA_ATTR last_error_code = 0;
static unsigned char RTC_DATA_ATTR forzar_espera_sntp = 0;
unsigned int RTC_DATA_ATTR last_on_time_seconds = 0;
int sntp_response_time_seconds = 0;
unsigned int tph_on_time = 0;
time_t wake_up_timestamp = 0;


char MQTT_suffix[200];
char MQTT_topic[250];
char MQTT_payload[200];
size_t MQTT_payload_length = 200;
char MQTT_valor[100];
size_t MQTT_values_count;

esp_mqtt_client_handle_t mqtt_client_handle = NULL;
esp_mqtt_client_config_t mqtt_client_config = { };
char* GCP_JWT = NULL;
bool mqtt_client_connected = false;
bool mqtt_disconnected_event_flag = false;


static bool mqtt_client_configure(void);

int T = 0, P = 0, H = 0; // las declaro global para probar rapidamente
uint8_t id_sensor_recibido;

//**** MQTT


void turn_led(int8_t led, int8_t status) {
 
 
    switch (led)
    {
    case LED_R:
        strcpy(led_r_status, "LED 1: OFF"); 
        if (status==ON) {strcpy(led_r_status, "LED 1: ON");}
        break;
    case LED_Y:
        strcpy(led_y_status, "LED 2: OFF"); 
        if (status==ON) {strcpy(led_y_status, "LED 2: ON");}
        break;
    case LED_G:
        strcpy(led_g_status, "LED 3: OFF"); 
        if (status==ON) {strcpy(led_g_status, "LED 3: ON");}
        break;
    case RELAY:
        strcpy(rele_status, "RELE: OFF"); 
        if (status==ON) {strcpy(rele_status, "RELE: ON");}
        break;    
    default:
        break;
    }
    gpio_set_level(led, status);
}

void initialize_sensor (void) {
    
    //----- CREATE THE I2C PORT -----
	i2c_param_config(I2C_NUM_0, &i2c_conf);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    printf("WRITING CONFIG ... ");

    uint16_t config = 0x0003; // disable comparator
    config |= 0x00E0; // rate 860 SPS (default)
    config |= 0x0100; // single-shot mode (default)
    config |= 0x0000; // Gain
    config |= 0x4000; // MUX single-ended AIN0 
    config |= 0x0000; // write: set to start a single-conversion

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1 | 0x00), true);    // Frame 1: Slave Address Byte (Write)
    i2c_master_write_byte(cmd, (0x01), true);                          // Frame 2: Address Pointer Register (Config)      
    i2c_master_write_byte(cmd, (uint8_t)config>>8, true);              // Frame 3: Data Byte 1
    i2c_master_write_byte(cmd, (uint8_t)config & 0xFF, true);          // Frame 4: Data Byte 2
    i2c_master_stop(cmd);
    
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, (TIMEOUT_MS / portTICK_RATE_MS));	
    
    if (i2c_ret == ESP_OK)
    {
        printf("OK\n");
    } else {
        printf("ERROR\n");    
    }
    i2c_cmd_link_delete(cmd);

}


uint16_t readCurrent(void) {
    uint16_t res = 0;    
    
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, (I2C_SLAVE_ADDR << 1 | 0x00), true);    // Frame 1: Slave Address Byte (Write)
    i2c_master_write_byte(cmd2, (0x00), true);                          // Frame 2: Address Pointer Register (Conv)     
    i2c_master_stop(cmd2);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd2, (TIMEOUT_MS / portTICK_RATE_MS));	
    if (i2c_ret != ESP_OK)
    {
        printf("READ COMMAND (1) ... ");
        printf("ERROR\n");    
    }

    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, (I2C_SLAVE_ADDR << 1 | 0x01 ), true);   // Frame 3: Slave Address Byte (Read)
    i2c_master_read_byte(cmd3, &data_h, true);                           // Frame 4: Data Byte 1 
    i2c_master_read_byte(cmd3, &data_l, true);                          // Frame 5: Data Byte 2
    i2c_master_stop(cmd3);
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd3, (TIMEOUT_MS / portTICK_RATE_MS));	
    
    if (i2c_ret == ESP_OK)
    {    
        res = (data_h << 8) | (data_l);
        //printf("%d\n", data_h);
        //printf("%d\n", data_l);
        //printf("%d\n", res);
    } else {
        printf("READ COMMAND (2) ... ");
        printf("ERROR\n");    
    }

    i2c_cmd_link_delete(cmd2);
    i2c_cmd_link_delete(cmd3);
    return res;

}

int64_t getTime() {

    struct timeval ts;
    gettimeofday(&ts, NULL);
    int64_t t = (int64_t)ts.tv_sec;
    return t;
}

float rmsCurrent(void) {

    int count = 0;
    uint16_t current_max = 0;
    uint16_t current_min = 10000;
    
    
    int64_t time_max = getTime() + 10;
    while (getTime() <= time_max) {
        uint16_t current_reading  = readCurrent();
    
        if (current_reading>current_max) {
            current_max = current_reading;
        }
        if (current_reading<current_min) {
            current_min = current_reading;
        }
        count++;
    }

    float current_rms = (current_max-current_min)/(2*sqrt(2));
    return current_rms * current_factor;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    cliente = event->client;
    switch (event->event_id) {
            case MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

                // Suscribirse a tema 'config' de Google Cloud IoT
                esp_mqtt_client_subscribe(event->client, "/devices/"IOTCORE_DEVICEID"/config", 0);

                // Suscribirse a tema 'commands' de Google Cloud IoT
                esp_mqtt_client_subscribe(event->client, "/devices/"IOTCORE_DEVICEID"/commands/#", 0);
                mqtt_client_connected = true;
                turn_led(LED_G, ON);
                break;

            case MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
                //ESP_LOGI(TAG, "Resetting MQTT Config");
                //mqtt_client_reset_config();
                mqtt_client_connected = false;
                turn_led(LED_G, OFF);
                mqtt_disconnected_event_flag = true;
                break;

            case MQTT_EVENT_SUBSCRIBED:
                ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);		
                break;

            case MQTT_EVENT_UNSUBSCRIBED:
                ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
                break;

            case MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
                last_error_count = 0;
                last_error_code = 0;
                last_on_time_seconds = 0;
                forzar_espera_sntp = 0;
                sntp_response_time_seconds = 0;
                break;

            case MQTT_EVENT_DATA:
                ESP_LOGI(TAG, "MQTT_EVENT_DATA: ");
                printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
                printf("DATA=%.*s\r\n", event->data_len, event->data);
                if (strcmp(event->data, "RL1")==0) {
                    printf("RELAY ON\n");
                    turn_led(RELAY, ON);
                };
                if (strcmp(event->data, "RL0")==0) {
                    printf("RELAY OFF\n");
                    turn_led(RELAY, OFF);
                };
                //printf(strcmp(event->data, "LR1"));
                if (strcmp(event->data, "LR1")==0) {
                    printf("LED R ON\n");
                    turn_led(LED_R, ON);
                };
                if (strcmp(event->data, "LR0")==0) {
                    printf("LED R OFF\n");
                    turn_led(LED_R, OFF);
                };
                break;

            case MQTT_EVENT_ERROR:
                ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
                // Si se presento error en la conexion MQTT, tal vez esta mal el horario del token.
                // Forzar espera de sincronizacion en el proximo reinicio.
                forzar_espera_sntp = 1;
                last_error_count ++;
                last_error_code |= ERROR_CODE_MQTT;
                break;

            default:
                ESP_LOGI(TAG, "Other event id:%d", event->event_id);
                break;
        }
        return ESP_OK;
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}



void mqtt_app_main_task(void * parm)
{
    ESP_LOGI(TAG, "Ingresa a mqtt_app_main_task()");

    mqtt_client_configure();

    esp_mqtt_client_handle_t mqtt_client_handle = esp_mqtt_client_init(&mqtt_client_config);
    esp_mqtt_client_register_event(mqtt_client_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client_handle);

    ESP_LOGI(TAG, "Arrancando MQTT client... ");
    esp_mqtt_client_start(mqtt_client_handle);


	while (1) {

        

		while(!mqtt_disconnected_event_flag) vTaskDelay(1000 / portTICK_PERIOD_MS);

		ESP_LOGI(TAG, "Actualizando configuracion Cliente MQTT... ");
		if (mqtt_client_configure()) {
			ESP_LOGI(TAG, "Seteando configuracion Cliente MQTT... ");
			esp_mqtt_set_config(mqtt_client_handle, &mqtt_client_config);
			ESP_LOGI(TAG, "Reseteando flag evento disconnected. ");		
			mqtt_disconnected_event_flag = false;			
		}

	}

	vTaskDelete(NULL);

}


static bool mqtt_client_configure(void) {

	ESP_LOGI(TAG, "Generando JWT Token... ");
    if (GCP_JWT != NULL) free(GCP_JWT);
	
	GCP_JWT = createGCPJWT(IOTCORE_PROJECTID, GCP_PEM_KEY, strlen(GCP_PEM_KEY), IOTCORE_TOKEN_EXPIRATION_TIME_MINUTES);

    if (GCP_JWT == 0) {
        last_error_count ++;
        last_error_code |= ERROR_CODE_JWT;
		ESP_LOGI(TAG, "Error al generar JWT Token... ");
		return false;
    }

    mqtt_client_config.uri = BROKER_URI;
    mqtt_client_config.username = IOTCORE_USERNAME;
    mqtt_client_config.password = GCP_JWT; //IOORE_TOKEN;
    mqtt_client_config.cert_pem = GCP_MIN_CA;
	//mqtt_client_config.disable_auto_reconnect = true;

    //printf("pass JWT: %s-FIN", GCP_JWT);
    mqtt_client_config.client_id = IOTCORE_CLIENTID;

	ESP_LOGI(TAG, "JWT Token generado... ");
	return true;
}


void publish_current(float current) {
    printf("PUBLISH CURRENT %f \n", current);

	char bufferJson[300];
	char bufferTopic[350];
	int msg_id;
    char buffer_curr_txt[15];
	char buffer_volt_txt[15];
	char buffer_rssi_txt[15];
	
	float volt = 220;
    int8_t rssi = 0;

	wifi_ap_record_t ap_info;

    uint32_t random_number;

    if (mqtt_client_connected == true) {
        // INICIO CICLO DE LECTURAS y publicaciones.
        vTaskDelay(TEMP_PUBLISH_INTERVAL_SECONDS * 1000 / portTICK_PERIOD_MS);
            
        // Consulto al modulo el nivel de señal que esta recibiendo.
        esp_wifi_sta_get_ap_info(&ap_info);
        rssi = ap_info.rssi;
        //ESP_LOGI(TAG, "RSSI: %d ", ap_info.rssi);

        // Convertir a texto los valores.
        snprintf(buffer_curr_txt, sizeof(buffer_curr_txt), "%.2f", current);
        snprintf(buffer_volt_txt, sizeof(buffer_volt_txt), "%.2f", volt);
        snprintf(buffer_rssi_txt, sizeof(buffer_rssi_txt), "%d", rssi);

        //ESP_LOGI("Muestra: ", "CURR %s - RSSI %s", buffer_curr_txt, buffer_rssi_txt);
            
        bufferJson[0] = 0;

        strcat(bufferJson, "{ \"dev_id\": \"");
        strcat(bufferJson, IOTCORE_DEVICEID);
        strcat(bufferJson, "\", \"voltage\": ");
        strcat(bufferJson, buffer_volt_txt);
        strcat(bufferJson, ", \"current\": ");
        strcat(bufferJson, buffer_curr_txt);
        strcat(bufferJson, ", \"rssi\": ");
        strcat(bufferJson, buffer_rssi_txt);


        strcat(bufferJson, ", \"led1\": \"");
        strcat(bufferJson, led_r_status);

        strcat(bufferJson, "\", \"led2\": \"");
        strcat(bufferJson, led_y_status);

        strcat(bufferJson, "\", \"led3\": \"");
        strcat(bufferJson, led_g_status);

        strcat(bufferJson, "\", \"rele\": \"");
        strcat(bufferJson, rele_status);
        strcat(bufferJson, "\"");

        strcat(bufferJson, " }");
        
        ESP_LOGI("JSON enviado:", " %s ", bufferJson);
        
        bufferTopic[0] = 0;
        strcat(bufferTopic, "/devices/");
        strcat(bufferTopic, IOTCORE_DEVICEID);
        strcat(bufferTopic, "/events/monitor");
        // powermeter/d1/current/r
        
        msg_id = esp_mqtt_client_publish(cliente, bufferTopic, bufferJson, 0, 1, 0);

        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        // FIN CICLO LECTURA Y ENVIO
    } else {
        printf("no conectado \n");
    }
    
}

//************************************************************************************************************************
//************************************************************************************************************************
//************************************************************************************************************************



void app_main(void)
{
    // GPIO OUTPUTS CONFIG
    //zero-initialize the config structure.
    gpio_config_t out_conf = {};
    //disable interrupt
    out_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    out_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    out_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    out_conf.pull_down_en = 0;
    //disable pull-up mode
    out_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&out_conf);

    turn_led(LED_R, ON);
    turn_led(LED_Y, ON);
    turn_led(LED_G, ON);
    vTaskDelay(1000/portTICK_RATE_MS);
    turn_led(LED_R, OFF);
    turn_led(LED_Y, OFF);
    turn_led(LED_G, OFF);


    printf("INI SENSOR \n");
    initialize_sensor();
    vTaskDelay(1000/portTICK_RATE_MS);
    printf("INI ESP \n");

    ESP_ERROR_CHECK( nvs_flash_init() );
    printf("INI WIFI \n");
    vTaskDelay(1000/portTICK_RATE_MS);
    
    initialize_wifi();
    vTaskDelay(1000/portTICK_RATE_MS);

    printf("INI SNTP \n");
    initialize_sntp();
    vTaskDelay(1000/portTICK_RATE_MS);

    while (!time_sinc_ok) vTaskDelay(100 * 1);
    
    xTaskCreate(mqtt_app_main_task, "mqtt_app_task", 4096 * 8, NULL, 3, NULL);

    
    float lectura = 0;
    while (true)
    {
        lectura = rmsCurrent();
        publish_current(lectura);
        
        if (lectura >  15.0 )  {
            turn_led(LED_Y, ON);
        } else {
            turn_led(LED_Y, OFF);
        }
        
        
    }
    
}
