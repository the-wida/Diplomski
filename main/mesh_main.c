#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include <freertos/event_groups.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "mqtt_client.h"
#include "esp_timer.h"
#include "DHT.h"
#include "general_init.h"
#include "handlers.h"
#include "i2cdev.h"
#include "bmp280.h"
#include "driver/gpio.h"

#define RX_SIZE         14
#define TX_SIZE         14
#define SEND_DELAY_MS	15000
#define SENSOR_DELAY_MS	7500
#define GREEN 25
#define YELLOW 26
#define SDA 4
#define SCL 16

char buffer[20];
char router_ssid[20];
char router_password[20];
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
int mesh_layer = -1;
int PROVISIONING_BIT = BIT0;
int mqtt_id;
int humidity_i = 0;
int temperature_i = 0;
int recv_hum;
int recv_temp;
esp_netif_t *provisioning_netif;
esp_netif_t *mesh_netif_station = NULL;
float returned_humidity = 0.;
float returned_temperature = 0.;
bool provisioning_recieved = false;
mesh_addr_t mesh_parent_addr;
EventGroupHandle_t is_provisioning_finished;
esp_mqtt_client_handle_t mqtt_client;
char hum_topic[39];
char temp_topic[42];
char from_mac_string[18];
int recieve_delay = SEND_DELAY_MS;
bool bme280p = false;
uint32_t b_pressure_i, recv_b_pressure_i;
int b_temperature_i, recv_b_temperature_i;
uint16_t b_humidity_i, recv_b_humidity_i;
char b_hum_topic[41];
char b_temp_topic[44];
char b_press_topic[41];

void sensor_read_dht(void *arg)
{
	setDHTgpio(33);
	while (true)
	{
		readDHT();
		returned_humidity = getHumidity();
		returned_temperature = getTemperature();
		printf("Hum: %.1f Tmp: %.1f\n", returned_humidity, returned_temperature);
		humidity_i = (int)(returned_humidity*10);
		temperature_i = (int)(returned_temperature*10);
		vTaskDelay(2000 / portTICK_RATE_MS);
		vTaskDelay((SENSOR_DELAY_MS-2000) / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}


void sensor_read_bmp(void *pvParameters)
{
    bmp280_params_t bmp_params;
    bmp280_init_default_params(&bmp_params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));
    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA, SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev, &bmp_params));
	if(dev.id == BME280_CHIP_ID)
	{
		bme280p = true;
	}
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
    float b_pressure, b_temperature, b_humidity;
    while (1)
    {
        vTaskDelay((SENSOR_DELAY_MS) / portTICK_RATE_MS);
        if (bmp280_read_float(&dev, &b_temperature, &b_pressure, &b_humidity) != ESP_OK)
        {
            printf("BMP/BME reading failed\n");
            continue;
        }
        printf("Pressure: %.2f Pa, Temperature: %.2f C", b_pressure, b_temperature);
        if (bme280p)
		{
            printf(", Humidity: %.2f\n", b_humidity);
		}
        else
		{
            printf("\n");
		}
		b_pressure_i = (uint32_t) (b_pressure *100);
		b_temperature_i = (int) (b_temperature *100);
		b_humidity_i = (uint16_t) (b_humidity *100);
    }
	vTaskDelete(NULL);
}

int get_base_recieve_delay()
{
	return SEND_DELAY_MS;
}
 
void mesh_send(void *arg)
{
    int send_count = 0;			
    mesh_data_t data;			
    data.data = tx_buf;			
    data.size = sizeof(tx_buf);	
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    while (true) 
	{
        send_count++;
		tx_buf[0] = send_count;
		tx_buf[1] = temperature_i & 0xFF;
		tx_buf[2] = (temperature_i >> 8) & 0xFF;
		tx_buf[3] = humidity_i & 0xFF;
		tx_buf[4] = (humidity_i >> 8) & 0xFF;
		tx_buf[5] = bme280p;
		tx_buf[6] = (b_pressure_i & 0xFF);
		tx_buf[7] = (b_pressure_i >> 8) & 0xFF;
		tx_buf[8] = (b_pressure_i >> 16) & 0xFF;
		tx_buf[9] = (b_pressure_i >> 24);
		tx_buf[10] = b_temperature_i & 0xFF;
		tx_buf[11] = (b_temperature_i >> 8) & 0xFF;
		tx_buf[12] = 0;
		tx_buf[13] = 0;
		if (bme280p)
		{
			tx_buf[12] = b_humidity_i & 0xFF;
			tx_buf[13] = (b_humidity_i >> 8) & 0xFF;
		}
		ESP_ERROR_CHECK(esp_mesh_send(NULL, &data, MESH_DATA_TODS , NULL, 0));
		printf("Sent data to root\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
		vTaskDelay((SEND_DELAY_MS-1000) / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void mesh_recieve(void *arg)
{
    int recv_count;
    esp_err_t mesh_recv_error;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;
    while (true) 
	{
        mesh_recv_error = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (mesh_recv_error != ESP_OK || !data.size) 
		{
            printf("MESH: Recieve error:0x%x, size:%d\n", mesh_recv_error, data.size);
            continue;
        }
		recv_count = data.data[0];
		recv_temp = ((data.data[2]<<8)|(data.data[1]&0xff));
		recv_hum = ((data.data[4]<<8)|(data.data[3]&0xff));
		recv_b_pressure_i = (data.data[9] << 24) | (data.data[8] << 16) | ( data.data[7] << 8 ) | (data.data[6]);
		recv_b_temperature_i = ((data.data[11]<<8)|(data.data[10]&0xff));
		if (data.data[5])
		{
			recv_b_humidity_i = ((data.data[13]<<8)|(data.data[12]&0xff));
		}
		sprintf(from_mac_string, "%02X:%02X:%02X:%02X:%02X:%02X", from.addr[0], from.addr[1], from.addr[2], from.addr[3], from.addr[4], from.addr[5]);
		sprintf(hum_topic, "%s%s%s","/topic/", from_mac_string, "/1891_humidity");
		sprintf(temp_topic, "%s%s%s","/topic/", from_mac_string, "/1891_temperature");
		sprintf(b_hum_topic, "%s%s%s","/topic/", from_mac_string, "/1891_humidity_b");
		sprintf(b_temp_topic, "%s%s%s","/topic/", from_mac_string, "/1891_temperature_b");
		sprintf(b_press_topic, "%s%s%s","/topic/", from_mac_string, "/1891_pressure_b");
        printf("MESH: Recieved from %s, current layer: %d, current parent:"MACSTR", Message number: %d\n", from_mac_string, mesh_layer, MAC2STR(mesh_parent_addr.addr), recv_count);
		printf("Humidity: %d, Temperature: %d\n",recv_hum, recv_temp);
		if (esp_mesh_is_root())
		{
			printf("Sending child data\n"); 
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, temp_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, temp_topic, itoa(recv_temp,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, hum_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, hum_topic,itoa(recv_hum,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_temp_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, b_temp_topic,itoa(recv_b_temperature_i,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_press_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, b_press_topic,itoa(recv_b_pressure_i,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			if (data.data[5])
			{
				mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_hum_topic, 1);
				printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
				mqtt_id = esp_mqtt_client_publish(mqtt_client, b_hum_topic,itoa(recv_b_humidity_i,buffer,10), 0, 1, 0);
				printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			}
		}
		vTaskDelay((recieve_delay-5) / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void send_own_data()
{
	uint8_t own_mac[6];
	esp_wifi_get_mac(WIFI_IF_STA, own_mac);
	static char own_mac_string[18];
	sprintf(own_mac_string, "%02X:%02X:%02X:%02X:%02X:%02X", own_mac[0], own_mac[1], own_mac[2], own_mac[3], own_mac[4], own_mac[5]);
	while (true) 
	{
		sprintf(hum_topic, "%s%s%s","/topic/", own_mac_string, "/1891_humidity");
		sprintf(temp_topic, "%s%s%s","/topic/", own_mac_string, "/1891_temperature");
		sprintf(b_hum_topic, "%s%s%s","/topic/", own_mac_string, "/1891_humidity_b");
		sprintf(b_temp_topic, "%s%s%s","/topic/", own_mac_string, "/1891_temperature_b");
		sprintf(b_press_topic, "%s%s%s","/topic/", own_mac_string, "/1891_pressure_b");
		printf("Sending own data\n");
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, temp_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, temp_topic, itoa(temperature_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, hum_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, hum_topic,itoa(humidity_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_temp_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, b_temp_topic,itoa(b_temperature_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_press_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, b_press_topic,itoa(b_pressure_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		if (bme280p)
		{
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, b_hum_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, b_hum_topic,itoa(b_humidity_i,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
		vTaskDelay((SEND_DELAY_MS-1000) / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	gpio_set_direction(GREEN, GPIO_MODE_OUTPUT);
	gpio_set_direction(YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_level(GREEN, 1);
    init_memory();
	mqtt_client = init_mqtt("mqtt://test.mosquitto.org");	
	aditional_init();
	register_handlers(mqtt_client);
	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
	provisioning_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
	start_provisioning();
	start_timer();
	xEventGroupWaitBits(is_provisioning_finished, PROVISIONING_BIT, false, true, portMAX_DELAY);
	gpio_set_level(GREEN, 0);
	restart_wifi(wifi_config);
	init_mesh();
	ESP_ERROR_CHECK(i2cdev_init());
	xTaskCreate(&sensor_read_dht, "Sens_d", 2048, NULL, 5, NULL);
	xTaskCreate(&sensor_read_bmp, "Sens_b", 2048, NULL, 5, NULL);
}
