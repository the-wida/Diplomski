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

#define RX_SIZE         5
#define TX_SIZE         5
#define SEND_DELAY_MS	15000
#define SENSOR_DELAY_MS	7500

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

void sensor_read(void *arg)
{
	setDHTgpio(19);
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
		sprintf(from_mac_string, "%02X:%02X:%02X:%02X:%02X:%02X", from.addr[0], from.addr[1], from.addr[2], from.addr[3], from.addr[4], from.addr[5]);
		sprintf(hum_topic, "%s%s%s","/topic/", from_mac_string, "/1891_humidity");
		sprintf(temp_topic, "%s%s%s","/topic/", from_mac_string, "/1891_temperature");
        printf("MESH: Recieved from %s, current layer: %d, current parent:"MACSTR", Message number: %d\n", from_mac_string, mesh_layer, MAC2STR(mesh_parent_addr.addr), recv_count);
		printf("Humidity: %d, Temperature: %d\n",recv_hum, recv_temp);
		if (esp_mesh_is_root())
		{
			printf("Sending root data\n"); 
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, temp_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, temp_topic, itoa(recv_temp,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, hum_topic, 1);
			printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, hum_topic,itoa(recv_hum,buffer,10), 0, 1, 0);
			printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		}
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
		printf("Sending own data\n");
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, temp_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, temp_topic, itoa(temperature_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_subscribe(mqtt_client, hum_topic, 1);
		printf("MQTT: Subscription successful, message id = %d\n", mqtt_id);
		mqtt_id = esp_mqtt_client_publish(mqtt_client, hum_topic,itoa(humidity_i,buffer,10), 0, 1, 0);
		printf("MQTT: Publishing successful, message id = %d\n", mqtt_id);
		vTaskDelay(1000 / portTICK_RATE_MS);
		vTaskDelay((SEND_DELAY_MS-1000) / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
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
	restart_wifi(wifi_config);
	init_mesh();
	xTaskCreate(&sensor_read, "Sens", 2048, NULL, 5, NULL);
}
