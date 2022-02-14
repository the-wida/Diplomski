#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include <freertos/event_groups.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include <string.h>
#include "driver/gpio.h"
#include "mqtt_client.h"

#define RX_SIZE         5
#define TX_SIZE         5
#define SEND_DELAY_MS	15000
#define SENSOR_DELAY_MS	7500

static const char *MESH_TAG = "mesh_main";
static const char *MQTT_TAG = "mqtt_main";
static const char *DHT_TAG = "dht_main";
static const char *PROV_TAG = "provisioning_main";
static const char *NO_TAG = "";
char buffer[20];
char *provisioning_ssid;
char *provisioning_password;
char router_ssid[20];
char router_password[20];

static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };

static int mesh_layer = -1;
const int PROVISIONING_BIT = BIT0;
int DHTgpio = GPIO_NUM_19;
int mqtt_id;
int humidity_i = 0;
int temperature_i = 0;
int recv_hum;
int recv_temp;

static esp_netif_t *provisioning_netif;
static esp_netif_t *mesh_netif_station = NULL;

float humidity = 0.;
float temperature = 0.;

static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static EventGroupHandle_t is_provisioning_finished;
esp_mqtt_client_handle_t mqtt_client;


uint8_t getTiming(int desired_interval, bool desired_state)
{
    uint8_t timing_micro_seconds = 0;
    while (gpio_get_level(DHTgpio) == desired_state)
    {
        if (timing_micro_seconds > desired_interval)
		{
            return -1;
		}
        timing_micro_seconds++;
        ets_delay_us(1);
    }
    return timing_micro_seconds;
}
 
int readDHT()
{
    int micro_seconds = 0;
    uint8_t dht_data[5];
    uint8_t data_byte = 0;
    uint8_t data_bit = 7;
    for (uint8_t k = 0; k < 5; k++)
	{
        dht_data[k] = 0;
	}
    gpio_set_direction(DHTgpio, GPIO_MODE_OUTPUT);
    gpio_set_level(DHTgpio, 0);
    ets_delay_us(3000);
    gpio_set_level(DHTgpio, 1);
    ets_delay_us(25);
    gpio_set_direction(DHTgpio, GPIO_MODE_INPUT);
    micro_seconds = getTiming(85, 0);
    if (micro_seconds < 0)
	{
        printf("DHT Timeout\n");
		return 2;
	}
    micro_seconds = getTiming(85, 1);
    if (micro_seconds < 0)
	{
        printf("DHT Timeout\n");
		return 2;
	}
    for (uint8_t k = 0; k < 40; k++)
    {
        micro_seconds = getTiming(56, 0);
        if (micro_seconds < 0)
		{
            printf("DHT Timeout\n");
			return 2;
		}
        micro_seconds = getTiming(75, 1);
        if (micro_seconds < 0)
		{
            printf("DHT Timeout\n");
			return 2;
		}
        if (micro_seconds > 40)
        {
            dht_data[data_byte] |= (1 << data_bit);
        }
        if (data_bit == 0)
        {
            data_bit = 7;
            data_byte++;
        }
        else
		{
            data_bit--;
		}
    }
    humidity = dht_data[0];
    humidity *= 0x100;
    humidity += dht_data[1];
	humidity /= 10;
    temperature = dht_data[2] & 0x7F;
    temperature *= 0x100;
    temperature += dht_data[3];
	temperature /= 10;
    if (dht_data[2] & 0x80) 
	{
        temperature *= -1;
	}
    if (dht_data[4] == ((dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]) & 0xFF))
	{
        return 0;
	}
    else
	{
		printf("DHT checksum err\n");
        return 1;
	}
}

void sensor_read(void *arg)
{
	while (true)
	{
		readDHT();
		ESP_LOGI(DHT_TAG, "Hum: %.1f Tmp: %.1f\n", humidity, temperature);
		humidity_i = (int)(humidity*10);
		temperature_i = (int)(temperature*10);
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
            ESP_LOGE(MESH_TAG, "MESH: Recieve error:0x%x, size:%d", mesh_recv_error, data.size);
            continue;
        }
		recv_count = data.data[0];
		recv_temp = ((data.data[2]<<8)|(data.data[1]&0xff));
		recv_hum = ((data.data[4]<<8)|(data.data[3]&0xff));
		char from_mac_string[18];
		sprintf(from_mac_string, "%02X:%02X:%02X:%02X:%02X:%02X", from.addr[0], from.addr[1], from.addr[2], from.addr[3], from.addr[4], from.addr[5]);
		char hum_topic[39];
		char temp_topic[42];
		sprintf(hum_topic, "%s%s%s","/topic/", from_mac_string, "/1891_humidity");
		sprintf(temp_topic, "%s%s%s","/topic/", from_mac_string, "/1891_temperature");
        ESP_LOGW(MESH_TAG, "MESH: Recieved from %s, current layer: %d, current parent:"MACSTR", Message number: %d", from_mac_string, mesh_layer, MAC2STR(mesh_parent_addr.addr), recv_count);
		ESP_LOGW(NO_TAG, "Humidity: %d, Temperature: %d",recv_hum, recv_temp);
		if (esp_mesh_is_root())
		{
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, temp_topic, 1);
			ESP_LOGI(MQTT_TAG, "MQTT: Subscription successful, message id = %d", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, temp_topic, itoa(recv_temp,buffer,10), 0, 1, 0);
			ESP_LOGI(MQTT_TAG, "MQTT: Publishing successful, message id = %d", mqtt_id);
			mqtt_id = esp_mqtt_client_subscribe(mqtt_client, hum_topic, 1);
			ESP_LOGI(MQTT_TAG, "MQTT: Subscription successful, message id = %d", mqtt_id);
			mqtt_id = esp_mqtt_client_publish(mqtt_client, hum_topic,itoa(recv_hum,buffer,10), 0, 1, 0);
			ESP_LOGI(MQTT_TAG, "MQTT: Publishing successful, message id = %d", mqtt_id);
		}
    }
    vTaskDelete(NULL);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t mqtt_event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) 
	{
		case MQTT_EVENT_CONNECTED:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Client connected to broker");
		}
		break;
		case MQTT_EVENT_DISCONNECTED:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Client disconnected from broker");
		}
		break;
		case MQTT_EVENT_SUBSCRIBED:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Client subscribed, subscription message = %d", mqtt_event->msg_id);
		}
		break;
		case MQTT_EVENT_UNSUBSCRIBED:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Client unsubscribed, unsubscription message = %d", mqtt_event->msg_id);
		}
		break;
		case MQTT_EVENT_PUBLISHED:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Message published, message id = %d", mqtt_event->msg_id);
		}
		break;
		case MQTT_EVENT_DATA:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Client recieved data:");
			printf("TOPIC = %.*s\r\n", mqtt_event->topic_len, mqtt_event->topic);
			printf("DATA = %.*s\r\n", mqtt_event->data_len, mqtt_event->data);
		}
		break;
		case MQTT_EVENT_ERROR:
		{
			ESP_LOGI(MQTT_TAG, "MQTT: Encountered an error");
		}
		break;
		default:
		break;
    }
}

void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    mesh_addr_t mesh_id = {0,};
    static uint16_t mesh_last_layer = 0;
    switch (event_id) 
	{
		case MESH_EVENT_STARTED: 
		{
			esp_mesh_get_id(&mesh_id);
			ESP_LOGI(MESH_TAG, "MESH: Started with id:"MACSTR"", MAC2STR(mesh_id.addr));
			is_mesh_connected = false;
			mesh_layer = esp_mesh_get_layer();
		}
		break;
		case MESH_EVENT_STOPPED: 
		{
			ESP_LOGI(MESH_TAG, "MESH: Stopped");
			is_mesh_connected = false;
			mesh_layer = esp_mesh_get_layer();
		}
		break;
		case MESH_EVENT_CHILD_CONNECTED: 
		{
			mesh_event_child_connected_t *mesh_child_connected = (mesh_event_child_connected_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Child connected, aid:%d, MAC: "MACSTR"", mesh_child_connected->aid, MAC2STR(mesh_child_connected->mac));
		}
		break; 
		case MESH_EVENT_CHILD_DISCONNECTED: 
		{
			mesh_event_child_disconnected_t *mesh_child_disconnected = (mesh_event_child_disconnected_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Child disconnected, aid:%d, MAC: "MACSTR"", mesh_child_disconnected->aid, MAC2STR(mesh_child_disconnected->mac));
		}
		break;
		case MESH_EVENT_ROUTING_TABLE_ADD: 
		{
			mesh_event_routing_table_change_t *mesh_routing_table = (mesh_event_routing_table_change_t *)event_data;
			ESP_LOGW(MESH_TAG, "MESH: Routing table changed: %d to: %d, layer:%d", mesh_routing_table->rt_size_change, mesh_routing_table->rt_size_new, mesh_layer);
		}
		break;
		case MESH_EVENT_ROUTING_TABLE_REMOVE: 
		{
			mesh_event_routing_table_change_t *mesh_routing_table = (mesh_event_routing_table_change_t *)event_data;
			ESP_LOGW(MESH_TAG, "MESH: Routing table removed: %d, layer:%d", mesh_routing_table->rt_size_change, mesh_layer);
		}
		break;
		case MESH_EVENT_NO_PARENT_FOUND: 
		{
			mesh_event_no_parent_found_t *mesh_no_parent = (mesh_event_no_parent_found_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: No parent found, scanned %d times", mesh_no_parent->scan_times);
		}
		break;
		case MESH_EVENT_PARENT_CONNECTED: 
		{
			mesh_event_connected_t *mesh_parent_connected = (mesh_event_connected_t *)event_data;
			esp_mesh_get_id(&mesh_id);
			mesh_layer = mesh_parent_connected->self_layer;
			memcpy(&mesh_parent_addr.addr, mesh_parent_connected->connected.bssid, 6);
			static char mesh_is_root_string[24] ="";
			if (esp_mesh_is_root())
			{
				strcpy(mesh_is_root_string, "This node is the root");
			}
			else if (mesh_layer == 2)
			{
				strcpy(mesh_is_root_string, "This node is on layer 2");
			}
			ESP_LOGI(MESH_TAG, "MESH: Parent connected, layer change: %d->%d, parent: "MACSTR"", mesh_last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr));
			ESP_LOGI(NO_TAG, "%s MESH id: "MACSTR", duty:%d", mesh_is_root_string, MAC2STR(mesh_id.addr), mesh_parent_connected->duty);
			mesh_last_layer = mesh_layer;
			is_mesh_connected = true;
			if (esp_mesh_is_root()) 
			{
				esp_netif_dhcpc_stop(mesh_netif_station);
				esp_netif_dhcpc_start(mesh_netif_station);
				esp_mqtt_client_start(mqtt_client);
				xTaskCreate(mesh_recieve, "MPRX", 3072, NULL, 5, NULL);
			}
			xTaskCreate(mesh_send, "MPTX", 3072, NULL, 5, NULL);
		}
		break;
		case MESH_EVENT_PARENT_DISCONNECTED: 
		{
			mesh_event_disconnected_t *mesh_parent_disconnected = (mesh_event_disconnected_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Parent disconnected, reason:%d", mesh_parent_disconnected->reason);
			is_mesh_connected = false;
			mesh_layer = esp_mesh_get_layer();
		}
		break;
		case MESH_EVENT_LAYER_CHANGE: 
		{
			mesh_event_layer_change_t *mesh_layer_change = (mesh_event_layer_change_t *)event_data;
			mesh_layer = mesh_layer_change->new_layer;
			static char mesh_layer_string[24] ="";
			if (esp_mesh_is_root())
			{
				strcpy(mesh_layer_string, "This node is the root");
			}
			else if (mesh_layer == 2)
			{
				strcpy(mesh_layer_string, "This node is on layer 2");
			}
			ESP_LOGI(MESH_TAG, "MESH: Layer change: %d->%d %s",mesh_last_layer, mesh_layer, mesh_layer_string);
			mesh_last_layer = mesh_layer;
		}
		break;
		case MESH_EVENT_ROOT_ADDRESS: 
		{
			mesh_event_root_address_t *mesh_root_addr = (mesh_event_root_address_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Root address obtained: "MACSTR"", MAC2STR(mesh_root_addr->addr));
		}
		break;
		case MESH_EVENT_VOTE_STARTED: 
		{
			mesh_event_vote_started_t *mesh_vote_started = (mesh_event_vote_started_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: vote started, attempts:%d, reason:%d, candidate:"MACSTR"", mesh_vote_started->attempts, mesh_vote_started->reason, MAC2STR(mesh_vote_started->rc_addr.addr));
		}
		break;
		case MESH_EVENT_VOTE_STOPPED: 
		{
			ESP_LOGI(MESH_TAG, "MESH: Vote stopped");
			break;
		}
		case MESH_EVENT_ROOT_SWITCH_REQ: 
		{
			mesh_event_root_switch_req_t *mesh_switch_req = (mesh_event_root_switch_req_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Root switch requested, reason:%d, requester: "MACSTR"", mesh_switch_req->reason, MAC2STR(mesh_switch_req->rc_addr.addr));
		}
		break;
		case MESH_EVENT_ROOT_SWITCH_ACK: 
		{
			ESP_LOGI(MESH_TAG, "MESH: Root switch acknowledged");
		}
		break;
		case MESH_EVENT_TODS_STATE: 
		{
			mesh_event_toDS_state_t *mesh_tods_state = (mesh_event_toDS_state_t *)event_data;
			if (mesh_tods_state == MESH_TODS_UNREACHABLE)
			{
				ESP_LOGI(MESH_TAG, "MESH: Outside IP network not reachable by root");
			}
			else
			{
				ESP_LOGI(MESH_TAG, "MESH: Outside IP network reachable by root");
			}
		}
		break;
		case MESH_EVENT_CHANNEL_SWITCH:
		{
			mesh_event_channel_switch_t *mesh_channel_switch = (mesh_event_channel_switch_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Switched to channel %d", mesh_channel_switch->channel);
		}
		break;
		case MESH_EVENT_NETWORK_STATE: 
		{
			mesh_event_network_state_t *mesh_network_state = (mesh_event_network_state_t *)event_data;
			if (mesh_network_state->is_rootless)
			{
				ESP_LOGI(MESH_TAG, "MESH: Mesh is rootless");
			}
			else
			{
				ESP_LOGI(MESH_TAG, "MESH: Mesh has a root");
			}
		}
		break;
		case MESH_EVENT_STOP_RECONNECTION: 
		{
			ESP_LOGI(MESH_TAG, "MESH: Devices stopped reconnecting");
		}
		break;
		case MESH_EVENT_FIND_NETWORK: 
		{
			mesh_event_find_network_t *mesh_find_network = (mesh_event_find_network_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Found new channel:%d, router BSSID:"MACSTR"", mesh_find_network->channel, MAC2STR(mesh_find_network->router_bssid));
		}
		break;
		case MESH_EVENT_ROUTER_SWITCH: 
		{
			mesh_event_router_switch_t *mesh_router_switch = (mesh_event_router_switch_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Router switched, new router:%s, channel:%d, "MACSTR"", mesh_router_switch->ssid, mesh_router_switch->channel, MAC2STR(mesh_router_switch->bssid));
		}
		break;
		default:
		break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&ip_event->ip_info.ip));
}

static void provisioning_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static int provisioning_retries;
    switch (event_id) 
	{
        case WIFI_PROV_CRED_RECV: 
		{
            wifi_sta_config_t *wifi_station_config = (wifi_sta_config_t *)event_data;
			provisioning_ssid = (char *) wifi_station_config->ssid;			
			provisioning_password = (char *) wifi_station_config->password;
            ESP_LOGI(PROV_TAG, "Received Wi-Fi credentials""\n\tSSID     : %s\n\tPassword : %s",provisioning_ssid,provisioning_password);
			strcpy(router_ssid, provisioning_ssid);
			strcpy(router_password, provisioning_password);  
        }
		break;
        case WIFI_PROV_CRED_FAIL: 
		{
            wifi_prov_sta_fail_reason_t *failure_reason = (wifi_prov_sta_fail_reason_t *)event_data;
			static char failure_reason_string[36];
			if(*failure_reason == WIFI_PROV_STA_AUTH_ERROR) 
			{
				strcpy(failure_reason_string, "Wi-Fi station authentication failed");
			}
			else
			{
				 strcpy(failure_reason_string, "Wi-Fi access-point not found");
			}
            ESP_LOGE(PROV_TAG, "Provisioning failed!\nReason: %s\nPlease reset to factory and retry provisioning",failure_reason_string);
            provisioning_retries++;
            if (provisioning_retries >= 3) 
			{
                ESP_LOGI(PROV_TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
                wifi_prov_mgr_reset_sm_state_on_failure();
                provisioning_retries = 0;
            } 
        }
		break;
        case WIFI_PROV_CRED_SUCCESS:
		{
            ESP_LOGI(PROV_TAG, "Provisioning successful");
            provisioning_retries = 0;
		}
		break;
        case WIFI_PROV_END:
		{
			wifi_prov_mgr_deinit();
			esp_wifi_disconnect();
			esp_netif_destroy_default_wifi(provisioning_netif);
			xEventGroupSetBits(is_provisioning_finished, PROVISIONING_BIT); 
		}
		break;
		default:
		break;
    } 
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_STA_START) 
		{
			esp_wifi_connect();
		} 
}

static void get_device_service_name(char *service_name, size_t print_size)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, print_size, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
	{
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
	esp_mqtt_client_config_t mqtt_config = 
	{
		.uri = "mqtt://test.mosquitto.org",
	};
	mqtt_client = esp_mqtt_client_init(&mqtt_config);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	is_provisioning_finished = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &provisioning_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
	ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
	provisioning_netif = esp_netif_create_default_wifi_sta();
	wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    wifi_prov_mgr_config_t provisioning_config = 
	{
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(provisioning_config));
    wifi_prov_mgr_reset_provisioning();
    ESP_LOGI(PROV_TAG, "Starting provisioning");
    char provisioning_service_name[12];
    get_device_service_name(provisioning_service_name, sizeof(provisioning_service_name));
    wifi_prov_security_t provisioning_security = WIFI_PROV_SECURITY_1;
    const char *provisioning_pop = "abcd1234";
    const char *provisioning_service_key = NULL;
    uint8_t provisioning_service_uuid[] = {0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,};
    wifi_prov_scheme_ble_set_service_uuid(provisioning_service_uuid);
    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(provisioning_security, provisioning_pop, provisioning_service_name, provisioning_service_key));
	xEventGroupWaitBits(is_provisioning_finished, PROVISIONING_BIT, false, true, portMAX_DELAY);
	esp_wifi_stop();
	esp_wifi_deinit();
	esp_wifi_init(&wifi_config);
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&mesh_netif_station, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(0.9));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
    mesh_cfg_t mesh_config = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &mesh_config.mesh_id, MESH_ID, 6);
    mesh_config.channel = 0;
    mesh_config.router.ssid_len = strlen(router_ssid);
    memcpy((uint8_t *) &mesh_config.router.ssid, router_ssid, mesh_config.router.ssid_len);
    memcpy((uint8_t *) &mesh_config.router.password, router_password, strlen(router_password));
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA_WPA2_PSK));
    mesh_config.mesh_ap.max_connection = 6;
    mesh_config.mesh_ap.nonmesh_max_connection = 0;
    memcpy((uint8_t *) &mesh_config.mesh_ap.password, "mesh_1891", strlen("mesh_1891"));
    ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_config));
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(5, MESH_PS_DEVICE_DUTY_DEMAND));
    ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(5, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE));
    ESP_LOGI(MESH_TAG, "MESH: Started successfully, Heap: %d, Power Saving is enabled",  esp_get_minimum_free_heap_size());
	printf("%d\n", esp_mesh_is_ps_enabled());
	xTaskCreate(&sensor_read, "Sens", 2048, NULL, 5, NULL);
}
