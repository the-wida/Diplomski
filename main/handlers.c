#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include <string.h>
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "general_init.h"
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include <freertos/event_groups.h>

const char *IP_TAG = "ip_main";
const char *MESH_TAG = "mesh_main";
const char *MQTT_TAG = "mqtt_main";
const char *PROV_TAG = "provisioning_main";
const char *NO_TAG = "";

int mesh_layer;
mesh_addr_t mesh_parent_addr;
esp_netif_t *mesh_netif_station;
esp_mqtt_client_handle_t mqtt_client;
char *provisioning_ssid;
char *provisioning_password;
char router_ssid[20];
char router_password[20];
bool provisioning_recieved;
esp_netif_t *provisioning_netif;
int PROVISIONING_BIT;
int recieve_delay;

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_STA_START) 
		{
			esp_wifi_connect();
		} 
}

void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(IP_TAG, "IP: Got new address: " IPSTR, IP2STR(&ip_event->ip_info.ip));
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
			mesh_layer = esp_mesh_get_layer();
		}
		break;
		case MESH_EVENT_STOPPED: 
		{
			ESP_LOGI(MESH_TAG, "MESH: Stopped");
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
			recieve_delay = get_base_recieve_delay()/(mesh_routing_table->rt_size_new);
			ESP_LOGW(MESH_TAG, "MESH: Recieve delay changed to: %d", recieve_delay);
		}
		break;
		case MESH_EVENT_ROUTING_TABLE_REMOVE: 
		{
			mesh_event_routing_table_change_t *mesh_routing_table = (mesh_event_routing_table_change_t *)event_data;
			ESP_LOGW(MESH_TAG, "MESH: Routing table removed: %d, layer:%d", mesh_routing_table->rt_size_change, mesh_layer);
			recieve_delay = get_base_recieve_delay()/(mesh_routing_table->rt_size_new);
			ESP_LOGW(MESH_TAG, "MESH: Recieve delay changed to: %d", recieve_delay);
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
			ESP_LOGI(NO_TAG, "%s MESH id: "MACSTR", parent duty:%d", mesh_is_root_string, MAC2STR(mesh_id.addr), mesh_parent_connected->duty);
			mesh_last_layer = mesh_layer;
			if (esp_mesh_is_root()) 
			{
				esp_netif_dhcpc_stop(mesh_netif_station);
				esp_netif_dhcpc_start(mesh_netif_station);
				esp_mqtt_client_start(mqtt_client);
				xTaskCreate(mesh_recieve, "MPRX", 3072, NULL, 5, NULL);
				xTaskCreate(send_own_data, "SROOT", 3072, NULL, 5, NULL);
				break;
			}
			xTaskCreate(mesh_send, "MPTX", 3072, NULL, 5, NULL);
		}
		break;
		case MESH_EVENT_PARENT_DISCONNECTED: 
		{
			mesh_event_disconnected_t *mesh_parent_disconnected = (mesh_event_disconnected_t *)event_data;
			ESP_LOGI(MESH_TAG, "MESH: Parent disconnected, reason:%d", mesh_parent_disconnected->reason);
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

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
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

void provisioning_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
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
			provisioning_recieved = true;
        }
		break;
        case WIFI_PROV_CRED_FAIL: 
		{
            wifi_prov_sta_fail_reason_t *failure_reason = (wifi_prov_sta_fail_reason_t *)event_data;
			char failure_reason_string[36];
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

