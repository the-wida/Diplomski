#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "handlers.h"
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "general_init.h"
#include <string.h>

esp_mqtt_client_handle_t init_mqtt_client;
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};

void get_device_service_name(char *service_name, size_t print_size)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, print_size, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

void init_memory()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
	{
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

esp_mqtt_client_handle_t init_mqtt(char *url)
{
	esp_mqtt_client_config_t mqtt_config = 
	{
		.uri = url,
	};
	init_mqtt_client = esp_mqtt_client_init(&mqtt_config);
	return init_mqtt_client;
}

void register_handlers(esp_mqtt_client_handle_t client)
{
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &provisioning_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
	ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
}

void start_provisioning()
{
	wifi_prov_mgr_config_t provisioning_config = 
	{
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(provisioning_config));
    wifi_prov_mgr_reset_provisioning();
    printf("Starting provisioning\n");
    char provisioning_service_name[12];
    get_device_service_name(provisioning_service_name, sizeof(provisioning_service_name));
    wifi_prov_security_t provisioning_security = WIFI_PROV_SECURITY_1;
    const char *provisioning_pop = "abcd1234";
    const char *provisioning_service_key = NULL;
    uint8_t provisioning_service_uuid[] = {0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,};
    wifi_prov_scheme_ble_set_service_uuid(provisioning_service_uuid);
	ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(provisioning_security, provisioning_pop, provisioning_service_name, provisioning_service_key));	
}

void init_mesh()
{
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
    printf("MESH: Started successfully, Heap: %d, Power Saving is enabled\n",  esp_get_minimum_free_heap_size());
}

void oneshot_timer_callback(void* arg)
{
	if (!provisioning_recieved)
	{
		wifi_prov_mgr_stop_provisioning();
		strcpy(router_ssid, "NO_ROUTER");
		strcpy(router_password, "NO_PASSWORD");  
		printf("Provisioning skipped\n");
	}
}

void start_timer()
{
	const esp_timer_create_args_t oneshot_timer_args = 
	{
            .callback = &oneshot_timer_callback,
            .name = "one-shot"
    };
    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
	ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 35000000));	
}

void restart_wifi(wifi_init_config_t config)
{
	esp_wifi_stop();
	esp_wifi_deinit();
	esp_wifi_init(&config);
}

void aditional_init()
{
	ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	is_provisioning_finished = xEventGroupCreate();
}