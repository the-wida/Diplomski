#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include <freertos/event_groups.h>
#include "esp_event.h"

extern int mesh_layer;
extern mesh_addr_t mesh_parent_addr;
extern esp_netif_t *mesh_netif_station;
extern esp_mqtt_client_handle_t mqtt_client;
extern char router_ssid[20];
extern char router_password[20];
extern bool provisioning_recieved;
extern esp_netif_t *provisioning_netif;
extern int PROVISIONING_BIT;
extern EventGroupHandle_t is_provisioning_finished;


void init_memory();
esp_mqtt_client_handle_t init_mqtt(char *url);
void mesh_recieve(void *arg);
void mesh_send(void *arg);
void register_handlers(esp_mqtt_client_handle_t client);
void start_provisioning();
void init_mesh();
void start_timer();
void restart_wifi(wifi_init_config_t config);
void aditional_init();
void oneshot_timer_callback(void* arg);
void send_own_data();