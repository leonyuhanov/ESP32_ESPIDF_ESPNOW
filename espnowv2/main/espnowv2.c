/* 
		THIS 2
			RX Module 30:AE:A4:21:28:44
		REMOTE 1
			TX Module 30:AE:A4:27:A9:48
*/

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "espnowv2.h"

#define CONFIG_ESPNOW_CHANNEL				1
#define ESPNOW_WIFI_MODE 					WIFI_MODE_STA
#define ESPNOW_WIFI_IF 						ESP_IF_WIFI_STA
#define MAX_ESPNOW_PACKET_SIZE				250


unsigned char CONFIG_ESPNOW_LMK[ESP_NOW_KEY_LEN];
static const char *TAG = "espnow_example";
uint8_t	localMac[6];

uint8_t espNowNode[ESP_NOW_ETH_ALEN] = {0x30, 0xAE, 0xA4, 0x27, 0xA9, 0x48};;
example_espnow_send_param_t *send_param;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

static esp_err_t example_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* WiFi should start before using ESPNOW */
static void initWifi(void)
{
    esp_err_t ret;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	uint8_t cnt=0;
	
	tcpip_adapter_init();
    ret = esp_event_loop_init(example_event_handler, NULL);
    ret = esp_wifi_init(&cfg);
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    ret = esp_wifi_set_mode( ESPNOW_WIFI_MODE );
    ret = esp_wifi_start();
    ret = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0);
	ret = esp_wifi_set_ps(WIFI_PS_NONE);
	ret = esp_wifi_get_mac(ESPNOW_WIFI_IF, localMac);
	printf("\r\n[%d]\r\n", ret);
	printf("\r\nWIFI_MODE_STA MAC Address:\t");
	for(cnt=0; cnt<6; cnt++)
	{
		if(cnt+1<6)
		{
			printf("%X:", localMac[cnt]);
		}
		else
		{
			printf("%X", localMac[cnt]);
		}
	}
	printf("\r\nEnabling Long range setting...[");
	ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N); //WIFI_PROTOCOL_LR
	printf(esp_err_to_name(ret));
	printf("]\r\n");
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
	int ret=0;
	
    if (mac_addr == NULL || data == NULL || len <= 0)
	{
        printf("\r\n\t\t\tERROR MAC OR DATA BOCKS NULL\r\n");
        return;
    }
	else
	{
		//printf("\r\n\tGot Data value\t[%d]\r\n", data[0]);
	}
	
	//send reply
    //send_param->unicast = true;
    //send_param->broadcast = false;
    //send_param->state = 0;
    //send_param->magic = esp_random();
    //send_param->count = 1;//CONFIG_ESPNOW_SEND_COUNT;
    //send_param->delay = 0;//CONFIG_ESPNOW_SEND_DELAY;
    //send_param->len = MAX_ESPNOW_PACKET_SIZE;//CONFIG_ESPNOW_SEND_LEN;
    //memcpy(send_param->dest_mac, espNowNode, ESP_NOW_ETH_ALEN);
    //memcpy(send_param->buffer, data, send_param->len);
	send_param->buffer[0] = data[0];
	if ((ret=esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len)) != ESP_OK)
	{
		printf("\r\n\t\t");
		printf(esp_err_to_name(ret));
		printf("\r\n");
		example_espnow_deinit(send_param);
		vTaskDelete(NULL);
	}
	else
	{
		//printf("\t\tReply sent!");
	}	
}

static esp_err_t example_espnow_init(void)
{

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
	
	//Add espnow node to reply to
	printf("\r\n\tAdd Remote node(peer) to local peer list...");
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
	memcpy(peer->peer_addr, espNowNode, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) ); 
	free(peer);
	printf("Remote Peer(the sender) Added!\r\n");


    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = 1;//CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = 0;//CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = MAX_ESPNOW_PACKET_SIZE;//CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(send_param->len);
    memcpy(send_param->dest_mac, espNowNode, ESP_NOW_ETH_ALEN);
    
	return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    esp_now_deinit();
}

void app_main()
{
    printf("\r\n\r\nReceiver Starting...\r\n");
	// Initialize NVS
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	printf("\r\nSetting Up WIFI....\r\n");
    initWifi();
	printf("\r\nWIFI Setup Complete!\r\n");
    example_espnow_init();
	while(true)
	{
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}