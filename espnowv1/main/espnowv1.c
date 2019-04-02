/* 
		THIS 1
			TX Module 30:AE:A4:27:A9:48
		REMOTE 2
			RX Module 30:AE:A4:21:28:44
			
*/#include <stdlib.h>
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
#include "espnowv1.h"
#include "esp_timer.h"

#define CONFIG_ESPNOW_CHANNEL				1
#define ESPNOW_WIFI_MODE 					WIFI_MODE_STA
#define ESPNOW_WIFI_IF 						ESP_IF_WIFI_STA
#define MAX_ESPNOW_PACKET_SIZE				250


unsigned char CONFIG_ESPNOW_LMK[ESP_NOW_KEY_LEN];
static const char *TAG = "espnow_example";
uint8_t	localMac[6];

static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0x30, 0xAE, 0xA4, 0x21, 0x28, 0x44 };
uint8_t espNowNode[ESP_NOW_ETH_ALEN] = {0x30, 0xAE, 0xA4, 0x27, 0xA9, 0x48}; 

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

typedef struct {
	uint8_t isReady;
	uint8_t previousSend;
	uint8_t currentData;
	
} esnowQueObject;



esnowQueObject globalQue;
long espnowTimers[3];
example_espnow_send_param_t *send_param_var;

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
	ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
	printf(esp_err_to_name(ret));
	printf("]\r\n");
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{

    if (mac_addr == NULL) 
	{
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }	
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }
	globalQue.currentData = data[0];
	if(globalQue.previousSend == globalQue.currentData)
	{
		globalQue.isReady=1;
		espnowTimers[1]=esp_timer_get_time()-espnowTimers[0];
		espnowTimers[2]+=espnowTimers[1];
	}
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espNowDataBlock *buf = (espNowDataBlock *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espNowDataBlock)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

static void sendESPNOWData(void *pvParameter)
{
    int ret;
	unsigned short int sendCount=0, maxSends=1000, missCnt=0;
	long maxTimeOut = 1000000;
    printf("\r\n");
    /* Start sending ESPNOW data. */
    
	send_param_var = (example_espnow_send_param_t *)pvParameter;
    
	while(sendCount<maxSends)
	{
		printf("\r\n\tCurrent sendCount\t%d\t", sendCount);
		send_param_var->buffer[0] = sendCount;
		if ((ret=esp_now_send(send_param_var->dest_mac, send_param_var->buffer, send_param_var->len)) != ESP_OK)
		{
			printf(esp_err_to_name(ret));
			example_espnow_deinit(send_param_var);
			vTaskDelete(NULL);
		}
		else
		{
			espnowTimers[0] = esp_timer_get_time();
			//wait for reply before trasmiting
			globalQue.previousSend = sendCount;
			while(!globalQue.isReady)
			{
				espnowTimers[1]=esp_timer_get_time()-espnowTimers[0];
				if(espnowTimers[1]>maxTimeOut)
				{
					globalQue.isReady=2;
				}
				//vTaskDelay(1 / portTICK_RATE_MS);
			}
			if(globalQue.isReady==1)
			{
				printf("RTT\t%lu microseconds", espnowTimers[1]);
			}
			else if(globalQue.isReady==2)
			{
				missCnt++;
				printf("TIMEOUT!");
			}
			globalQue.isReady=0;
			sendCount++;
		}
		vTaskDelay(5 / portTICK_RATE_MS);
	}
	printf("\r\n\r\n\t\t\tAvg RTT\t%lu\tSent\t%d\tMissed\t%d\r\n", espnowTimers[2]/(maxSends-missCnt), (maxSends-missCnt), missCnt);
	example_espnow_deinit(send_param_var);
	vTaskDelete(NULL);
	printf("\r\n\tOut of sendESPNOWData\r\n");
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    //free(peer);
	
	//Add espnow node to reply to
	printf("\r\n\tAdd Remote node(peer) to local peer list...");
	memset(peer, 0, sizeof(esp_now_peer_info_t));
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
	memcpy(peer->peer_addr, espNowNode, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) ); 
	free(peer);
	printf("Peer Added!\r\n");


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
    memcpy(send_param->dest_mac, example_broadcast_mac, ESP_NOW_ETH_ALEN);

	printf("\r\nESPNOW Set up complete!\r\n");
	xTaskCreate(sendESPNOWData, "sendESPNOWData", 2048, send_param, 4, NULL);

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
    // Initialize NVS
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	globalQue.isReady=0;
	globalQue.previousSend=255;
	globalQue.currentData=0;
	
	printf("\r\nSetting Up WIFI....\r\n");
    initWifi();
	printf("\r\nWIFI Setup Complete!\r\n");
	espnowTimers[2]=0;
    example_espnow_init();
}