/* 
		THIS 2
			RX Module 30:AE:A4:21:28:44
		REMOTE 1
			TX Module 30:AE:A4:21:B4:88
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
#define MAX_ESPNOW_PACKET_SIZE				10


unsigned char CONFIG_ESPNOW_LMK[ESP_NOW_KEY_LEN];
static const char *TAG = "espnow_example";
//static xQueueHandle example_espnow_queue;
uint8_t	localMac[6];

static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0x30, 0xAE, 0xA4, 0x21, 0x28, 0x44 };
uint8_t espNowNode[ESP_NOW_ETH_ALEN] = {0x30, 0xAE, 0xA4, 0x21, 0xB4, 0x88};
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };
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
	printf("\r\n");
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	//printf("\r\n\tData WAS Sent!\r\n");
	//example_espnow_event_t evt;
    //example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    //if (mac_addr == NULL) 
	//{
    //    ESP_LOGE(TAG, "Send cb arg error");
    //    return;
    //}

    //evt.id = EXAMPLE_ESPNOW_SEND_CB;
    //memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    //send_cb->status = status;
	//if (xQueueSend(example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
	//{
    //    ESP_LOGW(TAG, "Send send queue fail");
    //}
	
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    //example_espnow_event_t evt;
    //example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
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
	//example_espnow_send_param_t *send_param;
    //send_param = malloc(sizeof(example_espnow_send_param_t));
    //memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = 1;//CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = 0;//CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = MAX_ESPNOW_PACKET_SIZE;//CONFIG_ESPNOW_SEND_LEN;
    memcpy(send_param->dest_mac, espNowNode, ESP_NOW_ETH_ALEN);
    memcpy(send_param->buffer, data, send_param->len);
	
	//prepDataBlock(send_param);
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
	//free memory and exit
	//free(recv_cb->data);
	//free(send_param);

    //evt.id = EXAMPLE_ESPNOW_RECV_CB;
    //memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    //recv_cb->data = malloc(len);
    //if (recv_cb->data == NULL)
	//{
    //    ESP_LOGE(TAG, "Malloc receive data fail");
    //    return;
    //}
    //memcpy(recv_cb->data, data, len);
    //recv_cb->data_len = len;
    //if (xQueueSend(example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
	//{
    //    ESP_LOGW(TAG, "Send receive queue fail");
    //    free(recv_cb->data);
    //}
	
	//printf("\r\n\tGot something\r\n");
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

/* Prepare ESPNOW data to be sent. */
void prepDataBlock(example_espnow_send_param_t *send_param)
{
    espNowDataBlock *buf = (espNowDataBlock *)send_param->buffer;
    int i = 0;

    assert(send_param->len >= sizeof(espNowDataBlock));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    for (i = 0; i < send_param->len - sizeof(espNowDataBlock); i++) {
        buf->payload[i] = (uint8_t)esp_random();
    }
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static esp_err_t example_espnow_init(void)
{
    //example_espnow_send_param_t *send_param;

    //example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    //if (example_espnow_queue == NULL) 
	//{
    //    ESP_LOGE(TAG, "Create mutex fail");
    //    return ESP_FAIL;
    //}

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    //peer->channel = CONFIG_ESPNOW_CHANNEL;
    //peer->ifidx = ESPNOW_WIFI_IF;
    //peer->encrypt = false;
    //memcpy(peer->peer_addr, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    //ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    //free(peer);
	
	//Add espnow node to reply to
	printf("\r\n\tAdd Remote node(peer) to local peer list...");
	//peer = malloc(sizeof(esp_now_peer_info_t));
	//memset(peer, 0, sizeof(esp_now_peer_info_t));
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
    //prepDataBlock(send_param);
 
	//xTaskCreate(sendESPNOWData, "sendESPNOWData", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    //vSemaphoreDelete(example_espnow_queue); 
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