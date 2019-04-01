/* 
		THIS 1
			TX Module 30:AE:A4:21:B4:88
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
#define MAX_ESPNOW_PACKET_SIZE				10

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload


unsigned char CONFIG_ESPNOW_LMK[ESP_NOW_KEY_LEN];
static const char *TAG = "espnow_example";
//static xQueueHandle example_espnow_queue;
uint8_t	localMac[6];

static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0x30, 0xAE, 0xA4, 0x21, 0x28, 0x44 };
uint8_t espNowNode[ESP_NOW_ETH_ALEN] = {0x30, 0xAE, 0xA4, 0x21, 0xB4, 0x88};
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

typedef struct {
	uint8_t isReady;
	uint8_t previousSend;
	uint8_t currentData;
	
} esnowQueObject;



esnowQueObject globalQue;
long espnowTimers[2];
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
    //example_espnow_event_t evt;
    //example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) 
	{
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
	//printf("\tData has been sent!\r\n");
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

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }
	globalQue.currentData = data[0];
	if(globalQue.previousSend == globalQue.currentData)
	{
		globalQue.isReady=1;
	}
    //evt.id = EXAMPLE_ESPNOW_RECV_CB;
    //memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    //recv_cb->data = malloc(len);
    //if (recv_cb->data == NULL) {
    //    ESP_LOGE(TAG, "Malloc receive data fail");
    //    return;
    //}
    //memcpy(recv_cb->data, data, len);
    //recv_cb->data_len = len;
    //if (xQueueSend(example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
    //    ESP_LOGW(TAG, "Send receive queue fail");
    //    free(recv_cb->data);
    //}
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

static void sendESPNOWData(void *pvParameter)
{
    //example_espnow_event_t evt;
    //uint8_t recv_state = 0;
    //uint16_t recv_seq = 0;
    //int recv_magic = 0;
    //bool is_broadcast = false;
    int ret;
	unsigned short int sendCount=0, maxSends=10000;

    //vTaskDelay(1000 / portTICK_RATE_MS);
    printf("\r\n");
    /* Start sending ESPNOW data. */
    
	send_param_var = (example_espnow_send_param_t *)pvParameter;
    
	while(sendCount<maxSends)
	{
		printf("\tCurrent sendCount\t%d\r\n", sendCount);
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
				if( (esp_timer_get_time()-espnowTimers[0])>20000)
				{
					globalQue.isReady=2;
				}
				vTaskDelay(10 / portTICK_RATE_MS);
			}
			if(globalQue.isReady==1)
			{
				espnowTimers[1] = esp_timer_get_time();
				printf("\t\tRound trip took\t%lu microseconds\r\n", espnowTimers[1]-espnowTimers[0]);
			}
			else if(globalQue.isReady==2)
			{
				printf("\t\tTIMEOUT!\r\n");
			}
			globalQue.isReady=0;
			sendCount++;
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
	example_espnow_deinit(send_param_var);
    vTaskDelete(NULL);
	/*
    while (xQueueReceive(example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
	{
        printf("\tInside While loop\tEventID [%d]\t[EXAMPLE_ESPNOW_SEND_CB\t%d\tEXAMPLE_ESPNOW_RECV_CB\t%d\r\n", evt.id, EXAMPLE_ESPNOW_SEND_CB, EXAMPLE_ESPNOW_RECV_CB);
		switch (evt.id)
		{
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false))
				{
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                // Delay a while before sending the next data.
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                prepDataBlock(send_param);

                //Send the next data after the previous data is sent.
                if((ret = esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len)) != ESP_OK)
				{
                    printf("\tERROR\t%d\r\n", ret);
					ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                //if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST)
				//{
                    ESP_LOGI(TAG, "Receive %dth data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    // If MAC address does not exist in peer list, add it to peer list.
					if (esp_now_is_peer_exist(recv_cb->mac_addr) == false)
					{
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) ); 
                        free(peer);
                    }
					
					
                    //Indicates that the device has received broadcast ESPNOW data.
                    //if (send_param->state == 0) {
                    //    send_param->state = 1;
                    //}

                   
                    //if (recv_state == 1 || recv_state == 0)
					//{
                        
                        //if (send_param->unicast == false && send_param->magic >= recv_magic) {
						//if (send_param->unicast == true && send_param->magic >= recv_magic)
						//{
                    	    //ESP_LOGI(TAG, "Start sending unicast data");
                    	    //ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));
						if( esp_now_is_peer_exist(recv_cb->mac_addr) == false)
						{
							printf("\tAdding peer...\r\n");
							esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
							memset(peer, 0, sizeof(esp_now_peer_info_t));
							peer->channel = CONFIG_ESPNOW_CHANNEL;
							peer->ifidx = ESPNOW_WIFI_IF;
							peer->encrypt = false;
							memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
							memcpy(peer->peer_addr, espNowNode, ESP_NOW_ETH_ALEN);
							ESP_ERROR_CHECK( esp_now_add_peer(peer) ); 
							free(peer);
						}
						
						printf("\tSening ESPNOW Data...\r\n");
                    	    //Start sending unicast ESPNOW data
                            //memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
							memcpy(send_param->dest_mac, espNowNode, ESP_NOW_ETH_ALEN);
                            prepDataBlock(send_param);
                            if ((ret=esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len)) != ESP_OK)
							{
                                printf(esp_err_to_name(ret));
								ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else
							{
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                       // }
                    //}
                //}
                //else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST)
				//{
                //    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    // If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data.
                //    send_param->broadcast = false;
                ///}
                //else {
                //    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                //}
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
	*/
	printf("\r\n\tOut of sendESPNOWData\r\n");
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    //example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    //if (example_espnow_queue == NULL) 
	//{
    //    ESP_LOGE(TAG, "Create mutex fail");
    //    return ESP_FAIL;
    ///}

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
	//peer = malloc(sizeof(esp_now_peer_info_t));
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
    prepDataBlock(send_param);

 
	xTaskCreate(sendESPNOWData, "sendESPNOWData", 2048, send_param, 4, NULL);

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
    // Initialize NVS
    
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
	//globalQue = malloc(sizeof(espnowDataQue));
	globalQue.isReady=0;
	globalQue.previousSend=255;
	globalQue.currentData=0;
	
	printf("\r\nSetting Up WIFI....\r\n");
    initWifi();
	printf("\r\nWIFI Setup Complete!\r\n");
    example_espnow_init();
}