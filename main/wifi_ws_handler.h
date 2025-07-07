#pragma once /// include this library only once in compiling

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "mdns.h"
#include "nvs_flash.h"

#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_data_store.h"

#define ROBOT_WIFI_SSID "WiFi__5G"
#define ROBOT_WIFI_PSWD "#passwifi#"
#define ROBOT_WIFI_RETRY 10
#define ROBOT_MDNS_NAME "drone"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
    
#define ESP_WS_OTA_BUFF_SIZE 1024
#define ESP_WS_OTA_BUFF_SIZE_B 1024*16

static const char *WTAG = "wifi station";

class ESP_WIFI_DEV{
    static EventGroupHandle_t s_wifi_event_group;
    static NVS_DATA_STORE* nvsDataStorage;
public:
    static bool WIFI_CONNECTED_SUC;
    static bool WS_CLIENT_CNT;

    ESP_WIFI_DEV(NVS_DATA_STORE* _nvsDataStorage){
        ESP_WIFI_DEV::nvsDataStorage = _nvsDataStorage;
    }
    bool init(){
        //initilize nvs
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_LOGI(WTAG, "ESP_WIFI_MODE_STA");

        this->init_wifi_sta();
        if(ESP_WIFI_DEV::WIFI_CONNECTED_SUC){
            this->start_ws_server();
            this->init_mdns_service();
        }
        return ESP_WIFI_DEV::WIFI_CONNECTED_SUC;
    }

    template<typename Text, typename... TextArg>
    static esp_err_t ws_dt_Str_send(const char* type, Text str, TextArg... strag){
        if(!ESP_WIFI_DEV::WIFI_CONNECTED_SUC || ESP_WIFI_DEV::ws_connected_client_sockfd < 0) return ESP_FAIL;

        // httpd_ws_client_info_t cl = httpd_ws_get_fd_info(ESP_WIFI_DEV::ws_connected_client_h, ESP_WIFI_DEV::ws_connected_client_sockfd);
        // printf("http client is : %s", (cl == 0) ? "HTTPD_WS_CLIENT_INVALID" 
        //                             : (cl == 1) ? "HTTPD_WS_CLIENT_HTTP" : "HTTPD_WS_CLIENT_WEBSOCKET");
        // if(cl == 0)return ESP_FAIL;

        char tempStrDt[128];
        memcpy(tempStrDt, (char*)type, 3);
        // snprintf(tempStrDt + 3, 32, str, strag...);
        snprintf(tempStrDt + 3, sizeof(tempStrDt) - 3, str, strag...);

        // ESP_LOGI("WSDT", "%s", tempStrDt); ///////

        httpd_ws_frame_t ws_send_pkt;
        memset(&ws_send_pkt, 0, sizeof(ws_send_pkt));

        ws_send_pkt.type = HTTPD_WS_TYPE_TEXT;
        ws_send_pkt.payload = (uint8_t*)tempStrDt;
        ws_send_pkt.len = strlen(tempStrDt);

        return httpd_ws_send_frame_async(ESP_WIFI_DEV::ws_connected_client_h, ESP_WIFI_DEV::ws_connected_client_sockfd, &ws_send_pkt);
        // esp_err_t err = httpd_ws_send_data(hd, fd, &ws_send_pkt);
    }
    static void ws_dt_bin_send(uint8_t *data, const size_t len, const size_t chunk = 32*1024){
        if(!ESP_WIFI_DEV::WIFI_CONNECTED_SUC || ESP_WIFI_DEV::ws_connected_client_sockfd < 0) return;
        size_t sendSize = 0;
        size_t dtChunkS = chunk;
        bool finBuf = false;

        while(sendSize < len){
            if(len - sendSize > chunk) {
                dtChunkS = chunk;
            }else {
                dtChunkS = len - sendSize;
                finBuf = true;
            }
            // buffer = realloc(buffer, sizeof(buffer) + 4);
            // printf("get reall buf %p d: %s l: %d\n", buffer, (char*)buffer, sizeof(buffer));

            // memcpy((uint8_t*)buffer, DatakeysForWSInit[i], 4);
            // printf("get fin buf %p d: %s l: %d\n", buffer, (char*)buffer, sizeofbuffer);/////////////

            httpd_ws_frame_t ws_send_pkt;
            memset(&ws_send_pkt, 0, sizeof(ws_send_pkt));

            // ws_send_pkt.fragmented = true;
            // ws_send_pkt.final = finBuf;
            ws_send_pkt.type = HTTPD_WS_TYPE_BINARY;
            ws_send_pkt.payload = data + sendSize;
            ws_send_pkt.len = dtChunkS;

            httpd_ws_send_frame_async(ESP_WIFI_DEV::ws_connected_client_h, ESP_WIFI_DEV::ws_connected_client_sockfd, &ws_send_pkt);

            sendSize += dtChunkS;
        }
    }

    static void send_init_dt_ws(){
        if(!ESP_WIFI_DEV::WIFI_CONNECTED_SUC || ESP_WIFI_DEV::ws_connected_client_sockfd < 0) return;
        // return;
        
        const char* DatakeysForWSInit[] = {"motm", "motc", "Rpid", "Ppid", "Ypid", "ARid"};
        uint8_t numberofdata = sizeof(DatakeysForWSInit) / 4;

        for(uint8_t i = 0; i < numberofdata; i++){
            void* buffer = ESP_WIFI_DEV::nvsDataStorage->get_data_bytes(DatakeysForWSInit[i]);
            if(buffer == 0)return;

            size_t sizeofbuffer = *(uint8_t*)buffer;

            // buffer = realloc(buffer, sizeof(buffer) + 4);
            // printf("get reall buf %p d: %s l: %d\n", buffer, (char*)buffer, sizeof(buffer));

            memcpy((uint8_t*)buffer, DatakeysForWSInit[i], 4);
            // printf("get fin buf %p d: %s l: %d\n", buffer, (char*)buffer, sizeofbuffer);/////////////

            httpd_ws_frame_t ws_send_pkt;
            memset(&ws_send_pkt, 0, sizeof(ws_send_pkt));

            ws_send_pkt.type = HTTPD_WS_TYPE_BINARY;
            ws_send_pkt.payload = (uint8_t*)buffer;
            ws_send_pkt.len = sizeofbuffer;

            httpd_ws_send_frame_async(ESP_WIFI_DEV::ws_connected_client_h, ESP_WIFI_DEV::ws_connected_client_sockfd, &ws_send_pkt);
            free(buffer);
        }
    }
    
    static uint32_t ota_update_progress[3];
private:
    static esp_ota_handle_t esp_ota_h;
    static const esp_partition_t *esp_ota_partition;
    static bool esp_ota_in_prog;
    static httpd_handle_t ws_connected_client_h;
    static int ws_connected_client_sockfd;
    static bool ws_data_buffer_bin;

    void init_mdns_service(){
        esp_err_t err;

        err = mdns_init();
        if(err != ESP_OK){
            ESP_LOGW("MDNS", "mdns server failed %d", err);
            return;
        }

        mdns_hostname_set(ROBOT_MDNS_NAME);

        mdns_service_add(NULL, "_http", "_tcp", 81, NULL, 0);
        mdns_service_add(NULL, "_ws", "_tcp", 81, NULL, 0);

        ESP_LOGI("MDNS", "Mdns hostname : %s", ROBOT_MDNS_NAME);
    }
    void start_ws_server(){
        httpd_handle_t server = NULL;
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.max_uri_handlers = 3;
        config.server_port = 81;

        ESP_LOGI(WTAG, "Starting WS server...");
        if(httpd_start(&server, &config) == ESP_OK){
            httpd_uri_t ws = {
                .uri = "/",
                .method = HTTP_GET,
                .handler = ESP_WIFI_DEV::ws_data_handler,
                .user_ctx = NULL,
                .is_websocket = true,
            };

            httpd_register_uri_handler(server, &ws);

            ESP_LOGI(WTAG, "Websocket server Started");
        } else {
            ESP_LOGI(WTAG, "Fail to start WS");
        }
    }
    
    static esp_err_t ws_TEXT_handler(httpd_ws_frame_t *ws_pkt){
        char ws_dt_type[5];
        strlcpy(ws_dt_type, (char*)ws_pkt->payload, 5);

        // printf("wsRx: %s", (char*)ws_pkt->payload);

        if(strcmp(ws_dt_type, "ota0") == 0)
            ESP_WIFI_DEV::OTA_update_handler(ws_pkt, 1);
        else if(strcmp(ws_dt_type, "ota1") == 0)
            ESP_WIFI_DEV::OTA_update_handler(ws_pkt);
        else if(strcmp(ws_dt_type, "rest") == 0)
            esp_restart();
        else if(strcmp(ws_dt_type, "data") == 0){
            strlcpy(ws_dt_type, (char*)(ws_pkt->payload + 4), 5);
            ESP_LOGI("DTDT", "setting var : %s", ws_dt_type); /////////////
            if(strcmp(ws_dt_type, "runn") == 0){
                running_task = *(uint8_t*)(ws_pkt->payload + 8);
            } else if(strcmp(ws_dt_type, "Apid") == 0){
                ESP_WIFI_DEV::nvsDataStorage->writeToNVS("ARid", (ws_pkt->payload + 8));
                ESP_WIFI_DEV::nvsDataStorage->writeToNVS("APid", (ws_pkt->payload + 8));
            } else
                ESP_WIFI_DEV::nvsDataStorage->writeToNVS(ws_dt_type, (ws_pkt->payload + 8));
        }
        else if(strcmp(ws_dt_type, "swit") == 0){
            // set_led_flash();
        }
        else if(strcmp(ws_dt_type, "driv") == 0){
            ws_ctrl_data(ws_pkt->payload + 4);
            // int16_t xxx = ws_pkt->payload[5] << 8 | ws_pkt->payload[4] << 0;
            // int16_t ati = ws_pkt->payload[7] << 8 | ws_pkt->payload[6] << 0;
            // set_mot_spd(xxx, ati);
        }
        return ESP_OK;
    }

    static esp_err_t ws_data_handler(httpd_req_t *req){
        if(req->method == HTTP_GET){
            ESP_WIFI_DEV::ws_connected_client_h = req->handle;
            ESP_WIFI_DEV::ws_connected_client_sockfd = httpd_req_to_sockfd(req);
            return ESP_WIFI_DEV::ws_open_handler();
        }

        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(ws_pkt));

        size_t buffer_len = (ESP_WIFI_DEV::ws_data_buffer_bin) ? ESP_WS_OTA_BUFF_SIZE_B + 4 : ESP_WS_OTA_BUFF_SIZE + 4;
    
        void *buffer = malloc(buffer_len);
        if(!buffer){
            ESP_LOGE(WTAG, "Buffer alloc fail");
            return ESP_FAIL;
        }

        ws_pkt.payload = (uint8_t*)buffer;
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, buffer_len);

        if (ret != ESP_OK) {
            ESP_LOGE(WTAG, "WebSocket frame receive failed: %s", esp_err_to_name(ret));
            free(buffer);
            return ret;
        }

        // ESP_LOGI(WTAG, "Received WebSocket frame, type: %d, length: %d", ws_pkt.type, ws_pkt.len);

        if(ws_pkt.type == HTTPD_WS_TYPE_TEXT){
            ESP_WIFI_DEV::ws_TEXT_handler(&ws_pkt);
        }else if(ws_pkt.type == HTTPD_WS_TYPE_BINARY){
            ESP_WIFI_DEV::ws_TEXT_handler(&ws_pkt);
        }else {
            ESP_LOGW(WTAG, "unknown ws type: %d", ws_pkt.type);
        }

        free(buffer);
        return ESP_OK;
    }
    static esp_err_t OTA_update_handler(httpd_ws_frame_t* ws_pkt, bool ota_init = 0){
        if(ota_init){
            //stop motors for update
            ESP_WIFI_DEV::nvsDataStorage->writeToNVS("runn", 0);

            ESP_WIFI_DEV::ws_dt_Str_send("OTA", "PUPDATE STARTING..");
            if(ESP_WIFI_DEV::esp_ota_in_prog){
                ESP_LOGI("OTA", "Already on going update.. deleteing");
                esp_ota_abort(ESP_WIFI_DEV::esp_ota_h);
            }

            ESP_LOGI("OTA", "Staring OTA update...");

            ESP_WIFI_DEV::esp_ota_partition = esp_ota_get_next_update_partition(NULL);
            if(!ESP_WIFI_DEV::esp_ota_partition){
                ESP_LOGE("OTA", "Fail to get OTA partition");
                return ESP_FAIL;
            }
            ESP_LOGI("OTA", "OTA partition found: %s", ESP_WIFI_DEV::esp_ota_partition->label);

            esp_err_t err = esp_ota_begin(ESP_WIFI_DEV::esp_ota_partition, OTA_SIZE_UNKNOWN, &ESP_WIFI_DEV::esp_ota_h);
            if(err != ESP_OK){
                ESP_LOGE("OTA", "Failed to start OTA: %d", err);
                return err;
            }

            // ESP_WIFI_DEV::ota_update_progress[1] = strtoul((char*)(ws_pkt->payload + 3), (char**)"|", 10);
            memcpy(&ESP_WIFI_DEV::ota_update_progress[1], (ws_pkt->payload + 4), 4);
            ESP_WIFI_DEV::ws_dt_Str_send("DEB", "ota update size : %ld", ESP_WIFI_DEV::ota_update_progress[1]);

            ESP_WIFI_DEV::ws_data_buffer_bin = true;
            ESP_WIFI_DEV::esp_ota_in_prog = true;
            ESP_WIFI_DEV::ws_dt_Str_send("OTA", "wschunk");
        }
        else{
            if(ESP_WIFI_DEV::esp_ota_in_prog && ws_pkt->len > 0){
                esp_err_t err = esp_ota_write(ESP_WIFI_DEV::esp_ota_h, ws_pkt->payload + 4, ws_pkt->len -4);
                if(err != ESP_OK){
                    ESP_LOGE("OTA", "OTA write failed: %d", err);
                    return err;
                }

                ESP_WIFI_DEV::ota_update_progress[0] += ws_pkt->len -4;
                ESP_WIFI_DEV::ota_update_progress[2] = (uint8_t)(ESP_WIFI_DEV::ota_update_progress[0] * 100 / ESP_WIFI_DEV::ota_update_progress[1]);

                if(ws_pkt->len -4 < ESP_WS_OTA_BUFF_SIZE_B){
                    ESP_LOGI("OTA", "Finalizing OTA update...");
                    ESP_WIFI_DEV::ota_update_progress[2] = 100;
                    ESP_WIFI_DEV::ws_data_buffer_bin = false;

                    err = esp_ota_end(ESP_WIFI_DEV::esp_ota_h);
                    if(err == ESP_OK){
                        err = esp_ota_set_boot_partition(ESP_WIFI_DEV::esp_ota_partition);
                        if(err == ESP_OK){
                            ESP_LOGI("OTA", "OTA update complete, rebooting..");
                            ESP_WIFI_DEV::ws_dt_Str_send("OTA", "PUPDATE COMPLETE!");
                            esp_restart();
                        }else{
                            ESP_LOGE("OTA", "Failed to set boot partiton");
                        }
                    }else{
                        ESP_LOGE("OTA", "OTA update failed");
                    }
                    ESP_WIFI_DEV::esp_ota_in_prog = false;
                }
                ESP_WIFI_DEV::ws_dt_Str_send("OTA", "wschunk");
                ESP_WIFI_DEV::ws_dt_Str_send("OTA", "P%d", ESP_WIFI_DEV::ota_update_progress[2]);
            }
        }
        return ESP_OK;
    }

    static esp_err_t ws_open_handler(){
        ESP_LOGI(WTAG, "Websocket Opened");
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_WIFI_DEV::ws_dt_Str_send("CMD", "CONNECTED!");
        ESP_WIFI_DEV::send_init_dt_ws();
        ESP_WIFI_DEV::WS_CLIENT_CNT = true;
        return ESP_OK;
    }

    static uint8_t wifi_retry_num;

    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
        if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
        else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
            if(ESP_WIFI_DEV::wifi_retry_num < ROBOT_WIFI_RETRY){
                esp_wifi_connect();
                ESP_WIFI_DEV::wifi_retry_num++;
                ESP_LOGI(WTAG, "retry to connect to the AP");
            } else {
                xEventGroupSetBits(ESP_WIFI_DEV::s_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(WTAG,"connect to the AP fail");
        }
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(WTAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            ESP_WIFI_DEV::wifi_retry_num = 0;
            xEventGroupSetBits(ESP_WIFI_DEV::s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
    void init_wifi_sta(){
        ESP_WIFI_DEV::s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ESP_WIFI_DEV::wifi_event_handler, NULL, &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ESP_WIFI_DEV::wifi_event_handler, NULL, &instance_got_ip));

        wifi_config_t wifi_conf = {
            .sta = {
                .ssid = ROBOT_WIFI_SSID,
                .password = ROBOT_WIFI_PSWD,
                .threshold = {
                  .authmode = WIFI_AUTH_WPA2_PSK,
                },
            },
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_conf));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(WTAG, "wifi_init_sta finished.");

        EventBits_t wbits = xEventGroupWaitBits(ESP_WIFI_DEV::s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if(wbits & WIFI_CONNECTED_BIT){
            ESP_WIFI_DEV::WIFI_CONNECTED_SUC = true;
            ESP_LOGI(WTAG, "connected to ap SSID:%s password:%s", ROBOT_WIFI_SSID, ROBOT_WIFI_PSWD);
        }else if (wbits & WIFI_FAIL_BIT) {
            ESP_WIFI_DEV::WIFI_CONNECTED_SUC = false;
            ESP_LOGI(WTAG, "!!!Failed to connect to SSID:%s, password:%s", ROBOT_WIFI_SSID, ROBOT_WIFI_PSWD);
        } else {
            ESP_WIFI_DEV::WIFI_CONNECTED_SUC = false;
            ESP_LOGE(WTAG, "UNEXPECTED EVENT");
        }

        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
        vEventGroupDelete(ESP_WIFI_DEV::s_wifi_event_group);

        if(!ESP_WIFI_DEV::WIFI_CONNECTED_SUC){
            esp_wifi_deinit();
            esp_netif_deinit();
        }
    }
};

bool ESP_WIFI_DEV::WIFI_CONNECTED_SUC = false;
bool ESP_WIFI_DEV::WS_CLIENT_CNT = false;
uint8_t ESP_WIFI_DEV::wifi_retry_num = 0;
EventGroupHandle_t ESP_WIFI_DEV::s_wifi_event_group;

esp_ota_handle_t ESP_WIFI_DEV::esp_ota_h = 0;
const esp_partition_t *ESP_WIFI_DEV::esp_ota_partition = NULL;
bool ESP_WIFI_DEV::esp_ota_in_prog = false;
uint32_t ESP_WIFI_DEV::ota_update_progress[3];
httpd_handle_t ESP_WIFI_DEV::ws_connected_client_h;
int ESP_WIFI_DEV::ws_connected_client_sockfd = -1;
bool ESP_WIFI_DEV::ws_data_buffer_bin = false;
NVS_DATA_STORE* ESP_WIFI_DEV::nvsDataStorage;