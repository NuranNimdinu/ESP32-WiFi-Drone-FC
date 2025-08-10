#pragma once /// include this library only once in compiling

#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <esp_log.h>
#include "string.h"

#define numOfKeys 15

class NVS_DATA_STORE{
    uint8_t stored_dt_num = 0;
    const char *storeKeys[numOfKeys];
    void *storePointers[numOfKeys];
    size_t storePtrLen[numOfKeys];
    const char *storeName = "ESPD";

public:
    NVS_DATA_STORE(){

    }
    void init(){
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW("NVS", "Erasing NVS partition and re-initializing...");
            err = nvs_flash_erase();
            err = nvs_flash_init();
        }
        if(err != ESP_OK){
            ESP_LOGW("NVS", "NVS flash fail");
            return;
        }
    }
    void recoverData(){
        nvs_handle_t nvs_h;
        esp_err_t err;

        err = nvs_open(this->storeName, NVS_READONLY, &nvs_h);
        if(err != ESP_OK){
            ESP_LOGW("NVS", "ERROR OPENNING DATA %s", this->storeName);
            return;
        }

        for(uint8_t i = 0; i < numOfKeys; i++){
            err = nvs_get_blob(nvs_h, this->storeKeys[i], this->storePointers[i], &this->storePtrLen[i]);
            if(err != ESP_OK){
                ESP_LOGI("NVS", "data not found %s", this->storeKeys[i]);
            }
        }
        nvs_close(nvs_h);
    }
    void* get_data_bytes(const char* key){
        uint8_t ind = 0;
        void* dataptr;
        size_t datasize;
        for(ind = 0; ind <= this->stored_dt_num; ind++){
            if(ind == this->stored_dt_num)break;
            else if(strcmp(this->storeKeys[ind], key) == 0)break;
        }
        if(ind == this->stored_dt_num){
            ESP_LOGI("NVS", "DATA NOT REG BEFORE. REG ");
            return 0;
        }
        dataptr = (void*)this->storePointers[ind];
        datasize = this->storePtrLen[ind] + 4;
        
        void* bytebuf = calloc(sizeof(uint8_t), datasize);
        if(!bytebuf)return 0;

        memcpy((uint8_t*)bytebuf + 4, (uint8_t*)dataptr, (datasize - 4));
        *(uint8_t*)bytebuf = datasize;
            
        // printf("get nvs buf key %s | %p d: %s l: %d\n",key, dataptr, (char*)dataptr, datasize);//////////////
        // printf("get nvs buf key %s | %p d: %s l: %d\n",key, bytebuf, (char*)bytebuf, datasize);//////////////
        return bytebuf;
    }
    void reg_var_nvs(const char* key, void* ptr, size_t size){
        this->storeKeys[this->stored_dt_num] = key;
        this->storePointers[this->stored_dt_num] = ptr;
        this->storePtrLen[this->stored_dt_num] = size;

        ESP_LOGI("NVS_REG", "NVS REG key: %s ptr: %p size: %d", 
            this->storeKeys[this->stored_dt_num], this->storePointers[this->stored_dt_num], this->storePtrLen[this->stored_dt_num]);//////////////

        this->stored_dt_num++;
    }
    void writeToRAM(const char* key, uint8_t* bytes = 0){

        uint8_t elem_index = 0;
        for(elem_index = 0; elem_index <= this->stored_dt_num; elem_index++){
            if(elem_index == this->stored_dt_num)break;
            else if(strcmp(this->storeKeys[elem_index], key) == 0)break;
        }
        if(elem_index == this->stored_dt_num){
            ESP_LOGI("NVS", "DATA NOT REG BEFORE. REG ");
            return;
        }
        memcpy(this->storePointers[elem_index], bytes, this->storePtrLen[elem_index]);
    }

    void writeToNVS(const char* key, uint8_t* bytes = 0){
        nvs_handle_t nvs_h;
        esp_err_t err;

        uint8_t elem_index = 0;
        for(elem_index = 0; elem_index <= this->stored_dt_num; elem_index++){
            if(elem_index == this->stored_dt_num)break;
            else if(strcmp(this->storeKeys[elem_index], key) == 0)break;
        }
        if(elem_index == this->stored_dt_num){
            ESP_LOGI("NVS", "DATA NOT REG BEFORE. REG ");
            return;
        }

        if(bytes){
            memcpy(this->storePointers[elem_index], bytes, this->storePtrLen[elem_index]);
            // if(strcmp("runn", key) == 0)return;
        }

        err = nvs_open(this->storeName, NVS_READWRITE, &nvs_h);
        if(err != ESP_OK){
            ESP_LOGI("NVS", "NVS write open Error");
            return;
        }

        err = nvs_set_blob(nvs_h, key, this->storePointers[elem_index], this->storePtrLen[elem_index]);
        if(err != ESP_OK){
            ESP_LOGI("NVS", "NVS DATA WRITE ERROR %s", key);
        }

        err = nvs_commit(nvs_h);
        if(err != ESP_OK){
            ESP_LOGI("NVS", "NVS DATA COMMIT ERROR %s", key);
        }
        nvs_close(nvs_h);

        // if(strcmp(key, "motc") == 0){
        //     int16_t* values = (int16_t*)this->storePointers[elem_index];
        //     printf("||| motc m1c %d m2c %d \n", values[0], values[1]);
        // }
        // if(strcmp(key, "kpid") == 0)
        //     printf("|||NVS w kpid value %0.2f | %0.2f | %0.2f \n", (*(float*)this->storePointers[elem_index]), (*(float*)(this->storePointers[elem_index] + 4)), (*(float*)(this->storePointers[elem_index] + 8)));

        // ESP_LOGI("NVS_WRITE", "NVS WRITTEN : %s ptr: %p val: %ld", key, this->storePointers[elem_index], *(uint32_t*)this->storePointers[elem_index]);//////////////
    }
};