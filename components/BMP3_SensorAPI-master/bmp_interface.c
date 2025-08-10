/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmp3.h"
#include "bmp_interface.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"

/* Variable to store the device address */
static uint8_t dev_addr;
#define I2C_PORT I2C_NUM_1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3


BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return BMP3_OK;
}

BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return BMP3_OK;
}

BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){   
    return -1;
}

BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    return -1;
}
void bmp3_delay_us(uint32_t period, void *intf_ptr){
    esp_rom_delay_us(period);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf){
    int8_t rslt = BMP3_OK;

    if (bmp3 != NULL){
        /* Bus configuration : I2C */
        if (intf == BMP3_I2C_INTF){
            printf("I2C Interface\n");
            dev_addr = BMP3_ADDR_I2C_PRIM;
            bmp3->read = bmp3_i2c_read;
            bmp3->write = bmp3_i2c_write;
            bmp3->intf = BMP3_I2C_INTF;

            // i2c_config_t conf = {
            //     .mode = I2C_MODE_MASTER,
            //     .sda_io_num = I2C_SDA_PIN,
            //     .scl_io_num = I2C_SCL_PIN,
            //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
            //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
            //     .master = {
            //         .clk_speed = 400000 // 100 kHz
            //     },
            // };
            // i2c_param_config(I2C_PORT, &conf);
            // i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
        }
        /* Bus configuration : SPI */
        else if (intf == BMP3_SPI_INTF){
            return -1;
            // printf("SPI Interface - not working\n");
            // dev_addr = 0;
            // bmp3->read = bmp3_spi_read;
            // bmp3->write = bmp3_spi_write;
            // bmp3->intf = BMP3_SPI_INTF;
        }

        bmp3->delay_us = bmp3_delay_us;
        bmp3->intf_ptr = &dev_addr;
    }

    return rslt;
}
