// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <assert.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "iot_i2c_device.h"

static const char* I2C_DEVICE_TAG = "i2c_device";

#define I2C_DEVICE_CHECK(a, str, ret)  if(!(a)) { \
    ESP_LOGE(I2C_DEVICE_TAG,"%s:%d (%s):%s", \
             __FILE__, __LINE__, __FUNCTION__, str); \
    return (ret); \
    }

#define ACK_CHECK_EN   true    /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  false   /*!< I2C master will not check ack from slave */

typedef struct {
    i2c_bus_handle_t bus;
    uint8_t dev_addr;
    uint32_t timeout;
} i2c_device_t;

i2c_device_handle_t iot_i2c_device_create(i2c_bus_handle_t bus,
                                          uint8_t dev_addr,
                                          uint32_t timeout)
{
    I2C_DEVICE_CHECK(bus != NULL, "Bus should not be NULL", NULL);
    I2C_DEVICE_CHECK(timeout > 0, "Timeout should be > 0", NULL);

    i2c_device_t *i2c_dev = (i2c_device_t*) calloc(1, sizeof(i2c_device_t));
    i2c_dev->bus = bus;
    i2c_dev->dev_addr = dev_addr;
    i2c_dev->timeout = timeout;

    return (i2c_device_handle_t) i2c_dev;
}

esp_err_t iot_i2c_device_delete(i2c_device_handle_t device)
{
    i2c_device_t* i2c_dev = (i2c_device_t*) device;
    free(i2c_dev);
    return ESP_OK;
}

static i2c_cmd_handle_t iot_i2c_device_write_cmd(uint8_t dev_addr,
                                                 uint8_t address,
                                                 uint8_t data_len,
                                                 uint8_t *data_buf)
{
    assert((data_len == 0) || (data_buf != NULL));
    assert((address & BIT(7)) == 0);

    // start; write dev_addr + write cmd; write reg_addr;
    // start; write dev_addr + read cmd; write data byte (no ack); stop
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);

    if(data_len > 0)
        i2c_master_write(cmd, data_buf, data_len, ACK_CHECK_EN);

    i2c_master_stop(cmd);

    return cmd;
}

esp_err_t iot_i2c_device_write(i2c_device_handle_t device,
                               uint8_t start_address,
                               uint8_t read_num,
                               uint8_t *data_buf)
{
    i2c_device_t *i2c_dev = (i2c_device_t *) device;

    I2C_DEVICE_CHECK(i2c_dev != NULL, "I2C device should be initialized first",
                     ESP_ERR_INVALID_ARG);
    I2C_DEVICE_CHECK(data_buf != NULL, "data_buf shouldn't be NULL",
                     ESP_ERR_INVALID_ARG);

    // define and execute the i2c command
    i2c_cmd_handle_t cmd = iot_i2c_device_write_cmd(i2c_dev->dev_addr,
                                                    start_address,
                                                    read_num,
                                                    data_buf);

    esp_err_t ret = iot_i2c_bus_cmd_begin(i2c_dev->bus, cmd,
                                          i2c_dev->timeout / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static i2c_cmd_handle_t iot_i2c_device_read_cmd(uint8_t dev_addr,
                                                uint8_t address,
                                                uint8_t data_len,
                                                uint8_t *data)
{
    assert(data != NULL);
    assert(data_len > 0);
    assert((address & BIT(7)) == 0);

    // start; write dev_addr + write cmd; write reg_addr;
    // start; write dev_addr + read cmd; write data byte (no ack); stop
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE,
                          ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    /* read the first data_len - 1 bytes and ACK it */
    if (data_len > 1)
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);

    /* the last byte doesn't need ACK */
    i2c_master_read_byte(cmd, &(data[data_len - 1]), I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    return cmd;
}

esp_err_t iot_i2c_device_read(i2c_device_handle_t device,
                              uint8_t start_address,
                              uint8_t read_num,
                              uint8_t *data_buf)
{
    i2c_device_t* i2c_dev = (i2c_device_t*) device;

    I2C_DEVICE_CHECK(i2c_dev != NULL, "I2C device should be initialized first",
                     ESP_ERR_INVALID_ARG);
    I2C_DEVICE_CHECK(data_buf != NULL, "data_buf shouldn't be NULL",
                     ESP_ERR_INVALID_ARG);

    // define the cmd
    i2c_cmd_handle_t cmd = iot_i2c_device_read_cmd(i2c_dev->dev_addr,
                                                   start_address,
                                                   read_num,
                                                   data_buf);
    esp_err_t ret = iot_i2c_bus_cmd_begin(i2c_dev->bus,
                                          cmd,
                                          i2c_dev->timeout / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

i2c_bus_handle_t iot_i2c_device_get_bus(i2c_device_handle_t dev)
{
    i2c_device_t* device = (i2c_device_t*) dev;

    I2C_DEVICE_CHECK(device != NULL, "I2C device should be initialized first",
                     NULL);
    return device->bus;
}

esp_err_t iot_i2c_device_set_timeout(i2c_device_handle_t dev, uint32_t tout_ms)
{
    i2c_device_t* device = (i2c_device_t*) dev;
    device->timeout = tout_ms;
    return ESP_OK;
}

uint32_t iot_i2c_device_get_timeout(i2c_device_handle_t dev)
{
    i2c_device_t* device = (i2c_device_t*) dev;
    return device->timeout;
}
