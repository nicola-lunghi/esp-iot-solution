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
#ifndef _IOT_I2C_DEVICE_H_
#define _IOT_I2C_DEVICE_H_

#include <stdint.h>
#include "driver/i2c.h"
#include "iot_i2c_bus.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void* i2c_device_handle_t;

/**
 * @brief Create and init I2C device and return a I2C device handle
 *
 * @param bus I2C device handle
 * @param dev_addr address number of the device
 * @param timeout timeout to wait for i2c operation
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
i2c_device_handle_t iot_i2c_device_create(i2c_bus_handle_t bus,
                                          uint8_t dev_addr,
                                          uint32_t timeout);

/**
 * @brief Delete and release the I2C device object
 *
 * @param device I2C device handle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_i2c_device_delete(i2c_device_handle_t device);

/**
 * @brief   Write multiple byte of data to an I2C device
 *
 * @param device I2C device handle
 * @param start_address The first address of the data to be written
 * @param write_num The size of the data to be written
 * @param dat_buf Pointer to write data
 *
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t iot_i2c_device_write(i2c_device_handle_t device,
                               uint8_t start_address,
                               uint8_t write_num,
                               uint8_t *data_buf);

/**
 * @brief   Read multiple byte of data from an I2C device
 *
 * @param device I2C device handle
 * @param start_address The first address of the data to be read
 * @param write_num The size of the data to be read
 * @param dat_buf Pointer to read data
 *
 * @return
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t iot_i2c_device_read(i2c_device_handle_t dev,
                              uint8_t start_address,
                              uint8_t read_num,
                              uint8_t *data_buf);

/**
 * @brief get I2C device bus handle
 *
 * @param device I2C device handle
 *
 * @return
 *    - i2c_bus_handle_t bus object
 */
i2c_bus_handle_t iot_i2c_device_get_bus(i2c_device_handle_t dev);

/**
 * @brief get I2C device timeout
 *
 * @param device I2C device handle
 *
 * @return
 *    - uitn32_t timeout value in ms
 */
uint32_t iot_i2c_device_get_timeout(i2c_device_handle_t dev);

/**
 * @brief set I2C device timeout
 *
 * @param device I2C device handle
 *
 * @return
 *    - uitn32_t timeout value in ms
 */
esp_err_t iot_i2c_device_set_timeout(i2c_device_handle_t dev, uint32_t tout_ms);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * class of I2c Device
 */
class CI2CDevice
{
private:
    i2c_device_handle_t m_i2c_device_handle;
    CI2CBus *m_i2c_bus_handle;

    /**
     * prevent copy constructing
     */
    CI2CDevice(const CI2CDevice&);
    CI2CDevice& operator =(const CI2CDevice&);
public:
    /**
     * @brief Constructor for CI2CDevice class
     * @param bus I2C bus object
     * @param dev_addr I2C device address
     * @param ticks_to_wait tmeout for i2c read/write
     *
     */
    CI2CDevice(CI2CBus *bus, uint8_t dev_addr,
               uint32_t ticks_to_wait = 1000 / portTICK_RATE_MS);

    /**
     * @brief Destructor function of CI2CDevice class
     */
    ~CI2CDevice();

    /**
     * @brief   Write multiple byte of data to an I2C device
     *
     * @param start_address The first address of the data to be written
     * @param write_num The size of the data to be written
     * @param dat_buf Pointer to write data
     *
     * @return
     *    - ESP_OK Success
     *    - ESP_FAIL Fail
     */
    esp_err_t write(uint8_t start_address, uint8_t write_num,
                    uint8_t *data_buf);

    /**
     * @brief   Read multiple byte of data from an I2C device
     *
     * @param start_address The first address of the data to be read
     * @param write_num The size of the data to be read
     * @param dat_buf Pointer to read data
     *
     * @return
     *    - ESP_OK Success
     *    - ESP_FAIL Fail
     */
    esp_err_t read(uint8_t start_address, uint8_t read_num,
                   uint8_t *data_buf);

    /**
     * @brief Get I2C device handle
     * @return I2C device handle
     */
    i2c_device_handle_t get_device_handle();

    /**
     * @brief Get I2C bus handle
     * @return bus handle
     */
    CI2CBus *get_bus();

    uint32_t get_timeout();
    esp_err_t set_timeout(uint32_t timeout_ms);
};
#endif

#endif /* _IOT_I2C_DEVICE_H_ */

