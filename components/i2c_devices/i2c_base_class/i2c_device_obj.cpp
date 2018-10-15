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
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "iot_i2c_device.h"

CI2CDevice::CI2CDevice(CI2CBus *bus, uint8_t dev_addr,
                       uint32_t ticks_to_wait)
{
    m_i2c_bus_handle = bus;
    m_i2c_device_handle = iot_i2c_device_create(
        m_i2c_bus_handle->get_bus_handle(),
        dev_addr, ticks_to_wait);
}

CI2CDevice::~CI2CDevice()
{
    iot_i2c_device_delete(m_i2c_device_handle);
    m_i2c_device_handle = NULL;
}

esp_err_t CI2CDevice::write(uint8_t start_address, uint8_t write_num,
                            uint8_t *data_buf)
{
    return iot_i2c_device_write(m_i2c_device_handle, start_address, write_num,
                                data_buf);
}

esp_err_t CI2CDevice::read(uint8_t start_address, uint8_t read_num,
                           uint8_t *data_buf)
{
    return iot_i2c_device_read(m_i2c_device_handle, start_address, read_num,
                               data_buf);
}

i2c_bus_handle_t CI2CDevice::get_device_handle()
{
    return m_i2c_device_handle;
}

CI2CBus *CI2CDevice::get_bus()
{
    return m_i2c_bus_handle;
}

uint32_t CI2CDevice::get_timeout()
{
    return iot_i2c_device_get_timeout(m_i2c_device_handle);
}

esp_err_t CI2CDevice::set_timeout(uint32_t timeout_ms)
{
    return iot_i2c_device_set_timeout(m_i2c_device_handle, timeout_ms);
}
