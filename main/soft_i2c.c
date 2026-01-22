/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include "driver/dedic_gpio.h"
#include "driver/gpio.h"
#include "soft_i2c.h"
#include "esp_check.h"
#include "main.h"

#define ERR_CHECK_OR_GOTO(ret, label) do { if((ret) != ESP_OK ) goto label; } while (0)

 //I2C引脚定义
#define SCL_PIN GPIO_NUM_6
#define SDA_PIN GPIO_NUM_7

#define I2C_SCL_MASK    (1 << 0)
#define I2C_SDA_MASK    (1 << 1)
#define I2C_BOTH_MASK   ((I2C_SDA_MASK) | (I2C_SCL_MASK))

#define I2C_SCL_BIT 0
#define I2C_SDA_BIT 1

#define I2C_W 0
#define I2C_R 1

#define I2C_DEVICE_ADDRESS 0x39

static const char* TAG = "I2C";

soft_i2c_master_bus_t APDSL = NULL;
soft_i2c_master_bus_t APDSR = NULL;
soft_i2c_master_bus_t PCF8575 = NULL;

void soft_i2c_task1(void* arg) {
    esp_err_t ret = ESP_OK;

    soft_i2c_master_config_t configL = {
        .scl_pin = SCL_PIN,
        .sda_pin = SDA_PIN,
        .freq = SOFT_I2C_100KHZ
    };

    /* Initialize and configure the software I2C bus */
    ret = soft_i2c_master_new(&configL, &APDSL);
    ESP_LOGI(TAG, "APDSL: %d", ret);

    uint8_t wbuffer[] = { 0 };
    wbuffer[0] = 0x00;
    i2c_write(APDSL, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x02;
    i2c_write(APDSL, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x05;
    i2c_write(APDSL, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x05;
    i2c_write(APDSL, I2C_DEVICE_ADDRESS, 0x8F, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x8F: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t rbuf[1] = { 0 };
    uint8_t pbuffer[2];
    // i2c_read(bus, I2C_DEVICE_ADDRESS, 0x94, rbuffer, sizeof(rbuffer));
    // ESP_LOGI(TAG, "读取数据: %x ：%x %x", 0x94, rbuffer[0], rbuffer[1]);
    while (1) {
        i2c_read(APDSL, I2C_DEVICE_ADDRESS, 0x9C, rbuf, 1);
        grt.proximity[0] = rbuf[0];
        // ESP_LOGI(TAG, "读取左传感器数据: %d", grt.proximity[0]);

        // i2c_read(APDSL, 0x20, 0x00, pbuffer, sizeof(pbuffer));
        // ESP_LOGI(TAG, "读取IO数据: %d,%d", pbuffer[0], pbuffer[1]);
        // for (int i = 0; i < 8; ++i) {
        //     grt.bt_state[i] = (uint8_t)(pbuffer[0] >> i & 0x01);
        // }
        // grt.vacuo_state = (uint8_t)(pbuffer[1] >> 7 & 0x01);
        // // ESP_LOGI("G", "真空%d", grt.vacuo_state);
        // ESP_LOGI(TAG, "读取按钮状态: %d %d %d %d %d %d %d %d", grt.bt_state[0], grt.bt_state[1], grt.bt_state[2], grt.bt_state[3], grt.bt_state[4], grt.bt_state[5], grt.bt_state[6], grt.bt_state[7]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void soft_i2c_task2(void* arg) {
    esp_err_t ret = ESP_OK;

    soft_i2c_master_config_t configR = {
        .scl_pin = GPIO_NUM_15,
        .sda_pin = GPIO_NUM_16,
        .freq = SOFT_I2C_100KHZ
    };

    /* Initialize and configure the software I2C bus */
    ret = soft_i2c_master_new(&configR, &APDSR);
    ESP_LOGI(TAG, "APDSR: %d", ret);

    uint8_t wbuffer[] = { 0 };
    wbuffer[0] = 0x00;
    i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x02;
    i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x05;
    i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x80, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x80: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    wbuffer[0] = 0x05;
    i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x8F, wbuffer, 1);
    ESP_LOGI(TAG, "写入数据地址0x8F: %d", wbuffer[0]);
    vTaskDelay(pdMS_TO_TICKS(50));
    // wbuffer[0] = 0x4C;
    // i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x8F, wbuffer, 1);
    // ESP_LOGI(TAG, "写入数据地址0x8F: %d", wbuffer[0]);
    // vTaskDelay(pdMS_TO_TICKS(50));
    // wbuffer[0] = 0x10;
    // i2c_write(APDSR, I2C_DEVICE_ADDRESS, 0x90, wbuffer, 1);
    // ESP_LOGI(TAG, "写入数据地址0x90: %d", wbuffer[0]);
    // vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t rbuf2[1] = { 0 };
    uint8_t pbuffer[2];

    // i2c_read(bus, I2C_DEVICE_ADDRESS, 0x94, rbuffer, sizeof(rbuffer));
    // ESP_LOGI(TAG, "读取数据: %x ：%x %x", 0x94, rbuffer[0], rbuffer[1]);
    while (1) {
        i2c_read(APDSR, I2C_DEVICE_ADDRESS, 0x9C, rbuf2, 1);
        grt.proximity[1] = rbuf2[0];
        // ESP_LOGI(TAG, "读取右传感器数据: 右:%d", grt.proximity[1]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Forward declaration of static functions */
static uint32_t freq_to_delay(soft_i2c_master_freq_t freq);
static esp_err_t emulate_i2c_transfer(uint8_t device_address,
    const uint8_t* write_buffer, uint32_t write_size,
    uint8_t* read_buffer, uint32_t read_size,
    soft_i2c_master_bus_t bus);

/* Mutex required to enter critical sections */
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

const char* __attribute__((used)) SOFT_I2C_MASTER_TAG = "soft_i2c_master";

/***** Public API implementation *****/
struct i2c_master_bus_impl_t {
    uint32_t scl_pin;
    uint32_t sda_pin;

    uint32_t freq_delay;
    dedic_gpio_bundle_handle_t bundle;
};

esp_err_t soft_i2c_master_new(soft_i2c_master_config_t* config, soft_i2c_master_bus_t* bus) {
    esp_err_t ret;
    struct i2c_master_bus_impl_t* bus_impl = NULL;

    /* Check the parameters */
    ESP_GOTO_ON_FALSE(config != NULL && bus != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG,
        "Parameters must not be NULL");
    ESP_GOTO_ON_FALSE(config->freq < SOFT_I2C_FREQ_END, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG,
        "Invalid frequency");

    /* Emulate I2C bus thanks to fast GPIOs. Both SCL and SDA are bi-directional pins and in open-drain mode.
     * Using internal pull-ups will let SCL be clocked at around 100KHz. Using external pull-ups
     * will let the clock be faster. */
    const uint32_t scl = config->scl_pin;
    const uint32_t sda = config->sda_pin;
    ret = gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    ERR_CHECK_OR_GOTO(ret, error);
    ret = gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    ERR_CHECK_OR_GOTO(ret, error);
    ret = gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    ERR_CHECK_OR_GOTO(ret, error);
    ret = gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    ERR_CHECK_OR_GOTO(ret, error);

    /* Configure both I/Os as fast GPIOs */
    int bidir_array[] = { scl, sda };
    dedic_gpio_bundle_config_t bidir_config = {
        .gpio_array = bidir_array,
        .array_size = 2,
        .flags = {
            .out_en = 1,
            .in_en = 1
        }
    };

    /* Allocate the master bus structure now that we need it */
    bus_impl = malloc(sizeof(struct i2c_master_bus_impl_t));
    ESP_GOTO_ON_FALSE(bus_impl != NULL, ESP_ERR_NO_MEM, error, SOFT_I2C_MASTER_TAG, "No more memory available in the system");

    /* Initialize the dedicated GPIO bundle and fill the bus structure */
    ret = dedic_gpio_new_bundle(&bidir_config, &bus_impl->bundle);
    ERR_CHECK_OR_GOTO(ret, error);
    bus_impl->scl_pin = config->scl_pin;
    bus_impl->sda_pin = config->sda_pin;
    bus_impl->freq_delay = freq_to_delay(config->freq);

    *bus = bus_impl;

    /* Set both lines to high-impedance, which is I2C idle state */
    dedic_gpio_bundle_write(bus_impl->bundle, 3, 3);

    return ret;
error:
    if (bus_impl != NULL) {
        free(bus_impl);
    }
    return ret;
}

esp_err_t soft_i2c_master_del(soft_i2c_master_bus_t bus) {
    esp_err_t ret;
    ESP_GOTO_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG,
        "Bus must not be NULL");

    dedic_gpio_del_bundle(bus->bundle);

    free(bus);
error:
    return ret;
}

esp_err_t soft_i2c_master_write(soft_i2c_master_bus_t bus,
    uint8_t device_address,
    const uint8_t* write_buffer, size_t write_size) {
    esp_err_t ret;
    ESP_GOTO_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Bus must not be NULL");
    ESP_GOTO_ON_FALSE(device_address < 0x80, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Invalid device address");
    ESP_GOTO_ON_FALSE(write_buffer != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Buffer must not be NULL");
    ESP_GOTO_ON_FALSE(write_size != 0, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Buffer size must not be 0");

    portENTER_CRITICAL(&g_lock);
    ret = emulate_i2c_transfer(device_address,
        write_buffer, write_size,
        NULL, 0,
        bus);
    portEXIT_CRITICAL(&g_lock);

error:
    return ret;
}

esp_err_t soft_i2c_master_read(soft_i2c_master_bus_t bus,
    uint8_t device_address,
    uint8_t* read_buffer, size_t read_size) {
    esp_err_t ret;
    ESP_GOTO_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Bus must not be NULL");
    ESP_GOTO_ON_FALSE(device_address < 0x80, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Invalid device address");
    ESP_GOTO_ON_FALSE(read_buffer != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Buffer must not be NULL");

    portENTER_CRITICAL(&g_lock);
    ret = emulate_i2c_transfer(device_address,
        NULL, 0,
        read_buffer, read_size,
        bus);
    portEXIT_CRITICAL(&g_lock);

error:
    return ret;
}

esp_err_t soft_i2c_master_write_read(soft_i2c_master_bus_t bus,
    uint8_t device_address,
    const uint8_t* write_buffer, size_t write_size,
    uint8_t* read_buffer, size_t read_size) {
    esp_err_t ret;
    ESP_GOTO_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Bus must not be NULL");
    ESP_GOTO_ON_FALSE(device_address < 0x80, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG, "Invalid device address");
    ESP_GOTO_ON_FALSE(write_buffer != NULL && read_buffer != NULL, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG,
        "Buffers must not be NULL");
    ESP_GOTO_ON_FALSE(write_size != 0 && read_size != 0, ESP_ERR_INVALID_ARG, error, SOFT_I2C_MASTER_TAG,
        "Buffers sizes must not be 0");

    portENTER_CRITICAL(&g_lock);
    ret = emulate_i2c_transfer(device_address,
        write_buffer, write_size,
        read_buffer, read_size,
        bus);
    portEXIT_CRITICAL(&g_lock);

error:
    return ret;
}

/***** Private implementation *****/

static uint32_t freq_to_delay(soft_i2c_master_freq_t freq) {
    switch (freq) {
    case SOFT_I2C_100KHZ: return 3;
    case SOFT_I2C_200KHZ: return 2;
    case SOFT_I2C_300KHZ: return 1;
    default:
        assert(false);
        return 0;
    }
}

static inline void set_scl(dedic_gpio_bundle_handle_t bidir_bundle, int value, uint32_t delay) {
    dedic_gpio_bundle_write(bidir_bundle, I2C_SCL_MASK, value << I2C_SCL_BIT);
    esp_rom_delay_us(delay);
}

static inline void set_sda(dedic_gpio_bundle_handle_t bidir_bundle, int value, uint32_t delay) {
    dedic_gpio_bundle_write(bidir_bundle, I2C_SDA_MASK, value << I2C_SDA_BIT);
    esp_rom_delay_us(delay);
}

static inline int get_sda(dedic_gpio_bundle_handle_t bidir_bundle) {
    uint32_t offset = 0;
    /* We need the offset of the SDA pin in the bundle */
    dedic_gpio_get_in_offset(bidir_bundle, &offset);
    uint32_t in = dedic_gpio_bundle_read_in(bidir_bundle);
    return in >> (offset + I2C_SDA_BIT);
}

static inline void emulate_start(soft_i2c_master_bus_t bus) {
    dedic_gpio_bundle_handle_t bidir_bundle = bus->bundle;
    const uint32_t delay = bus->freq_delay;

    /* A Start consists in pulling SDA low when SCL is high. */
    set_scl(bidir_bundle, 1, delay);
    /* If SDA is low, pull it high first */
    set_sda(bidir_bundle, 1, delay);
    set_sda(bidir_bundle, 0, delay);
}

static inline void emulate_stop(soft_i2c_master_bus_t bus) {
    dedic_gpio_bundle_handle_t bidir_bundle = bus->bundle;
    const uint32_t delay = bus->freq_delay;

    /* Stop consists in pulling SDA high while SCL is also high. First, pull both low, make sure
     * it is not detected as a "start" so let's do it one by one. */
    set_scl(bidir_bundle, 0, delay);
    set_sda(bidir_bundle, 0, delay);
    /* Pull SCL high */
    set_scl(bidir_bundle, 1, delay);
    /* Pull SDA high */
    set_sda(bidir_bundle, 1, delay);
}

static inline int emulate_write_byte(soft_i2c_master_bus_t bus, uint_fast8_t byte) {
    dedic_gpio_bundle_handle_t bidir_bundle = bus->bundle;
    const uint32_t delay = bus->freq_delay;

    for (int i = 7; i >= 0; i--) {
        const uint_fast8_t bit = (byte >> i) & 1;
        /* Set SCL to low */
        set_scl(bidir_bundle, 0, delay);
        /* Set SDA value now */
        set_sda(bidir_bundle, bit, delay);
        /* Set SCL to high */
        set_scl(bidir_bundle, 1, delay);
    }

    /* Send one more bit to get the ACK/NACK */
    set_scl(bidir_bundle, 0, delay);
    set_sda(bidir_bundle, 1, delay);
    set_scl(bidir_bundle, 1, delay);

    /* Get the SDA bit */
    int ret = get_sda(bidir_bundle) == 0;

    /* Pull clock low, to tell the device to stop pulling SDA down */
    set_scl(bidir_bundle, 0, delay);
    return ret;
}

static inline uint8_t emulate_read_byte(soft_i2c_master_bus_t bus, int send_ack) {
    uint8_t result = 0;
    uint32_t in = 0;
    dedic_gpio_bundle_handle_t bidir_bundle = bus->bundle;
    const uint32_t delay = bus->freq_delay;

    /* Set SCL to low as we are going to put SDA in high-impedance */
    set_scl(bidir_bundle, 0, delay);
    set_sda(bidir_bundle, 1, delay);

    for (int i = 7; i >= 0; i--) {
        /* Set SCL to low */
        set_scl(bidir_bundle, 0, delay);
        /* Get SDA value now and store it in the final result */
        in = get_sda(bidir_bundle);
        result = (result << 1) | in;
        /* Set SCL to high */
        set_scl(bidir_bundle, 1, delay);
    }

    /* Send one more bit to set ACK/NACK */
    set_scl(bidir_bundle, 0, delay);
    set_sda(bidir_bundle, !send_ack, delay);
    set_scl(bidir_bundle, 1, delay);

    return result;
}

static esp_err_t emulate_i2c_transfer(uint8_t device_address,
    const uint8_t* write_buffer, uint32_t write_size,
    uint8_t* read_buffer, uint32_t read_size,
    soft_i2c_master_bus_t bus) {
    esp_err_t ret = ESP_OK;
    dedic_gpio_bundle_handle_t bidir_bundle = bus->bundle;

    /* Set both pins to high to start */
    dedic_gpio_bundle_write(bidir_bundle, I2C_BOTH_MASK, 0x3);

    /* Perform a start/write on the bus */
    if (write_buffer != NULL && write_size != 0) {
        emulate_start(bus);
        int ack = emulate_write_byte(bus, device_address << 1);
        if (!ack) {
            ret = ESP_ERR_NOT_FOUND;
        }

        for (int i = 0; i < write_size && ret == ESP_OK; i++) {
            /* Check the ACK returned by the device */
            ack = emulate_write_byte(bus, write_buffer[i]);
            if (!ack) {
                ret = ESP_FAIL;
            }
        }
    }

    /* Perform a (re)start/read on the bus */
    if (ret == ESP_OK && read_buffer != NULL && read_size != 0) {
        emulate_start(bus);
        int ack = emulate_write_byte(bus, ((device_address << 1) | 1));
        if (!ack) {
            ret = ESP_ERR_NOT_FOUND;
        }
        else {
            for (int i = 0; i < read_size; i++) {
                /* We must send an ACK after each byte read, except the last one */
                const int send_ack = i != (read_size - 1);
                read_buffer[i] = emulate_read_byte(bus, send_ack);
            }
        }
    }

    emulate_stop(bus);

    return ret;
}

int8_t i2c_read(soft_i2c_master_bus_t bus, uint8_t dev_addr, uint8_t _usAddress, uint8_t* _pReadBuf, uint16_t _usSize) {
    uint16_t i;
    /* 第1步：发起I2C总线启动信号 */
    emulate_start(bus);

    /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    emulate_write_byte(bus, (dev_addr << 1) | I2C_W);

    emulate_write_byte(bus, (uint8_t)_usAddress);

    /* 第6步：重新启动I2C总线。下面开始读取数据 */
    emulate_start(bus);

    /* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    emulate_write_byte(bus, (dev_addr << 1) | I2C_R);

    for (int i = 0; i < _usSize; i++) {
        /* We must send an ACK after each byte read, except the last one */
        const int send_ack = i != (_usSize - 1);
        _pReadBuf[i] = emulate_read_byte(bus, send_ack);
    }
    emulate_stop(bus);
    return 1;
}

int8_t i2c_write(soft_i2c_master_bus_t bus, uint8_t dev_addr, uint8_t _usAddress, uint8_t* _pWriteBuf, uint16_t _usSize) {
    uint16_t i;
    /* 第1步：发起I2C总线启动信号 */
    emulate_start(bus);

    /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
    emulate_write_byte(bus, (dev_addr << 1) | I2C_W);

    emulate_write_byte(bus, (uint8_t)_usAddress);

    for (i = 0; i < _usSize; i++) {
        emulate_write_byte(bus, _pWriteBuf[i]);
    }
    emulate_stop(bus);
    return 1;
}

// int8_t ium_i2c_write(uint8_t dev_addr,uint16_t _usAddress,uint8_t *_pWriteBuf,uint16_t _usSize)
// {
//     uint16_t i,m;
//     uint16_t usAddr;
//     /* 第1步：发起I2C总线启动信号 */
//     i2c_Start();

//     /* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
//     i2c_SendByte((dev_addr<<1) | I2C_WR); /* 此处是写指令 */

//     /* 第3步：等待ACK */
//     if (i2c_WaitAck() != 0)
//     {
//         goto cmd_fail;  /* EEPROM器件无应答 */
//     }
//     /* 第4步：发送字节地址*/
//     i2c_SendByte((uint8_t)_usAddress);
//     /* 第5步：等待ACK */
//     if (i2c_WaitAck() != 0)
//     {
//         goto cmd_fail;  /* EEPROM器件无应答 */
//     }
//     for(i=0;i<_usSize;i++)
//     {
//         /* 第6步：开始写入数据 */
//         i2c_SendByte(_pWriteBuf[i]);

//         /* 第7步：发送ACK */
//         if (i2c_WaitAck() != 0)
//         {
//             goto cmd_fail;  /* EEPROM器件无应答 */
//         }
//     }
//     /* 命令执行成功，发送I2C总线停止信号 */
//     i2c_Stop();
//     return BMI2_INTF_RET_SUCCESS;   /* 执行成功 */

// cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
//     /* 发送I2C总线停止信号 */
//     i2c_Stop();
//     return BMI2_E_COM_FAIL;
// }