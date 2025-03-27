#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"

// I2C definitions
#define I2C_MASTER_SCL_IO 3         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 2         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need TX buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need RX buffer */
#define I2C_TIMEOUT_MS 1000

// I2C device address (7-bit format for SA0 low)
#define LIS3DH_I2C_ADD_L 0x18U

// LIS3DH registers
#define WHO_AM_I_REG 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28

// Conversion factors for high-resolution mode (±2g)
// Sensitivity is 1 mg/LSB => 0.001 g/LSB, and 1 g = 9.81 m/s².
#define SENSITIVITY_G 0.001f
#define GRAVITY 9.81f

/**
 * @brief Initialize I2C master.
 */
void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief I2C read function used by the sensor driver.
 *
 * @param handle Pointer to the device address (uint8_t*)
 * @param reg Starting register address to read from.
 * @param bufp Pointer to the data buffer.
 * @param len Number of bytes to read.
 * @return int32_t ESP_OK if successful, otherwise error code.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    uint8_t dev_addr = *((uint8_t *)handle);

    // Write the register address to the device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Read the data from the device
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, bufp, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, bufp + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief I2C write function used by the sensor driver.
 *
 * @param handle Pointer to the device address (uint8_t*)
 * @param reg Register address to write to.
 * @param bufp Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return int32_t ESP_OK if successful, otherwise error code.
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    esp_err_t ret;
    uint8_t dev_addr = *((uint8_t *)handle);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, (uint8_t *)bufp, len, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return ret;
}

void app_main(void)
{
    // Initialize I2C
    i2c_master_init();

    uint8_t lis3dh_addr = LIS3DH_I2C_ADD_L;

    // Verify communication with the sensor via WHO_AM_I register (0x0F)
    uint8_t who_am_i = 0;
    if (platform_read(&lis3dh_addr, WHO_AM_I_REG, &who_am_i, 1) == ESP_OK)
    {
        printf("WHO_AM_I register: 0x%02X\n", who_am_i);
    }
    else
    {
        printf("Error reading sensor WHO_AM_I register\n");
        return;
    }

    // Configure CTRL_REG1: enable X, Y, Z axes and set a 100Hz data rate (0x57 = 0b01010111)
    uint8_t ctrl_reg1 = 0x57;
    if (platform_write(&lis3dh_addr, CTRL_REG1, &ctrl_reg1, 1) != ESP_OK)
    {
        printf("Error writing CTRL_REG1 register\n");
        return;
    }
    printf("CTRL_REG1 configured: 0x%02X\n", ctrl_reg1);

    // Configure CTRL_REG4: enable high resolution (HR = 1) and set ±2g range (FS bits = 00)
    uint8_t ctrl_reg4 = 0x08; // Bit 3 (HR) set to 1
    if (platform_write(&lis3dh_addr, CTRL_REG4, &ctrl_reg4, 1) != ESP_OK)
    {
        printf("Error writing CTRL_REG4 register\n");
        return;
    }
    printf("CTRL_REG4 configured: 0x%02X\n", ctrl_reg4);

    // Allow sensor configuration to settle
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        uint8_t data[6] = {0};
        // Read 6 consecutive registers starting at OUT_X_L with auto-increment (0x80 bit set)
        if (platform_read(&lis3dh_addr, OUT_X_L | 0x80, data, 6) == ESP_OK)
        {
            // Combine the low and high bytes for each axis and right-shift by 4 to obtain 12-bit data
            int16_t raw_x = ((int16_t)(data[1] << 8 | data[0])) >> 4;
            int16_t raw_y = ((int16_t)(data[3] << 8 | data[2])) >> 4;
            int16_t raw_z = ((int16_t)(data[5] << 8 | data[4])) >> 4;

            // Convert raw values to acceleration in m/s²
            // (raw value * sensitivity in g) * gravity (9.81 m/s²)
            float ax = raw_x * SENSITIVITY_G * GRAVITY;
            float ay = raw_y * SENSITIVITY_G * GRAVITY;
            float az = raw_z * SENSITIVITY_G * GRAVITY;

            printf("Acceleration (m/s^2) -> X: %.2f, Y: %.2f, Z: %.2f\n", ax, ay, az);
        }
        else
        {
            printf("Error reading acceleration data\n");
        }
        // Continuously read with a short delay; adjust as needed based on your sensor's data rate.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
