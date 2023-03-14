#include "freertos/FreeRTOS.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <stdbool.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <math.h>


#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"
#include "esp_int_wdt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"

//I2C parameters
#define I2C_SDA 21 //gpio for sda
#define I2C_SCL 22 //gpio for scl
#define I2C_CLK_SPEED 1000000
#define ACK_CHECK_EN 0x1
#define SENSOR_ADDR 0x21 //address in hex of sensor (0x21 and then groud?)

extern "C"
{
    void app_main(void);
}

// Function to set up the I2C sensor
void sensor_enable()
{
    //Set up the i2c bus driver
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_CLK_SPEED;

    i2c_param_config(I2C_NUM_0, &conf);
    //Install the I2C Bus driver - last three arguments for
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

bool sensor16bitsend(int16_t i2c_reg)
{
    esp_err_t err;
    //Create command handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //Queue slave address and check for ACK
    i2c_master_write_byte(cmd, (SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    //Queue command - first byte then second
    i2c_master_write_byte(cmd, (i2c_reg >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (i2c_reg & 0xFF), ACK_CHECK_EN);
    //Stop and send command
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err == ESP_OK)
    {
        return true;
    }
    else
    {
        printf("I2C Error: %d\n", err);
        return false;
    }
}

uint16_t sensor16bitread()
{
    uint8_t msb;
    uint8_t lsb;
    uint16_t ret;

    //Start i2c communication with sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //Send slave address and check for ack
    i2c_master_write_byte(cmd, (SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    //Read first byte received and send ACK
    i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
    //Read second byte and send NACK so sensor stops sending data
    i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != 0)
    {
        printf("I2C Error: %d\n", err);
        ret = 1; //TODO: Make this a meaningful value, or implement in a different way
    }
    else
    {
        ret = (msb << 8) | lsb;
    }
    return ret;
}

esp_err_t SoftReset()
{
    bool ret = sensor16bitsend(0x0006);
    return ret;

    vTaskDelay(20 / portTICK_RATE_MS); //From Sensirion Sample Code
}

bool StartContinousMeasurement()
{
    bool ret = sensor16bitsend(0x3603);
    return ret;
}

float flowFromPressure(float pressure)
{
    pressure = (float)pressure / 60;

//   if(pressure < 0) {
//     flow = - pow(-flow, power);
//   }
//   else {
//     flow = pow(flow, power);
//   }

    float flow = pressure;
    return flow;
}

void app_main()
{
    bool error;
 
    // Initialise sensor module
    sensor_enable();
    esp_task_wdt_reset();

    // reset the sensor
    SoftReset();

    
    //int time = esp_timer_get_time()/1000;

    while(1) {

        // start continous measurement
        error = StartContinousMeasurement();
        //int currentTime = esp_timer_get_time()/1000;

        while(error == 0){

            int16_t diffPressure = sensor16bitread();

            if(diffPressure == 1){ // Define a better flag
                break;
            }

            float flow = flowFromPressure(diffPressure);
            printf("%f\n", flow);

        }

  }
}
