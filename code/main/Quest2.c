// Nicholas Hardy - U97871602
//STD C library
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <math.h>

//RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

//GPIO task
#include "driver/gpio.h"

#include "driver/i2c.h"


#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define LED1 21
//register definitions
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         400000     // i2c master clock freq



static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


//Ultrasonic
static const adc_channel_t channel_ultra = ADC_CHANNEL_3;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten_ultra = ADC_ATTEN_DB_0;
static const adc_unit_t unit_ultra = ADC_UNIT_1;


//Photocell
static const adc_channel_t channel_solar = ADC_CHANNEL_0; // GPIO34 if ADC1, GPIO14 if ADC2
//static const adc_atten_t atten_solar = ADC_ATTEN_DB_0;
static const adc_unit_t unit_solar = ADC_UNIT_1;



uint32_t get_voltage()
{
  uint32_t adc_reading = 0;
  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++)
  {
    if (unit_solar == ADC_UNIT_1)
    {
      adc_reading += adc1_get_raw((adc1_channel_t)channel_solar);
    }
    else
    {
      int raw;
      adc2_get_raw((adc2_channel_t)channel_solar, ADC_WIDTH_BIT_12, &raw);
      adc_reading += raw;
    }
  }
  adc_reading /= NO_OF_SAMPLES;
  // Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  // vTaskDelay(pdMS_TO_TICKS(1000));
  return voltage;
}









int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}
// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  //printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      //printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {
      //printf("- No I2C devices found!" "\n");
      
  }
}
//i2c init from Master init
static void i2c_master_init(){
    // Debug
    //printf("\n>> i2c Config\n");
    int err;
    
    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    
    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {
        // printf("- parameters: ok\n");
        
    }
    
    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {//printf("- initialized: yes\n");
        
    }
        
        // Data in MSB mode
        i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Read and write to register Functions ///////////////////////////////////////////////////////////
    
    // Write one byte to register (single byte write)
    void writeRegister(uint8_t reg, uint8_t data) {
        // create i2c communication init
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);    // 1. Start (Master write start)
        i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK); // (Master write slave add + write bit)
        // wait for salve to ack
        i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); // (Master write register address)
        // wait for slave to ack
        i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);// master write data
        // wait for slave to ack
        i2c_master_stop(cmd); // 11. Stop
        // i2c communication done and delete
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        // no return here since data is written by master onto slave
    }
    
    // Read register (single byte read)
    uint16_t readRegister(uint8_t reg) {
        uint8_t data1; //first byte MSB
        uint8_t data2; //second byte LSB
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_cmd_handle_t cmd1 = i2c_cmd_link_create();
        
        // Start
        i2c_master_start(cmd);
        // Master write slave address + write bit
        i2c_master_write_byte(cmd, ( LIDARLite_ADDRESS << 1 ) | WRITE_BIT, I2C_MASTER_ACK);
        // Master write register address + send ack
        i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
        //master stops
        i2c_master_stop(cmd);
        // This starts the I2C communication
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        //master starts
        i2c_master_start(cmd1);
        // Master write slave address + read bit
        i2c_master_write_byte(cmd1, ( LIDARLite_ADDRESS << 1 ) | READ_BIT, I2C_MASTER_ACK);
        // Master reads in slave ack and data
        i2c_master_read_byte(cmd1, &data1 , I2C_MASTER_ACK);
        i2c_master_read_byte(cmd1, &data2 , I2C_MASTER_NACK);
        // Master nacks and stops.
        i2c_master_stop(cmd1);
        // This starts the I2C communication
        i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd1, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd1);
        
        
        uint16_t two_byte_data = (data1 << 8 | data2);
        return two_byte_data;
    }
    
    float test(){
        //printf("\n>> Polling Lidar\n");
        
        //write to register 0x00 the value 0x04
        writeRegister(0x00, 0x04);
        //READ REGISTER 0X01 UNTIL LSB GOES LOW
        //if LSB goes low then set flag to true
        int flag = 1;
        uint16_t data = readRegister(0x01);
        // printf("DATA: %d\n", data);
        flag = data & (1<<15);
        vTaskDelay(5);
        uint16_t distance = readRegister(RegisterHighLowB);
        float distance_m = (float)distance;
        //printf("%.2f,", distance_m);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        return distance_m;
    }
    
    
    
    
    static void check_efuse(void)
    {
        //Check TP is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
            //printf("eFuse Two Point: Supported\n");
        } else {
            //printf("eFuse Two Point: NOT supported\n");
        }
        
        //Check Vref is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
            //printf("eFuse Vref: Supported\n");
        } else {
            //printf("eFuse Vref: NOT supported\n");
        }
    }
    
    static void print_char_val_type(esp_adc_cal_value_t val_type)
    {
        if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
            //printf("Characterized using Two Point Value\n");
        } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
            //printf("Characterized using eFuse Vref\n");
        } else {
            //printf("Characterized using Default Vref\n");
        }
    }
    
    float convert_to_cel(int reading)
    {
        const float R1 = 5000.0; //value of resistor
        const float B = 3950.0; //value of B (+/- 1 percent)
        const float T0 = 276.0; //room temp in Kelvin
        float Rt = R1 / (4095.0 / (float)reading) - 1.0;
        float celsius = (1.0 / ((log(Rt / R1)) / B + (1.0/T0))) - 273.15;
        return celsius;
    }
    
    void app_main(void)
    {
        
        esp_rom_gpio_pad_select_gpio(LED1);
        gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
        
        i2c_master_init();
        i2c_scanner();
        //Check if Two Point or Vref are burned into eFuse
        check_efuse();
        //Configure ADC
        if (unit == ADC_UNIT_1) {
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(channel, atten);
        } else {
            adc2_config_channel_atten((adc2_channel_t)channel, atten);
        }
        
        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);
        
        
        
        
        
        
        if (unit_ultra == ADC_UNIT_1) {
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(channel_ultra, atten_ultra);
        } else {
            adc2_config_channel_atten((adc2_channel_t)channel_ultra, atten_ultra);
        }
        
        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type_ultra = esp_adc_cal_characterize(unit_ultra, atten_ultra, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type_ultra);
        
        float distance_ultra_current = 0;
        float distance_lidar_current = 0;
        float temp_current = 0;
        int led_state = 0;
        
        int sum = 0;
        int person1 = 0;
        int person2 = 0;
        int person3 = 0;
        int person4 = 0;
        
        int count  = 0;
        
        float bruh;
        gpio_set_level(LED1, 0);
        while (1) {
            
            
            bruh = test();
            
            
            
            
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                    adc_reading += raw;
                }
            }
            adc_reading /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            float Celsius = convert_to_cel(voltage);
            Celsius = Celsius - 26;
            //printf("%.2f,",Celsius);
            
            
            
            uint32_t adc_reading_ultra = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit_ultra == ADC_UNIT_1) {
                    adc_reading_ultra += adc1_get_raw((adc1_channel_t)channel_ultra);
                } else {
                    int raw_ultra;
                    adc2_get_raw((adc2_channel_t)channel_ultra, ADC_WIDTH_BIT_12, &raw_ultra);
                    adc_reading_ultra += raw_ultra;
                }
            }
            adc_reading_ultra /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            uint32_t voltage_ultra = esp_adc_cal_raw_to_voltage(adc_reading_ultra, adc_chars);
            //float distance = (voltage/1024)/5
            float distance_ultra = (voltage_ultra*5)/10;
            distance_ultra = distance_ultra - 18;
            //printf("%.2f,",distance_ultra);
            
            
            uint32_t voltage_Solar = get_voltage();
            //printf("%ld",voltage_Solar);
            
            printf("%.2f,%.2f,%.2f,%ld\n", bruh,Celsius,distance_ultra,voltage_Solar);
            
            if (count == 0)
            {
                temp_current = Celsius;
                distance_lidar_current = bruh;
                distance_ultra_current = distance_ultra;
                
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            if (Celsius - temp_current > 0.8)
            {
                person1 = 1;
                
            }
            else
            {
                person1 = 0;
                
            }
            
            if (voltage_Solar > 95)
            {
                person2 = 1;
                
            }
            
            else
            {
                person2 = 0;
                
            }
            
            if (distance_lidar_current - bruh > 10)
            {
                person3 = 1;
                
            }
            
            else
            {
                person3 = 0;
                
            }
            if (distance_ultra_current - distance_ultra > 50)
            {
                person4 = 1;
                
            }
            
            else
            {
                person4 = 0;
                
            }
            
            sum = person1 + person2 + person3 + person4;
            
            if(sum > 0)
            {
                gpio_set_level(LED1, 0);
            }
            
            if(sum == 0)
            {
                gpio_set_level(LED1, 1);
            }
            
            
            
            count++;
        }
    }
