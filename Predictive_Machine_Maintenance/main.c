/******************************************************************************
* File Name:   main.c
*
* Description: This is the main file for mtb-example-ml-deepcraft-deploy-motion 
* Code Example.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <float.h>
#include <stdbool.h>
#include "cyhal.h"
#include "cybsp.h"
#include "bmi270.h"
#include "cy_retarget_io.h"

/* Model to use */
#include <models/model.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define BMI270_ADDRESS (BMI2_I2C_PRIM_ADDR)

/* X, Y and Z axes */
#define AXIS_COUNT            (3)

/* Total number of sensors - Accelerometer & Gyroscope */
#define SENSOR_COUNT          (2)

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH         (9.80665f)

/* Accelerometer range in G. Must be one of 2, 4, 8, 16 */
#define IMU_ACCEL_RANGE_G     (8)

/* Gyro range in degrees per second. Must be one of 125, 250, 500, 1000, 2000 */
#define IMU_GYRO_RANGE_DPS    (500)

/* IMU Sample frequency (Hz), must be one of 25, 50, 100, 200, 400 */
#define IMU_FREQ              (50)

/* I2C Config */
#define _I2C_TIMEOUT_MS            (10U)
#define _READ_WRITE_LEN            (46U)
#define _SOFT_RESET_DELAY_US       (300)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void init_board(void);
static void halt_error(int code);
static void imu_init(struct bmi2_dev* imu);
static float imu_lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
static float imu_lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
static bool imu_read(struct bmi2_dev* imu, float* dest);
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);
static void _bmi2_delay_us(uint32_t us, void* intf_ptr);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It initializes BSP, IMU and the ML model.
* It reads data from IMU sensor continuously, processes it within the model and 
* displays the output.
*
* Parameters:
*  void
*
* Return:
*  int
*

*******************************************************************************/
int main(void)
{
    float data_buffer[SENSOR_COUNT * AXIS_COUNT];
    float label_scores[IMAI_DATA_OUT_COUNT];
    char *label_text[] = IMAI_DATA_OUT_SYMBOLS;
    struct bmi2_dev imu = {0};
    cy_rslt_t result;
    int16_t best_label;
    float max_score;

    /* Basic board setup */
    init_board();

    /* ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H\x1b[?25l");

    /* Initialize model */
    result = IMAI_init();
    halt_error(result);

    /* Initialize IMU sampling */
    imu_init(&imu);

    /* Initialize the User LED */
           result = cyhal_gpio_init(CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT,
                                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
           /* GPIO init failed. Stop program execution */
               if (result != CY_RSLT_SUCCESS)
               {
                   CY_ASSERT(0);
               }
               result = cyhal_gpio_init(CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT,
                                                   CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_ON);
    for (;;)
    {
        /* Move cursor home */
        printf("\033[H");
        printf("Predictive Machine Maintenance\r\n\n");
        //cyhal_gpio_write(CYBSP_USER_LED,CYBSP_LED_STATE_OFF);
        /* Read sensor data */
        memset(data_buffer, 0,  sizeof(data_buffer));
        if(!imu_read(&imu, data_buffer))
        {
            continue;
        }
        /* Give sensor data to model */
        result = IMAI_enqueue(data_buffer);
        halt_error(result);

        /* Check if there is any model output */
        best_label = 0;
        max_score = -1000.0f;
        switch(IMAI_dequeue(label_scores))
        {
            case IMAI_RET_SUCCESS:      /* We have data, display it */
                for(int i = 0; i < IMAI_DATA_OUT_COUNT; i++)
                {
                    printf("label: %-10s: score: %f\r\n", label_text[i], label_scores[i]);
                    if (label_scores[i] > max_score)
                    {
                        max_score = label_scores[i];
                        best_label = i;
                    }
                }
                printf("\r\n");
                printf("Output: %-30s\r\n", label_text[best_label]);
                printf("\r\n");
                printf("best lable: %d\r\n", best_label);
                if(best_label >0 )
                        {
                		cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_OFF); //Turn RED LED ON WHEN ANOMALLY Detected
                        }
                else {cyhal_gpio_write(CYBSP_USER_LED2,CYBSP_LED_STATE_ON);}

                break;
            case IMAI_RET_NODATA:   /* No new output, continue with sampling */
                break;
            case IMAI_RET_ERROR:    /* Abort on error */
                halt_error(IMAI_RET_ERROR);
                break;
        }
    }
}


/*******************************************************************************
* Function Name: init_board
********************************************************************************
* Summary:
*    This function is a one-time setup for the board that initializes the device 
*    and board peripherals
*
* Parameters:
*    void
*
* Return:
*    void
*
*
*******************************************************************************/
static void init_board(void)
{
    cy_rslt_t result;
    /* Clear watchdog timer so that it doesn't trigger a reset */
    #if defined (CY_DEVICE_SECURE)
        cyhal_wdt_t wdt_obj;
        result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
        CY_ASSERT(CY_RSLT_SUCCESS == result);
        cyhal_wdt_free(&wdt_obj);
    #endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    halt_error(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(
            CYBSP_DEBUG_UART_TX,
            CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,
            CYBSP_DEBUG_UART_RTS,
            CY_RETARGET_IO_BAUDRATE);
    halt_error(result);
}


/*******************************************************************************
* Function Name: halt_error
********************************************************************************
* Summary:
*    This function halts the execution using an infinite loop. If the given
*    parameter is 0 (for success) this function does nothing.    
*
* Parameters:
*    code          Return code from the calling function 
*                  
* Return:
*    void
*
*
*******************************************************************************/
static void halt_error(int code)
{
    if(code != 0) /* Universal success code */
    {
        for(;;) /* Infinite loop to halt the execution */
        {
        }
    }
}


/*******************************************************************************
* Function Name: imu_init
********************************************************************************
* Summary:
*    A function used to initialize and configure the BMI270 IMU sensor.
*
* Parameters:
*  imu          Pointer to the bmi2_dev structure.
*
* Return:
*     void
*
*
*******************************************************************************/
static void imu_init(struct bmi2_dev* imu)
{
    cy_rslt_t result;
    static cyhal_i2c_t i2c;
    struct bmi2_sens_config config[SENSOR_COUNT];
    uint8_t sens_list[SENSOR_COUNT] = { BMI2_ACCEL, BMI2_GYRO };

    /* I2C config structure */
    cyhal_i2c_cfg_t i2c_config =
    {
         .is_slave = false,
         .address = 0,
         .frequencyhal_hz = 400000
    };

    /* Initialize I2C */
    result = cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    halt_error(result);

    /* Configure the I2C */
    result = cyhal_i2c_configure(&i2c, &i2c_config);
    halt_error(result);

    /* Initializes the bmi270 IMU */
    imu->intf = BMI2_I2C_INTF;
    imu->read = _bmi2_i2c_read;
    imu->write = _bmi2_i2c_write;
    imu->delay_us = _bmi2_delay_us;
    imu->intf_ptr = &i2c;
    imu->read_write_len = _READ_WRITE_LEN;
    imu->config_file_ptr = NULL;
    result = bmi270_init(imu);
    halt_error(result);

    /* Get sensor configuration */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    result = bmi2_get_sensor_config(config, SENSOR_COUNT, imu);
    halt_error(result);

    /* Update the IMU configuration */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    switch(IMU_FREQ) 
    {
        case 25:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
            break;
        case 50:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
            break;
        case 100:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
            break;
        case 200:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
            break;
        case 400:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
            break;
        case 800:
            config[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
            break;
        default:
            halt_error(-1);
            return;
    }

    switch(IMU_ACCEL_RANGE_G) 
    {
        case 2: 
            config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
            break;
        case 4: 
            config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
            break;
        case 8: 
            config[0].cfg.acc.range = BMI2_ACC_RANGE_8G;
            break;
        case 16: 
            config[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
            break;
        default: 
            halt_error(-1);
            return;
    }

    switch(IMU_GYRO_RANGE_DPS) 
    {
        case 125: 
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_125;
            break;
        case 250: 
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_250;
            break;
        case 500: 
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
            break;
        case 1000: 
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_1000;
            break;
        case 2000: 
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            break;
        default: 
            halt_error(-1);
            return;
    }

    /* The bandwidth parameter is used to configure the number of sensor samples that are averaged.
     * If it is set to 2, then 2^(bandwidth parameter) samples are averaged, resulting in 4
     * averaged samples.
     * Note1 : For more information, refer to the datasheet.
     * Note2 : A higher number of averaged samples will result in a lower noise level of the signal,
     * but this has an adverse effect on the power consumed.
     */
    config[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;

    /* Enable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode (Default)
     * For more info refer to the datasheet.
     */
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

    /* Enable/Disable the noise performance mode for precision yaw rate sensing
     * There are two modes
     *  0 -> Ultra low power mode (Default)
     *  1 -> High performance mode
     */
    config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    /* Enable/Disable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode (Default)
     */
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    /* Apply the configuration */
    result = bmi2_set_sensor_config(config, SENSOR_COUNT, imu);
    halt_error(result);

    /* Enable the sensors */
    result = bmi2_sensor_enable(sens_list, SENSOR_COUNT, imu);
    halt_error(result);
}

/*******************************************************************************
* Function Name: imu_lsb_to_mps2
********************************************************************************
* Summary:
*    This function is used to convert lsb to meter per second squared for 16-bit accelerometer.
*
* Parameters:
*  val          The raw 16-bit integer value from the accelerometer.
*  g_range      The range of the accelerometer in g (e.g. 2g, 4g, 8g, etc.).
*  bit_width    The bit width of the accelerometer's ADC (e.g. 16-bit).
*
* Return:
*     float
*
*
*******************************************************************************/
static float imu_lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}


/*******************************************************************************
* Function Name: imu_lsb_to_dps
********************************************************************************
* Summary:
*    A function used to convert lsb to degree per second for 16-bit gyro.
*
* Parameters:
*  val          The raw 16-bit integer value from the gyro.
*  dps          The full scale range of the gyro in degrees per second.
*  bit_width    The bit width of the gyro's ADC (e.g. 16-bit).
*
* Return:
*     float
*
*
*******************************************************************************/
static float imu_lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}


/*******************************************************************************
* Function Name: imu_read
********************************************************************************
* Summary:
*    This function is used to read the current BMI270 data and convert it.
*
* Parameters:
*  imu          A pointer to the bmi2_dev structure.
*  dest         A pointer to a float array to store the scaled sensor data.
*
* Return:
*     bool
*
*
*******************************************************************************/
static bool imu_read(struct bmi2_dev* imu, float* dest)
{
    int8_t result;
    struct bmi2_sens_data data = { { 0 } };

    result = bmi2_get_sensor_data(&data, imu);

    if(result != BMI2_OK)
    {
        return false;
    }

    if(!(data.status & BMI2_DRDY_ACC) || !(data.status & BMI2_DRDY_GYR))
    {
        return false;
    }

    *dest++ = imu_lsb_to_mps2(data.acc.x, IMU_ACCEL_RANGE_G, imu->resolution);
    *dest++ = imu_lsb_to_mps2(data.acc.y, IMU_ACCEL_RANGE_G, imu->resolution);
    *dest++ = imu_lsb_to_mps2(data.acc.z, IMU_ACCEL_RANGE_G, imu->resolution);
    *dest++ = imu_lsb_to_dps(data.gyr.x, IMU_GYRO_RANGE_DPS, imu->resolution);
    *dest++ = imu_lsb_to_dps(data.gyr.y, IMU_GYRO_RANGE_DPS, imu->resolution);
    *dest++ = imu_lsb_to_dps(data.gyr.z, IMU_GYRO_RANGE_DPS, imu->resolution);

    return true;
}


/*****************************************************************************
* Function name: _bmi2_i2c_read
*****************************************************************************
* Summary:
*   This function handles I2C read operations
*
* Parameters:
*  reg_addr    8-bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_read(
        i2c,
        BMI270_ADDRESS,
        reg_addr,
        1,
        reg_data,
        (uint16_t)len,
        _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmi2_i2c_write
*****************************************************************************
* Summary:
*   This function handles I2C write operations.
*
* Parameters:
*  reg_addr    8-bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMI2_INTF_RETURN_TYPE _bmi2_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMI2_INTF_RETURN_TYPE)cyhal_i2c_master_mem_write(
        i2c,
        BMI270_ADDRESS,
        reg_addr,
        1,
        reg_data,
        (uint16_t)len,
        _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmi2_delay_us
*****************************************************************************
* Summary:
*   This function introduces specified delay
*
* Parameters:
*  us           The time period in microseconds
*  intf_ptr     Void pointer that can enable the linking of descriptors for
*               interface related callbacks
*
* Return:
*  void
*
*****************************************************************************/
static void _bmi2_delay_us(uint32_t us, void* intf_ptr)
{
    (void)(intf_ptr);

    cyhal_system_delay_us(us);
}
