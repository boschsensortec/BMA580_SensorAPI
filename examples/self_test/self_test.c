/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <math.h>

#include "common.h"
#include "bma580_features.h"

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 * @brief This internal API converts raw sensor values(LSB) to mg.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in mg
 *
 */
static float lsb_to_mg(int16_t val, float g_range, uint8_t bit_width);

/*!
 * @brief This internal API reads accel samples based on data ready interrupt
 *
 * @param[in] count         : Stores count of accel samples to be read.
 * @param[in] sens_data_x   : Stores average value of accel x axis data.
 * @param[in] sens_data_y   : Stores average value of accel y axis data.
 * @param[in] sens_data_z   : Stores average value of accel z axis data.
 * @param[in] dev           : Structure instance of bma5_dev.
 *
 * return Status of API
 */
static int8_t read_accel_samples(uint8_t count,
                                 int32_t *sens_data_x,
                                 int32_t *sens_data_y,
                                 int32_t *sens_data_z,
                                 struct bma5_dev *dev);

/******************************************************************************/
int main(void)
{
    int8_t rslt;
    struct bma5_dev dev;
    uint8_t count;
    uint8_t int_conf, get_int_conf;
    uint8_t int_map, get_int_map;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    uint8_t sensor_ctrl, get_sensor_ctrl;
    int32_t pos_exc_x, pos_exc_y, pos_exc_z;
    int32_t neg_exc_x, neg_exc_y, neg_exc_z;
    int32_t diff_x, diff_y, diff_z;
    int32_t sens_data_pos_exc_x = 0, sens_data_pos_exc_y = 0, sens_data_pos_exc_z = 0;
    int32_t sens_data_neg_exc_x = 0, sens_data_neg_exc_y = 0, sens_data_neg_exc_z = 0;

    uint8_t acc_self_test, get_acc_self_test;
    struct bma580_feat_eng_gpr_0 config = { 0 };
    struct bma580_int_map map_config = { 0 };

    enum bma5_context context;

    /* Assign context parameter selection */
    context = BMA5_SMARTPHONE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_I2C_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);

    /********************************************************************************/

    printf("# *******************************************************************************\n");
    printf("Disable features and interrupts\n");
    printf("***********************************************************************************\n");

    rslt = bma580_set_feat_eng_gpr_0(&config, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&config, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    printf("acc_foc_en : 0x%X\n", config.acc_foc_en);
    printf("gen_int1_en : 0x%X\n", config.gen_int1_en);
    printf("gen_int2_en : 0x%X\n", config.gen_int2_en);
    printf("gen_int3_en : 0x%X\n", config.gen_int3_en);

    rslt = bma580_set_int_map(&map_config, &dev);
    bma5_check_rslt("bma580_set_int_map", rslt);

    rslt = bma580_get_int_map(&map_config, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    printf("acc_drdy_int_map : 0x%X\n", map_config.acc_drdy_int_map);
    printf("fifo_full_int_map : 0x%X\n", map_config.fifo_full_int_map);
    printf("fifo_wm_int_map : 0x%X\n", map_config.fifo_wm_int_map);
    printf("gen_int1_int_map : 0x%X\n", map_config.gen_int1_int_map);
    printf("gen_int2_int_map :  0x%X\n", map_config.gen_int2_int_map);
    printf("gen_int3_int_map : 0x%X\n", map_config.gen_int3_int_map);
    printf("feat_eng_err_int_map : 0x%X\n", map_config.feat_eng_err_int_map);

    printf("\n# *******************************************************************************\n");
    printf("Activate Self-test\n");
    printf("***********************************************************************************\n");

    rslt = bma5_activate_self_test(&dev);
    bma5_check_rslt("bma5_activate_self_test", rslt);

    /********************************************************************************/

    printf("\n# Write Reg INT1_CONF(0x34) with 0x01 (latch)\n");

    int_conf = 0x01;
    rslt = bma5_set_regs(BMA5_REG_INT1_CONF, &int_conf, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma5_get_regs(BMA5_REG_INT1_CONF, &get_int_conf, 1, &dev);
    bma5_check_rslt("bma5_get_regs", rslt);

    printf("Value read from Reg INT1_CONF(0x34) : 0x%x\n", get_int_conf);

    /********************************************************************************/

    printf("*******************************************************************************\n");
    printf("\n# Write Reg INT_MAP0(0x36) with 0x01 (Acc data ready mapped to INT1)\n");

    int_map = 0x01;
    rslt = bma5_set_regs(BMA580_REG_INT_MAP_0, &int_map, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma5_get_regs(BMA580_REG_INT_MAP_0, &get_int_map, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    printf("Value read from Reg INT_MAP0(0x36) : 0x%x\n", get_int_map);

    /********************************************************************************/

    printf("\n# *******************************************************************************\n");
    printf("Self-Test with negative excitation\n");
    printf("***********************************************************************************\n");

    rslt = bma5_self_test_neg_excitation(&dev);
    bma5_check_rslt("bma5_self_test_neg_excitation", rslt);

    /********************************************************************************/

    printf("\n# Read accel data based on data ready interrupt\n");

    count = 4;

    rslt = read_accel_samples(count, &sens_data_neg_exc_x, &sens_data_neg_exc_y, &sens_data_neg_exc_z, &dev);
    bma5_check_rslt("read_accel_samples", rslt);

    printf("*******************************************************************************\n");
    printf("Mean LSB values for self-test with negative excitation (4 samples)\n");
    printf("*******************************************************************************\n");

    /* Negative excitation self-test values in LSB */
    neg_exc_x = (long int)(sens_data_neg_exc_x / 4);
    neg_exc_y = (long int)(sens_data_neg_exc_y / 4);
    neg_exc_z = (long int)(sens_data_neg_exc_z / 4);

    /* Print the data in LSB */
    printf("Acc_LSB_X : %ld, Acc_LSB_Y : %ld, Acc_LSB_Z : %ld\n",
           (long int)neg_exc_x,
           (long int)neg_exc_y,
           (long int)neg_exc_z);

    printf("*******************************************************************************\n");

    /********************************************************************************/

    printf("\n# *******************************************************************************\n");
    printf("Self-Test with positive excitation\n");
    printf("***********************************************************************************\n");

    rslt = bma5_self_test_pos_excitation(&dev);
    bma5_check_rslt("bma5_self_test_pos_excitation\n", rslt);

    /********************************************************************************/

    printf("\n# Read accel data based on data ready interrupt\n");

    count = 4;

    rslt = read_accel_samples(count, &sens_data_pos_exc_x, &sens_data_pos_exc_y, &sens_data_pos_exc_z, &dev);
    bma5_check_rslt("read_accel_samples", rslt);

    printf("*******************************************************************************\n");
    printf("Mean LSB values for self-test with positive excitation (4 samples)\n");
    printf("*******************************************************************************\n");

    /* Positive excitation self-test values in LSB */
    pos_exc_x = (long int)(sens_data_pos_exc_x / 4);
    pos_exc_y = (long int)(sens_data_pos_exc_y / 4);
    pos_exc_z = (long int)(sens_data_pos_exc_z / 4);

    /* Print the data in LSB */
    printf("Acc_LSB_X : %ld, Acc_LSB_Y : %ld, Acc_LSB_Z : %ld\n",
           (long int)pos_exc_x,
           (long int)pos_exc_y,
           (long int)pos_exc_z);

    /********************************************************************************/

    printf("\n# *******************************************************************************\n");
    printf("Disable self-test and reset signal path\n");

    acc_self_test = 0x00;
    rslt = bma5_set_regs(BMA5_REG_ACC_SELF_TEST, &acc_self_test, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma5_get_regs(BMA5_REG_ACC_SELF_TEST, &get_acc_self_test, 1, &dev);
    bma5_check_rslt("bma5_get_regs", rslt);

    printf("(Disable Self-test) Value read from Reg ACC_SELF_TEST(0x76) : 0x%x\n", get_acc_self_test);

    /* Disable accelerometer */
    sensor_ctrl = BMA5_SENSOR_CTRL_DISABLE;

    rslt = bma5_set_acc_conf_0(sensor_ctrl, &dev);
    bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    rslt = bma5_get_acc_conf_0(&get_sensor_ctrl, &dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    printf("(Disable accelerometer) ACC_CONF_0.sensor_ctrl : 0x%x\n", get_sensor_ctrl);

    /* Delay of 50ms */
    dev.delay_us(BMA5_SELF_TEST_ACCEL_DISABLE_DELAY, dev.intf_ptr);

    /* Enable accelerometer */
    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;

    rslt = bma5_set_acc_conf_0(sensor_ctrl, &dev);
    bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    rslt = bma5_get_acc_conf_0(&get_sensor_ctrl, &dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    printf("(Enable accelerometer) ACC_CONF_0.sensor_ctrl : 0x%x\n", get_sensor_ctrl);

    /********************************************************************************/

    printf("*******************************************************************************\n");
    printf("\n# Calculate difference of self-test results\n");

    diff_x = (long int)(pos_exc_x - neg_exc_x);
    diff_y = (long int)(pos_exc_y - neg_exc_y);
    diff_z = (long int)(pos_exc_z - neg_exc_z);

    printf("Diff_LSB_X : %ld  Diff_LSB_Y : %ld  Diff_LSB_Z : %ld\n# ",
           (long int)diff_x,
           (long int)diff_y,
           (long int)diff_z);

    if (diff_x >= BMA5_SELF_TEST_MIN_THRESHOLD_X)
    {
        printf("\n# Self-test pass for X-axis");
    }
    else
    {
        printf("\n# Self-test fail for X-axis");
    }

    if (diff_y >= BMA5_SELF_TEST_MIN_THRESHOLD_Y)
    {
        printf("\n# Self-test pass for Y-axis");
    }
    else
    {
        printf("\n# Self-test fail for Y-axis");
    }

    if (diff_z >= BMA5_SELF_TEST_MIN_THRESHOLD_Z)
    {
        printf("\n# Self-test pass for Z-axis");
    }
    else
    {
        printf("\n# Self-test fail for Z-axis\n# ");
    }

    bma5_coines_deinit();

    return rslt;
}

/******************************************************************************/
/*!           Static Function Definitions                                     */

/*!
 *  @brief This internal API converts raw sensor values(LSB) to mg.
 */
static float lsb_to_mg(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (val * g_range * 1000) / half_scale;
}

/*!
 * @brief This internal API reads accel samples based on data ready interrupt
 */
static int8_t read_accel_samples(uint8_t count,
                                 int32_t *sens_data_x,
                                 int32_t *sens_data_y,
                                 int32_t *sens_data_z,
                                 struct bma5_dev *dev)
{
    int8_t rslt = BMA5_E_COM_FAIL;
    struct bma5_sensor_status status;
    struct bma580_int_status_types int_status = { 0 };
    struct bma5_accel sens_data = { 0 };
    uint8_t n_status = 1;
    float x = 0, y = 0, z = 0;

    int_status.int_src = BMA580_INT_STATUS_INT1;

    printf("\nCount, Accel_LSB_X, Accel_LSB_Y, Accel_LSB_Z, Acc_mg_X, Acc_mg_Y, Acc_mg_Z\n");

    while (count >= 1)
    {
        /* Get accel data ready status */
        rslt = bma5_get_sensor_status(&status, dev);
        bma5_check_rslt("bma5_get_sensor_status", rslt);

        if (status.acc_data_rdy & BMA5_ENABLE)
        {
            /* Get accel data ready interrupt status */
            rslt = bma580_get_int_status(&int_status, n_status, dev);
            bma5_check_rslt("bma580_get_int_status", rslt);

            if (int_status.int_status.acc_drdy_int_status & BMA580_ACC_DRDY_INT_STATUS_MSK)
            {
                rslt = bma5_set_sensor_status(&status, dev);
                bma5_check_rslt("bma5_set_sensor_status", rslt);

                rslt = bma580_set_int_status(&int_status, n_status, dev);
                bma5_check_rslt("bma580_set_int_status", rslt);

                /* Get accel data and sensortime */
                rslt = bma5_get_acc(&sens_data, dev);
                bma5_check_rslt("bma5_get_acc", rslt);

                (*sens_data_x) += sens_data.x;
                (*sens_data_y) += sens_data.y;
                (*sens_data_z) += sens_data.z;

                /* Converting lsb to mg for 16 bit resolution at 8G range */
                x = lsb_to_mg(sens_data.x, (float)8, BMA5_16_BIT_RESOLUTION);
                y = lsb_to_mg(sens_data.y, (float)8, BMA5_16_BIT_RESOLUTION);
                z = lsb_to_mg(sens_data.z, (float)8, BMA5_16_BIT_RESOLUTION);

                /* Print the data in mg */
                printf("%d,  %d,  %d,  %d,  %4.2f,  %4.2f,  %4.2f\n",
                       count,
                       sens_data.x,
                       sens_data.y,
                       sens_data.z,
                       x,
                       y,
                       z);

                count--;
            }
        }
    }

    return rslt;
}
