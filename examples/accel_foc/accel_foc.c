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
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

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
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

/******************************************************************************/
int main(void)
{
    int8_t rslt;
    uint8_t n_ints = 2;
    uint8_t n_status = 2;
    uint8_t loop;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    uint8_t sensor_ctrl;

    float x, y, z;

    struct bma5_dev dev;
    struct bma580_int_map int_map = { 0 };
    struct bma580_int_status_types int_status[2];
    struct bma5_int_conf_types int_config[2];
    struct bma580_accel_foc_config conf;
    struct bma580_feat_eng_gpr_0 gpr_0;
    struct bma580_feat_eng_gp_flags gp_flags;
    struct bma5_accel_doff user_off;
    struct bma5_sensor_status status;
    struct bma5_accel sens_data = { 0 };

    /* Assign context parameter selection */
    enum bma5_context context;

    context = BMA5_SMARTPHONE;

    int_config[0].int_src = BMA5_INT_1;
    int_config[1].int_src = BMA5_INT_2;

    int_status[0].int_src = BMA580_INT_STATUS_INT1;
    int_status[1].int_src = BMA580_INT_STATUS_INT2;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_I2C_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    printf("BMA580 Chip ID is 0x%X\n", dev.chip_id);

    /* Get accel configurations */
    rslt = bma5_get_acc_conf_0(&sensor_ctrl, &dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    sensor_ctrl = BMA5_SENSOR_CTRL_DISABLE;

    /* disable accel */
    rslt = bma5_set_acc_conf_0(sensor_ctrl, &dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    /* Map accel drdy interrupt */
    int_map.acc_drdy_int_map = BMA580_ACC_DRDY_INT_MAP_INT1;

    /* Map accel foc interrupt */
    int_map.acc_foc_int_map = BMA580_ACC_FOC_INT_MAP_INT2;

    rslt = bma580_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_set_int_map_0", rslt);

    /* Get accel configurations */
    rslt = bma5_get_acc_conf_0(&sensor_ctrl, &dev);
    bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;

    /* enable accel */
    rslt = bma5_set_acc_conf_0(sensor_ctrl, &dev);
    bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config[0].int_conf.int_mode = BMA5_INT1_MODE_PULSED_LONG;
    int_config[0].int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config[0].int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_HIGH;

    int_config[1].int_conf.int_mode = BMA5_INT2_MODE_PULSED_LONG;
    int_config[1].int_conf.int_od = BMA5_INT2_OD_PUSH_PULL;
    int_config[1].int_conf.int_lvl = BMA5_INT2_LVL_ACTIVE_LOW;

    rslt = bma5_set_int_conf(int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    printf("Reading Accel values before FOC compensation\n");
    printf("\n# Count, Accel_LSB_X, Accel_LSB_Y, Accel_LSB_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

    loop = 0;
    while (loop < 10)
    {
        for (;;)
        {
            /* Get accel data ready status */
            rslt = bma5_get_sensor_status(&status, &dev);
            bma5_check_rslt("bma5_get_sensor_status", rslt);

            if (status.acc_data_rdy)
            {
                /* Get accel data ready interrupt status */
                rslt = bma580_get_int_status(int_status, n_status, &dev);
                bma5_check_rslt("bma580_get_int_status", rslt);

                if (int_status[0].int_status.acc_drdy_int_status & BMA580_ACC_DRDY_INT_STATUS_MSK)
                {
                    rslt = bma5_set_sensor_status(&status, &dev);
                    bma5_check_rslt("bma5_set_sensor_status", rslt);

                    rslt = bma580_set_int_status(int_status, n_status, &dev);
                    bma5_check_rslt("bma580_set_int_status_int1_0", rslt);

                    /* Get accel data */
                    rslt = bma5_get_acc(&sens_data, &dev);
                    bma5_check_rslt("bma5_get_acc", rslt);

                    /* Converting lsb to mg for 16 bit resolution at 8G range */
                    x = lsb_to_ms2(sens_data.x, (float)8, BMA5_16_BIT_RESOLUTION);
                    y = lsb_to_ms2(sens_data.y, (float)8, BMA5_16_BIT_RESOLUTION);
                    z = lsb_to_ms2(sens_data.z, (float)8, BMA5_16_BIT_RESOLUTION);

                    /* Print the data in mg */
                    printf("%d,  %d,  %d,  %d,  %4.2f,  %4.2f,  %4.2f\n",
                           loop + 1,
                           sens_data.x,
                           sens_data.y,
                           sens_data.z,
                           x,
                           y,
                           z);
                    break;
                }
            }
        }

        loop++;
    }

    printf("Reading the offset values before FOC compensation\n");

    rslt = bma5_get_acc_doff(&user_off, &dev);
    bma5_check_rslt("bma5_get_acc_doff", rslt);

    printf("USER_OFFSET_X %d\t USER_OFFSET_Y %d\t USER_OFFSET_Z %d\n", user_off.x_doff, user_off.y_doff,
           user_off.z_doff);

    rslt = bma580_get_accel_foc_config(&conf, &dev);
    bma5_check_rslt("bma580_get_accel_foc_config", rslt);

    printf("foc_off_x:0x%x\n", conf.foc_off_x);
    printf("foc_off_y:0x%x\n", conf.foc_off_y);
    printf("foc_off_z:0x%x\n", conf.foc_off_z);

    conf.foc_apply_corr = BMA5_ENABLE;
    conf.foc_filter_coeff = 4;
    conf.foc_axis_1g = BMA5_ACC_FOC_AXIS_Z_MINUS_1G;

    rslt = bma580_set_accel_foc_config(&conf, &dev);
    bma5_check_rslt("bma580_set_accel_foc_config", rslt);

    rslt = bma580_get_accel_foc_config(&conf, &dev);
    bma5_check_rslt("bma580_get_accel_foc_config", rslt);

    printf("foc_apply_corr:0x%x\n", conf.foc_apply_corr);
    printf("foc_filter_coeff:0x%x\n", conf.foc_filter_coeff);
    printf("foc_axis_1g:0x%x\n", conf.foc_axis_1g);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.acc_foc_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    printf("foc en : %d\n", gpr_0.acc_foc_en);

    rslt = bma580_get_feat_eng_gp_flags(&gp_flags, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gp_flags", rslt);

    printf("gp_flags.feat_init_stat : %d\n", gp_flags.feat_init_stat);
    printf("gp_flags.foc_running : %d\n", gp_flags.foc_running);

    printf("\nDo not move the board to perform ACCEL FOC\n");
    for (;;)
    {
        rslt = bma580_get_int_status(int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        if (int_status[0].int_status.acc_drdy_int_status & int_status[1].int_status.acc_foc_int_status & BMA5_ENABLE)
        {
            rslt = bma580_set_int_status(int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            printf("\nAccel FOC interrupt occurred\n");

            break;
        }
    }

    printf("\n# Count, Accel_LSB_X, Accel_LSB_Y, Accel_LSB_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");
    printf("Reading Accel values after FOC compensation\n");

    loop = 0;
    while (loop < 10)
    {
        for (;;)
        {
            /* Get accel data ready status */
            rslt = bma5_get_sensor_status(&status, &dev);
            bma5_check_rslt("bma5_get_sensor_status", rslt);

            if (status.acc_data_rdy)
            {
                /* Get accel data ready interrupt status */
                rslt = bma580_get_int_status(int_status, n_status, &dev);
                bma5_check_rslt("bma580_get_int_status", rslt);

                if (int_status[0].int_status.acc_drdy_int_status & BMA580_ACC_DRDY_INT_STATUS_MSK)
                {
                    rslt = bma5_set_sensor_status(&status, &dev);
                    bma5_check_rslt("bma5_set_sensor_status", rslt);

                    rslt = bma580_set_int_status(int_status, n_status, &dev);
                    bma5_check_rslt("bma580_set_int_status_int1_0", rslt);

                    /* Get accel data */
                    rslt = bma5_get_acc(&sens_data, &dev);
                    bma5_check_rslt("bma5_get_acc", rslt);

                    /* Converting lsb to mg for 16 bit resolution at 8G range */
                    x = lsb_to_ms2(sens_data.x, (float)8, BMA5_16_BIT_RESOLUTION);
                    y = lsb_to_ms2(sens_data.y, (float)8, BMA5_16_BIT_RESOLUTION);
                    z = lsb_to_ms2(sens_data.z, (float)8, BMA5_16_BIT_RESOLUTION);

                    /* Print the data in mg */
                    printf("%d,  %d,  %d,  %d,  %4.2f,  %4.2f,  %4.2f\n",
                           loop + 1,
                           sens_data.x,
                           sens_data.y,
                           sens_data.z,
                           x,
                           y,
                           z);
                    break;
                }
            }
        }

        loop++;
    }

    printf("Reading the offset values after FOC compensation\n");

    rslt = bma5_get_acc_doff(&user_off, &dev);
    bma5_check_rslt("bma5_get_acc_doff", rslt);

    printf("USER_OFFSET_X %d\t USER_OFFSET_Y %d\t USER_OFFSET_Z %d\n", user_off.x_doff, user_off.y_doff,
           user_off.z_doff);

    bma5_coines_deinit();

    return rslt;
}

/*******************************************************************/
/*                Static Function Definitions                      */
/*******************************************************************/

/*!
 *  @brief This internal API converts raw sensor values(LSB) to mg.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}
