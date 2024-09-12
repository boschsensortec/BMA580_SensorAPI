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
#include "common.h"
#include "bma580_features.h"

/******************************************************************************/
/*                  Global Definitions                                        */

/* Macros for setting various generic interrupt configuration */

/* Configuration parameters of generic interrupt which influences the sensitivity of the
* Motion Detection Algorithm
* The user can modify below parameters according to system requirements
*/
#define SLOPE_THRES    UINT16_C(160)
#define HYSTERESIS     UINT16_C(8)
#define WAIT_TIME      UINT8_C(3)
#define REF_ACC_X      INT16_C(0) /* without influence since acc_ref_up set to 1 */
#define REF_ACC_Y      INT16_C(0) /* without influence since acc_ref_up set to 1 */
#define REF_ACC_Z      INT16_C(2048) /* without influence since acc_ref_up set to 1 */
#define QUIET_TIME     UINT16_C(64) /* without influence for Android compliance */

/*
 * Fixed configuration parameters of generic interrupt for Motion Detection algorithm.
 * Not to be changed
 */
#define CRITERION_SEL  UINT8_C(1) /* Check for activity */
#define ACC_REF_UP     UINT8_C(1) /* Reference update always */
#define COMB_SEL       UINT8_C(0) /* OR combination of axes */
#define AXIS_SEL       UINT8_C(7) /* Select all axes (x, y, z) */
#define DURATION       UINT16_C(250) /* set to 5 seconds (value of 250) for Android compliance */

/******************************************************************************/
int main(void)
{
    struct bma5_dev dev;
    int8_t rslt;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    uint8_t n_ints = 1;
    uint8_t n_status = 1;

    /* Variable to hold interrupt mapping */
    struct bma580_int_map int_map;
    struct bma5_int_conf_types int_config;

    /* Variable to hold generic interrupt 1 configuration*/
    struct bma580_generic_interrupt_types conf;

    conf.generic_interrupt = BMA580_GEN_INT_1;

    /* Variable to store interrupt status */
    struct bma580_int_status_types int_status;
    struct bma580_feat_eng_gpr_0 gpr_0;

    /* Mapping to hardware interrupt pin 1 on sensor */
    int_config.int_src = BMA5_INT_1;

    /* Variable to hold configurations related to interrupt pin 1 */
    int_status.int_src = BMA580_INT_STATUS_INT1;

    /* Assign context parameter selection */
    enum bma5_context context;
    context = BMA5_SMARTPHONE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_I2C_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    /* Initialize the BMA5 device instance */
    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    printf("Chip ID :0x%X\n", dev.chip_id);

    /* Updating the generic interrupt 1 for motion detection */
    rslt = bma580_get_generic_int_config(&conf, n_ints, &dev);
    bma5_check_rslt("bma580_get_default_generic_int_config", rslt);

    /* Set Generic Interrupt 1 Configuration */
    conf.gen_int.slope_thres = SLOPE_THRES;
    conf.gen_int.comb_sel = COMB_SEL;
    conf.gen_int.axis_sel = AXIS_SEL;
    conf.gen_int.hysteresis = HYSTERESIS;
    conf.gen_int.criterion_sel = CRITERION_SEL;
    conf.gen_int.quiet_time = QUIET_TIME;
    conf.gen_int.duration = DURATION;
    conf.gen_int.wait_time = WAIT_TIME;
    conf.gen_int.acc_ref_up = ACC_REF_UP;
    conf.gen_int.ref_acc_x = REF_ACC_X;
    conf.gen_int.ref_acc_y = REF_ACC_Y;
    conf.gen_int.ref_acc_z = REF_ACC_Z;

    rslt = bma580_set_generic_int_config(&conf, n_ints, &dev);
    bma5_check_rslt("bma580_set_generic_int_config", rslt);

    if (rslt == BMA5_OK)
    {
        printf("Generic Interrupt configurations done\n");
    }

    /* Enabling the generic interrupt 1 using the feature engine general purpose register 0 */
    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    /* Enable generic interrupt 1 */
    gpr_0.gen_int1_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    if (rslt == BMA5_OK)
    {
        printf("Generic Interrupt 1 enabled\n");
    }

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    /* Map generic interrupt 1 to hardware interrupt pin 1 of the sensor*/
    int_map.gen_int1_int_map = BMA580_GEN_INT1_INT_MAP_INT1;
    rslt = bma580_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_set_int_map", rslt);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    /* Set the hardware interrupt pin configuration */
    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;
    int_config.int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    printf("Int Configurations:\n");
    printf("Int1 mode: %s\n", enum_to_string(BMA5_INT1_MODE_LATCHED));
    printf("Int1 OD: %s\n", enum_to_string(BMA5_INT1_OD_PUSH_PULL));
    printf("Int1 Level: %s\n", enum_to_string(BMA5_INT1_LVL_ACTIVE_HIGH));

    printf("Shake the board to trigger motion detected interrupt\n");

    for (;;)
    {
        /* Read the hardware interrupt pin 1 status */
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        /* Checking interrupt status to check motion detection */
        if (int_status.int_status.gen_int1_int_status & BMA5_ENABLE)
        {
            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            printf("Motion detected\n");
            break;

        }
    }

    /*
     * To realize Android compliance: After receiving an interrupt for motion detected, it is required to reset the Generic Interrupt for the next
     * motion detection interrupt to occur.
     */
    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.gen_int1_en = BMA5_DISABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    if (rslt == BMA5_OK)
    {
        printf("Generic Interrupt 1 disabled\n");
    }

    /*Delay for generic interrupt reset*/
    dev.delay_us(40000, dev.intf_ptr);

    bma5_coines_deinit();

    return rslt;
}
