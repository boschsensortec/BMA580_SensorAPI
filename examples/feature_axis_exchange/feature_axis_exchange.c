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
/*!           Static Function Declaration                                     */

/*!
 * @brief This internal API prints the selected XYZ axis.
 *
 *  @param[in] conf       : Structure instance of bma580_feat_axis.
 *
 *  @return void.
 *
 */
static void feature_axis_selection(const struct bma580_feat_axis *conf);

/******************************************************************************/
int main(void)
{
    int8_t rslt;
    uint8_t n_ints = 1;
    uint8_t n_status = 1;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;

    struct bma5_dev dev;
    struct bma580_feat_axis conf, get_conf;
    struct bma580_int_map int_map = { 0 };
    struct bma5_int_conf_types int_config;
    struct bma580_generic_interrupt_types gen_conf, set_gen_conf;
    struct bma580_int_status_types int_status;
    struct bma580_feat_eng_gpr_0 gpr_0;
    struct bma5_feat_eng_status feat_eng_status;

    gen_conf.generic_interrupt = BMA580_GEN_INT_1;
    set_gen_conf.generic_interrupt = BMA580_GEN_INT_1;
    int_status.int_src = BMA580_INT_STATUS_INT1;
    int_config.int_src = BMA5_INT_1;

    /* Assign context parameter selection */
    enum bma5_context context;
    context = BMA5_SMARTPHONE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_SPI_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    printf("BMA580 Chip ID is 0x%X\n", dev.chip_id);

    rslt = bma5_get_feat_eng_status(&feat_eng_status, &dev);
    bma5_check_rslt("bma5_get_feat_eng_status", rslt);

    printf("feat_eng_status.feat_eng_gpr_update_pending : %d\n", feat_eng_status.feat_eng_gpr_update_pending);
    printf("feat_eng_status.feat_eng_halted : %d\n", feat_eng_status.feat_eng_halted);
    printf("feat_eng_status.feat_eng_running : %d\n", feat_eng_status.feat_eng_running);
    printf("feat_eng_status.host_gpr_update_pending : %d\n\n", feat_eng_status.host_gpr_update_pending);

    rslt = bma580_get_feature_axis_config(&conf, &dev);
    bma5_check_rslt("bma580_get_feature_axis_config", rslt);

    conf.feat_axis_ex = BMA580_FEAT_AXIS_EX_ZXY;
    conf.feat_x_inv = BMA580_FEAT_X_INV_INVERT;
    conf.feat_y_inv = BMA580_FEAT_Y_INV_INVERT;
    conf.feat_z_inv = BMA580_FEAT_Z_INV_INVERT;

    rslt = bma580_set_feature_axis_config(&conf, &dev);
    bma5_check_rslt("bma580_set_feature_axis_config", rslt);

    rslt = bma580_get_feature_axis_config(&get_conf, &dev);
    bma5_check_rslt("bma580_get_feature_axis_config", rslt);

    feature_axis_selection(&get_conf);

    rslt = bma580_get_generic_int_config(&gen_conf, n_ints, &dev);
    bma5_check_rslt("bma580_get_generic_int_config", rslt);

    set_gen_conf.gen_int.slope_thres = 0xA;
    set_gen_conf.gen_int.comb_sel = 0x0;
    set_gen_conf.gen_int.axis_sel = BMA580_FEAT_AXIS_EX_SEL_Z;
    set_gen_conf.gen_int.hysteresis = 0x2;
    set_gen_conf.gen_int.criterion_sel = 0x1;
    set_gen_conf.gen_int.acc_ref_up = 0x1;
    set_gen_conf.gen_int.duration = 0xA;
    set_gen_conf.gen_int.wait_time = 0x3;
    set_gen_conf.gen_int.quiet_time = 0x40;
    set_gen_conf.gen_int.ref_acc_x = 0x0;
    set_gen_conf.gen_int.ref_acc_y = 0x0;
    set_gen_conf.gen_int.ref_acc_z = 0x800;

    rslt = bma580_set_generic_int_config(&set_gen_conf, n_ints, &dev);
    bma5_check_rslt("bma580_set_generic_int_config", rslt);

    rslt = bma580_get_generic_int_config(&gen_conf, n_ints, &dev);
    bma5_check_rslt("bma580_get_generic_int_config", rslt);

    printf("\nGeneric Interrupt 1 configurations\n");
    printf("slope_thres 0x%x\n", gen_conf.gen_int.slope_thres);
    printf("comb_sel 0x%x\n", gen_conf.gen_int.comb_sel);
    printf("axis_sel 0x%x\n", gen_conf.gen_int.axis_sel);
    printf("hysteresis 0x%x\n", gen_conf.gen_int.hysteresis);
    printf("criterion_sel 0x%x\n", gen_conf.gen_int.criterion_sel);
    printf("acc_ref_up 0x%x\n", gen_conf.gen_int.acc_ref_up);
    printf("duration 0x%x\n", gen_conf.gen_int.duration);
    printf("wait_time 0x%x\n", gen_conf.gen_int.wait_time);
    printf("quiet_time 0x%x\n", gen_conf.gen_int.quiet_time);
    printf("ref_acc_x 0x%x\n", gen_conf.gen_int.ref_acc_x);
    printf("ref_acc_y 0x%x\n", gen_conf.gen_int.ref_acc_y);
    printf("ref_acc_z 0x%x\n", gen_conf.gen_int.ref_acc_z);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.gen_int1_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    printf("\ngpr_0.gen_int1_en : %d\n", gpr_0.gen_int1_en);

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    /* Map generic interrupt 1 */
    int_map.gen_int1_int_map = BMA580_GEN_INT1_INT_MAP_INT1;
    rslt = bma580_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_set_int_map", rslt);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;
    int_config.int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    printf("\nShake the board to get generic interrupt 1 interrupt\n");

    for (;;)
    {
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        if (int_status.int_status.gen_int1_int_status & BMA5_ENABLE)
        {
            rslt = bma580_set_int_status(&int_status, n_ints, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            printf("Generic interrupt 1 interrupt occurred\n");

            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API prints the selected XYZ axis.
 */
static void feature_axis_selection(const struct bma580_feat_axis *conf)
{
    if ((conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_DEFAULT_0) ||
        (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_DEFAULT_6) || (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_DEFAULT_7))
    {
        printf("Selected axis is XYZ\n");
    }

    if (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_YXZ)
    {
        printf("Selected axis is YXZ\n");
    }

    if (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_XZY)
    {
        printf("Selected axis is XZY\n");
    }

    if (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_ZXY)
    {
        printf("Selected axis is ZXY\n");
    }

    if (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_YZX)
    {
        printf("Selected axis is YZX\n");
    }

    if (conf->feat_axis_ex == BMA580_FEAT_AXIS_EX_ZYX)
    {
        printf("Selected axis is ZYX\n");
    }

    if (conf->feat_x_inv == BMA580_FEAT_X_INV_DEFAULT)
    {
        printf("feat_x_inv : remains unchanged\n");
    }
    else
    {
        printf("feat_x_inv : -X\n");
    }

    if (conf->feat_y_inv == BMA580_FEAT_Y_INV_DEFAULT)
    {
        printf("feat_y_inv : remains unchanged\n");
    }
    else
    {
        printf("feat_y_inv : -Y\n");
    }

    if (conf->feat_z_inv == BMA580_FEAT_Z_INV_DEFAULT)
    {
        printf("feat_z_inv : remains unchanged\n");
    }
    else
    {
        printf("feat_z_inv : -Z\n");
    }
}
