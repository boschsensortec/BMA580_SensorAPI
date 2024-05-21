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
int main(void)
{
    struct bma5_dev dev;
    int8_t rslt;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    uint8_t n_ints = 1;
    uint8_t n_status = 1;

    struct bma580_int_map int_map;
    struct bma5_int_conf_types int_config;
    struct bma580_generic_interrupt_types conf, set_conf;
    struct bma580_int_status_types int_status;
    struct bma580_feat_eng_gpr_0 gpr_0;

    int_config.int_src = BMA5_INT_2;
    int_status.int_src = BMA580_INT_STATUS_INT2;
    conf.generic_interrupt = BMA580_GEN_INT_2;
    set_conf.generic_interrupt = BMA580_GEN_INT_2;

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

    printf("Default configurations\n");
    rslt = bma580_get_default_generic_int_config(&conf, n_ints, &dev);
    bma5_check_rslt("bma580_get_default_generic_int_config", rslt);

    printf("slope_thres 0x%x\n", conf.gen_int.slope_thres);
    printf("comb_sel 0x%x\n", conf.gen_int.comb_sel);
    printf("axis_sel 0x%x\n", conf.gen_int.axis_sel);
    printf("hysteresis 0x%x\n", conf.gen_int.hysteresis);
    printf("criterion_sel 0x%x\n", conf.gen_int.criterion_sel);
    printf("acc_ref_up 0x%x\n", conf.gen_int.acc_ref_up);
    printf("duration 0x%x\n", conf.gen_int.duration);
    printf("wait_time 0x%x\n", conf.gen_int.wait_time);
    printf("quiet_time 0x%x\n", conf.gen_int.quiet_time);
    printf("ref_acc_x 0x%x\n", conf.gen_int.ref_acc_x);
    printf("ref_acc_y 0x%x\n", conf.gen_int.ref_acc_y);
    printf("ref_acc_z 0x%x\n", conf.gen_int.ref_acc_z);

    set_conf.gen_int.slope_thres = 0xA;
    set_conf.gen_int.comb_sel = 0x1;
    set_conf.gen_int.axis_sel = 0x7;
    set_conf.gen_int.hysteresis = 0x2;
    set_conf.gen_int.criterion_sel = 0x0;
    set_conf.gen_int.acc_ref_up = 0x1;
    set_conf.gen_int.duration = 0xA;
    set_conf.gen_int.wait_time = 0x3;
    set_conf.gen_int.quiet_time = 0x40;
    set_conf.gen_int.ref_acc_x = 0x0;
    set_conf.gen_int.ref_acc_y = 0x0;
    set_conf.gen_int.ref_acc_z = 0x800;

    rslt = bma580_set_generic_int_config(&set_conf, n_ints, &dev);
    bma5_check_rslt("bma580_set_generic_int_config", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.gen_int2_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    /* Map generic interrupt 2 */
    int_map.gen_int2_int_map = BMA580_GEN_INT2_INT_MAP_INT2;
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

    printf("Do not shake the board to get interrupt for generic interrupt 2\n");

    for (;;)
    {
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status_int", rslt);

        if (int_status.int_status.gen_int2_int_status & BMA5_ENABLE)
        {
            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status_int", rslt);

            printf("Generic interrupt 2 interrupt occurred\n");

            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
