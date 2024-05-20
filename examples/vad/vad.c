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

    /*! Number of interrupt configuration to iterate through */
    uint8_t n_ints = 1;
    uint8_t n_status = 1;

    uint8_t gpr_ctrl_host = BMA5_ENABLE;

    struct bma580_int_map int_map = { 0 };
    struct bma5_int_conf_types int_config;
    struct bma580_feat_eng_gpr_0 gpr_0;
    struct bma580_int_status_types int_status;
    struct bma580_feat_eng_gpr_2 config;
    struct bma580_vad_config vad_config, get_vad_config;

    enum bma5_context context;

    int_config.int_src = BMA5_INT_2;
    int_status.int_src = BMA580_INT_STATUS_INT2;

    /* Assign context parameter selection */
    context = BMA5_HEARABLE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_SPI_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    printf("BMA580 Chip ID is 0x%X\n", dev.chip_id);

    rslt = bma580_get_vad_config(&vad_config, &dev);
    bma5_check_rslt("bma580_get_vad_config", rslt);

    vad_config.en_vad_x = BMA5_ENABLE;
    vad_config.en_vad_y = BMA5_ENABLE;
    vad_config.en_vad_z = BMA5_ENABLE;

    rslt = bma580_set_vad_config(&vad_config, &dev);
    bma5_check_rslt("bma580_get_vad_config", rslt);

    rslt = bma580_get_vad_config(&get_vad_config, &dev);
    bma5_check_rslt("bma580_get_vad_config", rslt);

    printf("get_vad_config.en_vad_x : %d\n", get_vad_config.en_vad_x);
    printf("get_vad_config.en_vad_y : %d\n", get_vad_config.en_vad_y);
    printf("get_vad_config.en_vad_z : %d\n", get_vad_config.en_vad_z);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config.int_conf.int_mode = BMA5_INT2_MODE_PULSED_LONG;
    int_config.int_conf.int_od = BMA5_INT2_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT2_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    /* Map VAD */
    int_map.vad_int_map = BMA580_VAD_INT_MAP_INT2;
    rslt = bma580_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_set_int_map", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.vad_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_int_status(&int_status, n_status, &dev);
    bma5_check_rslt("bma580_get_int_status", rslt);

    printf("Keep the board near to any vibration\n");

    for (;;)
    {
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        if (int_status.int_status.vad_int_status & BMA5_ENABLE)
        {
            printf("VAD interrupt occurred\n");

            rslt = bma580_get_feat_eng_gpr_2(&config, &dev);
            bma5_check_rslt("bma580_get_feat_eng_gpr_2", rslt);

            printf("VAD_STAT : %d\n", config.vad_stat);

            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
