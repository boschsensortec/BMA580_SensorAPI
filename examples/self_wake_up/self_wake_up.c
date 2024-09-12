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

/* Enum for power modes */
#define BMA580_POWER_MODE_LOW               0
#define BMA580_SELF_WAKE_UP_STAT_LOW_POWER  1

/******************************************************************************/
int main(void)
{
    struct bma5_dev dev;
    int8_t rslt;
    int8_t loop = 5;
    uint8_t n_ints = 1;
    uint8_t n_status = 1;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    struct bma580_int_map int_map = { 0 };
    struct bma5_int_conf_types int_config;
    struct bma580_feat_eng_gpr_0 gpr_0;
    struct bma5_acc_conf acc_conf;
    struct bma580_feat_eng_gpr_1 gpr_1;
    struct bma580_feat_eng_gpr_2 gpr_2;
    struct bma580_int_status_types int_status;
    struct bma580_self_wakeup_config self_wake_up_config, get_self_wake_up_config;

    int_config.int_src = BMA5_INT_2;
    int_status.int_src = BMA580_INT_STATUS_INT2;

    /* Assign context parameter selection */
    enum bma5_context context;
    context = BMA5_HEARABLE;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_SPI_INTF, context);
    bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma580_init(&dev);
    bma5_check_rslt("bma580_init", rslt);
    printf("Chip ID :0x%X\n", dev.chip_id);

    rslt = bma580_get_self_wakeup_config(&self_wake_up_config, &dev);
    bma5_check_rslt("bma580_get_self_wakeup_config", rslt);

    /* Set the self wake-up configuration */
    self_wake_up_config.acc_odr = BMA5_ACC_ODR_HZ_50;
    self_wake_up_config.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;
    self_wake_up_config.acc_perf_mode = BMA580_SELF_WAKE_UP_ACC_PERF_MODE_CIC_AVG;

    rslt = bma580_set_self_wakeup_config(&self_wake_up_config, &dev);
    bma5_check_rslt("bma580_set_self_wakeup_config", rslt);

    printf("\nAccel Configurations\n");
    printf("ODR: %s\t\n", enum_to_string(BMA5_ACC_ODR_HZ_50));
    printf("BWP: %s\t\n", enum_to_string(BMA5_ACC_BWP_NORM_AVG4));
    printf("Performance Mode: %s\t\n", enum_to_string(BMA580_SELF_WAKE_UP_ACC_PERF_MODE_CIC_AVG));

    rslt = bma580_get_self_wakeup_config(&get_self_wake_up_config, &dev);
    bma5_check_rslt("bma580_get_self_wakeup_config", rslt);

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config.int_conf.int_mode = BMA5_INT2_MODE_LATCHED;
    int_config.int_conf.int_od = BMA5_INT2_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT2_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_ints, &dev);
    bma5_check_rslt("bma5_set_int_conf", rslt);

    printf("\nInterrupt Configurations\n");
    printf("Int Mode: %s\t\n", enum_to_string(BMA5_INT2_MODE_LATCHED));
    printf("Int OD: %s\t\n", enum_to_string(BMA5_INT2_OD_PUSH_PULL));
    printf("Int Level: %s\t\n", enum_to_string(BMA5_INT2_LVL_ACTIVE_HIGH));

    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    int_map.gen_int1_int_map = BMA580_GEN_INT1_INT_MAP_INT2;
    int_map.gen_int2_int_map = BMA580_GEN_INT2_INT_MAP_INT2;
    int_map.self_wake_up_int_map = BMA580_SELF_WAKE_UP_INT_MAP_INT2;

    rslt = bma580_set_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_set_int_map", rslt);

    printf("\nGeneric Int Configurations\n");
    printf("Gen Int1 Map: %s\t\n", enum_to_string(BMA580_GEN_INT1_INT_MAP_INT2));
    printf("Gen Int2 Map: %s\t\n", enum_to_string(BMA580_GEN_INT2_INT_MAP_INT2));
    printf("Self Wake Up Map: %s\t\n", enum_to_string(BMA580_SELF_WAKE_UP_INT_MAP_INT2));

    rslt = bma580_get_feat_eng_gpr_1(&gpr_1, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_1", rslt);

    gpr_1.gen_int1_data_src = BMA580_GEN_INT1_DATA_SRC_DATA_SRC_4;

    rslt = bma580_set_feat_eng_gpr_1(&gpr_1, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_1", rslt);

    if (rslt == BMA5_OK)
    {
        printf("\nGeneric Interrupt 1 data source set\n");
    }

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_1(&gpr_1, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_1", rslt);

    printf("gpr_1.gen_int1_data_src : %d\n", gpr_1.gen_int1_data_src);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.gen_int1_en = BMA5_ENABLE;
    gpr_0.gen_int2_en = BMA5_ENABLE;
    gpr_0.self_wake_up_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    if (rslt == BMA5_OK)
    {
        printf("\nGeneric Interrupt 1, Generic Interrupt 2 and Self wake-up enabled\n");
    }

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    printf("gpr_0.gen_int1_en : %d\n", gpr_0.gen_int1_en);
    printf("gpr_0.gen_int2_en : %d\n", gpr_0.gen_int2_en);
    printf("gpr_0.self_wake_up_en : %d\n", gpr_0.self_wake_up_en);

    rslt = bma580_get_int_status(&int_status, n_status, &dev);
    bma5_check_rslt("bma580_get_int_status", rslt);

    printf("\nMove the board and keep idle and do the same for 5 to 10 times continuously\n");
    printf("In motion self wake-up state should be normal and in idle self wake-up state should be in low power mode\n");

    while (loop > 0)
    {
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        /* Check for the interrupt status */
        if (int_status.int_status.gen_int1_int_status == BMA5_ENABLE ||
            int_status.int_status.gen_int2_int_status == BMA5_ENABLE ||
            int_status.int_status.self_wake_up_int_status == BMA5_ENABLE)
        {
            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            rslt = bma5_get_acc_conf(&acc_conf, &dev);
            bma5_check_rslt("bma580_get_int_status", rslt);

            printf("\nInterrupt received:\n");

            if (int_status.int_status.gen_int1_int_status == BMA5_ENABLE)
            {
                printf("Generic Interrupt 1\n");
            }

            if (int_status.int_status.gen_int2_int_status == BMA5_ENABLE)
            {
                printf("Generic Interrupt 2\n");
            }

            if (int_status.int_status.self_wake_up_int_status == BMA5_ENABLE)
            {
                printf("Self wake-up\n");
            }

            rslt = bma580_get_feat_eng_gpr_2(&gpr_2, &dev);
            bma5_check_rslt("bma580_get_feat_eng_gpr_2", rslt);

            printf("gen_int1_stat : %d\n", gpr_2.gen_int1_stat);
            printf("gen_int2_stat : %d\n", gpr_2.gen_int2_stat);
            printf("self_wake_up_stat : %d\n", gpr_2.self_wake_up_stat);
            printf("Self Wake Up Mode: %s\n",
                   (gpr_2.self_wake_up_stat ==
                    BMA580_SELF_WAKE_UP_STAT_LOW_POWER) ? "Low Power" : "Normal (High Performance Mode)");

            printf("Accel Config Power Mode: %s\n",
                   (acc_conf.power_mode == BMA580_POWER_MODE_LOW) ? "Low Power" : "High Performance Mode");

            loop--;
        }

        if (loop == 0)
        {
            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
