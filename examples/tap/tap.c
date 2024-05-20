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
    uint8_t n_ints = 1;
    uint8_t n_status = 1;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;
    struct bma580_int_map int_map = { 0 };
    struct bma5_int_conf_types int_config;
    struct bma580_tap_config conf, set_conf;
    struct bma580_feat_eng_gpr_0 gpr_0;
    struct bma580_int_status_types int_status;
    uint8_t s_tap_count = 0, d_tap_count = 0, t_tap_count = 0;
    enum bma5_context context;

    int_config.int_src = BMA5_INT_1;
    int_status.int_src = BMA580_INT_STATUS_INT1;

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

    printf("\nDefault configurations\n\n");
    rslt = bma580_get_default_tap_config(&conf, &dev);
    bma5_check_rslt("bma580_get_default_tap_config", rslt);

    printf("axis_sel 0x%x\n", conf.axis_sel);
    printf("wait_for_timeout 0x%x\n", conf.wait_for_timeout);
    printf("max_peaks_for_tap 0x%x\n", conf.max_peaks_for_tap);
    printf("mode 0x%x\n", conf.mode);
    printf("s_tap_en %d\n", conf.s_tap_en);
    printf("d_tap_en %d\n", conf.d_tap_en);
    printf("t_tap_en %d\n", conf.t_tap_en);
    printf("tap_peak_thres 0x%x\n", conf.tap_peak_thres);
    printf("max_gesture_dur 0x%x\n", conf.max_gesture_dur);
    printf("max_dur_between_peaks 0x%x\n", conf.max_dur_between_peaks);
    printf("tap_shock_settling_dur 0x%x\n", conf.tap_shock_settling_dur);
    printf("min_quite_dur_between_taps 0x%x\n", conf.min_quite_dur_between_taps);
    printf("quite_time_after_gesture 0x%x\n", conf.quite_time_after_gesture);

    set_conf.axis_sel = BMA580_TAP_SEL_AXIS_Z;
    set_conf.wait_for_timeout = 0x1;
    set_conf.max_peaks_for_tap = 0x6;
    set_conf.mode = BMA580_TAP_MODE_NORMAL;
    set_conf.s_tap_en = BMA5_ENABLE;
    set_conf.d_tap_en = BMA5_ENABLE;
    set_conf.t_tap_en = BMA5_ENABLE;
    set_conf.tap_peak_thres = 0x100;
    set_conf.max_gesture_dur = 0x3B;
    set_conf.max_dur_between_peaks = 0x4;
    set_conf.tap_shock_settling_dur = 0x6;
    set_conf.min_quite_dur_between_taps = 0x8;
    set_conf.quite_time_after_gesture = 0x6;

    printf("\nSet configurations\n\n");
    printf("axis_sel 0x%x\n", set_conf.axis_sel);
    printf("wait_for_timeout 0x%x\n", set_conf.wait_for_timeout);
    printf("max_peaks_for_tap 0x%x\n", set_conf.max_peaks_for_tap);
    printf("mode 0x%x\n", set_conf.mode);
    printf("s_tap_en %d\n", set_conf.s_tap_en);
    printf("d_tap_en %d\n", set_conf.d_tap_en);
    printf("t_tap_en %d\n", set_conf.t_tap_en);
    printf("tap_peak_thres 0x%x\n", set_conf.tap_peak_thres);
    printf("max_gesture_dur 0x%x\n", set_conf.max_gesture_dur);
    printf("max_dur_between_peaks 0x%x\n", set_conf.max_dur_between_peaks);
    printf("tap_shock_settling_dur 0x%x\n", set_conf.tap_shock_settling_dur);
    printf("min_quite_dur_between_taps 0x%x\n", set_conf.min_quite_dur_between_taps);
    printf("quite_time_after_gesture 0x%x\n", set_conf.quite_time_after_gesture);

    rslt = bma580_set_tap_config(&set_conf, &dev);
    bma5_check_rslt("bma580_set_tap_config", rslt);

    rslt = bma580_get_tap_config(&conf, &dev);
    bma5_check_rslt("bma580_get_tap_config", rslt);

    printf("\nGet configurations\n\n");
    printf("axis_sel 0x%x\n", conf.axis_sel);
    printf("wait_for_timeout 0x%x\n", conf.wait_for_timeout);
    printf("max_peaks_for_tap 0x%x\n", conf.max_peaks_for_tap);
    printf("mode 0x%x\n", conf.mode);
    printf("s_tap_en %d\n", conf.s_tap_en);
    printf("d_tap_en %d\n", conf.d_tap_en);
    printf("t_tap_en %d\n", conf.t_tap_en);
    printf("tap_peak_thres 0x%x\n", conf.tap_peak_thres);
    printf("max_gesture_dur 0x%x\n", conf.max_gesture_dur);
    printf("max_dur_between_peaks 0x%x\n", conf.max_dur_between_peaks);
    printf("tap_shock_settling_dur 0x%x\n", conf.tap_shock_settling_dur);
    printf("min_quite_dur_between_taps 0x%x\n", conf.min_quite_dur_between_taps);
    printf("quite_time_after_gesture 0x%x\n", conf.quite_time_after_gesture);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    gpr_0.tap_en = BMA5_ENABLE;

    rslt = bma580_set_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_set_feat_eng_gpr_0", rslt);

    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, &dev);
    bma5_check_rslt("bma5_set_regs", rslt);

    rslt = bma580_get_feat_eng_gpr_0(&gpr_0, &dev);
    bma5_check_rslt("bma580_get_feat_eng_gpr_0", rslt);

    printf("\n\ngpr_0.tap_en : %d\n", gpr_0.tap_en);

    /* Map single tap */
    rslt = bma580_get_int_map(&int_map, &dev);
    bma5_check_rslt("bma580_get_int_map", rslt);

    int_map.stap_int_map = BMA580_STAP_INT_MAP_INT1;
    int_map.dtap_int_map = BMA580_DTAP_INT_MAP_INT1;
    int_map.ttap_int_map = BMA580_TTAP_INT_MAP_INT1;

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

    printf("\n\nTap the board to detect single, double, triple taps\n");

    for (;;)
    {
        rslt = bma580_get_int_status(&int_status, n_status, &dev);
        bma5_check_rslt("bma580_get_int_status", rslt);

        if (int_status.int_status.stap_int_status & BMA5_ENABLE)
        {
            printf("Single Tap interrupt occurred\n");

            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            s_tap_count++;
        }

        if (int_status.int_status.dtap_int_status & BMA5_ENABLE)
        {
            printf("Double Tap interrupt occurred\n");

            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            d_tap_count++;
        }

        if (int_status.int_status.ttap_int_status & BMA5_ENABLE)
        {
            printf("Triple Tap interrupt occurred\n");

            rslt = bma580_set_int_status(&int_status, n_status, &dev);
            bma5_check_rslt("bma580_set_int_status", rslt);

            t_tap_count++;
        }

        if ((s_tap_count > 0) && (d_tap_count > 0) && (t_tap_count > 0))
        {
            break;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
