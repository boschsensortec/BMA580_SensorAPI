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
    uint8_t loop = 0;
    struct bma5_temp_conf config;
    struct bma5_sensor_status status;
    uint8_t temperature = 0;
    int8_t temp_celsius = 0;
    uint8_t limit = 50;

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

    /* Get temperature config */
    rslt = bma5_get_temp_conf(&config, &dev);
    bma5_check_rslt("bma5_get_temp_conf", rslt);

    config.temp_rate = BMA5_TEMP_RATE_HZ_25;
    config.temp_meas_src = BMA5_TEMP_MEAS_SRC_TMP_INT;
    config.temp_ext_sel = BMA5_TEMP_EXT_SEL_INT2;

    /* Set temperature config */
    rslt = bma5_set_temp_conf(&config, &dev);
    bma5_check_rslt("bma5_set_temp_conf", rslt);

    printf("\nTemp Configurations\n");
    printf("Temp Rate: %s\n", enum_to_string(BMA5_TEMP_RATE_HZ_25));
    printf("Temp Meas Src: %s\n", enum_to_string(BMA5_TEMP_MEAS_SRC_TMP_INT));
    printf("Temp Ext Sel: %s\n", enum_to_string(BMA5_TEMP_EXT_SEL_INT2));

    printf("\nCount, Temp\n");

    while (loop < limit)
    {
        /* Get temperature data ready status */
        rslt = bma5_get_sensor_status(&status, &dev);
        bma5_check_rslt("bma5_get_sensor_status", rslt);

        /* Check if temperature data is ready */
        if (status.temperature_rdy & BMA5_ENABLE)
        {
            rslt = bma5_set_sensor_status(&status, &dev);
            bma5_check_rslt("bma5_set_sensor_status", rslt);

            /* Get temperature data */
            rslt = bma5_get_temp_data(&temperature, &dev);
            bma5_check_rslt("bma5_get_temp_data", rslt);

            if (rslt == BMA5_OK)
            {
                temp_celsius = (int8_t)(temperature + 23);
            }

            printf("%d \t%d(Deg C)\n", loop, temp_celsius);

            loop++;
        }
    }

    bma5_coines_deinit();

    return rslt;
}
