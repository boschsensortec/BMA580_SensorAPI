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

#ifndef _COMMON_H
#define _COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "bma5.h"

/******************************************************************************/
/*!                       Macro definitions                                   */

/*! Enum to string converter*/
#ifndef enum_to_string
#define enum_to_string(a)  #a
#endif

/***************************************************************************/

/*!                 User function prototypes
 ****************************************************************************/

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[out] reg_data     : Pointer to the data buffer to store the read data.
 *  @param[in] length        : No of bytes to read.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA5_INTF_RET_SUCCESS -> Fail.
 *
 */
BMA5_INTF_RET_TYPE bma5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[out] reg_data     : Pointer to the data buffer to store the read data.
 *  @param[in] length        : No of bytes to read.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA5_INTF_RET_SUCCESS -> Fail.
 *
 */
BMA5_INTF_RET_TYPE bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[in] reg_data      : Pointer to the data buffer whose data has to be written.
 *  @param[in] length        : No of bytes to write.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval  != BMA5_INTF_RET_SUCCESS -> Fail.
 *
 */
BMA5_INTF_RET_TYPE bma5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr      : Register address.
 *  @param[in] reg_data      : Pointer to the data buffer whose value is to be written.
 *  @param[in] length        : No of bytes to write.
 *  @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA5_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA5_INTF_RET_SUCCESS -> Failure.
 *
 */
BMA5_INTF_RET_TYPE bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us      : The required wait time in microsecond.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *  @return void.
 *
 */
void bma5_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *         Also to initialize coines platform.
 *
 *  @param[in] bma5     : Structure instance of bma5_dev
 *  @param[in] intf     : Interface selection parameter
 *  @param[in] context  : Context parameter selection
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bma5_interface_init(struct bma5_dev *bma5, uint8_t intf, enum bma5_context context);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bma5_check_rslt(const char api_name[], int8_t rslt);

/*!
 * @brief This function deinitializes coines platform
 *
 *  @return void.
 *
 */
void bma5_coines_deinit(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _COMMON_H */
