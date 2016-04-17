/*******************************************************************************
################################################################################
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License version 2 and only version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
********************************************************************************/

/**
 * @file VL6180x_cfg.h
 */

#ifndef VL6180x_CFG_H_
#define VL6180x_CFG_H_

/**
 * @defgroup Configuration  API Application Configuration
 * @ingroup porting
 *
 * @brief File vl6180x_cfg.h holds static implementation options and features activation.
 * It also contains per product recommended features support.
 *
 * End user can change \p #define configuration values to suits  application and platform needs.\n
 * Do not delete #define, instead set value 0 that stands for "disable" for most options.
 */


/**
 * @brief  Select The product in used
 *
 * Only one of PRODUCT_VL6180 or PRODUCT_VL6180X must be defined at a time
 * @ingroup   Configuration
 */
#define PRODUCT_VL6180
/* #define PRODUCT_VL6180X */


#if defined(PRODUCT_VL6180)

/** @ingroup Configuration
 * @{*/


/**
 * @def VL6180x_UPSCALE_SUPPORT
 * @brief Configure up-scale support and default up-scale factor @a #VL6180x_EXTENDED_RANGE
 *
 * @li __1__ fixed scaling by 1 (no up-scaling support)
 * @li __2__ fixed scaling by 2
 * @li __3__ fixed scaling by 3
 * @li  \b -1 \b -2 \b -3 run time programmable \n
 *  @note initial starting scaling factor is -VL6180x_UPSCALE_SUPPORT
 */
#define VL6180x_UPSCALE_SUPPORT -3

/**
 * @def VL6180x_ALS_SUPPORT
 * @brief Enable the support for ALS operation
 *
 * Set __0__ if ALS is not used \n
 * It can help reduce code size if it is a concern.
 */
#define VL6180x_ALS_SUPPORT      0

/**
 * @def VL6180x_HAVE_DMAX_RANGING
 * @brief Enable support of DMax type ranging functionality.
 */
#define VL6180x_HAVE_DMAX_RANGING   1

/**
 * @def VL6180x_WRAP_AROUND_FILTER_SUPPORT
 * @brief Support for wrap around filter.
 *
 * @li __0__  Filter is not supported, no filtering code is included related API is minimal.
 * @li __1__  Filter is supported and active by default.
 * @li @b -1 Filter is supported but is not active by default @a VL6180x_FilterSetState(). Can turn it ON or OFF at any time.
 */
#define VL6180x_WRAP_AROUND_FILTER_SUPPORT   1

/**
 * @def VL6180x_EXTENDED_RANGE
 * @brief Enable extended ranging support
 *
 * Device that do not formally supports extended ranging should only be used with a scaling factor of 1.
 * Correct operation with scaling factor other than 1 (>200mm ) is not granted.
 */
#define VL6180x_EXTENDED_RANGE 1

/** @}*/
#elif defined(PRODUCT_VL6180X)
/* VL6180X Product recommended operation  */
  #define VL6180x_ALS_SUPPORT  1
  #define VL6180x_UPSCALE_SUPPORT 1
  #define VL6180x_HAVE_DMAX_RANGING 1
  #define VL6180x_WRAP_AROUND_FILTER_SUPPORT 0
  #define VL6180x_EXTENDED_RANGE 0
#endif /* #elif defined(PRODUCT_VL6180X) */

#if (VL6180x_UPSCALE_SUPPORT!=1) && (VL6180x_EXTENDED_RANGE==0)
#warning "Never device that do not formally support extended ranging should be used with a scaling factor different of 1"
#endif

#if (VL6180x_EXTENDED_RANGE) && (VL6180x_ALS_SUPPORT)
#warning "Als support should be OFF for extended range"
#endif

/**
 * @def VL6180x_SINGLE_DEVICE_DRIVER
 * @brief Enable lightweight single vl6180x device driver.
 *
 * Value __1__ =>  Single device capable.
 * Configure optimized API for single device driver with static data and minimal use of ref pointer. \n
 *				Limited to single device driver or application in non multi thread/core environment. \n
 *
 * Value __0__ =>  Multiple device capable. User must review "device" structure and type in porting files.
 * @ingroup Configuration
 */
#define VL6180x_SINGLE_DEVICE_DRIVER 1

/**
 * @def I2C_BUFFER_CONFIG
 *
 * @brief Configure  low level device register to I2C access translation required buffer implementation.
 *
 * __0__ On GLOBAL buffer	Use one global buffer of MAX_I2C_XFER_SIZE byte in data space. \n
 *				This solution is not multi-device compliant nor multi-thread cpu safe. \n
 *				It can be the best option for small 8/16 bit MCU without stack and limited ram  (STM8s, 80C51 ...).
 *
 *  __1__ ON_STACK/local    Use local variable (on stack) buffer \n
 *			   This solution is multi-thread with use of i2c resource lock or mutex see @a VL6180x_GetI2CAccess().
 *
 * __2__ User defined\n		Per device potentially dynamic allocated requires @a VL6180x_GetI2cBuffer() user porting.
 * @ingroup Configuration
 */
#define I2C_BUFFER_CONFIG 1

/**
 * @def VL6180x_RANGE_STATUS_ERRSTRING
 * @brief When define include range status Error string and related
 *
 * The string table lookup require some space in read only area.
 * @ingroup Configuration
 */
#define VL6180x_RANGE_STATUS_ERRSTRING  1

/**
 * @def VL6180X_SAFE_POLLING_ENTER
 *
 * @brief Ensure safe polling method when set
 *
 * Polling for a condition can be hazardous and result in infinite looping if any previous interrupt status
 * condition is not cleared. \n
 * Setting these flags enforce error clearing on start of polling method to avoid it.
 * the drawback are : \n
 * @li Extra use-less I2C bus usage and traffic.
 * @li Potentially slower measure rate.
 * If application ensures interrupt get clear on mode or interrupt configuration changes then keep option disabled. \n
 * To be safe, set this option to \b 1
 * @ingroup Configuration
 */
#define VL6180X_SAFE_POLLING_ENTER  0


/**
 * @brief Enable function start/end logging
 *
 * Requires porting  @a #LOG_FUNCTION_START @a #LOG_FUNCTION_END @a #LOG_FUNCTION_END_FMT
 * @ingroup Configuration
 */
#define VL6180X_LOG_ENABLE  0



#endif

/* VL6180x_CFG_H_ */
