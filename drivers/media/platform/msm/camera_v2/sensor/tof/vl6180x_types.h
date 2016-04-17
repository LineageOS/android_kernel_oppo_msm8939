/*******************************************************************************
################################################################################
#                             (C) STMicroelectronics 2014
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
 * @file  vl6180x_types.h
 * @brief vl6180x types definition
 */

#ifndef VL6180x_TYPES_H_
#define VL6180x_TYPES_H_


//#include <stdint.h>
#include <stddef.h>
#include "msm_sd.h"
#ifndef NULL
#error "TODO review  NULL definition or add required include "
#define NULL 0
#endif

#if 0
#if ! defined(STDINT_H) &&  !defined(_GCC_STDINT_H) &&!defined(__STDINT_DECLS) && !defined(_GCC_WRAP_STDINT_H)

#pragma message("Please review  type definition of STDINT define for your platform and add to list above ")

 /*
  *  target platform do not provide stdint or use a different #define than above
  *  to avoid seeing the message below addapt the #define list above or implement
  *  all type and delete these pragma
  */

/** @ingroup porting_type
 * @{
 */

/** @brief Typedef defining 32 bit unsigned int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned int uint32_t;

/** @brief Typedef defining 32 bit int type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef int int32_t;

/** @brief Typedef defining 16 bit unsigned short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned short uint16_t;

/** @brief Typedef defining 16 bit short type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef short int16_t;

/** @brief Typedef defining 8 bit unsigned char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef unsigned char uint8_t;

/** @brief Typedef defining 8 bit char type.\n
 * The developer should modify this to suit the platform being deployed.
 */
typedef signed char int8_t;

/** @}  */
#endif /* _STDINT_H */
#endif
#endif /* VL6180x_TYPES_H_ */
