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
 * @file vl6180x_port.h
 *
 * @brief All end user OS/platform/application porting
 */

/**
 * @defgroup porting  API porting
 * @brief Platform and application specific porting, user has to check or modify the following modules:
 */

/** @defgroup porting_type  Basic Types Definition
 *  @ingroup  porting
 *
 *  @brief  file vl6180x_types.h files contains basic type definition that may require porting.
 *  It contains types that must be defined for the targeted platform.\n
 *  When target platform and compiler provide stdint.h and stddef.h it is enough to include thems.\n
 *  If stdint.h is not available review and adapt all signed and unsigned 8/16/32 bits basic types. \n
 *  If stddef.h is not available review and adapt NULL definition.
 */

/**
 * @defgroup porting_i2c  I2C device register access
 * @ingroup  porting
 *
 * @brief  End user __must provide__ at least @a VL6180x_I2CRead() and @a VL6180x_I2CWrite() basic rd/wr functions. \n
 * Some adaption via macro is needed for multi-threading and for multiple device support .
 *
 * file vl6180x_i2c.c implement device register access via i2c it is not mandatory to review or adapt that file.\n
 * if application and platform has no multi-thread multi-cpu and use single device then nothing else is required than the 2 mandatory function.\n
 * In over case review and customize @a VL6180x_GetI2CAccess() @a VL6180x_DoneI2CAccess() @a #VL6180x_I2C_USER_VAR
 * should be enough to conform to a wide range of platform OS and application requirements .\n
 *
 * if your configured i2c for per device buffer via  @a # I2C_BUFFER_CONFIG ==2 you must implement @a VL6180x_GetI2cBuffer()
 *
 * __I2C Port sample__ \n
 * A __linux kernel__ port need a "long flags" var for its spin_lock in all function \n
 * it will include a spin lock "lock" on its custom device structure \n
 * @code
struct MyVL6180Dev_t {
     struct VL6180xDevData_t StData;
     ...
     spinlock_t i2c_lock;
};
typedef struct MyVL6180Dev_t *VL6180xDev_t;

#define VL6180x_I2C_USER_VAR   unsigned long flags;
#define GetI2CAccess(dev)      spin_lock_irqsave(dev->i2c_lock, flags)
#define DoneI2CAccess(dev)     spin_unlock_irqrestore(dev->i2c_lock,flags)
@endcode

*  __POSIX pthread__ application port could go as :\n
* @code
struct MyVL6180Dev_t {
    struct VL6180xDevData_t StData;
    ...
    pthread_mutex_t *lock;
};
typedef struct MyVL6180Dev_t *VL6180xDev_t;

#define VL6180x_I2C_USER_VAR        //no need
#define VL6180x_GetI2CAccess(dev)   pthread_mutex_lock(dev->lock)
#define VL6180x_DoneI2CAcces(dev)   pthread_mutex_unlock(dev->lock)
 * @endcode
 */


/**
 * @defgroup porting_device  Device Type definition
 * @ingroup  porting
 *
 * @brief   User must provide  ::VL6180xDev_t : @copybrief ::VL6180xDev_t .
 *
 * All API calls and macros do have VL6180xDev_t dev as first argument
 * @code
 * int VL6180x_xxxx(VL6180xDev_t dev, ... )
 * @endcode
 * dev is passed from function to function down to final platform abstraction layer that handles final access . \n
 * In single device case, that can be as simple as an integer being the i2c device address
 *  @snippet vl6180x_port.h  device_type_int
 *
 * For more elaborated platform, it can be a pointer to a structure containing all necessary items for the platform.
 *  @snippet vl6180x_port.h  device_type_multi
 *
 *
 * For Multiple device support refer to specific porting section @ref porting_multi
 */



/**
 * @defgroup porting_multi  Multiple device support
 * @brief  Requirement for multi device support
 *
 * When multiple device support is active end user must manage per device instance @a VL6180xDevData_t data storage and provide data accessing macro/function to it. \n
 * A typical implemention will embed ST data structure in user specific device structure and will define 2 macro for access the internal ST Data member
 *
 * @snippet  vl6180x_port.h device_type_multi
 * @ingroup porting
 */


#ifndef VL6180x_PORT
#define VL6180x_PORT


#include "msm_sensor.h"
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

#include "vl6180x_def.h"


#if VL6180x_SINGLE_DEVICE_DRIVER
/**
 * @typedef  VL6180xDev_t
 * @brief    Generic VL6180x device type that does link between API and platform abstraction layer
 *
 * @ingroup porting_device
 */
    /*! [device_type_int] */
    typedef struct msm_camera_i2c_client *VL6180xDev_t; /* simplest single device example "dev" is the i2c device address in the platform AL*/

/*
 * int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len)
 * {
 *  return Myi2cDev_Write(dev, buff, len);
 * }
 *
 */
    /*! [device_type_int]  */

#else /* VL6180x_SINGLE_DEVICE_DRIVER */

 /*! [device_type_multi] */

struct MyDev_t {
    struct VL6180xDevData_t Data;          /*!< embed ST VL6180 Dev  data as "Data"*/
#if I2C_BUFFER_CONFIG == 2
    uint8_t i2c_buffer[VL6180x_MAX_I2C_XFER_SIZE];  /*!< Example per device i2c buffer declaration and it's accessing macro  */
    #define VL6180x_GetI2cBuffer(dev, n) ((dev)->i2c_buffer)
#endif
    //TODO ADD HERE any extra device data
    /*!< user specific field */
    int     i2c_bus_num;                   /*!< i2c bus number user specific field */
    int     i2c_dev_addr;                  /*!< i2c devcie address user specific field */
    mutex_t dev_lock   ;                   /*!< mutex user specific field */
    int     i2c_file;                      /*!< sample i2c file handle */
};
typedef struct msm_camera_i2c_client *VL6180xDev_t;


#endif /* #else VL6180x_SINGLE_DEVICE_DRIVER */

/**
 * @def VL6180xDevDataGet
 * @brief Get ST private structure @a VL6180xDevData_t data access
 *
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like VL6180xDevDataGet(FilterData.field)[i] or VL6180xDevDataGet(FilterData.MeasurementIndex)++
 * @ingroup porting_multi
 */
//#define VL6180xDevDataGet(dev, field) (dev->Data.field)

/**
 * @def VL6180xDevDataSet(dev, field, data)
 * @brief  Set ST private structure @a VL6180xDevData_t data field
 * @param dev    The device
 * @param field  ST structure field name
 * @param data   Data to be set
 * @ingroup porting_multi
 */
//#define VL6180xDevDataSet(dev, field, data) (dev->Data.field)=(data)
/*! [device_type_multi] */


/**
 * @brief Carry out a delay in all polling API call @a VL6180x_RangePollMeasurement() or @a VL6180x_AlsPollMeasurement()
 *
 * A typical multi-thread or RTO's implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 *
 * @param dev The device
 * @ingroup porting_misc
 */
/* TODO if you implement these function delete the "void" macro below */
void VL6180x_PollDelay(VL6180xDev_t dev);

/** @def VL6180x_PollDelay
 *  @brief Default value: do nothing. Macro to be deleted it you implement a real function
 * @ingroup porting_misc
 */
#define VL6180x_PollDelay(dev)  (void)0


/**
 * @brief       Write data buffer to VL6180x device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup porting_i2c
 */
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len);

/**
 *
 * @brief       Read data buffer from VL6180x device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup  porting_i2c
 */
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len);


/**
 * @brief Declare any required variable for use by i2c lock @a VL6180x_DoneI2CAccess() @a VL6180x_GetI2CAccess()
 *  and buffer access  @a VL6180x_GetI2cBuffer()
 *
 * @ingroup porting_i2c
 */
#define VL6180x_I2C_USER_VAR

/**
 *  @brief Acquire lock or mutex for access to i2c data buffer and bus\n
 *  delete default VL6180x_GetI2CAccess do nothing macro below if implementing that function
 *
 *  These porting function is used to perform i2c bus level and multiple access locking required for multi thread/proccess system\n
 *  multiple access (read and update) will lock once and do multiple basic i2c rd/wr to complete the overall transfer\n
 *  when no locking is needed these can be a void macro\n
 *
 * @param dev  the device
 * @ingroup porting_i2c
 */
void VL6180x_GetI2CAccess(VL6180xDev_t dev);

/**
 * @def VL6180x_GetI2CAccess
 * @brief Default value: do nothing. Macro for @a VL6180x_GetI2CAccess(), delete if used
 * @ingroup porting_i2c
 */
#define VL6180x_GetI2CAccess(dev) (void)0 /* TODO delete if function used */

/**
 * @brief Release acquired lock or mutex for i2c access\n
 * delete default VL6180x_DoneI2CAccess do nothing macro below if implementing that function
 *
 * These porting function is used to release acquire lock when done using the i2c bus and buffer
 * @param dev The device
 * @ingroup porting_i2c
 */
void VL6180x_DoneI2CAccess(VL6180xDev_t dev);

/** @def VL6180x_DoneI2CAcces
 * @brief Default value: do nothing. Macro for @a VL6180x_DoneI2CAcces(), delete if used
 * @ingroup porting_i2c
 */
#define VL6180x_DoneI2CAcces(dev) (void)0  /*TODO delete  if function used */

/**
 * @brief Provided data buffer for i2c access for at least n_byte.
 *
 * User must implement it when i2c @a #I2C_BUFFER_CONFIG is set to per device user provided buffer\n
 * Used in context with #VL6180x_I2C_USER_VAR
 *
 * @param dev     The device
 * @param n_byte  Minimal number of byte
 * @return        The buffer (cannot fail return not checked)
 * @ingroup porting_i2c
 */
uint8_t *VL6180x_GetI2cBuffer(VL6180xDev_t dev, int n_byte);
#if I2C_BUFFER_CONFIG == 2
#error /* TODO add your macro of code here for VL6180x_GetI2cBuffer */
#endif




/**
 * @defgroup porting_misc Miscellaneous
 * @ingroup  porting
 * @brief    A set of macro and types, to be customized at user convenience. It concerns data memory location and core polling loop delay.
 */

/**
 * @brief For user convenience to place or give any required data attribute to the built-in single device instance.
 * Required when Configuration @a #VL6180x_SINGLE_DEVICE_DRIVER is active.
 * @ingroup porting_misc
 */
#define VL6180x_DEV_DATA_ATTR

/**
* @def ROMABLE_DATA
* @brief API Read-Only data that can be place in rom/flash are declared with that extra keyword
*
* For user convenience, use compiler specific attribute or keyword to place all read-only in required data area. \n
* For example, using gcc section  :
*  @code
*  #define ROMABLE_DATA  __attribute__ ((section ("user_rom")))
*  // you may need to edit your link script file to place user_rom section in flash/rom memory
*  @endcode
*
* @ingroup porting_misc
*/
#define ROMABLE_DATA
/*  #define ROMABLE_DATA  __attribute__ ((section ("user_rom"))) */



/**
 * @defgroup porting_log API Log and Trace functions
 * @ingroup  porting
 * @brief    API provided built-in facilities to log and trace function entry/leave.
 * It can generates traces log to help problem tracking analysis and solving. \n
 * It requires porting.
 *
 * It is activated by configuring @a #VL6180X_LOG_ENABLE \n
 * If activated, you must implement the above porting macro or function defined in vl6180x_port.h\n
 */

#if VL6180X_LOG_ENABLE

/**
 * @def LOG_FUNCTION_START
 * @brief Log function start.
 *
 * @param  fmt  Text and printf formating for any extra argument\n
 *              is " " when there is nothing else than the function entry to log.
 * @param  ...  (\__VA_ARGS__) extra argument if any, but it can be none.
 *
 * _Note:_  Formatting a string can take considerable amount of cycles. Typically "sprinting" 4 int argument and function name take 5-10K on common 32 bit ÂµC\n
 * It depends on implementation. The tracing is very intrusive and resources consuming .
 *
 * Example time log using printf and LOG_GET_TIME() macro \n
 * @code
 * #define LOG_FUNCTION_START(fmt, ... )  printf("beg %s @%d\t" fmt "\n", __func__, LOG_GET_TIME(), ##__VA_ARGS__)
 * @endcode
 * @ingroup porting_log
 */
#define LOG_FUNCTION_START(fmt, ... ) TODO add your code here

/**
 * @def LOG_FUNCTION_END
 * @brief  Logging function end with status.
 *
 * Example time log using printf and LOG_GET_TIME() macro \n
 * @code
 * #define LOG_FUNCTION_END(status)   printf("end %s @%d %d\n", __func__, LOG_GET_TIME(), (int)status)
 * @endcode
 * @param  status final status (%d)
 * @ingroup porting_log
 */
#define LOG_FUNCTION_END(status) TODO add your code here

/**
 * @brief  Log function end along with extra optional formated arguments.
 *
 * Example time log using printf and LOG_GET_TIME() macro. \n
 * @code
 * #define LOG_FUNCTION_END_FMT(status, fmt, ... )  printf("End %s @%d %d\t"fmt"\n" , __func__, LOG_GET_TIME(), (int)status,##__VA_ARGS__)
 * @endcode
 * @param  status final status (%d)
 * @param  fmt printf format and string
 * @param  ... (\__VA_ARGS__) extra argument for printf or va_args type function
 * @ingroup porting_log
 */
#define LOG_FUNCTION_END_FMT(status, fmt, ... ) TODO add your code here

/**
 * @def VL6180x_ErrLog(msg, ... )
 * @brief call to log error raised in API
 *
 * implementation may abort execution but it's not an API requirement
 *
 * @param  msg      text and printf like format list
 * @param  ...      (\__VA_ARGS__)    optional variable to be formated
 *
 * Example time log using printf and LOG_GET_TIME() macro \n
 * @code
 * #define VL6180x_ErrLog(msg, ... ) printf("Err in  %s line %d @%dt" msg "\n" , __func__, __LINE__, LOG_GET_TIME(), msg ,##__VA_ARGS__)
 * @endcode
 *
 *
 * @ingroup porting_log
 */
#define VL6180x_ErrLog(msg, ... ) TODO add your code here

#else /* VL6180X_LOG_ENABLE no logging */
    #define LOG_FUNCTION_START(...) (void)0
    #define LOG_FUNCTION_END(...) (void)0
    #define LOG_FUNCTION_END_FMT(...) (void)0
    #define VL6180x_ErrLog(... ) (void)0
#endif

#endif  /* VL6180x_PORT */
