/*******************************************************************************
* Copyright (c)  2014- 2014  Guangdong OPPO Mobile Telecommunications Corp., Ltd
* CONFIG_MACH_OPPO
* Description: Source file for CBufferList.
*           To allocate and free memory block safely.
* Version   : 0.0
* Date      : 2014-07-30
* Author    : Lijiada @Bsp.charge
* ---------------------------------- Revision History: -------------------------
* <version>           <date>          < author >              <desc>
* Revision 0.0        2014-07-30      Lijiada @Bsp.charge
* Modified to be suitable to the new coding rules in all functions.
*******************************************************************************/

#ifndef _OPPO_BQ2022A_H_
#define _OPPO_BQ2022A_H_

#ifdef OPPO_BQ2022A_PAR
#define OPPO_BQ2022A_EXT
#else
#define OPPO_BQ2022A_EXT extern
#endif

OPPO_BQ2022A_EXT bool oppo_battery_status_init(int batt_id_gpio);

#endif /*_OPPO_BQ2022A_H_*/
