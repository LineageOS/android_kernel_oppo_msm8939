/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oppo_boot.h
** CONFIG_MACH_OPPO
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
**
** Description:
**     change define of boot_mode here for other place to use it
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**	<author>	<data>			<desc>
** tong.han@BasicDrv.TP&LCD 11/01/2014 add this file
************************************************************************************/

#ifndef _OPPO_BOOT_H
#define _OPPO_BOOT_H

enum {
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__RECOVERY,
};

int get_boot_mode(void);

#endif
