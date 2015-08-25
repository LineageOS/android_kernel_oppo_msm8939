#ifndef __LINUX_KEYBOARD_H
#define __LINUX_KEYBOARD_H

#include <uapi/linux/keyboard.h>
#ifdef VENDOR_EDIT
//Modify by Tong.han@Bsp.Group.Tp for increase input array,2015-4-18
#include <linux/input.h>
#endif/*VENDOR_EDIT*/

struct notifier_block;
extern unsigned short *key_maps[MAX_NR_KEYMAPS];
#ifndef VENDOR_EDIT
//Modify by Tong.han@Bsp.Group.Tp for increase input array,2015-4-18
extern unsigned short plain_map[NR_KEYS];
#else
extern unsigned short plain_map[KEY_CNT];
#endif
struct keyboard_notifier_param {
	struct vc_data *vc;	/* VC on which the keyboard press was done */
	int down;		/* Pressure of the key? */
	int shift;		/* Current shift mask */
	int ledstate;		/* Current led state */
	unsigned int value;	/* keycode, unicode value or keysym */
};

extern int register_keyboard_notifier(struct notifier_block *nb);
extern int unregister_keyboard_notifier(struct notifier_block *nb);
#endif
