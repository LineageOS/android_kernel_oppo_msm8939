/*
 * ak4375.c  --  audio driver for AK4375
 *
 * Copyright (C) 2014 Asahi Kasei Microdevices Corporation
 *  Author                Date        Revision
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      14/06/25	    1.1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <soc/oppo/oppo_project.h>
#include <linux/of_gpio.h>


#include "ak4375.h"

//#define AK4375_DEBUG			//used at debug mode
//#define AK4375_CONTIF_DEBUG		//used at debug mode

#ifdef AK4375_DEBUG
#define akdbgprt printk
#else
#define akdbgprt(format, arg...) do {} while (0)
#endif

/* AK4375 Codec Private Data */
struct ak4375_priv {
	struct snd_soc_codec codec;
	u8 reg_cache[AK4375_MAX_REGISTERS];
	int fs1;
	int fs2;
	int rclk;			//Master Clock
	int nSeldain;		//0:Bypass, 1:SRC	(Dependent on a register bit)
	int nBickFreq;		//0:32fs, 1:48fs, 2:64fs
	int nSrcOutFsSel;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
	int nPllMode;		//0:PLL OFF, 1: PLL ON
	int nPllMCKI;		//0:PLL not use, 1: PLL use
	int nSmt;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
	int dfsrc8fs;		//DFTHR bit and SRCO8FS bit
	/*OPPO 2014-08-22 zhzhyon Add for reason*/
	int audio_vdd_en_gpio;
	int ak4375_reset_gpio;
	/*OPPO 2014-08-22 zhzhyon Add end*/
};

static struct snd_soc_codec *ak4375_codec;
static struct ak4375_priv *ak4375_data;

/* ak4375 register cache & default register settings */
static const u8 ak4375_reg[AK4375_MAX_REGISTERS] = {
	0x00,	/*	0x00	AK4375_00_POWER_MANAGEMENT1			*/
	0x00,	/*	0x01	AK4375_01_POWER_MANAGEMENT2			*/
	0x00,	/*	0x02	AK4375_02_POWER_MANAGEMENT3			*/
	0x00,	/*	0x03	AK4375_03_POWER_MANAGEMENT4			*/
	0x00,	/*	0x04	AK4375_04_OUTPUT_MODE_SETTING		*/
	0x00,	/*	0x05	AK4375_05_CLOCK_MODE_SELECT			*/
	0x00,	/*	0x06	AK4375_06_DIGITAL_FILTER_SELECT		*/
	0x00,	/*	0x07	AK4375_07_DAC_MONO_MIXING			*/
	0x00,	/*	0x08	AK4375_08_JITTER_CLEANER_SETTING1	*/
	0x00,	/*	0x09	AK4375_09_JITTER_CLEANER_SETTING2	*/
	0x00,	/*	0x0A	AK4375_0A_JITTER_CLEANER_SETTING3	*/
	0x19,	/*	0x0B	AK4375_0B_LCH_OUTPUT_VOLUME			*/
	0x19,	/*	0x0C	AK4375_0C_RCH_OUTPUT_VOLUME			*/
	0x75,	/*	0x0D	AK4375_0D_HP_VOLUME_CONTROL			*/
	0x01,	/*	0x0E	AK4375_0E_PLL_CLK_SOURCE_SELECT		*/
	0x00,	/*	0x0F	AK4375_0F_PLL_REF_CLK_DIVIDER1		*/
	0x00,	/*	0x10	AK4375_10_PLL_REF_CLK_DIVIDER2		*/
	0x00,	/*	0x11	AK4375_11_PLL_FB_CLK_DIVIDER1		*/
	0x00,	/*	0x12	AK4375_12_PLL_FB_CLK_DIVIDER2		*/
	0x00,	/*	0x13	AK4375_13_SRC_CLK_SOURCE			*/
	0x00,	/*	0x14	AK4375_14_DAC_CLK_DIVIDER			*/
	0x00,	/*	0x15	AK4375_15_AUDIO_IF_FORMAT			*/
	0x00,	/*	0x16	AK4375_16_DUMMY						*/
	0x00,	/*	0x17	AK4375_17_DUMMY						*/
	0x00,	/*	0x18	AK4375_18_DUMMY						*/
	0x00,	/*	0x19	AK4375_19_DUMMY						*/
	0x00,	/*	0x1A	AK4375_1A_DUMMY						*/
	0x00,	/*	0x1B	AK4375_1B_DUMMY						*/
	0x00,	/*	0x1C	AK4375_1C_DUMMY						*/
	0x00,	/*	0x1D	AK4375_1D_DUMMY						*/
	0x00,	/*	0x1E	AK4375_1E_DUMMY						*/
	0x00,	/*	0x1F	AK4375_1F_DUMMY						*/
	0x00,	/*	0x20	AK4375_20_DUMMY						*/
	0x00,	/*	0x21	AK4375_21_DUMMY						*/
	0x00,	/*	0x22	AK4375_22_DUMMY						*/
	0x00,	/*	0x23	AK4375_23_DUMMY						*/
	0x00,	/*	0x24	AK4375_24_MODE_CONTROL				*/
};

static const struct {
	int readable;   /* Mask of readable bits */
	int writable;   /* Mask of writable bits */
} ak4375_access_masks[] = {
    { 0xFF, 0xFF },	//0x00
    { 0xFF, 0xFF },	//0x01
    { 0xFF, 0xFF },	//0x02
    { 0xFF, 0xFF },	//0x03
    { 0xFF, 0xFF },	//0x04
    { 0xFF, 0xFF },	//0x05
    { 0xFF, 0xFF },	//0x06
    { 0xFF, 0xFF },	//0x07
    { 0xFF, 0xFF },	//0x08
    { 0xFF, 0xFF },	//0x09
    { 0xFF, 0xFF },	//0x0A
    { 0xFF, 0xFF },	//0x0B
    { 0xFF, 0xFF },	//0x0C
    { 0xFF, 0xFF },	//0x0D
    { 0xFF, 0xFF },	//0x0E
    { 0xFF, 0xFF },	//0x0F
    { 0xFF, 0xFF },	//0x10
    { 0xFF, 0xFF },	//0x11
    { 0xFF, 0xFF },	//0x12
    { 0xFF, 0xFF },	//0x13
    { 0xFF, 0xFF },	//0x14
    { 0xFF, 0xFF },	//0x15
    { 0x00, 0x00 },	//0x16	//DUMMY
    { 0x00, 0x00 },	//0x17	//DUMMY
    { 0x00, 0x00 },	//0x18	//DUMMY
    { 0x00, 0x00 },	//0x19	//DUMMY
    { 0x00, 0x00 },	//0x1A	//DUMMY
    { 0x00, 0x00 },	//0x1B	//DUMMY
    { 0x00, 0x00 },	//0x1C	//DUMMY
    { 0x00, 0x00 },	//0x1D	//DUMMY
    { 0x00, 0x00 },	//0x1E	//DUMMY
    { 0x00, 0x00 },	//0x1F	//DUMMY
    { 0x00, 0x00 },	//0x20	//DUMMY
    { 0x00, 0x00 },	//0x21	//DUMMY
    { 0x00, 0x00 },	//0x22	//DUMMY
    { 0x00, 0x00 },	//0x23	//DUMMY
    { 0xFF, 0xFF },	//0x24
};

/* Output Digital volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB) */
static DECLARE_TLV_DB_SCALE(ovl_tlv, -1250, 50, 0);
static DECLARE_TLV_DB_SCALE(ovr_tlv, -1250, 50, 0);

/* HP-Amp Analog volume control:
 * from -42 to 6 dB in 2 dB steps (mute instead of -42 dB) */
static DECLARE_TLV_DB_SCALE(hpg_tlv, -4200, 20, 0);

static const char *ak4375_ovolcn_select_texts[] = {"Dependent", "Independent"};
static const char *ak4375_mdacl_select_texts[] = {"x1", "x1/2"};
static const char *ak4375_mdacr_select_texts[] = {"x1", "x1/2"};
static const char *ak4375_invl_select_texts[] = {"Normal", "Inverting"};
static const char *ak4375_invr_select_texts[] = {"Normal", "Inverting"};
static const char *ak4375_cpmod_select_texts[] =
		{"Automatic Switching", "+-VDD Operation", "+-1/2VDD Operation"};
static const char *ak4375_hphl_select_texts[] = {"9ohm", "200kohm"};
static const char *ak4375_hphr_select_texts[] = {"9ohm", "200kohm"};
static const char *ak4375_dacfil_select_texts[]  =
		{"Sharp Roll-Off", "Slow Roll-Off",
					"Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};
static const char *ak4375_srcfil_select_texts[] =
		{"Sharp Roll-Off", "Slow Roll-Off",
					"Short Delay Sharp Roll-Off", "Short Delay Slow Roll-Off"};

static const struct soc_enum ak4375_dac_enum[] = {
	SOC_ENUM_SINGLE(AK4375_0B_LCH_OUTPUT_VOLUME, 7,
			ARRAY_SIZE(ak4375_ovolcn_select_texts), ak4375_ovolcn_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 2,
			ARRAY_SIZE(ak4375_mdacl_select_texts), ak4375_mdacl_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 6,
			ARRAY_SIZE(ak4375_mdacr_select_texts), ak4375_mdacr_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 3,
			ARRAY_SIZE(ak4375_invl_select_texts), ak4375_invl_select_texts),
	SOC_ENUM_SINGLE(AK4375_07_DAC_MONO_MIXING, 7,
			ARRAY_SIZE(ak4375_invr_select_texts), ak4375_invr_select_texts),
	SOC_ENUM_SINGLE(AK4375_03_POWER_MANAGEMENT4, 2,
			ARRAY_SIZE(ak4375_cpmod_select_texts), ak4375_cpmod_select_texts),
	SOC_ENUM_SINGLE(AK4375_04_OUTPUT_MODE_SETTING, 0,
			ARRAY_SIZE(ak4375_hphl_select_texts), ak4375_hphl_select_texts),
	SOC_ENUM_SINGLE(AK4375_04_OUTPUT_MODE_SETTING, 1,
			ARRAY_SIZE(ak4375_hphr_select_texts), ak4375_hphr_select_texts),
    SOC_ENUM_SINGLE(AK4375_06_DIGITAL_FILTER_SELECT, 6,
		ARRAY_SIZE(ak4375_dacfil_select_texts), ak4375_dacfil_select_texts),
	SOC_ENUM_SINGLE(AK4375_09_JITTER_CLEANER_SETTING2, 4,
			ARRAY_SIZE(ak4375_srcfil_select_texts), ak4375_srcfil_select_texts),
};

static const char *bickfreq_on_select[] = {"32fs", "48fs", "64fs"};

static const char *srcoutfs_on_select[] =
#ifdef SRC_OUT_FS_48K
	{"48kHz", "96kHz", "192kHz"};
#else
	{"44.1kHz", "88.2kHz", "176.4kHz"};
#endif

static const char *pllmode_on_select[] = {"OFF", "ON"};
static const char *smtcycle_on_select[] = {"1024", "2048", "4096", "8192"};
static const char *dfsrc8fs_on_select[] = {"Digital Filter", "Bypass", "8fs mode"};

static const struct soc_enum ak4375_bitset_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bickfreq_on_select), bickfreq_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(srcoutfs_on_select), srcoutfs_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(pllmode_on_select), pllmode_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smtcycle_on_select), smtcycle_on_select),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dfsrc8fs_on_select), dfsrc8fs_on_select),
};


static int ak4375_writeMask(struct snd_soc_codec *, u16, u16, u16);
static inline u32 ak4375_read_reg_cache(struct snd_soc_codec *, u16);

static int get_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4375->nBickFreq;

    return 0;
}

static int ak4375_set_bickfs(struct snd_soc_codec *codec)
{
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

	if ( ak4375->nBickFreq == 0 ) {		//32fs
		ak4375_writeMask(codec, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
	}
	else if( ak4375->nBickFreq == 1 ) {	//48fs
		ak4375_writeMask(codec, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x00);	//DL1-0=00(24bit, >=48fs)
	}
	else if( ak4375->nBickFreq == 2 ) {								//64fs
		ak4375_writeMask(codec, AK4375_15_AUDIO_IF_FORMAT, 0x02, 0x02);	//DL1-0=1x(32bit, >=64fs)
	}

	return 0;
}

static int set_bickfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

	ak4375->nBickFreq = ucontrol->value.enumerated.item[0];

	ak4375_set_bickfs(codec);

    return 0;
}

static int get_srcfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4375->nSrcOutFsSel;

    return 0;
}

static int set_srcfs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

	ak4375->nSrcOutFsSel = ucontrol->value.enumerated.item[0];

    return 0;
}

//static int get_smtcycle(
//struct snd_kcontrol       *kcontrol,
//struct snd_ctl_elem_value  *ucontrol)
//{
//    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
//	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);
//
//    ucontrol->value.enumerated.item[0] = ak4375->nSmt;
//
//    return 0;
//}
//
//static int set_smtcycle(
//struct snd_kcontrol       *kcontrol,
//struct snd_ctl_elem_value  *ucontrol)
//{
//    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
//	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);
//
//	ak4375->nSmt = ucontrol->value.enumerated.item[0];
//
//	//0:1024, 1:2048, 2:4096, 3:8192
//	ak4375_writeMask(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x0C, ((ak4375->nSmt) << 2));
//
//    return 0;
//}

static int get_dfsrc8fs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

    ucontrol->value.enumerated.item[0] = ak4375->dfsrc8fs;

    return 0;
}

static int ak4375_set_dfsrc8fs(struct snd_soc_codec *codec)
{
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

	switch (ak4375->dfsrc8fs) {
	case 0:		//DAC Filter
		ak4375_writeMask(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x00);	//DFTHR=0
		ak4375_writeMask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00); //SRCO8FS=0
		break;
	case 1:		//Bypass
		ak4375_writeMask(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x08);	//DFTHR=1
		ak4375_writeMask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x00); //SRCO8FS=0
		break;
	case 2:		//8fs mode
		ak4375_writeMask(codec, AK4375_06_DIGITAL_FILTER_SELECT, 0x08, 0x08);	//DFTHR=1
		ak4375_writeMask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0x20, 0x20); //SRCO8FS=1
		break;
	}
    return 0;
}

static int set_dfsrc8fs(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);

	ak4375->dfsrc8fs = ucontrol->value.enumerated.item[0];

	ak4375_set_dfsrc8fs(codec);

	return 0;
}

#ifdef AK4375_DEBUG

static const char *test_reg_select[]   =
{
    "read AK4375 Reg 00:24",
};

static const struct soc_enum ak4375_enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(test_reg_select), test_reg_select),
};

static int nTestRegNo = 0;

static int get_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    /* Get the current output routing */
    ucontrol->value.enumerated.item[0] = nTestRegNo;

    return 0;
}

static int set_test_reg(
struct snd_kcontrol       *kcontrol,
struct snd_ctl_elem_value  *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
    u32    currMode = ucontrol->value.enumerated.item[0];
	int    i, value;
	int	   regs, rege;

	nTestRegNo = currMode;

	regs = 0x00;
	rege = 0x15;

	for ( i = regs ; i <= rege ; i++ ){
		value = snd_soc_read(codec, i);
		printk("***AK4375 Addr,Reg=(%x, %x)\n", i, value);
	}
	value = snd_soc_read(codec, 0x24);
	printk("***AK4375 Addr,Reg=(%x, %x)\n", 0x24, value);

	return 0;
}
#endif

static const struct snd_kcontrol_new ak4375_snd_controls[] = {
	SOC_SINGLE_TLV("AK4375 Digital Output VolumeL",
			AK4375_0B_LCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovl_tlv),
	SOC_SINGLE_TLV("AK4375 Digital Output VolumeR",
			AK4375_0C_RCH_OUTPUT_VOLUME, 0, 0x1F, 0, ovr_tlv),
	SOC_SINGLE_TLV("AK4375 HP-Amp Analog Volume",
			AK4375_0D_HP_VOLUME_CONTROL, 0, 0x1F, 0, hpg_tlv),

	SOC_ENUM("AK4375 Digital Volume Control", ak4375_dac_enum[0]),
	SOC_ENUM("AK4375 DACL Signal Level", ak4375_dac_enum[1]),
	SOC_ENUM("AK4375 DACR Signal Level", ak4375_dac_enum[2]),
	SOC_ENUM("AK4375 DACL Signal Invert", ak4375_dac_enum[3]),
	SOC_ENUM("AK4375 DACR Signal Invert", ak4375_dac_enum[4]),
	SOC_ENUM("AK4375 Charge Pump Mode", ak4375_dac_enum[5]),
	SOC_ENUM("AK4375 HPL Power-down Resistor", ak4375_dac_enum[6]),
	SOC_ENUM("AK4375 HPR Power-down Resistor", ak4375_dac_enum[7]),
	SOC_ENUM("AK4375 DAC Digital Filter Mode", ak4375_dac_enum[8]),
	SOC_ENUM("AK4375 SRC Digital Filter Mode", ak4375_dac_enum[9]),

	SOC_ENUM_EXT("AK4375 Data Output mode", ak4375_bitset_enum[4], get_dfsrc8fs, set_dfsrc8fs),
	SOC_ENUM_EXT("AK4375 BICK Frequency Select", ak4375_bitset_enum[0], get_bickfs, set_bickfs),
	SOC_ENUM_EXT("AK4375 SRC Output FS", ak4375_bitset_enum[1], get_srcfs, set_srcfs),
//	SOC_ENUM_EXT("AK4375 Soft Mute Cycle Select", ak4375_bitset_enum[3], get_smtcycle, set_smtcycle),

	SOC_SINGLE("AK4375 SRC Semi-Auto Mode", AK4375_09_JITTER_CLEANER_SETTING2, 1, 1, 0),
	SOC_SINGLE("AK4375 SRC Dither", AK4375_0A_JITTER_CLEANER_SETTING3, 4, 1, 0),
	SOC_SINGLE("AK4375 Soft Mute Control", AK4375_09_JITTER_CLEANER_SETTING2, 0, 1, 0),

#ifdef AK4375_DEBUG
	SOC_ENUM_EXT("Reg Read", ak4375_enum[0], get_test_reg, set_test_reg),
#endif

};
/*OPPO 2014-08-22 zhzhyon Delete for reason*/
#if 0
static int ak4375_set_PLL_MCKI(struct snd_soc_codec *codec, int pll)
{
	int PLDbit, PLMbit, MDIVbit, DIVbit;
	int nTemp;

	if (pll) {	//PLL use
		PLDbit = 8 - 1;
		PLMbit = 40 - 1;
		MDIVbit = 1 - 1;
		DIVbit = 1;

		//PLD15-0
		snd_soc_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
		snd_soc_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
		//PLM15-0
		snd_soc_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
		snd_soc_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
		//DIVbit
		nTemp = snd_soc_read(codec, AK4375_13_SRC_CLK_SOURCE);
		nTemp &= ~0x10;
		nTemp |= ( DIVbit << 4 );
		snd_soc_write(codec, AK4375_13_SRC_CLK_SOURCE, nTemp);
		//MDIV7-0
		snd_soc_write(codec, AK4375_14_DAC_CLK_DIVIDER, MDIVbit);
		//PLL=ON
		ak4375_writeMask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x01);	//PMPLL=1
		ak4375_writeMask(codec, AK4375_13_SRC_CLK_SOURCE, 0x01, 0x01);		//SRCCKS=1
		ak4375_data->nPllMCKI=1;
	}
	else {		//PLL not use
		ak4375_writeMask(codec, AK4375_13_SRC_CLK_SOURCE, 0x01, 0x00);		//SRCCKS=0
		ak4375_writeMask(codec, AK4375_00_POWER_MANAGEMENT1, 0x01, 0x00);	//PMPLL=0
		ak4375_data->nPllMCKI=0;
	}

	return 0;
}
#endif
/*OPPO 2014-08-22 zhzhyon Delete end*/

/* DAC control */
static int ak4375_dac_event2(struct snd_soc_codec *codec, int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:	/* before widget power up */
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x01);	//PMCP1=1
		mdelay(6);															//wait 6ms
		udelay(500);														//wait 0.5ms
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x30);	//PMLDO1P/N=1
		mdelay(1);															//wait 1ms
		break;
	case SND_SOC_DAPM_POST_PMU:	/* after widget power up */
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x02);	//PMCP2=1
		mdelay(4);															//wait 4ms
		udelay(500);														//wait 0.5ms
		break;
	case SND_SOC_DAPM_PRE_PMD:	/* before widget power down */
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x02,0x00);	//PMCP2=0
		break;
	case SND_SOC_DAPM_POST_PMD:	/* after widget power down */
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x30,0x00);	//PMLDO1P/N=0
		ak4375_writeMask(codec, AK4375_01_POWER_MANAGEMENT2, 0x01,0x00);	//PMCP1=0
//		if (ak4375_data->nPllMode==1) ak4375_set_PLL_MCKI(codec, 0);
		break;
	}
	return 0;
}

static int ak4375_dac_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event) //CONFIG_LINF
{
	struct snd_soc_codec *codec = w->codec;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4375_dac_event2(codec, event);

	return 0;
}

/* DAC MUX */
static const char *ak4375_seldain_select_texts[] =
		{"SDTI", "SRC"};

static const struct soc_enum ak4375_seldain_mux_enum =
	SOC_ENUM_SINGLE(AK4375_0A_JITTER_CLEANER_SETTING3, 1,
			ARRAY_SIZE(ak4375_seldain_select_texts), ak4375_seldain_select_texts);

static const struct snd_kcontrol_new ak4375_seldain_mux_control =
	SOC_DAPM_ENUM("SRC Select", ak4375_seldain_mux_enum);

/* HPL Mixer */
static const struct snd_kcontrol_new ak4375_hpl_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACL", AK4375_07_DAC_MONO_MIXING, 0, 1, 0),
	SOC_DAPM_SINGLE("RDACL", AK4375_07_DAC_MONO_MIXING, 1, 1, 0),
};

/* HPR Mixer */
static const struct snd_kcontrol_new ak4375_hpr_mixer_controls[] = {
	SOC_DAPM_SINGLE("LDACR", AK4375_07_DAC_MONO_MIXING, 4, 1, 0),
	SOC_DAPM_SINGLE("RDACR", AK4375_07_DAC_MONO_MIXING, 5, 1, 0),
};


/* ak4375 dapm widgets */
static const struct snd_soc_dapm_widget ak4375_dapm_widgets[] = {
// DAC
	SND_SOC_DAPM_DAC_E("AK4375 DAC", "NULL", AK4375_02_POWER_MANAGEMENT3, 0, 0,
			ak4375_dac_event, (SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD
                            |SND_SOC_DAPM_PRE_PMU |SND_SOC_DAPM_POST_PMD)),

#ifdef PLL_BICK_MODE
	SND_SOC_DAPM_SUPPLY("AK4375 PLL", AK4375_00_POWER_MANAGEMENT1, 0, 0, NULL, 0),
#endif
	SND_SOC_DAPM_SUPPLY("AK4375 OSC", AK4375_00_POWER_MANAGEMENT1, 4, 0, NULL, 0),

	SND_SOC_DAPM_MUX("AK4375 DAC MUX", SND_SOC_NOPM, 0, 0, &ak4375_seldain_mux_control),

	SND_SOC_DAPM_AIF_IN("AK4375 SRC", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AK4375 SDTI", "Playback", 0, SND_SOC_NOPM, 0, 0),

// Analog Output
	SND_SOC_DAPM_OUTPUT("AK4375 HPL"),
	SND_SOC_DAPM_OUTPUT("AK4375 HPR"),

	SND_SOC_DAPM_MIXER("AK4375 HPR Mixer", AK4375_03_POWER_MANAGEMENT4, 1, 0,
			&ak4375_hpr_mixer_controls[0], ARRAY_SIZE(ak4375_hpr_mixer_controls)),

	SND_SOC_DAPM_MIXER("AK4375 HPL Mixer", AK4375_03_POWER_MANAGEMENT4, 0, 0,
			&ak4375_hpl_mixer_controls[0], ARRAY_SIZE(ak4375_hpl_mixer_controls)),

};

static const struct snd_soc_dapm_route ak4375_intercon[] =
{

#ifdef PLL_BICK_MODE
	{"AK4375 DAC", "NULL", "AK4375 PLL"},
#endif

	{"AK4375 SRC", "NULL", "AK4375 OSC"},
	{"AK4375 DAC MUX", "SRC", "AK4375 SRC"},
	{"AK4375 DAC MUX", "SDTI", "AK4375 SDTI"},
	{"AK4375 DAC", "NULL", "AK4375 DAC MUX"},

	{"AK4375 HPL Mixer", "LDACL", "AK4375 DAC"},
	{"AK4375 HPL Mixer", "RDACL", "AK4375 DAC"},
	{"AK4375 HPR Mixer", "LDACR", "AK4375 DAC"},
	{"AK4375 HPR Mixer", "RDACR", "AK4375 DAC"},

	{"AK4375 HPL", "NULL", "AK4375 HPL Mixer"},
	{"AK4375 HPR", "NULL", "AK4375 HPR Mixer"},

};

static int ak4375_set_mcki(struct snd_soc_codec *codec, int fs, int rclk)
{
	u8 mode;
	u8 mode2;
	int mcki_rate;

	akdbgprt("\t[AK4375] %s fs=%d rclk=%d\n",__FUNCTION__, fs, rclk);

	if ((fs != 0)&&(rclk != 0)) {
		if (rclk > 28800000) return -EINVAL;
		mcki_rate = rclk/fs;

		mode = snd_soc_read(codec, AK4375_05_CLOCK_MODE_SELECT);
		mode &= ~AK4375_CM;

		if (ak4375_data->nSeldain == 0) {	//SRC Bypass Mode
			switch (mcki_rate) {
			case 128:
				mode |= AK4375_CM_3;
				break;
			case 256:
				mode |= AK4375_CM_0;
				mode2 = snd_soc_read(codec, AK4375_24_MODE_CONTROL);
				if ( fs <= 12000 ) {
					mode2 &= 0x40;	//DSMLP=1
					snd_soc_write(codec, AK4375_24_MODE_CONTROL, mode2);
				}
				else {
					mode2 &= ~0x40;	//DSMLP=0
					snd_soc_write(codec, AK4375_24_MODE_CONTROL, mode2);
				}
				break;
			case 512:
				mode |= AK4375_CM_1;
				break;
			case 1024:
				mode |= AK4375_CM_2;
				break;
			default:
				return -EINVAL;
			}
		}
		else {								//SRC Mode
			switch (mcki_rate) {
			case 256:
				mode |= AK4375_CM_0;
				break;
			case 512:
				mode |= AK4375_CM_1;
				break;
			case 1024:
				mode |= AK4375_CM_2;
				break;
//			case 128:
//				mode |= AK4375_CM_0;
//				if (fs <= 96000) return -EINVAL;
//				ak4375_set_PLL_MCKI(codec, 1);
//				break;
			default:
				return -EINVAL;
			}
		}
		snd_soc_write(codec, AK4375_05_CLOCK_MODE_SELECT, mode);

	}

	return 0;
}

static int ak4375_set_src_mcki(struct snd_soc_codec *codec, int fs)
{
	u8 nrate;
	int oclk_rate;
	int src_out_fs;

	nrate = snd_soc_read(codec, AK4375_08_JITTER_CLEANER_SETTING1);
	nrate &= ~0x7F;	//CM21-0 bits, FS24-0 bits

#ifdef SRC_OUT_FS_48K
	src_out_fs = 48000 * ( 1 << (ak4375_data->nSrcOutFsSel));
#else
	src_out_fs = 44100 * ( 1 << (ak4375_data->nSrcOutFsSel));
#endif
	switch (src_out_fs) {
	case 44100:
		nrate |= AK4375_FS_44_1KHZ;
		break;
	case 48000:
		nrate |= AK4375_FS_48KHZ;
		break;
	case 88200:
		nrate |= AK4375_FS_88_2KHZ;
		break;
	case 96000:
		nrate |= AK4375_FS_96KHZ;
		break;
	case 176400:
		nrate |= AK4375_FS_176_4KHZ;
		break;
	case 192000:
		nrate |= AK4375_FS_192KHZ;
		break;
	default:
		return -EINVAL;
	}

	oclk_rate = XTAL_OSC_FS/src_out_fs;
	switch (oclk_rate) {
	case 128:
		nrate |= AK4375_CM_3;
		break;
	case 256:
		nrate |= AK4375_CM_0;
		break;
	case 512:
		nrate |= AK4375_CM_1;
		break;
	case 1024:
		nrate |= AK4375_CM_2;
		break;
	default:
		return -EINVAL;
	}

	ak4375_data->fs2 = src_out_fs;
	snd_soc_write(codec, AK4375_08_JITTER_CLEANER_SETTING1, nrate);

	return 0;
}

static int ak4375_set_pllblock(struct snd_soc_codec *codec, int fs)
{
	u8 mode;
	int nMClk, nPLLClk, nRefClk;
	int PLDbit, PLMbit, MDIVbit, DIVbit;
	int nTemp;

	mode = snd_soc_read(codec, AK4375_05_CLOCK_MODE_SELECT);
	mode &= ~AK4375_CM;

	if (ak4375_data->nSeldain == 0)
	{	//SRC bypass
		if ( fs <= 24000 )
		{
			mode |= AK4375_CM_1;
			nMClk = 512 * fs;
		}
		else if ( fs <= 96000 )
		{
			mode |= AK4375_CM_0;
			nMClk = 256 * fs;
		}
		else
		{
			mode |= AK4375_CM_3;
			nMClk = 128 * fs;
		}
	}
	else
	{
		//SRC
		if ( fs <= 24000 )
		{
			mode |= AK4375_CM_1;
			nMClk = 512 * fs;
		}
		else  {
			mode |= AK4375_CM_0;
			nMClk = 256 * fs;
		}
	}
	snd_soc_write(codec, AK4375_05_CLOCK_MODE_SELECT, mode);

	if ( (fs % 8000) == 0 )
	{
		nPLLClk = 122880000;
	}
	else if ( (fs == 11025 ) && ( ak4375_data->nBickFreq == 1 ))
	{
		nPLLClk = 101606400;
	}
	else
	{
		nPLLClk = 112896000;
	}

	if ( ak4375_data->nBickFreq == 0 )
	{		//32fs
		if ( fs <= 96000 ) PLDbit = 1;
		else PLDbit = 2;
		nRefClk = 32 * fs / PLDbit;
	}
	else if ( ak4375_data->nBickFreq == 1 )
	{	//48fs
		if ( fs <= 16000 ) PLDbit = 1;
		else PLDbit = 3;
		nRefClk = 48 * fs / PLDbit;
	}
	else
	{									// 64fs
		if ( fs <= 48000 ) PLDbit = 1;
		else if ( fs <= 96000 ) PLDbit = 2;
		else PLDbit = 4;
		nRefClk = 64 * fs / PLDbit;
	}

	PLMbit = nPLLClk / nRefClk;

	if ( ( ak4375_data->nSeldain == 0 ) || ( fs <= 96000 ) )
	{
		MDIVbit = nPLLClk / nMClk;
		DIVbit = 0;
	}
	else
	{
		MDIVbit = 5;
		DIVbit = 1;
	}

	PLDbit--;
	PLMbit--;
	MDIVbit--;

	/*OPPO 2015-07-15 qiujianfeng enable for dynamic set*/
	//PLD15-0
	snd_soc_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, ((PLDbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, ((PLDbit & 0x00FF) >> 0));
	//PLM15-0
	snd_soc_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, ((PLMbit & 0xFF00) >> 8));
	snd_soc_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, ((PLMbit & 0x00FF) >> 0));
	/*OPPO 2015-07-15 qiujianfeng enable for dynamic set end*/

	//DIVbit
	nTemp = snd_soc_read(codec, AK4375_13_SRC_CLK_SOURCE);
	nTemp &= ~0x10;
	nTemp |= ( DIVbit << 4 );
	snd_soc_write(codec, AK4375_13_SRC_CLK_SOURCE, (nTemp|0x01));		//DIV=0or1,SRCCKS=1(SRC Clock Select=PLL) set
	snd_soc_write(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x01);	//PLS=1(BCLK)//2015-07-15 qiujianfeng add for set BCLK
/*OPPO 2015-07-15 qiujianfeng delete for dynamic set*/
#if 0
	snd_soc_write(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x00);	//PLS=0(MCLK)//zhzhyon mark
	snd_soc_write(codec, AK4375_0F_PLL_REF_CLK_DIVIDER1, 0x00);
	snd_soc_write(codec, AK4375_10_PLL_REF_CLK_DIVIDER2, 0x04);
	snd_soc_write(codec, AK4375_11_PLL_FB_CLK_DIVIDER1, 0x00);
	snd_soc_write(codec, AK4375_12_PLL_FB_CLK_DIVIDER2, 0x3F);
#endif
/*OPPO 2015-07-15 qiujianfeng delete for dynamic set end*/

	//MDIV7-0
	snd_soc_write(codec, AK4375_14_DAC_CLK_DIVIDER, MDIVbit);

	return 0;
}

static int ak4375_set_timer(struct snd_soc_codec *codec)
{
	int ret, curdata;
	int count, tm, nfs;
	int lvdtm, vddtm, hptm;

	lvdtm = 0;
	vddtm = 0;
	hptm = 0;

	if ( ak4375_data->nSeldain == 1 ) nfs = ak4375_data->fs2;
	else	nfs = ak4375_data->fs1;

	//LVDTM2-0 bits set
	ret = snd_soc_read(codec, AK4375_03_POWER_MANAGEMENT4);
	curdata = (ret & 0x70) >> 4;	//Current data Save
	ret &= ~0x70;
	do {
       count = 1000 * (64 << lvdtm);
       tm = count / nfs;
       if ( tm > LVDTM_HOLD_TIME ) break;
       lvdtm++;
    } while ( lvdtm < 7 );			//LVDTM2-0 = 0~7
	if ( curdata != lvdtm) {
			snd_soc_write(codec, AK4375_03_POWER_MANAGEMENT4, (ret | (lvdtm << 4)));
	}

	//VDDTM3-0 bits set
	ret = snd_soc_read(codec, AK4375_04_OUTPUT_MODE_SETTING);
	curdata = (ret & 0x3C) >> 2;	//Current data Save
	ret &= ~0x3C;
	do {
       count = 1000 * (1024 << vddtm);
       tm = count / nfs;
       if ( tm > VDDTM_HOLD_TIME ) break;
       vddtm++;
    } while ( vddtm < 8 );			//VDDTM3-0 = 0~8
	if ( curdata != vddtm) {
			snd_soc_write(codec, AK4375_04_OUTPUT_MODE_SETTING, (ret | (vddtm<<2)));
	}

	//HPTM2-0 bits set
	ret = snd_soc_read(codec, AK4375_0D_HP_VOLUME_CONTROL);
	curdata = (ret & 0xE0) >> 5;	//Current data Save
	ret &= ~0xE0;
	do {
       count = 1000 * (128 << hptm);
       tm = count / nfs;
       if ( tm > HPTM_HOLD_TIME ) break;
       hptm++;
    } while ( hptm < 4 );			//HPTM2-0 = 0~4
	if ( curdata != hptm) {
			snd_soc_write(codec, AK4375_0D_HP_VOLUME_CONTROL, (ret | (hptm<<5)));
	}

	return 0;
}

static int ak4375_hw_params_set(struct snd_soc_codec *codec, int nfs1)
{
	u8	fs;
	u8  src;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	src = snd_soc_read(codec, AK4375_0A_JITTER_CLEANER_SETTING3);
	src = (src & 0x02) >> 1;

	fs = snd_soc_read(codec, AK4375_05_CLOCK_MODE_SELECT);
	fs &= ~AK4375_FS;

//	ak4375_data->fs1 = params_rate(params);

	switch (nfs1) {
	case 8000:
		fs |= AK4375_FS_8KHZ;
		break;
	case 11025:
		fs |= AK4375_FS_11_025KHZ;
		break;
	case 16000:
		fs |= AK4375_FS_16KHZ;
		break;
	case 22050:
		fs |= AK4375_FS_22_05KHZ;
		break;
	case 32000:
		fs |= AK4375_FS_32KHZ;
		break;
	case 44100:
		fs |= AK4375_FS_44_1KHZ;
		break;
	case 48000:
		fs |= AK4375_FS_48KHZ;
		break;
	case 88200:
		fs |= AK4375_FS_88_2KHZ;
		break;
	case 96000:
		fs |= AK4375_FS_96KHZ;
		break;
	case 176400:
		fs |= AK4375_FS_176_4KHZ;
		break;
	case 192000:
		fs |= AK4375_FS_192KHZ;
		break;
	default:
		return -EINVAL;
	}
	snd_soc_write(codec, AK4375_05_CLOCK_MODE_SELECT, fs);

	if ( ak4375_data->nPllMode == 0 )
	{	//Not PLL mode
		ak4375_set_mcki(codec, nfs1, ak4375_data->rclk);
	}
	else
	{								//PLL mode
		ak4375_set_pllblock(codec, nfs1);
	}

	if ( src == 1 ) {				//SRC mode
		ak4375_data->nSeldain = 1;
		ak4375_writeMask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0xC2);	//XCKSEL=XCKCPSEL=SELDAIN=1
		ak4375_set_src_mcki(codec, nfs1);
	}
	else {							//SRC Bypass mode
		ak4375_data->nSeldain = 0;
		ak4375_writeMask(codec, AK4375_0A_JITTER_CLEANER_SETTING3, 0xC2, 0x00);	//XCKSEL=XCKCPSEL=SELDAIN=0
	}

	ak4375_set_timer(codec);

	return 0;
}

static int ak4375_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	ak4375_data->fs1 = params_rate(params);

	ak4375_hw_params_set(codec, ak4375_data->fs1);

	return 0;
}

static int ak4375_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4375_data->rclk = freq;

	if ( ak4375_data->nPllMode == 0 ) {	//Not PLL mode
		ak4375_set_mcki(codec, ak4375_data->fs1, freq);
	}

	return 0;
}

static int ak4375_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = dai->codec;
	u8 format;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	/* set master/slave audio interface */
	format = snd_soc_read(codec, AK4375_15_AUDIO_IF_FORMAT);
	format &= ~AK4375_DIF;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= AK4375_DIF_I2S_MODE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= AK4375_DIF_MSB_MODE;
		break;
	default:
		return -EINVAL;
	}

	/* set format */
	snd_soc_write(codec, AK4375_15_AUDIO_IF_FORMAT, format);

	return 0;
}

static int ak4375_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	int	ret;

	switch (reg) {
//		case :
//			ret = 1;
		default:
			ret = 0;
			break;
	}
	return ret;
}

static int ak4375_readable(struct snd_soc_codec *codec, unsigned int reg)
{

	if (reg >= ARRAY_SIZE(ak4375_access_masks))
		return 0;
	return ak4375_access_masks[reg].readable != 0;
}

/*
* Read ak4375 register cache
 */
static inline u32 ak4375_read_reg_cache(struct snd_soc_codec *codec, u16 reg)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4375_reg));
    return (u32)cache[reg];
}

#ifdef AK4375_CONTIF_DEBUG
/*
 * Write ak4375 register cache
 */
static inline void ak4375_write_reg_cache(
struct snd_soc_codec *codec,
u16 reg,
u16 value)
{
    u8 *cache = codec->reg_cache;
    BUG_ON(reg > ARRAY_SIZE(ak4375_reg));
    cache[reg] = (u8)value;
}

unsigned int ak4375_i2c_read(struct snd_soc_codec *codec, unsigned int reg)
{

	int ret;

	if ( reg == AK4375_16_DUMMY ) { // Dummy Register.
		ret = ak4375_read_reg_cache(codec, reg);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(codec->control_data, (u8)(reg & 0xFF));
//	ret = ak4375_read_reg_cache(codec, reg);

	if (ret < 0) {
		akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int ak4375_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	ak4375_write_reg_cache(codec, reg, value);

	akdbgprt("\t[ak4375] %s: (addr,data)=(%x, %x)\n",__FUNCTION__, reg, value);

	if ( reg == AK4375_16_DUMMY ) return 0;  // Dummy Register.

	if(i2c_smbus_write_byte_data(codec->control_data, (u8)(reg & 0xFF), (u8)(value & 0xFF))<0) {
		akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);
		return EIO;
	}

	return 0;
}
#endif

/*
 * Write with Mask to  AK4375 register space
 */
static int ak4375_writeMask(
struct snd_soc_codec *codec,
u16 reg,
u16 mask,
u16 value)
{
    u16 olddata;
    u16 newdata;

	if ( (mask == 0) || (mask == 0xFF) ) {
		newdata = value;
	}
	else {
		olddata = ak4375_read_reg_cache(codec, reg);
	    newdata = (olddata & ~(mask)) | value;
	}

	snd_soc_write(codec, (unsigned int)reg, (unsigned int)newdata);

	akdbgprt("\t[ak4375_writeMask] %s(%d): (addr,data)=(%x, %x)\n",__FUNCTION__,__LINE__, reg, newdata);

    return 0;
}

// * for AK4375
static int ak4375_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *codec_dai)
{
	int	ret = 0;
 //   struct snd_soc_codec *codec = codec_dai->codec;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	return ret;
}


static int ak4375_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int ak4375_set_dai_mute2(struct snd_soc_codec *codec, int mute)
{
	int ret = 0;
	int nfs, ndt, ndt2;

	if ( ak4375_data->nSeldain == 1 ) nfs = ak4375_data->fs2;
	else	nfs = ak4375_data->fs1;

	akdbgprt("\t[AK4375] %s mute[%s]\n",__FUNCTION__, mute ? "ON":"OFF");


	if (mute) {	//SMUTE: 1 , MUTE
		if (ak4375_data->nSeldain) {
			ret = snd_soc_update_bits(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x01, 0x01);
			ndt = (1024000 << ak4375_data->nSmt) / nfs;
			mdelay(ndt);
			ret = snd_soc_update_bits(codec, AK4375_02_POWER_MANAGEMENT3, 0x80, 0x00);
		}
	}
	else {		// SMUTE: 0 ,NORMAL operation
		ak4375_data->nSmt = (ak4375_data->nSrcOutFsSel + SMUTE_TIME_MODE);
		ret = snd_soc_update_bits(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x0C, (ak4375_data->nSmt << 2));
		ndt = (26 * nfs) / 44100;		//for After HP-Amp Power up
		if (ak4375_data->nSeldain) {
			ret = snd_soc_update_bits(codec, AK4375_02_POWER_MANAGEMENT3, 0x80, 0x80);
			ndt2 = (1024000 << ak4375_data->nSmt) / nfs;
			ndt -= ndt2;
			if (ndt < 4) ndt=4;
			mdelay(ndt);
			ret =snd_soc_update_bits(codec, AK4375_09_JITTER_CLEANER_SETTING2, 0x01, 0x00);
			mdelay(ndt2);
		}
		else {
			mdelay(ndt);
		}
	}
	return ret;
}

static int ak4375_set_dai_mute(struct snd_soc_dai *dai, int mute)
{
    struct snd_soc_codec *codec = dai->codec;

	ak4375_set_dai_mute2(codec, mute);

	return 0;
}

#define AK4375_RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
				SNDRV_PCM_RATE_192000)

#define AK4375_FORMATS		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE


static struct snd_soc_dai_ops ak4375_dai_ops = {
	.hw_params	= ak4375_hw_params,
	.set_sysclk	= ak4375_set_dai_sysclk,
	.set_fmt	= ak4375_set_dai_fmt,
	.trigger = ak4375_trigger,
	.digital_mute = ak4375_set_dai_mute,
};

struct snd_soc_dai_driver ak4375_dai[] = {
	{
		.name = "ak4375-AIF1",
		.playback = {
		       .stream_name = "Playback",
		       .channels_min = 1,
		       .channels_max = 2,
		       .rates = AK4375_RATES,
		       .formats = AK4375_FORMATS,
		},
		.ops = &ak4375_dai_ops,
	},
};
/*OPPO 2014-08-22 zhzhyon Delete for reason*/
#if 0
static int ak4375_write_cache_reg(
struct snd_soc_codec *codec,
u16  regs,
u16  rege)
{
	u32	reg, cache_data;

	reg = regs;
	do {
		cache_data = ak4375_read_reg_cache(codec, reg);
		snd_soc_write(codec, (unsigned int)reg, (unsigned int)cache_data);
		reg ++;
	} while (reg <= rege);

	return 0;
}
#endif
/*OPPO 2014-08-22 zhzhyon Delete end*/

static int ak4375_init_reg(struct snd_soc_codec *codec)
{
	udelay(800);
	ak4375_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	akdbgprt("\t[AK4375 bias] %s(%d)\n",__FUNCTION__,__LINE__);

#ifdef PLL_BICK_MODE
		ak4375_writeMask(codec, AK4375_13_SRC_CLK_SOURCE, 0x01, 0x01);			//SRCCKS=1(SRC Clock Select=PLL)
		ak4375_writeMask(codec, AK4375_0E_PLL_CLK_SOURCE_SELECT, 0x01, 0x01);	//PLS=1(BICK)
#endif

	return 0;
}

static int ak4375_probe(struct snd_soc_codec *codec)
{
	struct ak4375_priv *ak4375 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

#ifdef AK4375_CONTIF_DEBUG
	codec->write = ak4375_i2c_write;
	codec->read = ak4375_i2c_read;
#endif
	ak4375_codec = codec;

//	akdbgprt("\t[AK4375] %s(%d) ak4375=%x\n",__FUNCTION__,__LINE__, (int)ak4375);

	ak4375_init_reg(codec);

	akdbgprt("\t[AK4375 Effect] %s(%d)\n",__FUNCTION__,__LINE__);

	//snd_soc_add_controls(codec, ak4375_snd_controls,
	                    //ARRAY_SIZE(ak4375_snd_controls));

	/*OPPO 2014-08-22 zhzhyon Add for ak4375*/
	snd_soc_add_codec_controls(codec, ak4375_snd_controls,
	                    ARRAY_SIZE(ak4375_snd_controls));

	/*OPPO 2014-08-22 zhzhyon Add end*/

	ak4375->fs1 = 48000;
	ak4375->fs2 = 48000;
	ak4375->rclk = 0;
	ak4375->nSeldain = 0;		//0:Bypass, 1:SRC	(Dependent on a register bit)
	ak4375->nBickFreq = 0;		//0:32fs, 1:48fs, 2:64fs
	//
	ak4375_writeMask(codec, AK4375_15_AUDIO_IF_FORMAT, 0x03, 0x01);	//DL1-0=01(16bit, >=32fs)
	//
	ak4375->nSrcOutFsSel = 0;	//0:48(44.1)kHz, 1:96(88.2)kHz, 2:192(176.4)kHz
#ifdef PLL_BICK_MODE
	ak4375->nPllMode = 1;		//1: PLL ON
#else
	ak4375->nPllMode = 0;		//0:PLL OFF
#endif
	ak4375->nSmt = 0;			//0:1024/FSO, 1:2048/FSO, 2:4096/FSO, 3:8192/FSO
	ak4375->dfsrc8fs = 0;		//0:DAC Filter, 1:Bypass, 2:8fs mode
	return ret;
}

static int ak4375_remove(struct snd_soc_codec *codec)
{

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);

	ak4375_set_bias_level(codec, SND_SOC_BIAS_OFF);


	return 0;
}

static int ak4375_suspend(struct snd_soc_codec *codec)
{
	ak4375_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ak4375_resume(struct snd_soc_codec *codec)
{

	ak4375_init_reg(codec);

	return 0;
}


struct snd_soc_codec_driver soc_codec_dev_ak4375 = {
	.probe = ak4375_probe,
	.remove = ak4375_remove,
	.suspend =	ak4375_suspend,
	.resume =	ak4375_resume,

	.set_bias_level = ak4375_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ak4375_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ak4375_reg,
	.readable_register = ak4375_readable,
	.volatile_register = ak4375_volatile,
	.dapm_widgets = ak4375_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4375_dapm_widgets),
	.dapm_routes = ak4375_intercon,
	.num_dapm_routes = ARRAY_SIZE(ak4375_intercon),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_ak4375);

static int ak4375_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	struct ak4375_priv *ak4375;
	struct snd_soc_codec *codec;
	int ret=0;

	akdbgprt("\t[AK4375] %s(%d)\n",__FUNCTION__,__LINE__);


	ak4375 = kzalloc(sizeof(struct ak4375_priv), GFP_KERNEL);
	if (ak4375 == NULL) return -ENOMEM;

	/*OPPO 2014-08-22 zhzhyon Add for reason*/
	ak4375->audio_vdd_en_gpio = of_get_named_gpio(i2c->dev.of_node,
					"audio-vdd-enable-gpios", 0);
	if (ak4375->audio_vdd_en_gpio < 0)
	{
		dev_err(&i2c->dev,
			"property %s in node %s not found %d\n",
			"audio-vdd-enable-gpios", i2c->dev.of_node->full_name,
			ak4375->audio_vdd_en_gpio);
	}

	if (gpio_is_valid(ak4375->audio_vdd_en_gpio))
	{
		gpio_request(ak4375->audio_vdd_en_gpio,"audio_vdd_enable");
		pr_err("zhzhyon:set ak4375 vdd gpio\n");
		gpio_direction_output(ak4375->audio_vdd_en_gpio, 1);
	}
	/*OPPO 2014-08-22 zhzhyon Add end*/

	/*OPPO 2014-08-22 zhzhyon Add for reason*/
	usleep(800);
	ak4375->ak4375_reset_gpio = of_get_named_gpio(i2c->dev.of_node,
					"ak4375-reset-gpios", 0);
	if (ak4375->ak4375_reset_gpio < 0)
	{
		dev_err(&i2c->dev,
			"property %s in node %s not found %d\n",
			"audio-vdd-enable-gpios", i2c->dev.of_node->full_name,
			ak4375->ak4375_reset_gpio);
	}

	if (gpio_is_valid(ak4375->ak4375_reset_gpio))
	{
		gpio_request(ak4375->ak4375_reset_gpio,"ak4375_reset_gpio");
		pr_err("zhzhyon:set ak4375_reset_gpio\n");
		gpio_direction_output(ak4375->ak4375_reset_gpio, 1);
	}
	/*OPPO 2014-08-22 zhzhyon Add end*/



	codec = &ak4375->codec;
	i2c_set_clientdata(i2c, ak4375);
	codec->control_data = i2c;
	ak4375_data = ak4375;

	codec->dev = &i2c->dev;
	snd_soc_codec_set_drvdata(codec, ak4375);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_ak4375, &ak4375_dai[0], ARRAY_SIZE(ak4375_dai));
	if (ret < 0){
		kfree(ak4375);
		akdbgprt("\t[AK4375 Error!] %s(%d)\n",__FUNCTION__,__LINE__);
	}
	return ret;
}

static int  ak4375_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ak4375_i2c_id[] = {
	{ "ak4375", 0 },
	{ }
};
/*OPPO 2014-07-26 zhzhyon Add for i2c*/
#ifdef CONFIG_OF
static struct of_device_id ak4375_match_tbl[] = {
	{ .compatible = "ak4375" },
	{ },
};
#endif
/*OPPO 2014-07-26 zhzhyon Add end*/
MODULE_DEVICE_TABLE(i2c, ak4375_i2c_id);

static struct i2c_driver ak4375_i2c_driver = {
	.driver = {
		.name = "ak4375",
		.owner = THIS_MODULE,
		/*OPPO 2014-07-26 zhzhyon Add for i2c*/
		.of_match_table = of_match_ptr(ak4375_match_tbl),
		/*OPPO 2014-07-26 zhzhyon Add end*/

	},
	.probe = ak4375_i2c_probe,
	.remove = ak4375_i2c_remove,
	.id_table = ak4375_i2c_id,
};

static int __init ak4375_modinit(void)
{

	akdbgprt("\t[AK4375] %s(%d)\n", __FUNCTION__,__LINE__);
	/*OPPO 2014-08-22 zhzhyon Add for reason*/
	if((is_project(OPPO_15018)) || (is_project(OPPO_15011)) || (is_project(OPPO_15022)))
	{
		return i2c_add_driver(&ak4375_i2c_driver);
	}
	else
	{
		pr_err("The project is not 14045,don't register ak4375\n");
		return 0;
	}
	/*OPPO 2014-08-22 zhzhyon Add end*/
}

module_init(ak4375_modinit);

static void __exit ak4375_exit(void)
{
	/*OPPO 2014-08-22 zhzhyon Add for reason*/
	if((is_project(OPPO_15018)) || (is_project(OPPO_15011)) || (is_project(OPPO_15022)))
	{
		i2c_del_driver(&ak4375_i2c_driver);
	}
	else
	{
		pr_err("The project is not 14045,don't consider ak4375\n");
	}
	/*OPPO 2014-08-22 zhzhyon Add end*/
}
module_exit(ak4375_exit);

MODULE_DESCRIPTION("ASoC ak4375 codec driver");
MODULE_LICENSE("GPL");
