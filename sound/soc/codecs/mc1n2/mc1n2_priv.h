/*
 * MC-1N2 ASoC codec driver - private header
 *
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef MC1N2_PRIV_H
#define MC1N2_PRIV_H

#include "mcdriver.h"

#if 1 // Add for Venturi
#ifdef CONFIG_VENTURI_KOR // Only KOR, Enable analog volume feature.
#define CONFIG_MUSIC_CODEC_GAIN
#endif
#define CONFIG_FMRADIO_CODEC_GAIN
#define CONFIG_VOIP
#define CODEC_DEBUG
#define MC1N2_LOAD_PRD_FOR_SPL
#endif

#ifdef CONFIG_MUSIC_CODEC_GAIN
#define MUSIC_DEFAUTL_VOL_LEVEL 15
#define MUSIC_VOL_LEVEL 31
#endif

#ifdef CONFIG_FMRADIO_CODEC_GAIN
#define FM_VOL_LEVEL 31

unsigned int McDrv_Ctrl_set_fm_vol(unsigned int volume);
unsigned char McDrv_Ctrl_get_fm_vol(void);
unsigned int McDrv_Ctrl_fm_mute(void);
#else
#define FM_FIXED_GAIN 0x0f01
#endif

#ifdef CONFIG_VOIP
int mc1n2_get_voip_status(void);
void mc1n2_start_voip(void);
void mc1n2_end_voip(void);
void mc1n2_set_voip_playback_parameters(int micpath);
void mc1n2_set_voip_mic_parameters(int path);
#endif

/*
 * Virtual registers
 */
enum {
	MC1N2_RCV_VOL_L = 0,
	MC1N2_RCV_VOL_R,
	MC1N2_SPEAKER_VOL_L,
	MC1N2_SPEAKER_VOL_R,
	MC1N2_HEADPHONE_VOL_L,
	MC1N2_HEADPHONE_VOL_R,
	MC1N2_ADC_VOL_L,
	MC1N2_ADC_VOL_R,
	MC1N2_MIC_ADC_VOL,
	MC1N2_N_REG,
	MC1N2_MIC1_GAIN,
	MC1N2_MIC2_GAIN,
	MC1N2_DAC_VOL_L,
	MC1N2_DAC_MASTER,
	MC1N2_DAC_DAT_VAL,
};

/*
 * Path settings
 */
enum {
	PLAYBACK_OFF = 0,
	RCV,
	SPK,
	HP,
	BT,
	DUAL,
	RING_SPK,
	RING_HP,
	RING_DUAL,
	EXTRA_DOCK_SPEAKER,
	TV_OUT
};

enum {
	MIC_MAIN = 0,
	MIC_SUB,
	MIC_BT
};

enum {
	FMR_OFF = 0,
	FMR_SPK,
	FMR_HP,
	FMR_SPK_MIX,
	FMR_HP_MIX,
	FMR_DUAL_MIX
};


#define CMD_FMR_INPUT_DEACTIVE		0  /* Codec Input PGA off */
#define CMD_FMR_INPUT_ACTIVE		1  /* Codec Input PGA on */
#define CMD_FMR_FLAG_CLEAR		2  /* Radio flag clear for shutdown */
#define CMD_FMR_END			3  /* Codec off in FM radio mode */
#define CMD_CALL_FLAG_CLEAR		4  /* Call flag clear for shutdown */
#define CMD_CALL_END			5  /* Codec off in call mode */
#define CMD_RECOGNITION_DEACTIVE	6  /* Voice recognition off */
#define CMD_RECOGNITION_ACTIVE		7  /* Voice recognition on */
#ifdef CONFIG_MUSIC_CODEC_GAIN
#define CMD_MUSIC_START                 8  /* Start music stream type */
#define CMD_MUSIC_STOP                  9  /* Stop music stream type */
#endif
#ifdef CONFIG_VOIP
#define CMD_VOIP_START                  10 /* Start VoIP */
#define CMD_VOIP_STOP                   11 /* Stop VoIP */
#endif


#define MC1N2_DSOURCE_OFF		0
#define MC1N2_DSOURCE_ADC		1
#define MC1N2_DSOURCE_DIR0		2
#define MC1N2_DSOURCE_DIR1		3
#define MC1N2_DSOURCE_DIR2		4
#define MC1N2_DSOURCE_MIX		5

#define MC1N2_AE_PARAM_1		0
#define MC1N2_AE_PARAM_2		1
#define MC1N2_AE_PARAM_3		2
#define MC1N2_AE_PARAM_4		3
#define MC1N2_AE_PARAM_5		4

#define mc1n2_i2c_read_byte(c,r) i2c_smbus_read_byte_data((c), (r)<<1)

extern struct i2c_client *mc1n2_get_i2c_client(void);

/*
 * For debugging
 */
#ifdef CONFIG_SND_SOC_MC1N2_DEBUG

#define dbg_info(format, arg...) snd_printd(KERN_INFO format, ## arg)
#define TRACE_FUNC() snd_printd(KERN_INFO "<trace> %s()\n", __FUNCTION__)


#define _McDrv_Ctrl McDrv_Ctrl_dbg
extern SINT32 McDrv_Ctrl_dbg(UINT32 dCmd, void *pvPrm, UINT32 dPrm);

#else

#define dbg_info(format, arg...)
#define TRACE_FUNC()

#define _McDrv_Ctrl McDrv_Ctrl

#endif /* CONFIG_SND_SOC_MC1N2_DEBUG */

#ifdef CODEC_DEBUG
#define SUBJECT "mc1n2"
#define CODECDBG(format, ...)\
	printk(KERN_INFO "[ "SUBJECT " (%s,%d) ] " format "\n", \
			__func__, __LINE__, ## __VA_ARGS__);
#else
#define CODECDBG(format, ...)
#endif


#endif /* MC1N2_PRIV_H */
