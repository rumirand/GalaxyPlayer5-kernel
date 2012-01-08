/*
 * MC-1N2 ASoC codec driver
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <sound/hwdep.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/file.h>
#include <asm/io.h>
#include <asm/gpio.h> 
#include <plat/gpio-cfg.h> 
#include <plat/map-base.h>
#include <mach/regs-clock.h> 
#include <mach/gpio.h> 

#include "mc1n2.h"
#include "mc1n2_priv.h"
#include "mc1n2_cfg.h"
#include "mcresctrl.h"

#define MC1N2_DRIVER_VERSION "0.9.8"

unsigned char fmradio_state;
unsigned char fmradio_closed;

int voice_recognition_state;

#ifdef CONFIG_MUSIC_CODEC_GAIN
int music_start_flag;
#endif
#ifdef CONFIG_VOIP
int voip_status;
#endif

/* For ear-jack control(MIC-Bias) */
extern short int get_headset_status(void);

// ========================= EUR volume table 100DB SPL =============================
#define HEADPHONE_VOL_EUR               54      // 0~63
#define HEADPHONE_VOL_FM_EUR            46      // 0~63
#define DUAL_VOL_FOR_HEADPHONE_EUR      22      // 0~63
#define MIC2_GAIN_TEST_EUR              2       // default 0: ear mic 0=15db, 1=20db,2=25db,3=30db
#define MC1N2_MIC2_ADC_VOL_EUR          220     // default 190:mic2 volume Left  0~239
#define MIC1_GAIN_REC_EUR               1
#define MC1N2_MIC1_ADC_VOL_REC_EUR      202

#ifdef CONFIG_MUSIC_CODEC_GAIN
/*                                           0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 */
int gAnalVolHpIndex_eur[MUSIC_VOL_LEVEL] = {43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,44,45,46,47,48,49,51,54,54,54,54,54,54,54};
#endif

/* ========================= KOREA  volume table 107DB SPL ============================= */
#define HEADPHONE_VOL_KOR               55      // 0~63
#define HEADPHONE_VOL_FM_KOR            54      // 0~63
#define DUAL_VOL_FOR_HEADPHONE_KOR      26      // 0~63
#define MIC2_GAIN_TEST_KOR              3       // default 0: ear mic 0=15db, 1=20db,2=25db,3=30db
#define MC1N2_MIC2_ADC_VOL_KOR          228     // default 190:mic2 volume Left  0~239
#define MIC1_GAIN_REC_KOR               2
#define MC1N2_MIC1_ADC_VOL_REC_KOR      194

#ifdef CONFIG_MUSIC_CODEC_GAIN
/*                                           0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 */ 
int gAnalVolHpIndex_kor[MUSIC_VOL_LEVEL] = {41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,43,45,48,51,54};
#endif

/* ========================= COMMON value  ============================= */
#define SPEAKER_VOL                     53      // 0~63
#define SPEAKER_VOL_FM                  44      // 0~63
#define RCV_VOL                         63      // 0~63
#define DUAL_VOL_FOR_SPEAKER            52      // 0~63

#define MIC1_GAIN_TEST                  3       // default 0: main mic 0=15db, 1=20db,2=25db,3=30db
#define MC1N2_MIC1_ADC_VOL         239     // default 190:mic1 volume Left  0~239
#define FM_ADC_VOL                      200     // default 96:fm radio volume Left  0~239


unsigned int gMicGain = MIC1_GAIN_REC_KOR;
unsigned int gMicADC = MC1N2_MIC1_ADC_VOL_REC_KOR;

int g_HEADPHONE_VOL = HEADPHONE_VOL_KOR;
int g_HEADPHONE_VOL_FM = HEADPHONE_VOL_FM_KOR;
int g_DUAL_VOL_FOR_HEADPHONE = DUAL_VOL_FOR_HEADPHONE_KOR;
int g_MIC2_GAIN_TEST = MIC2_GAIN_TEST_KOR;
int g_MC1N2_MIC2_ADC_VOL = MC1N2_MIC2_ADC_VOL_KOR;
#ifdef CONFIG_MUSIC_CODEC_GAIN
int *gAnalVolHpIndex = gAnalVolHpIndex_kor;
#endif

#ifdef CONFIG_VOIP
/* VoIP gain */
#define MC1N2_VOIP_SPEAKER_GAIN                 55
#define MC1N2_VOIP_RECEIVER_GAIN                63
#define MC1N2_VOIP_HEADPHONE_GAIN               44

#define MC1N2_VOIP_SPEAKER_MIC_GAIN             3
#define MC1N2_VOIP_SPEAKER_MIC_ADC              177

#define MC1N2_VOIP_HEADSET_MIC_GAIN             3
#define MC1N2_VOIP_HEADSET_ADC_GAIN             209
#define MC1N2_VOIP_HEADSET_DIT                  192

#define MC1N2_VOIP_SPEAKER_DAC                  192
#define MC1N2_VOIP_SPEAKER_DAC_MASTER           192
#endif

#ifdef MP3_SNDCONFIG
#include <linux/string.h>
#include <linux/kernel.h>

int isLoadSoundConfig = 1;
char *token;
char *last;

unsigned int gMicGainTest = MIC1_GAIN_TEST;
unsigned int gMicADCTest = MC1N2_MIC1_ADC_VOL;
#endif 

#ifdef LOAD_VOIP_CONFIG 
#include <linux/string.h>
#include <linux/kernel.h>

int isLoadVoipConfig = 1;
char *token;
char *last;

/* playback */
int gSpeakerGain;
int gReceiverGain;
int gHeadPhoneGain;

/* mic */ 
int gSpeaker_mic_gain;
int gSpeaker_mic_adc;
int gHeadPhone_mic_gain;
int gHeadPhone_mic_adc;
int gHeadPhone_mic_dit;

int gSpeakerGainDac;
int gSpeakerDacMaster;
int gSpeakerDitVol;
#endif


#define MC1N2_NAME "mc1n2"

#define MC1N2_I2S_RATE (SNDRV_PCM_RATE_8000_48000)
#define MC1N2_I2S_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	 SNDRV_PCM_FMTBIT_S24_3LE)

#define MC1N2_PCM_RATE (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000)
#define MC1N2_PCM_FORMATS \
	(SNDRV_PCM_FMTBIT_S8 | \
	 SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
	 SNDRV_PCM_FMTBIT_A_LAW | SNDRV_PCM_FMTBIT_MU_LAW)

#define MC1N2_HWDEP_ID "mc1n2"

#define MC1N2_HW_ID_AA 0x78
#define MC1N2_HW_ID_AB 0x79

#define mc1n2_is_in_playback(p) ((p)->stream & (1 << SNDRV_PCM_STREAM_PLAYBACK))
#define mc1n2_is_in_capture(p)  ((p)->stream & (1 << SNDRV_PCM_STREAM_CAPTURE))

static UINT8 mc1n2_hwid;
static struct snd_soc_codec *mc1n2_codec;


struct snd_soc_codec *mc1n2_get_codec_data(void)
{
        return mc1n2_codec;
}

static void mc1n2_set_codec_data(struct snd_soc_codec *codec)
{
        mc1n2_codec = codec;
}

/* deliver i2c access to machdep */
struct i2c_client *mc1n2_get_i2c_client(void)
{
        return mc1n2_codec->control_data;
}

/*
 * DAI (PCM interface)
 */
/* SRC_RATE settings @ 73728000Hz (ideal PLL output) */
static int mc1n2_src_rate[][SNDRV_PCM_STREAM_LAST+1] = {
        /* DIR, DIT */
        {32768, 4096},                  /* MCDRV_FS_48000 */
        {30106, 4458},                  /* MCDRV_FS_44100 */
        {21845, 6144},                  /* MCDRV_FS_32000 */
        {0, 0},                         /* N/A */
        {0, 0},                         /* N/A */
        {15053, 8916},                  /* MCDRV_FS_22050 */
        {10923, 12288},                 /* MCDRV_FS_16000 */
        {0, 0},                         /* N/A */
        {0, 0},                         /* N/A */
        {7526, 17833},                  /* MCDRV_FS_11025 */
        {5461, 24576},                  /* MCDRV_FS_8000 */
};

#define mc1n2_fs_to_srcrate(rate,dir) mc1n2_src_rate[(rate)][(dir)];


#ifdef MP3_SNDCONFIG
int ReadSoundConfigFile(char *Filename, int StartPos)
{
        struct file *filp;
        char *Buffer;
        mm_segment_t oldfs;
        int BytesRead;

        Buffer = kmalloc(256, GFP_KERNEL);
        if (!Buffer) {
                CODECDBG("Buffer kmalloc error");
                return -1;
        }

        filp = filp_open(Filename, 00, O_RDONLY);
        if (IS_ERR(filp) || !filp) {
                CODECDBG("ReadSoundconfig open error");
                return -1;
        }

        if (!filp->f_op->read) {
                CODECDBG("ReadSoundconfig open read error");
                return -1;  /* File(system) doesn't allow reads */
        }

        /* Now read 4096 bytes from postion "StartPos" */
        filp->f_pos = StartPos;
        
        oldfs = get_fs();
        set_fs(KERNEL_DS);
        
        BytesRead = filp->f_op->read(filp,Buffer,256,&filp->f_pos);
        set_fs(oldfs);

        last = Buffer;
        token = strsep(&last, ",");
        gMicGainTest = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gMicADCTest = simple_strtoul(token, NULL, 10);

        CODECDBG("gMicGain, gMicADC=%d,%d", gMicGainTest, gMicADCTest);

        /* Close the file */
        fput(filp);
        /* release allocate memeory */
        kfree(Buffer);
        
        return 0;
}
#endif

#ifdef LOAD_VOIP_CONFIG
int ReadVoipConfigFile(char *Filename, int StartPos)
{
        struct file *filp;  
        char *Buffer;
        mm_segment_t oldfs;
        int BytesRead;

        Buffer = kmalloc(256, GFP_KERNEL);
        if (!Buffer) 
                return -1;

        filp = filp_open(Filename, 00, O_RDONLY);
        if (IS_ERR(filp) || !filp) {
                CODECDBG("ReadSoundconfig open error");
                return -1;  /* Or do something else */
        }

        if (!filp->f_op->read) {
                CODECDBG("ReadSoundconfig open read error");
                return -1;  /* File(system) doesn't allow reads */
        }

        /* Now read 4096 bytes from postion "StartPos" */
        filp->f_pos = StartPos;

        oldfs = get_fs();
        set_fs(KERNEL_DS);

        BytesRead = filp->f_op->read(filp,Buffer, 256, &filp->f_pos);
        set_fs(oldfs);

        /* playback */
        last = Buffer;
        token = strsep(&last, ",");
        gSpeakerGain = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gReceiverGain = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gHeadPhoneGain = simple_strtoul(token, NULL, 10);

        /* mic */ 
        token = strsep(&last, ",");
        gSpeaker_mic_gain = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gSpeaker_mic_adc = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gHeadPhone_mic_gain = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gHeadPhone_mic_adc = simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gSpeakerGainDac= simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gSpeakerDacMaster= simple_strtoul(token, NULL, 10);
        token = strsep(&last, ",");
        gSpeakerDitVol= simple_strtoul(token, NULL, 10);

        /* Close the file */
        fput(filp);

        /* release allocate memeory */
        kfree(Buffer);

        return 0;
}
#endif 

#ifdef MC1N2_LOAD_PRD_FOR_SPL
#define MC1N2_SPL_100DB 1
#define MC1N2_SPL_107DB 2
#define MC1N2_PRD_PATH "/efs/buyer_code.dat"

int gSplFlag = MC1N2_SPL_107DB;
int isLoadPRDconfig = 1;

#define EUR_VOL_COUNT 19
const char *EURVol[] = {"EDC","EUR","NEE","ROM","TPH","XEE","XEF","XEH","XEN","XEO","XET","XEU","XEZ","SSA","XFA","XEG","TUR","TRA","SEB"};

static int LoadPRDFile(void)
{
        struct file *filp;
        char *Buffer;
        mm_segment_t oldfs;
        int BytesRead,i;
        char prd[4] = {0};
        int err = 0;

        Buffer = kmalloc(256, GFP_KERNEL);
        if (!Buffer) {
                CODECDBG("Buffer kmalloc error");
                err = -1;
                goto default_case;
        }

        filp = filp_open(MC1N2_PRD_PATH, 00, O_RDONLY);

        if (IS_ERR(filp) || !filp) {
                CODECDBG("LoadPRDFile open error");
                err = -1;
                goto default_case;
        }

        if (!filp->f_op->read) {
                CODECDBG("LoadPRDFile open read error");
                err = -1;
                goto default_case;
        }

        /* Now read 4096 bytes from postion "StartPos" */
        filp->f_pos = 0;

        oldfs = get_fs();
        set_fs(KERNEL_DS);

        BytesRead = filp->f_op->read(filp,Buffer,256,&filp->f_pos);
        set_fs(oldfs);

        /* copy prd data from read buffer */
        prd[0] = Buffer[0]; 
        prd[1] = Buffer[1]; 
        prd[2] = Buffer[2];  
        prd[3] = '\0';

        CODECDBG("prd=%s\n", prd);

        gSplFlag = MC1N2_SPL_107DB;
        for (i = 0; i < EUR_VOL_COUNT; i++) {
                if (!strcmp(prd,EURVol[i])) {
                        gSplFlag = MC1N2_SPL_100DB;
                        break;
                }
        }

default_case:
        CODECDBG("current spl level =%d\n", gSplFlag);

        /* Close the file */
        if (!filp)
                fput(filp);

        /* release allocate memeory */
        kfree(Buffer);

        if (MC1N2_SPL_100DB == gSplFlag) {
                /* Load 100DB Spl level */
                g_HEADPHONE_VOL = HEADPHONE_VOL_EUR;
                g_HEADPHONE_VOL_FM = HEADPHONE_VOL_FM_EUR;
                g_DUAL_VOL_FOR_HEADPHONE = DUAL_VOL_FOR_HEADPHONE_EUR;
                gMicGain = MIC1_GAIN_REC_EUR;
                gMicADC = MC1N2_MIC1_ADC_VOL_REC_EUR;
                g_MIC2_GAIN_TEST = MIC2_GAIN_TEST_EUR;
                g_MC1N2_MIC2_ADC_VOL = MC1N2_MIC2_ADC_VOL_EUR;
#ifdef CONFIG_MUSIC_CODEC_GAIN
                /* load analog volume table */
                gAnalVolHpIndex = gAnalVolHpIndex_eur;
#endif
                CODECDBG("load spl 100DB\n");
        } else {
                /* Load 107DB Spl level */
                g_HEADPHONE_VOL = HEADPHONE_VOL_KOR;
                g_HEADPHONE_VOL_FM = HEADPHONE_VOL_FM_KOR;
                g_DUAL_VOL_FOR_HEADPHONE = DUAL_VOL_FOR_HEADPHONE_KOR;
                gMicGain = MIC1_GAIN_REC_KOR;
                gMicADC = MC1N2_MIC1_ADC_VOL_REC_KOR;
                g_MIC2_GAIN_TEST = MIC2_GAIN_TEST_KOR;
                g_MC1N2_MIC2_ADC_VOL = MC1N2_MIC2_ADC_VOL_KOR;
#ifdef CONFIG_MUSIC_CODEC_GAIN
                /* load analog volume table */
                gAnalVolHpIndex = gAnalVolHpIndex_kor;
#endif
                CODECDBG("load spl 107DB\n");
        }

        return err;
}
#endif

static int mc1n2_setup_dai(struct mc1n2_data *mc1n2, int id, int mode, int dir)
{
	MCDRV_DIO_INFO dio;
	MCDRV_DIO_PORT *port = &dio.asPortInfo[id];
	struct mc1n2_setup *setup = &mc1n2->setup;
	struct mc1n2_port_params *par = &mc1n2->port[id];
	UINT32 update = 0;
	int i;

	memset(&dio, 0, sizeof(MCDRV_DIO_INFO));

	if (par->stream == 0) {
		port->sDioCommon.bMasterSlave = par->master;
		port->sDioCommon.bAutoFs = MCDRV_AUTOFS_OFF;
		port->sDioCommon.bFs = par->rate;
		port->sDioCommon.bBckFs = par->bckfs;
		port->sDioCommon.bInterface = mode;
		port->sDioCommon.bBckInvert = par->inv;
		if (mode == MCDRV_DIO_PCM) {
			port->sDioCommon.bPcmHizTim = setup->pcm_hiz_redge[id];
			port->sDioCommon.bPcmClkDown = par->pcm_clkdown;
			port->sDioCommon.bPcmFrame = par->format;
			port->sDioCommon.bPcmHighPeriod = setup->pcm_hperiod[id];
		}
		update |= MCDRV_DIO0_COM_UPDATE_FLAG;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		port->sDir.wSrcRate = mc1n2_fs_to_srcrate(par->rate, dir);
		if (mode == MCDRV_DIO_DA) {
			port->sDir.sDaFormat.bBitSel = par->bits[dir];
			port->sDir.sDaFormat.bMode = par->format;
		} else {
			port->sDir.sPcmFormat.bMono = par->pcm_mono[dir];
			port->sDir.sPcmFormat.bOrder = par->pcm_order[dir];
			if (setup->pcm_extend[id]) {
				port->sDir.sPcmFormat.bOrder |=
					(1 << setup->pcm_extend[id]);
			}
			port->sDir.sPcmFormat.bLaw = par->pcm_law[dir];
			port->sDir.sPcmFormat.bBitSel = par->bits[dir];
		}
		for (i = 0; i < DIO_CHANNELS; i++) {
			if (i && par->channels == 1) {
				port->sDir.abSlot[i] = port->sDir.abSlot[i-1];
			} else {
				port->sDir.abSlot[i] = setup->slot[id][dir][i];
			}

		}
		update |= MCDRV_DIO0_DIR_UPDATE_FLAG;
	}

	if (dir == SNDRV_PCM_STREAM_CAPTURE) {
		port->sDit.wSrcRate = mc1n2_fs_to_srcrate(par->rate, dir);
		if (mode == MCDRV_DIO_DA) {
			port->sDit.sDaFormat.bBitSel = par->bits[dir];
			port->sDit.sDaFormat.bMode = par->format;
		} else {
			port->sDit.sPcmFormat.bMono = par->pcm_mono[dir];
			port->sDit.sPcmFormat.bOrder = par->pcm_order[dir];
			if (setup->pcm_extend[id]) {
				port->sDit.sPcmFormat.bOrder |=
					(1 << setup->pcm_extend[id]);
			}
			port->sDit.sPcmFormat.bLaw = par->pcm_law[dir];
			port->sDit.sPcmFormat.bBitSel = par->bits[dir];
		}
		for (i = 0; i < DIO_CHANNELS; i++) {
			port->sDit.abSlot[i] = setup->slot[id][dir][i];
		}
		update |= MCDRV_DIO0_DIT_UPDATE_FLAG;
	}

	return _McDrv_Ctrl(MCDRV_SET_DIGITALIO, &dio, update << (id*3));
}

static int mc1n2_control_dir(struct mc1n2_data *mc1n2, int id, int enable)
{
	MCDRV_PATH_INFO info;
	MCDRV_CHANNEL *ch;
	int activate;
	int i;

	memset(&info, 0, sizeof(MCDRV_PATH_INFO));

	for (i = 0; i < MC1N2_N_PATH_CHANNELS; i++) {

		switch (i) {
		case 0:
			ch = &info.asDit0[0];
#ifdef DIO0_DAI_ENABLE
			activate = enable && mc1n2_is_in_capture(&mc1n2->port[0]);
#else
			activate = enable;
#endif
			break;

		case 1:
			ch = &info.asDit1[0];
#ifdef DIO1_DAI_ENABLE
			activate = enable && mc1n2_is_in_capture(&mc1n2->port[1]);
#else
			activate = enable;
#endif
			break;
		case 2:
			ch = &info.asDit2[0];
#ifdef DIO2_DAI_ENABLE
			activate = enable && mc1n2_is_in_capture(&mc1n2->port[2]);
#else
			activate = enable;
#endif
			break;
		case 3:
			ch = &info.asHpOut[0];
			activate = enable;
			break;
		case 4:
			ch = &info.asHpOut[1];
			activate = enable;
			break;
		case 5:
			ch = &info.asSpOut[0];
			activate = enable;
			break;
		case 6:
			ch = &info.asSpOut[1];
			activate = enable;
			break;
		case 7:
			ch = &info.asRcOut[0];
			activate = enable;
			break;
		case 8:
			ch = &info.asLout1[0];
			activate = enable;
			break;
		case 9:
			ch = &info.asLout1[1];
			activate = enable;
			break;
		case 10:
			ch = &info.asLout2[0];
			activate = enable;
			break;
		case 11:
			ch = &info.asLout2[1];
			activate = enable;
			break;
		case 12:
			ch = &info.asDac[0];
			activate = enable;
			break;
		case 13:
			ch = &info.asDac[1];
			activate = enable;
			break;
		case 14:
			ch = &info.asAe[0];
			activate = enable;
			break;
		case 15:
			ch = &info.asAdc0[0];
			activate = enable;
			break;
		case 16:
			ch = &info.asAdc0[1];
			activate = enable;
			break;
		case 17:
			ch = &info.asMix[0];
			activate = enable;
			break;
		case 18:
			ch = &info.asBias[0];
			activate = enable;
			break;

		default:
			activate = enable;
			break;
		}

		if (mc1n2->port[id].dir[i]) {
			ch->abSrcOnOff[3] = 0x1 << (id * 2 + !activate);
		}
	}


        if (fmradio_closed) {
                CODECDBG("Close FM radio path");
                
                info.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                info.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_OFF;
                info.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_OFF;
                info.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                info.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                info.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                info.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                info.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                info.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                
                fmradio_closed = 0;
        }

	return _McDrv_Ctrl(MCDRV_SET_PATH, &info, 0);
}

static int mc1n2_control_dit(struct mc1n2_data *mc1n2, int id, int enable)
{
	MCDRV_PATH_INFO info;
	MCDRV_CHANNEL *ch = info.asDit0 + id;
	int stream;
	int i;

	memset(&info, 0, sizeof(MCDRV_PATH_INFO));

	for (i = 0; i < SOURCE_BLOCK_NUM; i++) {
		if (i == 3) {
			stream = 0;
#ifdef DIO0_DAI_ENABLE
			stream |= mc1n2_is_in_playback(&mc1n2->port[0]);
#endif
#ifdef DIO1_DAI_ENABLE
			stream |= mc1n2_is_in_playback(&mc1n2->port[1]) << 2;
#endif
#ifdef DIO2_DAI_ENABLE
			stream |= mc1n2_is_in_playback(&mc1n2->port[2]) << 4;
#endif

			ch->abSrcOnOff[3] = (stream & mc1n2->port[id].dit.abSrcOnOff[3]) << !enable;
		} else {
			ch->abSrcOnOff[i] = mc1n2->port[id].dit.abSrcOnOff[i] << !enable;
		}
	}

        /* Turn off main mic_bias */
        if (!enable)
                info.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF;

	return _McDrv_Ctrl(MCDRV_SET_PATH, &info, 0);
}

static int mc1n2_update_clock(struct mc1n2_data *mc1n2)
{
	MCDRV_CLOCK_INFO info;

	memset(&info, 0, sizeof(MCDRV_CLOCK_INFO));
	info.bCkSel = mc1n2->setup.init.bCkSel;
	info.bDivR0 = mc1n2->setup.init.bDivR0;
	info.bDivF0 = mc1n2->setup.init.bDivF0;
	info.bDivR1 = mc1n2->setup.init.bDivR1;
	info.bDivF1 = mc1n2->setup.init.bDivF1;

	return _McDrv_Ctrl(MCDRV_UPDATE_CLOCK, &info, 0);
}

static int mc1n2_set_clkdiv_common(struct mc1n2_data *mc1n2, int div_id, int div)
{
	struct mc1n2_setup *setup = &mc1n2->setup;
	CODECDBG("div_id= %d, div = %d  \n", div_id,div);
	
	switch (div_id) {
	case MC1N2_CKSEL:
		switch (div) {
		case 0:
			setup->init.bCkSel = MCDRV_CKSEL_CMOS;
			break;
		case 1:
			setup->init.bCkSel = MCDRV_CKSEL_TCXO;
			break;
		case 2:
			setup->init.bCkSel = MCDRV_CKSEL_CMOS_TCXO;
			break;
		case 3:
			setup->init.bCkSel = MCDRV_CKSEL_TCXO_CMOS;
			break;
		default:
			return -EINVAL;
		}
		break;
	case MC1N2_DIVR0:
		if ((div < 1) || (div > 127)) {
			return -EINVAL;
		}
		setup->init.bDivR0 = div;
		break;
	case MC1N2_DIVF0:
		if ((div < 1) || (div > 255)) {
			return -EINVAL;
		}
		setup->init.bDivF0 = div;
		break;
	case MC1N2_DIVR1:
		if ((div < 1) || (div > 127)) {
			return -EINVAL;
		}
		setup->init.bDivR1 = div;
		break;
	case MC1N2_DIVF1:
		if ((div < 1) || (div > 255)) {
			return -EINVAL;
		}
		setup->init.bDivF1 = div;
		break;
	default:
		return -EINVAL;
	}

	mc1n2->clk_update = 1;

	return 0;
}

static int mc1n2_set_fmt_common(struct mc1n2_port_params *port, unsigned int fmt)
{
        CODECDBG("fmt=%d",fmt);

	/* master */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		port->master = MCDRV_DIO_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		port->master = MCDRV_DIO_SLAVE;
		break;
	default:
		return -EINVAL;
	}

	/* inv */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		port->inv = MCDRV_BCLK_NORMAL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		port->inv = MCDRV_BCLK_INVERT;
		break;
	default:
		return -EINVAL;
	}

#ifdef ALSA_VER_1_0_19
	/* clock */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_SYNC:
		/* just validating */
		break;
	default:
		return -EINVAL;
	}
#endif

	return 0;
}

static int mc1n2_i2s_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];


	CODECDBG("div_id= %d, div=%d", div_id, div);
        
	switch (div_id) {
	case MC1N2_BCLK_MULT:
		switch (div) {
		case MC1N2_LRCK_X32:
			port->bckfs = MCDRV_BCKFS_32;
			break;
		case MC1N2_LRCK_X48:
			port->bckfs = MCDRV_BCKFS_48;
			break;
		case MC1N2_LRCK_X64:
			port->bckfs = MCDRV_BCKFS_64;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return mc1n2_set_clkdiv_common(mc1n2, div_id, div);
	}

	return 0;
}


static int mc1n2_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id]; 


	CODECDBG("fmt= %d", fmt);

	/* format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		port->format = MCDRV_DAMODE_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		port->format = MCDRV_DAMODE_TAILALIGN;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		port->format = MCDRV_DAMODE_HEADALIGN;
		break;
	default:
		return -EINVAL;
	}

	return mc1n2_set_fmt_common(port, fmt);
}

static int mc1n2_i2s_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *runtime = snd_pcm_substream_chip(substream);
#ifdef ALSA_VER_1_0_19
	struct snd_soc_codec *codec = runtime->socdev->codec;
#else
	struct snd_soc_codec *codec = runtime->socdev->card->codec;
#endif
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];
	int dir = substream->stream;
	int rate;
	int err = 0;


	CODECDBG("hw_params: [%d] name=%s, dir=%d, rate=%d, bits=%d, ch=%d\n",
		 dai->id, substream->name, dir,
		 params_rate(params), params_format(params), params_channels(params));

	/* format (bits) */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		port->bits[dir] = MCDRV_BITSEL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		port->bits[dir] = MCDRV_BITSEL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		port->bits[dir] = MCDRV_BITSEL_24;
		break;
	default:
		return -EINVAL;
	}

	/* rate */
	switch (params_rate(params)) {
	case 8000:
		rate = MCDRV_FS_8000;
		break;
	case 11025:
		rate = MCDRV_FS_11025;
		break;
	case 16000:
		rate = MCDRV_FS_16000;
		break;
	case 22050:
		rate = MCDRV_FS_22050;
		break;
	case 32000:
		rate = MCDRV_FS_32000;
		break;
	case 44100:
		rate = MCDRV_FS_44100;
		break;
	case 48000:
		rate = MCDRV_FS_48000;
		break;
	default:
		return -EINVAL;
	}
        
        /* fix samplerate for recording */
        if (dir == 1) 
                rate = MCDRV_FS_44100;

	mutex_lock(&mc1n2->mutex);

	if ((port->stream & ~(1 << dir)) && (rate != port->rate)) {
		err = -EBUSY;
		goto error;
	}

	port->rate = rate;
	port->channels = params_channels(params);

	err = mc1n2_update_clock(mc1n2);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_update_clock\n", err);
		err = -EIO;
		goto error;
	}

	err = mc1n2_setup_dai(mc1n2, dai->id, MCDRV_DIO_DA, dir);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_setup_dai\n", err);
		err = -EIO;
		goto error;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		err = mc1n2_control_dir(mc1n2, dai->id, 1);
	} else {
		err = mc1n2_control_dit(mc1n2, dai->id, 1);
	}
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_control_dir/dit\n", err);
		err = -EIO;
		goto error;
	}

	port->stream |= (1 << dir);

error:
	mutex_unlock(&mc1n2->mutex);

	return err;
}

#ifdef CONFIG_SND_S5P_RP
extern volatile int s5p_rp_is_running;
#endif

static int mc1n2_hw_free(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *runtime = snd_pcm_substream_chip(substream);
#ifdef ALSA_VER_1_0_19
	struct snd_soc_codec *codec = runtime->socdev->codec;
#else
	struct snd_soc_codec *codec = runtime->socdev->card->codec;
#endif
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];
	int dir = substream->stream;
	int err;


        if (port->stream & (1 << dir)) {
#ifdef CONFIG_SND_S5P_RP
                /* Because RP use dummy data, shutdown is called during RP is running.
                * So, we should check RP flag and skip the shutdown routine.
                */
                if (s5p_rp_is_running && (dir == SNDRV_PCM_STREAM_PLAYBACK)) {
                        CODECDBG("Skip disable playback path");
                        return 0;
                }

                CODECDBG("stream=(%d) S5p_rp_is_running(%d)",dir, s5p_rp_is_running);
#else
                CODECDBG("stream=(%d)", dir);
#endif
        }

	mutex_lock(&mc1n2->mutex);

	if (!(port->stream & (1 << dir))) {
		err = 0;
		goto error;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		err = mc1n2_control_dir(mc1n2, dai->id, 0);
	} else {
		err = mc1n2_control_dit(mc1n2, dai->id, 0);
	}
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_control_dir/dit\n", err);
		err = -EIO;
		goto error;
	}

	port->stream &= ~(1 << dir);

error:
	mutex_unlock(&mc1n2->mutex);

	return err;
}

static int mc1n2_pcm_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];

	switch (div_id) {
	case MC1N2_BCLK_MULT:
		switch (div) {
		case MC1N2_LRCK_X8:
			port->bckfs = MCDRV_BCKFS_16;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_HALF;
			break;
		case MC1N2_LRCK_X16:
			port->bckfs = MCDRV_BCKFS_16;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X24:
			port->bckfs = MCDRV_BCKFS_48;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_HALF;
			break;
		case MC1N2_LRCK_X32:
			port->bckfs = MCDRV_BCKFS_32;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X48:
			port->bckfs = MCDRV_BCKFS_48;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X64:
			port->bckfs = MCDRV_BCKFS_64;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X128:
			port->bckfs = MCDRV_BCKFS_128;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X256:
			port->bckfs = MCDRV_BCKFS_256;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		case MC1N2_LRCK_X512:
			port->bckfs = MCDRV_BCKFS_512;
			port->pcm_clkdown = MCDRV_PCM_CLKDOWN_OFF;
			break;
		}
		break;
	default:
		return mc1n2_set_clkdiv_common(mc1n2, div_id, div);
	}

	return 0;
}

static int mc1n2_pcm_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];
 	/* format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		port->format = MCDRV_PCM_SHORTFRAME;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		port->format = MCDRV_PCM_LONGFRAME;
		break;
	default:
		return -EINVAL;
	}

	return mc1n2_set_fmt_common(port, fmt);
}

static int mc1n2_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *runtime = snd_pcm_substream_chip(substream);
#ifdef ALSA_VER_1_0_19
	struct snd_soc_codec *codec = runtime->socdev->codec;
#else
	struct snd_soc_codec *codec = runtime->socdev->card->codec;
#endif
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_port_params *port = &mc1n2->port[dai->id];
	int dir = substream->stream;
	int rate;
	int err;

	CODECDBG("hw_params: [%d] name=%s, dir=%d, rate=%d, bits=%d, ch=%d",
		 dai->id, substream->name, dir,
		 params_rate(params), params_format(params), params_channels(params));

	/* channels */
	switch (params_channels(params)) {
	case 1:
		port->pcm_mono[dir] = MCDRV_PCM_MONO;
		break;
	case 2:
		port->pcm_mono[dir] = MCDRV_PCM_STEREO;
		break;
	default:
		return -EINVAL;
	}

	/* format (bits) */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		port->bits[dir] = MCDRV_PCM_BITSEL_8;
		port->pcm_order[dir] = MCDRV_PCM_MSB_FIRST;
		port->pcm_law[dir] = MCDRV_PCM_LINEAR;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		port->bits[dir] = MCDRV_PCM_BITSEL_16;
		port->pcm_order[dir] = MCDRV_PCM_LSB_FIRST;
		port->pcm_law[dir] = MCDRV_PCM_LINEAR;
		break;
	case SNDRV_PCM_FORMAT_S16_BE:
		port->bits[dir] = MCDRV_PCM_BITSEL_16;
		port->pcm_order[dir] = MCDRV_PCM_MSB_FIRST;
		port->pcm_law[dir] = MCDRV_PCM_LINEAR;
		break;
	case SNDRV_PCM_FORMAT_A_LAW:
		port->bits[dir] = MCDRV_PCM_BITSEL_8;
		port->pcm_order[dir] = MCDRV_PCM_MSB_FIRST;
		port->pcm_law[dir] = MCDRV_PCM_ALAW;
		break;
	case SNDRV_PCM_FORMAT_MU_LAW:
		port->bits[dir] = MCDRV_PCM_BITSEL_8;
		port->pcm_order[dir] = MCDRV_PCM_MSB_FIRST;
		port->pcm_law[dir] = MCDRV_PCM_MULAW;
		break;
	default:
		return -EINVAL;
	}

	/* rate */
	switch (params_rate(params)) {
	case 8000:
		rate = MCDRV_FS_8000;
		break;
	case 16000:
		rate = MCDRV_FS_16000;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&mc1n2->mutex);

	if ((port->stream & ~(1 << dir)) && (rate != port->rate)) {
		err = -EBUSY;
		goto error;
	}

	port->rate = rate;
	port->channels = params_channels(params);

	err = mc1n2_update_clock(mc1n2);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_update_clock\n", err);
		err = -EIO;
		goto error;
	}

	err = mc1n2_setup_dai(mc1n2, dai->id, MCDRV_DIO_PCM, dir);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_setup_dai\n", err);
		err = -EIO;
		goto error;
	}

	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		err = mc1n2_control_dir(mc1n2, dai->id, 1);
	} else {
		err = mc1n2_control_dit(mc1n2, dai->id, 1);
	}
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in mc1n2_control_dir/dit\n", err);
		err = -EIO;
		goto error;
	}

	port->stream |= (1 << dir);

error:
	mutex_unlock(&mc1n2->mutex);

	return err;
}

#ifndef ALSA_VER_1_0_19
static struct snd_soc_dai_ops mc1n2_dai_ops[] = {
	{
		.set_clkdiv = mc1n2_i2s_set_clkdiv,
		.set_fmt = mc1n2_i2s_set_fmt,
		.hw_params = mc1n2_i2s_hw_params,
		.hw_free = mc1n2_hw_free,
	},
	{
		.set_clkdiv = mc1n2_pcm_set_clkdiv,
		.set_fmt = mc1n2_pcm_set_fmt,
		.hw_params = mc1n2_pcm_hw_params,
		.hw_free = mc1n2_hw_free,
	},
	{
		.set_clkdiv = mc1n2_i2s_set_clkdiv,
		.set_fmt = mc1n2_i2s_set_fmt,
		.hw_params = mc1n2_i2s_hw_params,
		.hw_free = mc1n2_hw_free,
	},
	{
		.set_clkdiv = mc1n2_pcm_set_clkdiv,
		.set_fmt = mc1n2_pcm_set_fmt,
		.hw_params = mc1n2_pcm_hw_params,
		.hw_free = mc1n2_hw_free,
	},
	{
		.set_clkdiv = mc1n2_i2s_set_clkdiv,
		.set_fmt = mc1n2_i2s_set_fmt,
		.hw_params = mc1n2_i2s_hw_params,
		.hw_free = mc1n2_hw_free,
	},
	{
		.set_clkdiv = mc1n2_pcm_set_clkdiv,
		.set_fmt = mc1n2_pcm_set_fmt,
		.hw_params = mc1n2_pcm_hw_params,
		.hw_free = mc1n2_hw_free,
	}
};
#endif

struct snd_soc_dai mc1n2_dai[] = {
	{
		.name = MC1N2_NAME "-da0",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_i2s_set_clkdiv,
			.set_fmt = mc1n2_i2s_set_fmt,
			.hw_params = mc1n2_i2s_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[0]
#endif
	},
	{
		.name = MC1N2_NAME "-da0",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_pcm_set_clkdiv,
			.set_fmt = mc1n2_pcm_set_fmt,
			.hw_params = mc1n2_pcm_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[1]
#endif
	},
	{
		.name = MC1N2_NAME "-da1",
		.id = 1,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_i2s_set_clkdiv,
			.set_fmt = mc1n2_i2s_set_fmt,
			.hw_params = mc1n2_i2s_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[2]
#endif
	},
	{
		.name = MC1N2_NAME "-da1",
		.id = 1,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_pcm_set_clkdiv,
			.set_fmt = mc1n2_pcm_set_fmt,
			.hw_params = mc1n2_pcm_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[3]
#endif
	},
	{
		.name = MC1N2_NAME "-da2",
		.id = 2,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_I2S_RATE,
			.formats = MC1N2_I2S_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_i2s_set_clkdiv,
			.set_fmt = mc1n2_i2s_set_fmt,
			.hw_params = mc1n2_i2s_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[4]
#endif
	},
	{
		.name = MC1N2_NAME "-da2",
		.id = 2,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC1N2_PCM_RATE,
			.formats = MC1N2_PCM_FORMATS,
		},
#ifdef ALSA_VER_1_0_19
		.ops = {
			.set_clkdiv = mc1n2_pcm_set_clkdiv,
			.set_fmt = mc1n2_pcm_set_fmt,
			.hw_params = mc1n2_pcm_hw_params,
			.hw_free = mc1n2_hw_free,
		}
#else
		.ops = &mc1n2_dai_ops[5]
#endif
	},
};
EXPORT_SYMBOL_GPL(mc1n2_dai);

/*
 * Control interface
 */
#ifdef CONFIG_FMRADIO_CODEC_GAIN
/* Volume map for FM radio codec line-input volume */
static SINT16 mc1n2_vol_fm[] = {
        0xe501, 0xe6ff, 0xe801, 0xea01, 0xec01, 0xed81, 0xf001, 0xf181, 0xf301, 0xf481, 
        0xf601, 0xf781, 0xf901, 0xfa81, 0xfc01, 0xfd81, 0x0001, 0x0181, 0x0301, 0x0401, 
        0x0501, 0x0601, 0x0701, 0x0801, 0x0901, 0x0a01, 0x0b01, 0x0c01, 0x0d01, 0x0e01, 
        0x0f01
};
#endif

#ifdef CONFIG_FMRADIO_CODEC_GAIN
unsigned char McDrv_Ctrl_get_fm_vol(void)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        CODECDBG("volume = %d", mc1n2->fm_volume);

        return mc1n2->fm_volume;
}
EXPORT_SYMBOL(McDrv_Ctrl_get_fm_vol);

unsigned int McDrv_Ctrl_set_fm_vol(unsigned int volume)
{
        int err;
        MCDRV_VOL_INFO fm_vol;
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        if (volume >= FM_VOL_LEVEL)
                mc1n2->fm_volume = FM_VOL_LEVEL-1;
        else if (volume < 0)
                mc1n2->fm_volume = 0;
        else
                mc1n2->fm_volume = volume;
        

        if (!fmradio_state)
                return 0;


        McResCtrl_GetVolInfo(&fm_vol);
        memset(&fm_vol, 0, sizeof(MCDRV_VOL_INFO));

        CODECDBG("volume = %d", volume);

        fm_vol.aswA_Ad0[0] = mc1n2_vol_fm[mc1n2->fm_volume];
        fm_vol.aswA_Ad0[1] = mc1n2_vol_fm[mc1n2->fm_volume];
        mc1n2->vol_store.aswA_Ad0[0] = mc1n2_vol_fm[mc1n2->fm_volume];
        mc1n2->vol_store.aswA_Ad0[1] = mc1n2_vol_fm[mc1n2->fm_volume];

        err = _McDrv_Ctrl(MCDRV_SET_VOLUME,(void*) &fm_vol, 0);
        
        if (err != MCDRV_SUCCESS) {
                CODECDBG("%d: Error in MCDRV_SET_VOLUME", err);
                return -EIO;
        }

        return 0;
}
EXPORT_SYMBOL(McDrv_Ctrl_set_fm_vol);

unsigned int McDrv_Ctrl_fm_mute(void)
{
        MCDRV_VOL_INFO fm_vol;


        CODECDBG("");

        McResCtrl_GetVolInfo(&fm_vol);

        memset(&fm_vol, 0, sizeof(MCDRV_VOL_INFO));

        return 0;
}
EXPORT_SYMBOL(McDrv_Ctrl_fm_mute);
#else
unsigned int McDrv_Ctrl_set_fixed_fm_vol(void)
{
        int err;
        MCDRV_VOL_INFO fm_vol;
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        if (!fmradio_state)
                return 0;


        McResCtrl_GetVolInfo(&fm_vol);
        memset(&fm_vol, 0, sizeof(MCDRV_VOL_INFO));

        fm_vol.aswA_Ad0[0] = FM_FIXED_GAIN;
        fm_vol.aswA_Ad0[1] = FM_FIXED_GAIN;
        mc1n2->vol_store.aswA_Ad0[0] = FM_FIXED_GAIN;
        mc1n2->vol_store.aswA_Ad0[1] = FM_FIXED_GAIN;

        err = _McDrv_Ctrl(MCDRV_SET_VOLUME,(void*) &fm_vol, 0);
        
        if (err != MCDRV_SUCCESS) {
                CODECDBG("%d: Error in MCDRV_SET_VOLUME", err);
                return -EIO;
        }

        return 0;
}
#endif

void McDrv_Ctrl_MICBIAS2(int en)
{
        MCDRV_PATH_INFO path;

        McResCtrl_GetPathInfoVirtual(&path);

        CODECDBG("(%d)", en);

        if (en)
                path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC2_BLOCK] = MCDRV_SRC0_MIC2_ON;
        else
                path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC2_BLOCK] = MCDRV_SRC0_MIC2_OFF;

        _McDrv_Ctrl(MCDRV_SET_PATH, &path, 0);

        return;
}
EXPORT_SYMBOL(McDrv_Ctrl_MICBIAS2);

int mc1n2_set_path(struct snd_soc_codec *codec, MCDRV_PATH_INFO *info)
{
	struct mc1n2_data *mc1n2 = codec->drvdata;
	MCDRV_CHANNEL *ch;
	int i, j;

	mutex_lock(&mc1n2->mutex);

	/* preserve DIR settings */
	for (i = 0; i < MC1N2_N_PATH_CHANNELS; i++) {
		switch (i) {
		case 0:
			ch = &info->asDit0[0];
			break;
		case 1:
			ch = &info->asDit1[0];
			break;
		case 2:
			ch = &info->asDit2[0];
			break;
		case 3:
			ch = &info->asHpOut[0];
			break;
		case 4:
			ch = &info->asHpOut[1];
			break;
		case 5:
			ch = &info->asSpOut[0];
			break;
		case 6:
			ch = &info->asSpOut[1];
			break;
		case 7:
			ch = &info->asRcOut[0];
			break;
		case 8:
			ch = &info->asLout1[0];
			break;
		case 9:
			ch = &info->asLout1[1];
			break;
		case 10:
			ch = &info->asLout2[0];
			break;
		case 11:
			ch = &info->asLout2[1];
			break;
		case 12:
			ch = &info->asDac[0];
			break;
		case 13:
			ch = &info->asDac[1];
			break;
		case 14:
			ch = &info->asAe[0];
			break;
		case 15:
			ch = &info->asAdc0[0];
			break;
		case 16:
			ch = &info->asAdc0[1];
			break;
		case 17:
			ch = &info->asMix[0];
			break;
		case 18:
			ch = &info->asBias[0];
			break;
		}

#ifdef DIO0_DAI_ENABLE
		switch ((ch->abSrcOnOff[3]) & 0x3) {
		case 1:
			mc1n2->port[0].dir[i] = 1;
			break;
		case 2:
			mc1n2->port[0].dir[i] = 0;
			break;
		}
#endif
#ifdef DIO1_DAI_ENABLE
		switch ((ch->abSrcOnOff[3] >> 2) & 0x3) {
		case 1:
			mc1n2->port[1].dir[i] = 1;
			break;
		case 2:
			mc1n2->port[1].dir[i] = 0;
			break;
		}
#endif
#ifdef DIO2_DAI_ENABLE
		switch ((ch->abSrcOnOff[3] >> 4) & 0x3) {
		case 1:
			mc1n2->port[2].dir[i] = 1;
			break;
		case 2:
			mc1n2->port[2].dir[i] = 0;
			break;
		}
#endif
	}

	/* preserve DIT settings */
#ifdef DIO0_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		mc1n2->port[0].dit.abSrcOnOff[j] |=
			info->asDit0[0].abSrcOnOff[j] & 0x55;
		mc1n2->port[0].dit.abSrcOnOff[j] &=
			~((info->asDit0[0].abSrcOnOff[j] & 0xaa) >> 1);
	}
#endif
#ifdef DIO1_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		mc1n2->port[1].dit.abSrcOnOff[j] |=
			info->asDit1[0].abSrcOnOff[j] & 0x55;
		mc1n2->port[1].dit.abSrcOnOff[j] &=
			~((info->asDit0[1].abSrcOnOff[j] & 0xaa) >> 1);
	}
#endif
#ifdef DIO2_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		mc1n2->port[2].dit.abSrcOnOff[j] |=
			info->asDit2[0].abSrcOnOff[j] & 0x55;
		mc1n2->port[2].dit.abSrcOnOff[j] &=
			~((info->asDit2[0].abSrcOnOff[j] & 0xaa) >> 1);
	}
#endif

	/* modify path */
#ifdef DIO0_DAI_ENABLE
	if (!mc1n2_is_in_playback(&mc1n2->port[0])) {
		info->asDit0[0].abSrcOnOff[3] &= ~(0x3);
		info->asDit1[0].abSrcOnOff[3] &= ~(0x3);
		info->asDit2[0].abSrcOnOff[3] &= ~(0x3);
		info->asHpOut[0].abSrcOnOff[3] &= ~(0x3);
		info->asHpOut[1].abSrcOnOff[3] &= ~(0x3);
		info->asSpOut[0].abSrcOnOff[3] &= ~(0x3);
		info->asSpOut[1].abSrcOnOff[3] &= ~(0x3);
		info->asRcOut[0].abSrcOnOff[3] &= ~(0x3);
		info->asLout1[0].abSrcOnOff[3] &= ~(0x3);
		info->asLout1[1].abSrcOnOff[3] &= ~(0x3);
		info->asLout2[0].abSrcOnOff[3] &= ~(0x3);
		info->asLout2[1].abSrcOnOff[3] &= ~(0x3);
		info->asDac[0].abSrcOnOff[3] &= ~(0x3);
		info->asDac[1].abSrcOnOff[3] &= ~(0x3);
		info->asAe[0].abSrcOnOff[3] &= ~(0x3);
		info->asAdc0[0].abSrcOnOff[3] &= ~(0x3);
		info->asAdc0[1].abSrcOnOff[3] &= ~(0x3);
		info->asMix[0].abSrcOnOff[3] &= ~(0x3);
		info->asBias[0].abSrcOnOff[3] &= ~(0x3);
	}
#endif
#ifdef DIO1_DAI_ENABLE
	if (!mc1n2_is_in_playback(&mc1n2->port[1])) {
		info->asDit0[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asDit1[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asDit2[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asHpOut[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asHpOut[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asSpOut[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asSpOut[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asRcOut[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asLout1[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asLout1[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asLout2[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asLout2[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asDac[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asDac[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asAe[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asAdc0[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asAdc0[1].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asMix[0].abSrcOnOff[3] &= ~(0x3 << 2);
		info->asBias[0].abSrcOnOff[3] &= ~(0x3 << 2);
	}
#endif
#ifdef DIO2_DAI_ENABLE
	if (!mc1n2_is_in_playback(&mc1n2->port[2])) {
		info->asDit0[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asDit1[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asDit2[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asHpOut[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asHpOut[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asSpOut[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asSpOut[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asRcOut[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asLout1[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asLout1[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asLout2[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asLout2[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asDac[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asDac[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asAe[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asAdc0[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asAdc0[1].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asMix[0].abSrcOnOff[3] &= ~(0x3 << 4);
		info->asBias[0].abSrcOnOff[3] &= ~(0x3 << 4);
	}
#endif

#ifdef DIO0_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		if (!mc1n2_is_in_capture(&mc1n2->port[0])) {
			info->asDit0[0].abSrcOnOff[j] = 0;
		}
	}
#endif
#ifdef DIO1_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		if (!mc1n2_is_in_capture(&mc1n2->port[1])) {
			info->asDit1[0].abSrcOnOff[j] = 0;
		}
	}
#endif
#ifdef DIO2_DAI_ENABLE
	for (j = 0; j < SOURCE_BLOCK_NUM; j++) {
		if (!mc1n2_is_in_capture(&mc1n2->port[2])) {
			info->asDit2[0].abSrcOnOff[j] = 0;
		}
	}
#endif

	_McDrv_Ctrl(MCDRV_SET_PATH, info, 0);

	mutex_unlock(&mc1n2->mutex);

	return 0;
}

static const SINT16 pvol[64] = {
	-14593, -14337, -14081, -13825, -13569, -13313, -13057, -12801,
	-12545, -12289, -12033, -11777, -11521, -11265, -11009, -10753,
	-10497, -10241,  -9985,  -9729,  -9473,  -9217,  -8961,  -8705,
	 -8449,  -8193,  -7937,  -7681,  -7425,  -7169,  -6913,  -6657,
	 -6401,  -6145,  -5889,  -5633,  -5377,  -5121,  -4865,  -4609,
	 -4353,  -4097,  -3841,  -3585,  -3329,  -3073,  -2817,  -2561,
	 -2305,  -2049,  -1793,  -1537,  -1281,  -1025,   -769,   -513,
	  -257,      1,    257,    513,    769,   1025,    1281,  1537
};

static const SINT16 cvol[240] = {
	-24575, -18335, -18239, -18143, -18047, -17951, -17855, -17759,
	-17663, -17567, -17471, -17375, -17279, -17183, -17087, -16991,
	-16895, -16799, -16703, -16607, -16511, -16415, -16319, -16223,
	-16127, -16031, -15935, -15839, -15743, -15647, -15551, -15455,
	-15359, -15263, -15167, -15071, -14975, -14879, -14783, -14687,
	-14591, -14495, -14399, -14303, -14207, -14111, -14015, -13919,
	-13823, -13727, -13631, -13535, -13439, -13343, -13247, -13151,
	-13055, -12959, -12863, -12767, -12671, -12575, -12479, -12383,
	-12287, -12191, -12095, -11999, -11903, -11807, -11711, -11615,
	-11519, -11423, -11327, -11231, -11135, -11039, -10943, -10847,
	-10751, -10655, -10559, -10463, -10367, -10271, -10175, -10079,
	 -9983,  -9887,  -9791,  -9695,  -9599,  -9503,  -9407,  -9311,
	 -9215,  -9119,  -9023,  -8927,  -8831,  -8735,  -8639,  -8543,
	 -8447,  -8351,  -8255,  -8159,  -8063,  -7967,  -7871,  -7775,
	 -7679,  -7583,  -7487,  -7391,  -7295,  -7199,  -7103,  -7007,
	 -6911,  -6815,  -6719,  -6623,  -6527,  -6431,  -6335,  -6239,
	 -6143,  -6047,  -5951,  -5855,  -5759,  -5663,  -5567,  -5471,
	 -5375,  -5279,  -5183,  -5087,  -4991,  -4895,  -4799,  -4703,
	 -4607,  -4511,  -4415,  -4319,  -4223,  -4127,  -4031,  -3935,
	 -3839,  -3743,  -3647,  -3551,  -3455,  -3359,  -3263,  -3167,
	 -3071,  -2975,  -2879,  -2783,  -2687,  -2591,  -2495,  -2399,
	 -2303,  -2207,  -2111,  -2015,  -1919,  -1823,  -1727,  -1631,
	 -1535,  -1439,  -1343,  -1247,  -1151,  -1055,   -959,   -863,
	  -767,   -671,   -575,   -479,   -383,   -287,   -191,    -95,
	     1,     97,    193,    289,    385,    481,    577,    673,
	   769,    865,    961,   1057,   1153,   1249,   1345,   1441,
	  1537,   1633,   1729,   1825,   1921,   2017,   2113,   2209,
	  2305,   2401,   2497,   2593,   2689,   2785,   2881,   2977,
	  3073,   3169,   3265,   3361,   3457,   3553,   3649,   3745,
	  3841,   3937,   4033,   4129,   4225,   4321,   4417,   4513
};

static const SINT16 mvol[4] = {
        3841, 5121, 6401, 7681
};

unsigned int mc1n2_read_reg(struct snd_soc_codec *codec, unsigned int reg)
{
        return ((u16 *)codec->reg_cache)[reg];
}

int mc1n2_write_reg(struct snd_soc_codec *codec,
			   unsigned int reg, unsigned int value)
{
        u16 *cp;
        struct mc1n2_data *mc1n2 = codec->drvdata;
        MCDRV_VOL_INFO vol;

        cp = (u16 *)codec->reg_cache + reg;

        memset(&vol, 0, sizeof(vol));

        switch (reg) {
        case MC1N2_RCV_VOL_L:
                mc1n2->rcv_vol_l = value;
                
                if (value > 63) 
                        value = 63;
                
                vol.aswA_Rc[0] = pvol[value];
                break;
                
        case MC1N2_RCV_VOL_R:
                mc1n2->rcv_vol_r = value;
                break;
        
        case MC1N2_SPEAKER_VOL_L:
                mc1n2->sp_vol_l = value;
                
                if (value > 63)
                        value = 63;
                
                vol.aswA_Sp[0] = pvol[value];
        break;
        
        case MC1N2_SPEAKER_VOL_R:
                mc1n2->sp_vol_r = value;
                
                if (value > 63)
                        value = 63;
                
                vol.aswA_Sp[1] = pvol[value];
                break;

        case MC1N2_HEADPHONE_VOL_L:
                mc1n2->hp_vol_l = value;
                
                if (value > 63)
                        value = 63;

                vol.aswA_Hp[0] = pvol[value];
                break;
        
        case MC1N2_HEADPHONE_VOL_R:
                mc1n2->hp_vol_r = value;
                
                if (value > 63)
                        value = 63;

                vol.aswA_Hp[1] = pvol[value];
                break;
        
        case MC1N2_ADC_VOL_L:
                mc1n2->ad_vol_l = value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_Ad0[0] = cvol[value];
                vol.aswD_Ad0[1] = cvol[value];
                break;

        case MC1N2_DAC_VOL_L:
                mc1n2->da_vol_l = value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_Dir0[0] = cvol[value];
                break;
        
        case MC1N2_DAC_MASTER:
                mc1n2->da_mas_vol=value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_DacMaster[0] = cvol[value];
                vol.aswD_DacMaster[1] = cvol[value];
                break;

        case MC1N2_DAC_DAT_VAL:
                mc1n2->da_dit_vol=value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_Dit0[0] = cvol[value];
                vol.aswD_Dit0[1] = cvol[value];
                break;

        case MC1N2_ADC_VOL_R:
                mc1n2->ad_vol_r = value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_Ad0[1] = cvol[value];
                break;

        case MC1N2_MIC_ADC_VOL:
                mc1n2->ad_vol_r = value;
                
                if (value > 239)
                        value = 239;

                vol.aswD_Ad1[0] = cvol[value];
                break;
        
        case MC1N2_MIC1_GAIN:
                if (value > 3)
                        value = 3;
                
                vol.aswA_Mic1Gain[0] = mvol[value];
                break;
        
        case MC1N2_MIC2_GAIN:
                if (value > 3)
                        value = 3;
                
                vol.aswA_Mic2Gain[0] = mvol[value];
                break;
        
        default:
                break;
        }

        _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&vol, 0);

        *cp = value;

        return 0;
}

static int mc1n2_get_playback_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	ucontrol->value.integer.value[0] = mc1n2->playback_path;

	return 0;
}

static int mc1n2_set_playback_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int path_num = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;
        MCDRV_PATH_INFO path;
        MCDRV_DIO_INFO diohdmi;
        static MCDRV_DIO_INFO dioorg;

#ifdef MP3_SNDCONFIG
        if (isLoadSoundConfig) {
                if (!ReadSoundConfigFile("/sdcard/soundcfg/analog.txt", 0))
                        isLoadSoundConfig = 0;
        }
#endif

#ifdef MC1N2_LOAD_PRD_FOR_SPL
        if (isLoadPRDconfig) {
                LoadPRDFile();
                isLoadPRDconfig = 0;
        }
#endif 

        memset(&path, 0, sizeof(path));

        CODECDBG("path_num = %d",path_num);

        switch (path_num) {
        case PLAYBACK_OFF:
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case RCV:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Rcv, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackRcv, 0);

#ifdef CONFIG_VOIP
                if (mc1n2_get_voip_status()) {
                        mc1n2_set_voip_playback_parameters(RCV);
                        mc1n2_set_voip_mic_parameters(MIC_MAIN);
                } else
#endif
                {
                        mc1n2_write_reg(codec, MC1N2_RCV_VOL_L, RCV_VOL);
                        mc1n2_write_reg(codec, MC1N2_RCV_VOL_R, RCV_VOL);
                        CODECDBG("RCV : gain(%d)", RCV_VOL);
                }
                
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON | MCDRV_SRC5_DAC_R_ON;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case SPK:
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackSpk, 0);

#ifdef CONFIG_VOIP
                if (mc1n2_get_voip_status()) {
                        mc1n2_set_voip_playback_parameters(SPK);
                        mc1n2_set_voip_mic_parameters(MIC_MAIN);

                        path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                        CODECDBG("VoIP SPK : Turn off right speaker");
                } else 
#endif
                {
                        mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL);
                        mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL);
                        CODECDBG("SPK : gain(%d)", SPEAKER_VOL);

                        path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;                
                }

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case HP:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Hp, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackHp, 0);

#ifdef CONFIG_VOIP
                if (mc1n2_get_voip_status()) {
                        mc1n2_set_voip_playback_parameters(HP);
                        mc1n2_set_voip_mic_parameters(MIC_SUB);
                } else 
#endif
                {
#ifdef CONFIG_MUSIC_CODEC_GAIN
                        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gAnalVolHpIndex[mc1n2->analog_vol]);
                        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gAnalVolHpIndex[mc1n2->analog_vol]);
                        CODECDBG("HP : gain(%d)", gAnalVolHpIndex[mc1n2->analog_vol]);
#else
                        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL);
                        mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL);
                        CODECDBG("HP : gain(%d)", g_HEADPHONE_VOL);
#endif
                }

                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case BT:
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case DUAL:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Dual, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackDual, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, DUAL_VOL_FOR_SPEAKER);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, DUAL_VOL_FOR_SPEAKER);
                CODECDBG("DUAL : SPK gain(%d)", DUAL_VOL_FOR_SPEAKER);
                
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_DUAL_VOL_FOR_HEADPHONE);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_DUAL_VOL_FOR_HEADPHONE);
                CODECDBG("DUAL : HP gain(%d)", g_DUAL_VOL_FOR_HEADPHONE);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case RING_SPK:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Spk, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackRingSpk, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL);
                CODECDBG("RING_SPK : SPK gain(%d)", SPEAKER_VOL);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case RING_HP:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Hp, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackRingHp, 0);

#ifdef CONFIG_MUSIC_CODEC_GAIN
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gAnalVolHpIndex[mc1n2->analog_vol]);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gAnalVolHpIndex[mc1n2->analog_vol]);
                CODECDBG("RING_HP : HP gain(%d)", gAnalVolHpIndex[mc1n2->analog_vol]);
#else
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL);
                CODECDBG("RING_HP : HP gain(%d)", g_HEADPHONE_VOL);
#endif
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case RING_DUAL:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Dual, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackRingDual, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL);
                
#ifdef CONFIG_MUSIC_CODEC_GAIN
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gAnalVolHpIndex[mc1n2->analog_vol]);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gAnalVolHpIndex[mc1n2->analog_vol]);
                CODECDBG("RING_DUAL : SPK gain(%d), HP gain(%d)", SPEAKER_VOL, gAnalVolHpIndex[mc1n2->analog_vol]);
#else
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL);
                CODECDBG("RING_DUAL : SPK gain(%d), HP gain(%d)", SPEAKER_VOL, g_HEADPHONE_VOL);
#endif
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case EXTRA_DOCK_SPEAKER:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Dock, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_PlaybackDock, 0);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                break;

        case TV_OUT:
                /* exit HDMI mode */
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDac[1].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                _McDrv_Ctrl(MCDRV_SET_PATH, &path, 0);

                _McDrv_Ctrl(MCDRV_SET_DIGITALIO, &dioorg, MCDRV_DIO0_COM_UPDATE_FLAG);
                
                /* enter HDMI mode */
                _McDrv_Ctrl(MCDRV_GET_DIGITALIO, &dioorg, 0);
                diohdmi.asPortInfo[0].sDioCommon.bMasterSlave = MCDRV_DIO_MASTER;
                diohdmi.asPortInfo[0].sDioCommon.bAutoFs = MCDRV_AUTOFS_ON;
                diohdmi.asPortInfo[0].sDioCommon.bFs = MCDRV_FS_44100;
                diohdmi.asPortInfo[0].sDioCommon.bBckFs = MCDRV_BCKFS_32;
                diohdmi.asPortInfo[0].sDioCommon.bInterface = MCDRV_DIO_DA;

                memset(&path, 0, sizeof(path));
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asDac[1].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;

                _McDrv_Ctrl(MCDRV_SET_PATH, &path, 0);
                _McDrv_Ctrl(MCDRV_SET_DIGITALIO, &diohdmi, MCDRV_DIO0_COM_UPDATE_FLAG);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                break;

        default:
                break;
        }

        mc1n2_set_path(codec, &path);

        mc1n2->playback_path = path_num;

        /* Reset call path */
        mc1n2->call_path = PLAYBACK_OFF;

        return 0;
}

static int mc1n2_get_call_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	ucontrol->value.integer.value[0] = mc1n2->call_path;

	return 0;
}

static int mc1n2_set_call_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int path_num = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;
        MCDRV_PATH_INFO path;


#ifdef CONFIG_VOIP
        if (mc1n2_get_voip_status()) {
                CODECDBG("VoIP path = %d", path_num);
                mc1n2_set_playback_path(kcontrol, ucontrol);
                return 0;
        }
#endif

        memset(&path, 0, sizeof(path));
        CODECDBG("path_num %d",path_num);

        switch (path_num) {
        case PLAYBACK_OFF:
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR1_BLOCK] = MCDRV_SRC3_DIR1_OFF;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                break;

        case SPK:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Spk, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_CallSpk, 0);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR1_BLOCK] = MCDRV_SRC3_DIR1_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;

                switch(mc1n2->mic_path) {
                case MIC_MAIN:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                        
                case MIC_SUB:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                        
                default:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                }
                break;

        case RCV:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Rcv, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_CallRcv, 0);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR1_BLOCK] = MCDRV_SRC3_DIR1_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON | MCDRV_SRC5_DAC_R_ON;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;

                switch(mc1n2->mic_path) {
                case MIC_MAIN:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                
                case MIC_SUB:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                
                default:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                }
                break;

        case HP:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Hp, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_CallHp, 0);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR1_BLOCK] = MCDRV_SRC3_DIR1_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;

                switch(mc1n2->mic_path) {
                case MIC_MAIN:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                
                case MIC_SUB:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                
                default:
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                        path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                        break;
                }
                break;

        case BT:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Bt, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_CallBt, 0);

                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR1_BLOCK] = MCDRV_SRC3_DIR1_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
                path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_ON;
                break;

        default:
                break;
        }

        mc1n2_set_path(codec, &path);

        mc1n2->call_path = path_num;

        /* Reset playback back */
        mc1n2->playback_path = PLAYBACK_OFF;

        return 0;
}

static int mc1n2_get_mic_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	ucontrol->value.integer.value[0] = mc1n2->mic_path;

	return 0;
}

static int mc1n2_set_mic_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int path_num = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;
        MCDRV_PATH_INFO path;


        CODECDBG("path_num = %d", path_num);

        memset(&path, 0, sizeof(path));

        switch (path_num) {
        case MIC_MAIN:
#ifdef CONFIG_VOIP
                if (mc1n2_get_voip_status()) {
                        CODECDBG("VoIP : MIC_MAIN");
                        mc1n2_set_voip_mic_parameters(MIC_MAIN);
                } else 
#endif
                {

#ifdef MP3_SNDCONFIG
                        mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, gMicGainTest);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gMicADCTest);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gMicADCTest);
                        CODECDBG("TEST MIC_MAIN : Mic1Gain=%d, MicADC=%d ", gMicGainTest, gMicADCTest);
#else
                        if (voice_recognition_state) {
                                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, gMicGain);
                                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gMicADC);
                                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gMicADC);
                                CODECDBG("Voice Recognition MIC_MAIN : Mic1Gain=%d, MicADC=%d", gMicGain, gMicADC);
                        } else {
                                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, MIC1_GAIN_TEST);
                                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_MIC1_ADC_VOL);
                                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_MIC1_ADC_VOL);
                                CODECDBG("MIC_MAIN : Mic1Gain=%d, MicADC=%d", MIC1_GAIN_TEST, MC1N2_MIC1_ADC_VOL);
                        }
#endif                   

                        msleep (50);

                        path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON; 
                        path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF; 
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
                        path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC3_OFF;
                }
                break; 

        case MIC_SUB:
#ifdef CONFIG_VOIP
                if (mc1n2_get_voip_status()) {
                        CODECDBG("VoIP : MIC_SUB");
                        mc1n2_set_voip_mic_parameters(MIC_SUB);
                } else 
#endif
                {
#ifdef MP3_SNDCONFIG
                        mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, gMicGainTest); 
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gMicADCTest);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gMicADCTest);
                        CODECDBG("TEST MIC_SUB : Mic2Gain=%d, MicADC=%d", gMicGainTest, gMicADCTest);
#else
                        mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, g_MIC2_GAIN_TEST);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, g_MC1N2_MIC2_ADC_VOL);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, g_MC1N2_MIC2_ADC_VOL);
                        CODECDBG("MIC_SUB : Mic2Gain=%d, MicADC=%d", g_MIC2_GAIN_TEST, g_MC1N2_MIC2_ADC_VOL);                        
#endif 

                        msleep(50);

                        path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON; 
                        path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF; 
                        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF; 
                        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF; 
                        path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON;
                }
                break;

        case MIC_BT:
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                break;

        default:
                break;
        }

        mc1n2_set_path(codec, &path);
        
        mc1n2->mic_path = path_num;

        /* For avoiding pop noise when start google voice search */
        if ((mc1n2->mic_path == MIC_MAIN) 
                && (voice_recognition_state)
#ifdef CONFIG_VOIP
                && !mc1n2_get_voip_status()
#endif
        ) {
                CODECDBG("Set delay for voice recognition");
                msleep(200);
        }

        return 0;
}

static int mc1n2_get_fmradio_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	ucontrol->value.integer.value[0] = mc1n2->fmr_path;

	return 0;
}

static int mc1n2_set_fmradio_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int path_num = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;
        MCDRV_PATH_INFO path;
        MCDRV_VOL_INFO vol;


        CODECDBG("mc1n2_set_fmradio_path = %d ",path_num);

        /* same as the previous setting, nothing to do */
        if (mc1n2->fmr_path == path_num) {
                CODECDBG("mc1n2_set_fmradio_path returned");
                return 0;
        }

        memset(&path, 0, sizeof(path));

        switch (path_num) {
        case FMR_OFF:
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;

                fmradio_state = 0;
                break;

        case FMR_SPK:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Spk, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_FmMixSpk, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL_FM);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL_FM);
                CODECDBG("FMR_SPK : SPK gain(%d)", SPEAKER_VOL_FM);

                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_ON;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;

                fmradio_state = 1;
                break;

        case FMR_HP:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Hp, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_FmMixHp, 0);

                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL_FM);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL_FM);
                CODECDBG("FMR_HP : HP gain(%d)", g_HEADPHONE_VOL_FM);
                
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_ON;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;

                fmradio_state = 1;
                break;

        case FMR_SPK_MIX:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Spk, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_FmMixSpk, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL_FM);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL_FM);
                CODECDBG("FMR_SPK_MIX : SPK gain(%d)", SPEAKER_VOL_FM);

                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_ON;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;

                fmradio_state = 1;
                break;

        case FMR_HP_MIX:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Hp, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_FmMixHp, 0);

                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL_FM);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL_FM);
                CODECDBG("FMR_HP_MIX : HP gain(%d)", g_HEADPHONE_VOL_FM);
                
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_ON;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;

                fmradio_state = 1;
                break;

        case FMR_DUAL_MIX:
                _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Dual, 0x1FF);
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_FmMixDual, 0);

                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, SPEAKER_VOL_FM);
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, SPEAKER_VOL_FM);
                CODECDBG("FMR_DUAL_MIX : SPK gain(%d)", SPEAKER_VOL_FM);
                
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, g_HEADPHONE_VOL_FM-18);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, g_HEADPHONE_VOL_FM-18);
                CODECDBG("FMR_DUAL_MIX : HP gain(%d)", g_HEADPHONE_VOL_FM);

                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
                path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_ON;
                path.asAe[0].abSrcOnOff[MCDRV_SRC_MIX_BLOCK] = MCDRV_SRC6_MIX_ON;
                path.asDac[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_ON;
                path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;
                path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_ON;
                path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_ON;

                fmradio_state = 1;
                break;

        default:
                break;
        }

        mc1n2_set_path(codec, &path);

        if (path_num == FMR_OFF) {
                memset(&vol, 0, sizeof(vol));

                vol.aswD_Ad0[0] = cvol[mc1n2->ad_vol_l];
                vol.aswD_Ad0[1] = cvol[mc1n2->ad_vol_l];

                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&vol, 0);
        } else {
                if (path_num == FMR_HP || path_num == FMR_SPK)	{
                        mc1n2->ad_vol_l = FM_ADC_VOL;
                        mc1n2->ad_vol_r = FM_ADC_VOL;

                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, FM_ADC_VOL);
                        mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, FM_ADC_VOL);

#ifdef CONFIG_FMRADIO_CODEC_GAIN
                        mc1n2->vol_store.aswA_Ad0[0] = mc1n2_vol_fm[mc1n2->fm_volume];
                        mc1n2->vol_store.aswA_Ad0[1] = mc1n2_vol_fm[mc1n2->fm_volume];
#else
                        mc1n2->vol_store.aswA_Ad0[0] = FM_FIXED_GAIN;
                        mc1n2->vol_store.aswA_Ad0[1] = FM_FIXED_GAIN;
#endif

                        CODECDBG("MC1N2_ADC_VOL=%d ",FM_ADC_VOL);
                }

#ifdef CONFIG_FMRADIO_CODEC_GAIN
                McDrv_Ctrl_set_fm_vol(mc1n2->fm_volume);
#else
                McDrv_Ctrl_set_fixed_fm_vol();
#endif
        }

        mc1n2->fmr_path = path_num;

        CODECDBG("mc1n2_set_fmradio_path end= %d ",path_num);

        return 0;
}

static int mc1n2_fmradio_path_parameters(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;
	int control_data = ucontrol->value.integer.value[0];


	if (!fmradio_state) {
                CODECDBG("Already closed path, %d",control_data);
		return 0;
	}
        
	CODECDBG("%d",control_data);

	switch (control_data) {
	case CMD_FMR_INPUT_DEACTIVE:
		mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, 0);
		mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, 0);
		break;
                
	case CMD_FMR_INPUT_ACTIVE:
		mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, FM_ADC_VOL);
		mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, FM_ADC_VOL);
		break;
                
	case CMD_FMR_FLAG_CLEAR:
	case CMD_FMR_END:
		mc1n2->fmr_path = FMR_OFF;
		fmradio_state = 0;
                fmradio_closed = 1;
                CODECDBG("Set fmradio_closed flag");
		break;
                
	default:
		break;
	}

	return 0;
}

static int mc1n2_set_recognition_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	MCDRV_PATH_INFO path;
	int control_data = ucontrol->value.integer.value[0];


	CODECDBG("(%d)",control_data);
	memset(&path, 0, sizeof(path));

	if (control_data == CMD_RECOGNITION_DEACTIVE) {
		path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF; 
		path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
		path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
		path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC3_OFF;
                
		mc1n2_set_path(codec, &path);
	}
        
	return 0;
}

#ifdef CONFIG_VOIP
void mc1n2_start_voip(void)
{
        CODECDBG("");
        
        voip_status = 1;

#ifdef LOAD_VOIP_CONFIG
        if (isLoadVoipConfig) {
                if (!ReadVoipConfigFile("/sdcard/soundcfg/voip.txt", 0))
                        isLoadVoipConfig = 0;
        }
#endif
}

void mc1n2_end_voip(void)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        MCDRV_PATH_INFO path;


        CODECDBG("");
        
        memset(&path, 0, sizeof(path));

        mc1n2_write_reg(codec, MC1N2_DAC_VOL_L, MC1N2_VOIP_SPEAKER_DAC);
        mc1n2_write_reg(codec, MC1N2_DAC_MASTER, MC1N2_VOIP_SPEAKER_DAC_MASTER);
        mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, MC1N2_VOIP_HEADSET_DIT);

        /* voip volup recovery */
        _McDrv_Ctrl(MCDRV_SET_AUDIOENGINE, (void *)&stAeInfo_Rcv, 0x1FF);

        /* turn off mic path */
        path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
        path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
        path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC3_OFF;
        mc1n2_set_path(codec, &path);

        voip_status = 0;

        return;       
}

int mc1n2_get_voip_status(void)
{
        return voip_status;
}

void mc1n2_set_voip_playback_parameters(int path)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();


        if (SPK == path) {
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_R, 0);        
                CODECDBG("VOIP_SPK : SPK_R mute");
                
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, gSpeakerGain);
                mc1n2_write_reg(codec, MC1N2_DAC_VOL_L, gSpeakerGainDac);
                mc1n2_write_reg(codec, MC1N2_DAC_MASTER, gSpeakerDacMaster);
                CODECDBG("VOIP_SPK TEST : SPK_L gain(%d)", gSpeakerGain);
#else
                mc1n2_write_reg(codec, MC1N2_SPEAKER_VOL_L, MC1N2_VOIP_SPEAKER_GAIN);
                mc1n2_write_reg(codec, MC1N2_DAC_VOL_L, MC1N2_VOIP_SPEAKER_DAC);
                mc1n2_write_reg(codec, MC1N2_DAC_MASTER, MC1N2_VOIP_SPEAKER_DAC_MASTER);
                CODECDBG("VOIP_SPK : SPK_L gain(%d)", MC1N2_VOIP_SPEAKER_GAIN);
#endif 
        } else if (RCV == path) {
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_RCV_VOL_L, gReceiverGain);
                mc1n2_write_reg(codec, MC1N2_RCV_VOL_R, gReceiverGain);
                CODECDBG("VOIP_RCV TEST : RCV gain(%d)", gReceiverGain);
#else 
                mc1n2_write_reg(codec, MC1N2_RCV_VOL_L, MC1N2_VOIP_RECEIVER_GAIN);
                mc1n2_write_reg(codec, MC1N2_RCV_VOL_R, MC1N2_VOIP_RECEIVER_GAIN);
                CODECDBG("VOIP_RCV : RCV gain(%d)", MC1N2_VOIP_RECEIVER_GAIN);
#endif         
        } else if (HP == path) {
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gHeadPhoneGain);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gHeadPhoneGain);
                CODECDBG("VOIP_HP TEST : HP gain(%d)", gHeadPhoneGain);
#else 
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, MC1N2_VOIP_HEADPHONE_GAIN);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, MC1N2_VOIP_HEADPHONE_GAIN);
                CODECDBG("VOIP_HP : HP gain(%d)", MC1N2_VOIP_HEADPHONE_GAIN);
#endif         
        }        
}

void mc1n2_set_voip_mic_parameters(int micpath)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        MCDRV_PATH_INFO path;

       
        memset(&path, 0, sizeof(path));

        if (MIC_MAIN == micpath) {
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, gSpeaker_mic_gain);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gSpeaker_mic_adc);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gSpeaker_mic_adc);

                CODECDBG("VoIP MIC_MAIN TEST : Mic1Gain=%d, MicADC=%d", 
                        gSpeaker_mic_gain, gSpeaker_mic_adc);
#else
                mc1n2_write_reg(codec, MC1N2_MIC1_GAIN, MC1N2_VOIP_SPEAKER_MIC_GAIN);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_VOIP_SPEAKER_MIC_ADC);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_VOIP_SPEAKER_MIC_ADC);

                CODECDBG("VoIP MIC_MAIN : Mic1Gain=%d, MicADC=%d", 
                        MC1N2_VOIP_SPEAKER_MIC_GAIN, MC1N2_VOIP_SPEAKER_MIC_ADC);
#endif


                msleep (50);

                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON; 
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF; 
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF; 
                path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_ON | MCDRV_SRC0_MIC3_OFF;

                mc1n2_set_path(codec, &path);
        }else if (MIC_SUB == micpath) {   
#ifdef LOAD_VOIP_CONFIG
                mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, gHeadPhone_mic_gain);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, gHeadPhone_mic_adc);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, gHeadPhone_mic_adc);
                mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, gHeadPhone_mic_dit);
                
                CODECDBG("VoIP MIC_SUB TEST : Mic2Gain=%d, MicADC=%d", gHeadPhone_mic_gain, gHeadPhone_mic_adc);
#else
                mc1n2_write_reg(codec, MC1N2_MIC2_GAIN, MC1N2_VOIP_HEADSET_MIC_GAIN);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_L, MC1N2_VOIP_HEADSET_ADC_GAIN);
                mc1n2_write_reg(codec, MC1N2_ADC_VOL_R, MC1N2_VOIP_HEADSET_ADC_GAIN);
                mc1n2_write_reg(codec, MC1N2_DAC_DAT_VAL, MC1N2_VOIP_HEADSET_DIT);

                CODECDBG("VoIP MIC_SUB : Mic2Gain=%d, MicADC=%d", MC1N2_VOIP_HEADSET_MIC_GAIN, MC1N2_VOIP_HEADSET_ADC_GAIN);
#endif

                msleep(50);

                path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_ON;
                path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
                path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON | MCDRV_SRC0_MIC3_OFF;
                path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_ON;

                mc1n2_set_path(codec, &path);
        }        
}
#endif

static int mc1n2_get_codec_tuning(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	ucontrol->value.integer.value[0] = mc1n2->codec_tuning;

	return mc1n2->config_flag;
}

static int mc1n2_set_codec_tuning(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int tuning = ucontrol->value.integer.value[0];
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;

	mc1n2->config_flag = tuning;

	mc1n2->codec_tuning = tuning;

	return 0;
}

#ifdef CONFIG_MUSIC_CODEC_GAIN
static int mc1n2_get_music_volume(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        CODECDBG("Get music volume = [%d]", mc1n2->music_vol);

        ucontrol->value.integer.value[0] = mc1n2->music_vol;

        return 0;
}

void mc1n2_set_music_codec_gain(int volume)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        if (music_start_flag
                && (mc1n2->playback_path == HP)
                && (mc1n2->fmr_path == FMR_OFF)
#ifdef CONFIG_VOIP
                && !mc1n2_get_voip_status()
#endif
                && (volume != mc1n2->analog_vol)
        ) {

                mc1n2->analog_vol = volume;
                
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gAnalVolHpIndex[mc1n2->analog_vol]);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gAnalVolHpIndex[mc1n2->analog_vol]);

                CODECDBG (" : HP gain(%d)", gAnalVolHpIndex[mc1n2->analog_vol]);
        }
}

static int mc1n2_set_music_volume(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int vol = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        if (vol >= MUSIC_VOL_LEVEL)
                mc1n2->music_vol = MUSIC_VOL_LEVEL-1;
        else if (vol < 0)
                mc1n2->music_vol = 0;
        else
                mc1n2->music_vol = vol;


        CODECDBG("Set music volume = [%d]", mc1n2->music_vol);

        mc1n2_set_music_codec_gain(mc1n2->music_vol);

        return 0;
}

void mc1n2_set_default_codec_gain(void)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        mc1n2->analog_vol = MUSIC_DEFAUTL_VOL_LEVEL;

        if ((mc1n2->playback_path == HP)
                && (mc1n2->fmr_path == FMR_OFF)
#ifdef CONFIG_VOIP
                && !mc1n2_get_voip_status()
#endif
        ) {

                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_L, gAnalVolHpIndex[mc1n2->analog_vol]);
                mc1n2_write_reg(codec, MC1N2_HEADPHONE_VOL_R, gAnalVolHpIndex[mc1n2->analog_vol]);

                CODECDBG (" : HP gain(%d)", gAnalVolHpIndex[mc1n2->analog_vol]);
        }
}
#endif

static int mc1n2_get_output_source(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        return 0;
}

static int mc1n2_set_output_source(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)        
{
        return 0;
}

static int mc1n2_get_codec_status(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;

        ucontrol->value.integer.value[0] = mc1n2->codec_status;

        return 0;
}

static int mc1n2_set_codec_status(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int control_data = ucontrol->value.integer.value[0];
        struct snd_soc_codec *codec = mc1n2_get_codec_data();
        struct mc1n2_data *mc1n2 = codec->drvdata;


        CODECDBG(" (%d)", control_data);
        
        switch(control_data) {
        case CMD_FMR_INPUT_DEACTIVE:
        case CMD_FMR_INPUT_ACTIVE:
        case CMD_FMR_FLAG_CLEAR:
        case CMD_FMR_END:
                mc1n2_fmradio_path_parameters(kcontrol, ucontrol);
                break;
        
        case CMD_RECOGNITION_DEACTIVE:
                CODECDBG("CMD_RECOGNITION_DEACTIVE");
                voice_recognition_state = 0;
                
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_RecognitionOff, 0);
                mc1n2_set_recognition_path(kcontrol, ucontrol);
                break;
        
        case CMD_RECOGNITION_ACTIVE:
                CODECDBG("CMD_RECOGNITION_ACTIVE");
                voice_recognition_state = 1;
                
                stVolInfo_RecognitionOn.aswA_Sp[0] = pvol[SPEAKER_VOL];
                stVolInfo_RecognitionOn.aswA_Sp[1] = pvol[SPEAKER_VOL];

#ifdef CONFIG_MUSIC_CODEC_GAIN
                stVolInfo_RecognitionOn.aswA_Hp[0] = pvol[gAnalVolHpIndex[mc1n2->analog_vol]];
                stVolInfo_RecognitionOn.aswA_Hp[1] = pvol[gAnalVolHpIndex[mc1n2->analog_vol]];
#else
                stVolInfo_RecognitionOn.aswA_Hp[0] = pvol[g_HEADPHONE_VOL];
                stVolInfo_RecognitionOn.aswA_Hp[1] = pvol[g_HEADPHONE_VOL];
#endif
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_RecognitionOn, 0);
                break;
        
        case CMD_CALL_FLAG_CLEAR:
                CODECDBG("CMD_CALL_FLAG_CLEAR ");
                break;
        
        case CMD_CALL_END:
                CODECDBG("CMD_CALL_END");
                break;

#ifdef CONFIG_MUSIC_CODEC_GAIN
	case CMD_MUSIC_START:
		CODECDBG("CMD_MUSIC_START (Volume=%d)", mc1n2->music_vol);
		music_start_flag = 1;
                mc1n2_set_music_codec_gain(mc1n2->music_vol);
		break;

	case CMD_MUSIC_STOP:
		CODECDBG("CMD_MUSIC_STOP");
		music_start_flag = 0;
                mc1n2_set_default_codec_gain();
		break;
#endif

#ifdef CONFIG_VOIP
        case CMD_VOIP_START:
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_VoipOn, 0);	
                CODECDBG("CMD_VOIP_START");
                mc1n2_start_voip();
                break;
        
        case CMD_VOIP_STOP:
                CODECDBG("CMD_VOIP_STOP");
                _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_VoipOff, 0);
                mc1n2_end_voip();             
                break;
#endif

        default:
                break;
        }

        mc1n2->codec_status = control_data;
        return 0;
}

static const unsigned int mc1n2_tlv_digital_rcv[] = {TLV_DB_LINEAR_ITEM(-5700, 600)};
static const unsigned int mc1n2_tlv_digital_spkr[] = {TLV_DB_LINEAR_ITEM(-5700, 600)};
static const unsigned int mc1n2_tlv_digital_headphone[] = {TLV_DB_LINEAR_ITEM(-5700, 600)};
static const unsigned int mc1n2_tlv_digital_mic[] = {TLV_DB_LINEAR_ITEM(-7162, 1762)};

static const char *playback_path[] = {
        "OFF", "RCV", "SPK", "HP", "BT", "DUAL", "RING_SPK", "RING_HP", 
        "RING_DUAL", "EXTRA_DOCK_SPEAKER", "TV_OUT"
};
static const char *voicecall_path[] = {
        "OFF", "RCV", "SPK", "HP", "BT"
};
static const char *mic_path[] = {
        "Main Mic", "Hands Free Mic"
};
static const char *fmradio_path[] = {
        "FMR_OFF", "FMR_SPK", "FMR_HP", "FMR_SPK_MIX", "FMR_HP_MIX", "FMR_DUAL_MIX"
};
static const char *codec_tuning_control[] = {
        "OFF", "ON"
};
static const char *codec_status_control[] = {
        "FMR_VOL_0", "FMR_VOL_1", "FMR_OFF", "REC_OFF", "REC_ON"
};
static const char *output_source_state[] = {
        "Default Output", "Ring Tone", "VoIP Output"
};
#ifdef CONFIG_MUSIC_CODEC_GAIN
static const char *music_volume[] = {
        "MUSIC_VOL_0", "MUSIC_VOL_MAX"
};
#endif


static const struct soc_enum path_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(playback_path), playback_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voicecall_path), voicecall_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mic_path), mic_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fmradio_path), fmradio_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(codec_tuning_control), codec_tuning_control),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(codec_status_control), codec_status_control),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(output_source_state), output_source_state),
#ifdef CONFIG_MUSIC_CODEC_GAIN
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(music_volume), music_volume),
#endif
};

static const struct snd_kcontrol_new mc1n2_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Playback Volume", MC1N2_RCV_VOL_L, MC1N2_RCV_VOL_R, 0, 0x3F, 0, mc1n2_tlv_digital_rcv),
	SOC_DOUBLE_R_TLV("Playback Spkr Volume", MC1N2_SPEAKER_VOL_L, MC1N2_SPEAKER_VOL_R, 0, 0x3F, 0, mc1n2_tlv_digital_spkr),
	SOC_DOUBLE_R_TLV("Playback Headset Volume", MC1N2_HEADPHONE_VOL_L, MC1N2_HEADPHONE_VOL_R, 0, 0x3F, 0, mc1n2_tlv_digital_headphone),
	SOC_SINGLE_TLV("Capture Volume", MC1N2_ADC_VOL_L, 0, 0xEF, 0, mc1n2_tlv_digital_mic),
	SOC_ENUM_EXT("Playback Path", path_control_enum[0], mc1n2_get_playback_path, mc1n2_set_playback_path),
	SOC_ENUM_EXT("Voice Call Path", path_control_enum[1], mc1n2_get_call_path, mc1n2_set_call_path),
	SOC_ENUM_EXT("Capture MIC Path", path_control_enum[2], mc1n2_get_mic_path, mc1n2_set_mic_path),
	SOC_ENUM_EXT("FM Radio Path", path_control_enum[3], mc1n2_get_fmradio_path, mc1n2_set_fmradio_path),
	SOC_ENUM_EXT("Codec Tuning", path_control_enum[4], mc1n2_get_codec_tuning, mc1n2_set_codec_tuning),
	SOC_ENUM_EXT("Codec Status", path_control_enum[5], mc1n2_get_codec_status, mc1n2_set_codec_status),
	SOC_ENUM_EXT("Output Source", path_control_enum[6], mc1n2_get_output_source, mc1n2_set_output_source),
#ifdef CONFIG_MUSIC_CODEC_GAIN
	SOC_ENUM_EXT("Music Volume", path_control_enum[7], mc1n2_get_music_volume, mc1n2_set_music_volume),
#endif
};

/*
 * Same as snd_soc_add_controls supported in alsa-driver 1.0.19 or later.
 * This function is implimented for compatibility with linux 2.6.29.
 */
static int mc1n2_add_controls(struct snd_soc_codec *codec,
			      const struct snd_kcontrol_new *controls, int n)
{
	int err, i;

	for (i = 0; i < n; i++, controls++) {
		if ((err = snd_ctl_add(codec->card,
				       snd_soc_cnew(controls, codec, NULL))) < 0) {
			return err;
		}
	}

	return 0;
}

int audio_power(int en)
{
	u32 val;


	CODECDBG("(%d)", en);
        
	if (en) {
		/* Forbid to turn off MCLK in sleep mode */
		val = __raw_readl(S5P_SLEEP_CFG);
		val |= (S5P_SLEEP_CFG_USBOSC_EN);
		__raw_writel(val , S5P_SLEEP_CFG);

                /* Wait for warming up */
		msleep(10);

		/* Turn on master clock */
                __raw_writel(__raw_readl(S5P_OTHERS) | (3<<8) , S5P_OTHERS);
                __raw_writel(__raw_readl(S5P_CLK_OUT) | (0x1) , S5P_CLK_OUT);

                /* Enable ear-path switch */
                gpio_set_value(GPIO_MUTE_ON, 1);
	} else {
		MCDRV_PATH_INFO path;
		memset(&path, 0, sizeof(path));

                /* Disable ear-path switch */
                gpio_set_value(GPIO_MUTE_ON, 0);

		/* MIC off */
		path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
		path.asDit0[0].abSrcOnOff[MCDRV_SRC_DIR2_BLOCK] = MCDRV_SRC3_DIR2_OFF;
		path.asAdc0[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
		path.asAdc0[1].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF | MCDRV_SRC0_MIC3_OFF;
                
                if (get_headset_status() == 1) // 4pole type.
        		path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF;
                else
        		path.asBias[0].abSrcOnOff[MCDRV_SRC_MIC1_BLOCK] = MCDRV_SRC0_MIC1_OFF | MCDRV_SRC0_MIC2_OFF;

		/* Lineout off */
		path.asDit0[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;
		path.asAdc0[0].abSrcOnOff[MCDRV_SRC_LINE1_L_BLOCK] = MCDRV_SRC1_LINE1_L_OFF;
		path.asAdc0[1].abSrcOnOff[MCDRV_SRC_LINE1_R_BLOCK] = MCDRV_SRC1_LINE1_R_OFF;
		path.asMix[0].abSrcOnOff[MCDRV_SRC_ADC0_BLOCK] = MCDRV_SRC4_ADC0_OFF;

		/* Playback off */
		path.asMix[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
		path.asSpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
		path.asSpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
		path.asHpOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
		path.asHpOut[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
		path.asRcOut[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF | MCDRV_SRC5_DAC_R_OFF;
		path.asLout1[0].abSrcOnOff[MCDRV_SRC_DAC_L_BLOCK] = MCDRV_SRC5_DAC_L_OFF;
		path.asLout1[1].abSrcOnOff[MCDRV_SRC_DAC_R_BLOCK] = MCDRV_SRC5_DAC_R_OFF;
		path.asDit2[0].abSrcOnOff[MCDRV_SRC_AE_BLOCK] = MCDRV_SRC6_AE_OFF;
		path.asDit1[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
		path.asDac[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
		path.asDac[1].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
		path.asAe[0].abSrcOnOff[MCDRV_SRC_DIR0_BLOCK] = MCDRV_SRC3_DIR0_OFF;
		_McDrv_Ctrl(MCDRV_SET_PATH, &path, 0);
		
		/* Turn off master clock */
		__raw_writel(__raw_readl(S5P_OTHERS) & (~(0x3<<8)) , S5P_OTHERS);
		__raw_writel(__raw_readl(S5P_CLK_OUT) & (0xFFFFFFFE) , S5P_CLK_OUT);

		/* Allow to turn off MCLK in sleep mode */
		val = __raw_readl(S5P_SLEEP_CFG);
		val &= ~(S5P_SLEEP_CFG_USBOSC_EN);
		__raw_writel(val , S5P_SLEEP_CFG);
	}
	
	return 0;
}


/*
 * Codec device
 */
static int mc1n2_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = mc1n2_get_codec_data();
	struct mc1n2_data *mc1n2 = codec->drvdata;
	struct mc1n2_setup *setup = socdev->codec_data;
	int err;
	UINT32 update = 0;

	TRACE_FUNC();
	CODECDBG("");

	audio_power(1);

	if (!codec) {
		dev_err(socdev->dev, "I2C bus is not probed successfully\n");
		err = -ENODEV;
		goto error_codec_data;
	}
        
#ifdef ALSA_VER_1_0_19
	socdev->codec = codec;
#else
	socdev->card->codec = codec;
#endif

	/* init hardware */
	if (!setup) {
		dev_err(socdev->dev, "No initialization parameters given\n");
		err = -EINVAL;
		goto error_init_hw;
	}
	memcpy(&mc1n2->setup, setup, sizeof(struct mc1n2_setup));
	err = _McDrv_Ctrl(MCDRV_INIT, &mc1n2->setup.init, 0);
	if (err != MCDRV_SUCCESS) {
		dev_err(socdev->dev, "%d: Error in MCDRV_INIT\n", err);
		err = -EIO;
		goto error_init_hw;
	}

	/* pcm */
	err = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in snd_soc_new_pcms\n", err);
		goto error_new_pcm;
	}

	/* controls */
	err = mc1n2_add_controls(codec, mc1n2_snd_controls,
				 ARRAY_SIZE(mc1n2_snd_controls));
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in mc1n2_add_controls\n", err);
		goto error_add_ctl;
	}

#if 0
	err = mc1n2_add_widgets(codec);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in mc1n2_add_widgets\n", err);
		CODECDBG(KERN_ALERT "mc1n2_add_widgets error\n");
		goto error_add_ctl;
	}
#endif

#if (defined ALSA_VER_1_0_19) || (defined ALSA_VER_1_0_21)
	err = snd_soc_init_card(socdev);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in snd_soc_init_card\n", err);
		goto error_init_card;
	}
#endif

#ifndef DIO0_DAI_ENABLE
	update |= (MCDRV_DIO0_COM_UPDATE_FLAG | MCDRV_DIO0_DIR_UPDATE_FLAG | MCDRV_DIO0_DIT_UPDATE_FLAG);
#endif

#ifndef DIO1_DAI_ENABLE
	update |= (MCDRV_DIO1_COM_UPDATE_FLAG | MCDRV_DIO1_DIR_UPDATE_FLAG | MCDRV_DIO1_DIT_UPDATE_FLAG);
#endif

#ifndef DIO2_DAI_ENABLE
	update |= (MCDRV_DIO2_COM_UPDATE_FLAG | MCDRV_DIO2_DIR_UPDATE_FLAG | MCDRV_DIO2_DIT_UPDATE_FLAG);
#endif

	err = _McDrv_Ctrl(MCDRV_SET_DIGITALIO, (void *)&stDioInfo_Default, update);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_DIGITALIO\n", err);
		goto error_set_mode;
	}

	err = _McDrv_Ctrl(MCDRV_SET_DAC, (void *)&stDacInfo_Default, 0x7);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_DAC\n", err);
		goto error_set_mode;
	}

	err = _McDrv_Ctrl(MCDRV_SET_ADC, (void *)&stAdcInfo_Default, 0x7);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_ADC\n", err);
		goto error_set_mode;
	}

	err = _McDrv_Ctrl(MCDRV_SET_SP, (void *)&stSpInfo_Default, 0);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_SP\n", err);
		goto error_set_mode;
	}

	err = _McDrv_Ctrl(MCDRV_SET_DNG, (void *)&stDngInfo_Default, 0x3F3F3F);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_DNG\n", err);
		goto error_set_mode;
	}

	if (mc1n2_hwid == MC1N2_HW_ID_AB) {
		err = _McDrv_Ctrl(MCDRV_SET_SYSEQ, (void *)&stSyseqInfo_Default, 0x3);

		if (err < 0) {
			dev_err(socdev->dev, "%d: Error in MCDRV_SET_SYSEQ\n", err);
			goto error_set_mode;
		}
	}

	err = _McDrv_Ctrl(MCDRV_SET_VOLUME, (void *)&stVolInfo_Default, 0);
	if (err < 0) {
		dev_err(socdev->dev, "%d: Error in MCDRV_SET_DNG\n", err);
		goto error_set_mode;
	}

        /* analog volume init*/
#ifdef CONFIG_MUSIC_CODEC_GAIN
        mc1n2->analog_vol = MUSIC_DEFAUTL_VOL_LEVEL; 
#endif
#ifdef CONFIG_FMRADIO_CODEC_GAIN
        mc1n2->fm_volume = 0;
#endif

	return 0;
	
error_set_mode:
#if (defined ALSA_VER_1_0_19) || (defined ALSA_VER_1_0_21)
error_init_card:
#endif
error_add_ctl:
	snd_soc_free_pcms(socdev);
error_new_pcm:
	_McDrv_Ctrl(MCDRV_TERM, NULL, 0);
error_init_hw:
#ifdef ALSA_VER_1_0_19
	socdev->codec = NULL;
#else
	socdev->card->codec = NULL;
#endif
error_codec_data:
	return err;
}

static int mc1n2_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	int err;

	TRACE_FUNC();

#ifdef ALSA_VER_1_0_19
	if (socdev->codec) {
#else
	if (socdev->card->codec) {
#endif
		snd_soc_free_pcms(socdev);

		err = _McDrv_Ctrl(MCDRV_TERM, NULL, 0);
		if (err != MCDRV_SUCCESS) {
			dev_err(socdev->dev, "%d: Error in MCDRV_TERM\n", err);
			return -EIO;
		}
	}

	return 0;
}

struct mc1n2_info_store {
	UINT32 get;
	UINT32 set;
	size_t offset;
	UINT32 flags;
};

struct mc1n2_info_store mc1n2_info_store_tbl[] = {
	{MCDRV_GET_DIGITALIO, MCDRV_SET_DIGITALIO,
	 offsetof(struct mc1n2_data, dio_store), 0x1ff}, //20101012 yamaha
	{MCDRV_GET_DAC, MCDRV_SET_DAC,
	 offsetof(struct mc1n2_data, dac_store), 0x7},
	{MCDRV_GET_ADC, MCDRV_SET_ADC,
	 offsetof(struct mc1n2_data, adc_store), 0x7},
	{MCDRV_GET_SP, MCDRV_SET_SP,
	 offsetof(struct mc1n2_data, sp_store), 0},
	{MCDRV_GET_DNG, MCDRV_SET_DNG,
	 offsetof(struct mc1n2_data, dng_store), 0x3f3f3f},
	{MCDRV_GET_SYSEQ, MCDRV_SET_SYSEQ,
	 offsetof(struct mc1n2_data, syseq_store), 0x3},
	{0, MCDRV_SET_AUDIOENGINE,
	 offsetof(struct mc1n2_data, ae_store), 0x1ff},
	{MCDRV_GET_PDM, MCDRV_SET_PDM,
	 offsetof(struct mc1n2_data, pdm_store), 0x7f},
	{MCDRV_GET_PATH, MCDRV_SET_PATH,
	 offsetof(struct mc1n2_data, path_store), 0},
	{MCDRV_GET_VOLUME, MCDRV_SET_VOLUME,
	 offsetof(struct mc1n2_data, vol_store), 0},
};
#define MC1N2_N_INFO_STORE (sizeof(mc1n2_info_store_tbl) / sizeof(struct mc1n2_info_store))

static int mc1n2_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
#ifdef ALSA_VER_1_0_19
	struct snd_soc_codec *codec = socdev->codec;
#else
	struct snd_soc_codec *codec = socdev->card->codec;
#endif
	struct mc1n2_data *mc1n2 = codec->drvdata;
	int err, i;

	TRACE_FUNC();

	if (fmradio_state) /* Do not enter suspend mode during FM radio */
		return 0;
	

	mutex_lock(&mc1n2->mutex);

	/* store parameters */
	for (i = 0; i < MC1N2_N_INFO_STORE; i++) {
		struct mc1n2_info_store *store = &mc1n2_info_store_tbl[i];
		if (store->get) {
			err = _McDrv_Ctrl(store->get, (void *)mc1n2 + store->offset, 0);
			if (err != MCDRV_SUCCESS) {
				dev_err(codec->dev,
					"%d: Error in mc1n2_suspend\n", err);
				err = -EIO;
				goto error;
			} else {
				err = 0;
			}
		}
	}

#if 0
	err = _McDrv_Ctrl(MCDRV_TERM, NULL, 0);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in MCDRV_TERM\n", err);
		err = -EIO;
	} else {
		err = 0;
	}
#endif

	audio_power(0);

error:
	mutex_unlock(&mc1n2->mutex);

	return err;
}

static int mc1n2_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
#ifdef ALSA_VER_1_0_19
	struct snd_soc_codec *codec = socdev->codec;
#else
	struct snd_soc_codec *codec = socdev->card->codec;
#endif
	struct mc1n2_data *mc1n2 = codec->drvdata;
	SINT16 *vol = (SINT16 *)&mc1n2->vol_store;
	int err, i;

	TRACE_FUNC();

	if (fmradio_state) /* Do not enter resume mode during FM radio */
		return 0;

	audio_power(1);
	
	mutex_lock(&mc1n2->mutex);

#if 0
	err = _McDrv_Ctrl(MCDRV_INIT, &mc1n2->setup.init, 0);
	if (err != MCDRV_SUCCESS) {
		dev_err(codec->dev, "%d: Error in MCDRV_INIT\n", err);
		err = -EIO;
		goto error;
	} else {
		err = 0;
	}
#endif

	/* restore parameters */
	for (i = 0; i < sizeof(MCDRV_VOL_INFO)/sizeof(SINT16); i++, vol++) {
		*vol |= 0x0001;
	}

	for (i = 0; i < MC1N2_N_INFO_STORE; i++) {
		struct mc1n2_info_store *store = &mc1n2_info_store_tbl[i];
		if (store->set) {
			err = _McDrv_Ctrl(store->set, (void *)mc1n2 + store->offset,
					  store->flags);
			if (err != MCDRV_SUCCESS) {
				dev_err(codec->dev,
					"%d: Error in mc1n2_resume\n", err);
				err = -EIO;
				goto error;
			} else {
				err = 0;
			}
		}
	}

error:
	mutex_unlock(&mc1n2->mutex);

	return err;
}

struct snd_soc_codec_device soc_codec_dev_mc1n2 = {
	.probe = mc1n2_probe,
	.remove = mc1n2_remove,
#ifdef CONFIG_PM	
	.suspend = mc1n2_suspend,
	.resume = mc1n2_resume
#endif
};
EXPORT_SYMBOL_GPL(soc_codec_dev_mc1n2);

/*
 * I2C client
 */
static int mc1n2_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	UINT8	bHwid = mc1n2_i2c_read_byte(client, 8);

	if (bHwid != MC1N2_HW_ID_AB && bHwid != MC1N2_HW_ID_AA) {
		return -ENODEV;
	}
        
	mc1n2_hwid = bHwid;

	return 0;
}

static int mc1n2_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct snd_soc_codec *codec;
	struct mc1n2_data *mc1n2;
	int err, i;

	TRACE_FUNC();
	CODECDBG("");

	/* setup codec data */
	if (!(codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL))) {
		err = -ENOMEM;
		goto err_alloc_codec;
	}
	codec->name = MC1N2_NAME;
	codec->owner = THIS_MODULE;
	mutex_init(&codec->mutex);
	codec->dev = &client->dev;

	if (!(mc1n2 = kzalloc(sizeof(struct mc1n2_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto err_alloc_data;
	}
	mutex_init(&mc1n2->mutex);
	codec->drvdata = mc1n2;

	client->adapter = i2c_get_adapter(4);

	/* setup i2c client data */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto err_i2c;
	}
	if ((err = mc1n2_i2c_detect(client, NULL)) < 0) {
		goto err_i2c;
	}
	i2c_set_clientdata(client, codec);

	codec->control_data = client;
	codec->read = mc1n2_read_reg;
	codec->write = mc1n2_write_reg;
	codec->hw_write = NULL;
	codec->hw_read = NULL;
	codec->reg_cache = kzalloc(sizeof(u16) * MC1N2_N_REG, GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		err = -ENOMEM;
		goto err_alloc_cache;
	}
	codec->reg_cache_size = MC1N2_N_REG;
	codec->reg_cache_step = 1;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->dai = mc1n2_dai;
	codec->num_dai = ARRAY_SIZE(mc1n2_dai);
	mc1n2_set_codec_data(codec);

	if ((err = snd_soc_register_codec(codec)) < 0) {
		goto err_reg_codec;
	}

	/* setup DAI data */
	for (i = 0; i < ARRAY_SIZE(mc1n2_dai); i++) {
		mc1n2_dai[i].dev = &client->dev;
	}
        if ((err = snd_soc_register_dais(mc1n2_dai, ARRAY_SIZE(mc1n2_dai))) < 0) {
		goto err_reg_dai;
	}

	return 0;

err_reg_dai:
	snd_soc_unregister_codec(codec);
err_reg_codec:
	kfree(codec->reg_cache);
err_alloc_cache:
	i2c_set_clientdata(client, NULL);
err_i2c:
	kfree(mc1n2);
err_alloc_data:
	kfree(codec);
err_alloc_codec:
	dev_err(&client->dev, "err=%d: failed to probe MC-1N2\n", err);
	return err;
}

static int mc1n2_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	struct mc1n2_data *mc1n2;

	TRACE_FUNC();

	if (codec) {
		mc1n2 = codec->drvdata;
		snd_soc_unregister_dais(mc1n2_dai, ARRAY_SIZE(mc1n2_dai));
		snd_soc_unregister_codec(codec);

		mutex_destroy(&mc1n2->mutex);
		kfree(mc1n2);

		mutex_destroy(&codec->mutex);
		kfree(codec);
	}

	return 0;
}

static const struct i2c_device_id mc1n2_i2c_id[] = {
	{MC1N2_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mc1n2_i2c_id);

static struct i2c_driver mc1n2_i2c_driver = {
	.driver = {
		.name = MC1N2_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mc1n2_i2c_probe,
	.remove = mc1n2_i2c_remove,
	.id_table = mc1n2_i2c_id,
};

/*
 * Module init and exit
 */
static int __init mc1n2_init(void)
{
	return i2c_add_driver(&mc1n2_i2c_driver);
}
module_init(mc1n2_init);

static void __exit mc1n2_exit(void)
{
	i2c_del_driver(&mc1n2_i2c_driver);
}
module_exit(mc1n2_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_DESCRIPTION("Yamaha MC-1N2 ALSA SoC codec driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MC1N2_DRIVER_VERSION);
