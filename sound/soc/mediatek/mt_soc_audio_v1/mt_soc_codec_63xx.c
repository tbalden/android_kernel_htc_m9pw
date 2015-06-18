/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */





#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "mt_soc_analog_type.h"
#include <mach/mt_clkbuf_ctl.h>
#include <sound/mt_soc_audio.h>
#include <mach/vow_api.h>
#ifdef CONFIG_MTK_SPEAKER
#include "mt_soc_codec_speaker_63xx.h"
#endif

#include "mt_soc_pcm_common.h"

#include <sound/htc_acoustic_alsa.h>
#ifdef CONFIG_HTC_RECEIVER
#include <linux/htc_receiver.h>
#endif


#ifndef CONFIG_MTK_FPGA
extern void mt6331_upmu_set_rg_audmicbias1lowpen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audmicbias1dcswnen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audmicbias1dcswpen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audpwdbmicbias1(kal_uint32 val);

extern void mt6331_upmu_set_rg_audmicbias0lowpen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audmicbias0dcswnen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audmicbias0dcswpen(kal_uint32 val);
extern void mt6331_upmu_set_rg_audpwdbmicbias0(kal_uint32 val);
#endif

static void HeadsetRVolumeSet(void);
static void HeadsetLVolumeSet(void);
static bool AudioPreAmp1_Sel(int Mul_Sel);
static bool GetAdcStatus(void);
static void Apply_Speaker_Gain(void);
static bool TurnOnVOWDigitalHW(bool enable);
static void TurnOffDacPower(void);
static void TurnOnDacPower(void);
static void OpenClassH(void);
static void OpenClassAB(void);
static void SetDcCompenSation(void);
static bool GetDLStatus(void);
static void EnableMicBias(uint32_t Mic, bool bEnable);
static void OpenMicbias0(bool bEanble);
static void OpenMicbias1(bool bEanble);
static void OpenMicbias2(bool bEanble);
static void OpenMicbias3(bool bEanble);
static void Audio_ADC1_Set_Input(int Input);
static void Audio_ADC2_Set_Input(int Input);
static void Audio_ADC3_Set_Input(int Input);
static void Audio_ADC4_Set_Input(int Input);
static void Voice_Amp_Change(bool enable);
static void Speaker_Amp_Change(bool enable);
static bool TurnOnVOWADcPowerACC(int ADCType, bool enable);
static void Audio_Amp_Change(int channels , bool enable);
void OpenAnalogHeadphone(bool bEnable);


#ifndef CONFIG_MTK_FPGA
extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
#endif

static mt6331_Codec_Data_Priv *mCodec_data = NULL;
static uint32 mBlockSampleRate[AUDIO_ANALOG_DEVICE_INOUT_MAX] = {48000, 48000, 48000};
static DEFINE_MUTEX(Ana_Ctrl_Mutex);
static DEFINE_MUTEX(Ana_buf_Ctrl_Mutex);
static DEFINE_MUTEX(Ana_Clk_Mutex);
static DEFINE_MUTEX(Ana_Power_Mutex);
static DEFINE_MUTEX(AudAna_lock);

static int mAudio_Analog_Mic1_mode  = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic2_mode  = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic3_mode  = AUDIO_ANALOGUL_MODE_ACC;
static int mAudio_Analog_Mic4_mode  = AUDIO_ANALOGUL_MODE_ACC;

static int mAudio_Vow_Analog_Func_Enable = false;
static int mAudio_Vow_Digital_Func_Enable = false;
static int TrimOffset = 2048;
static const int DC1unit_in_uv = 19184; 
static const int DC1devider = 8; 

#ifdef CONFIG_MTK_SPEAKER
static int Speaker_mode = AUDIO_SPEAKER_MODE_AB;
static unsigned int Speaker_pga_gain = 1 ; 
static bool mSpeaker_Ocflag = false;
static int mEnableAuxAdc = 0;
#endif
static int mAdc_Power_Mode = 0;
static unsigned int dAuxAdcChannel = 16;
static const int mDcOffsetTrimChannel = 9;
static bool mInitCodec = false;

static int reg_AFE_VOW_CFG0 = 0xffff;   
static int reg_AFE_VOW_CFG1 = 0x0200;   
static int reg_AFE_VOW_CFG2 = 0x2424;   
static int reg_AFE_VOW_CFG3 = 0x8767;   
static int reg_AFE_VOW_CFG4 = 0x000c;   
static int reg_AFE_VOW_CFG5 = 0x0000;   

static void  SavePowerState(void)
{
    int i = 0;
    for (i = 0; i < AUDIO_ANALOG_VOLUME_TYPE_MAX ; i++)
    {
        mCodec_data->mAudio_BackUpAna_DevicePower[i] = mCodec_data->mAudio_Ana_DevicePower[i];
    }
}

static void  RestorePowerState(void)
{
    int i = 0;
    for (i = 0; i < AUDIO_ANALOG_VOLUME_TYPE_MAX ; i++)
    {
        mCodec_data->mAudio_Ana_DevicePower[i] = mCodec_data->mAudio_BackUpAna_DevicePower[i];
    }
}


static bool mAnaSuspend = false;
void SetAnalogSuspend(bool bEnable)
{
    printk("+%s bEnable ==%d mAnaSuspend = %d \n", __func__, bEnable, mAnaSuspend);
    if ((bEnable == true) && (mAnaSuspend == false))
    {
        
        SavePowerState();
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == true)
        {
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = false;
            Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , false);
        }
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == true)
        {
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = false;
            Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , false);
        }
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] == true)
        {
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = false;
            Voice_Amp_Change(false);
        }
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] == true)
        {
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = false;
            Speaker_Amp_Change(false);
        }
        
        mAnaSuspend = true;
    }
    else if ((bEnable == false) && (mAnaSuspend == true))
    {
        
        if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == true)
        {
            Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , true);
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = true;
        }
        if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == true)
        {
            Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , true);
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = false;
        }
        if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] == true)
        {
            Voice_Amp_Change(true);
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = false;
        }
        if (mCodec_data->mAudio_BackUpAna_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] == true)
        {
            Speaker_Amp_Change(true);
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = false;
        }
        RestorePowerState();
        
        mAnaSuspend = false;
    }
    printk("-%s bEnable ==%d mAnaSuspend = %d \n", __func__, bEnable, mAnaSuspend);
}


static int audck_buf_Count = 0;
void audckbufEnable(bool enable)
{
    printk("audckbufEnable audck_buf_Count = %d enable = %d \n", audck_buf_Count, enable);
    mutex_lock(&Ana_buf_Ctrl_Mutex);
    if (enable)
    {
        if (audck_buf_Count == 0)
        {
            printk("+clk_buf_ctrl(CLK_BUF_AUDIO,true)\n");
            clk_buf_ctrl(CLK_BUF_AUDIO, true);
            printk("-clk_buf_ctrl(CLK_BUF_AUDIO,true)\n");
        }
        audck_buf_Count++;
    }
    else
    {
        audck_buf_Count--;
        if (audck_buf_Count == 0)
        {
            printk("+clk_buf_ctrl(CLK_BUF_AUDIO,false)\n");
            clk_buf_ctrl(CLK_BUF_AUDIO, false);
            printk("-clk_buf_ctrl(CLK_BUF_AUDIO,false)\n");
        }
        if (audck_buf_Count < 0)
        {
            printk("audck_buf_Count count <0 \n");
            audck_buf_Count = 0;
        }
    }
    mutex_unlock(&Ana_buf_Ctrl_Mutex);
}

static int ClsqAuxCount = 0;
static void ClsqAuxEnable(bool enable)
{
    printk("ClsqAuxEnable ClsqAuxCount = %d enable = %d \n", ClsqAuxCount, enable);
    mutex_lock(& AudAna_lock);
    if (enable)
    {
        if (ClsqAuxCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ, 0x0002, 0x0002); 
        }
        ClsqAuxCount++;
    }
    else
    {
        ClsqAuxCount--;
        if (ClsqAuxCount < 0)
        {
            printk("ClsqAuxEnable count <0 \n");
            ClsqAuxCount = 0;
        }
        if (ClsqAuxCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ, 0x0000, 0x0002);
        }
    }
    mutex_unlock(& AudAna_lock);
}

static int ClsqCount = 0;
static void ClsqEnable(bool enable)
{
    printk("ClsqEnable ClsqAuxCount = %d enable = %d \n", ClsqCount, enable);
    mutex_lock(& AudAna_lock);
    if (enable)
    {
        if (ClsqCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ, 0x0001, 0x0001); 
        }
        ClsqCount++;
    }
    else
    {
        ClsqCount--;
        if (ClsqCount < 0)
        {
            printk("ClsqEnable count <0 \n");
            ClsqCount = 0;
        }
        if (ClsqCount == 0)
        {
            Ana_Set_Reg(TOP_CLKSQ, 0x0000, 0x0001);
        }
    }
    mutex_unlock(& AudAna_lock);
}

static int TopCkCount = 0;
static void Topck_Enable(bool enable)
{
    printk("Topck_Enable enable = %d TopCkCount = %d \n", enable, TopCkCount);
    mutex_lock(&Ana_Clk_Mutex);
    if (enable == true)
    {
        if (TopCkCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x3000, 0x3000);  
        }
        TopCkCount++;
    }
    else
    {
        TopCkCount--;
        if (TopCkCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x3000, 0x3000);
        }
        if (TopCkCount <= 0)
        {
            printk("TopCkCount <0 =%d\n ", TopCkCount);
            TopCkCount = 0;
        }
    }
    mutex_unlock(&Ana_Clk_Mutex);
}

static int NvRegCount = 0;
static void NvregEnable(bool enable)
{
    printk("NvregEnable NvRegCount == %d enable = %d \n", NvRegCount, enable);
    mutex_lock(&Ana_Clk_Mutex);
    if (enable == true)
    {
        if (NvRegCount == 0)
        {
            Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0000, 0xffff);  
        }
        NvRegCount++;
    }
    else
    {
        NvRegCount--;
        if (NvRegCount == 0)
        {
            Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0001, 0xffff);
        }
        if (NvRegCount < 0)
        {
            printk("NvRegCount <0 =%d\n ", NvRegCount);
            NvRegCount = 0;
        }
    }
    mutex_unlock(&Ana_Clk_Mutex);
}

static int AdcClockCount = 0;
static void AdcClockEnable(bool enable)
{
    mutex_lock(&Ana_Clk_Mutex);
    if (enable == true)
    {
        if (AdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x3000, 0xffff);  
        }
        AdcClockCount++;
    }
    else
    {
        AdcClockCount--;
        if (AdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x3000, 0xffff);
        }
        if (AdcClockCount <= 0)
        {
            printk("TopCkCount <0 =%d\n ", AdcClockCount);
            AdcClockCount = 0;
        }
    }
    mutex_unlock(&Ana_Clk_Mutex);
}

#if defined (MTK_VOW_SUPPORT)
void vow_irq_handler(void)
{
    printk("audio irq event....\n");
    
    TurnOnVOWDigitalHW(false);
#if defined(VOW_TONE_TEST)
    EnableSideGenHw(Soc_Aud_InterConnectionOutput_O03, Soc_Aud_MemIF_Direction_DIRECTION_OUTPUT, true);
#endif
    VowDrv_ChangeStatus();
}
#endif

static int LowPowerAdcClockCount = 0;
static void LowPowerAdcClockEnable(bool enable)
{
    mutex_lock(&Ana_Clk_Mutex);
    if (enable == true)
    {
        if (LowPowerAdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x8000, 0xffff);  
        }
        LowPowerAdcClockCount++;
    }
    else
    {
        LowPowerAdcClockCount--;
        if (LowPowerAdcClockCount == 0)
        {
            Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x8000, 0xffff);
        }
        if (LowPowerAdcClockCount < 0)
        {
            printk("LowPowerAdcClockCount <0 =%d\n ", LowPowerAdcClockCount);
            LowPowerAdcClockCount = 0;
        }
    }
    mutex_unlock(&Ana_Clk_Mutex);
}


#ifdef CONFIG_MTK_SPEAKER
static void Apply_Speaker_Gain(void)
{
    Ana_Set_Reg(SPK_CON9,  Speaker_pga_gain << 8, 0xf << 8);
}
#else
static void Apply_Speaker_Gain(void)
{

}
#endif

void setOffsetTrimMux(unsigned int Mux)
{
    printk("%s Mux = %d\n", __func__, Mux);
    Ana_Set_Reg(AUDBUF_CFG7 , Mux << 12, 0xf << 12); 
}

void setOffsetTrimBufferGain(unsigned int gain)
{
    Ana_Set_Reg(AUDBUF_CFG7 , gain << 10, 0x3 << 10); 
}

static int mHplTrimOffset = 2048;
static int mHprTrimOffset = 2048;

void SetHplTrimOffset(int Offset)
{
    printk("%s Offset = %d\n",__func__,Offset);
    mHplTrimOffset = Offset;
    if ((Offset > 2098) || (Offset < 1998))
    {
        mHplTrimOffset = 2048;
        printk("SetHplTrimOffset offset may be invalid offset = %d\n", Offset);
    }
}

void SetHprTrimOffset(int Offset)
{
    printk("%s Offset = %d\n",__func__,Offset);
    mHprTrimOffset = Offset;
    if ((Offset > 2098) || (Offset < 1998))
    {
        mHplTrimOffset = 2048;
        printk("SetHplTrimOffset offset may be invalid offset = %d\n", Offset);
    }
}

void EnableTrimbuffer(bool benable)
{
    if (benable == true)
    {
        Ana_Set_Reg(AUDBUF_CFG8 , 0x1 << 13, 0x1 << 13); 
    }
    else
    {
        Ana_Set_Reg(AUDBUF_CFG8 , 0x0 << 13, 0x1 << 13); 
    }
}

void OpenTrimBufferHardware(bool enable)
{
    if (enable)
    {
        TurnOnDacPower();
        printk("%s \n", __func__);
        Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0000, 0xffff); 
        OpenClassAB();
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        msleep(1);
        Ana_Set_Reg(ZCD_CON0,   0x0000, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
        Ana_Set_Reg(ZCD_CON2,  0x0489 , 0xffff); 
        Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000c , 0x000c); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x0003 , 0x0003); 
    }
    else
    {
        printk("Audio_Amp_Change off amp\n");
        Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000e, 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000d, 0xffff); 
        Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1800 , 0xffffffff);
        Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
        TurnOffDacPower();
    }
}


void OpenAnalogTrimHardware(bool enable)
{
    if (enable)
    {
        TurnOnDacPower();
        printk("%s \n", __func__);
        
        OpenClassAB();
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        msleep(1);
        Ana_Set_Reg(ZCD_CON0,   0x0000, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
        Ana_Set_Reg(ZCD_CON2,  0x0489 , 0xffff); 
        Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000c , 0x000c); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x0003 , 0x0003); 
    }
    else
    {
        printk("Audio_Amp_Change off amp\n");
        Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000e, 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000d, 0xffff); 
        Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1800 , 0xffffffff);
        Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
        TurnOffDacPower();
    }
}

void OpenAnalogHeadphone(bool bEnable)
{
    printk("OpenAnalogHeadphone bEnable = %d", bEnable);
    if (bEnable)
    {
        SetHplTrimOffset(2048);
        SetHprTrimOffset(2048);
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = 44100;
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = true;
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = true;
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = false;
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , false);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = false;
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , false);
    }
}

bool OpenHeadPhoneImpedanceSetting(bool bEnable)
{
    printk("%s benable = %d\n", __func__, bEnable);
    if (GetDLStatus() == true)
    {
        return false;
    }

    if (bEnable == true)
    {
        TurnOnDacPower();
        OpenClassAB();
        Ana_Set_Reg(AUDBUF_CFG5, 0x0003, 0x0003); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(ZCD_CON0, 0x0000, 0xffff);
        Ana_Set_Reg(AUDBUF_CFG0, 0xE000, 0xffff);
        Ana_Set_Reg(AUDBUF_CFG1, 0x0000, 0xffff);
        Ana_Set_Reg(AUDBUF_CFG8, 0x4000, 0xffff);
        Ana_Set_Reg(IBIASDIST_CFG0, 0x0092, 0xffff);
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501, 0xffff);
        Ana_Set_Reg(AUDDAC_CFG0, 0x0009, 0xffff);
        Ana_Set_Reg(AUDBUF_CFG6, 0x4800, 0xffff);
    }
    else
    {
        Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG8, 0x0000, 0xffff);
        Ana_Set_Reg(AUDBUF_CFG5, 0x0000, 0x0003); 
        TurnOffDacPower();
        Ana_Set_Reg(AUDBUF_CFG6, 0x0000, 0xffff);
    }
    return true;
}

void setHpGainZero(void)
{
    Ana_Set_Reg(ZCD_CON2, 0x8 << 7, 0x0f80);
    Ana_Set_Reg(ZCD_CON2, 0x8 , 0x001f);
}

void SetSdmLevel(unsigned int level)
{
    Ana_Set_Reg(AFE_DL_SDM_CON1, level, 0xffffffff);
}


static void SetHprOffset(int OffsetTrimming)
{
    short Dccompsentation = 0;
    int DCoffsetValue = 0;
    unsigned short RegValue = 0;
    printk("%s OffsetTrimming = %d \n",__func__,OffsetTrimming);
    DCoffsetValue = OffsetTrimming *  1000000;
    DCoffsetValue = (DCoffsetValue / DC1devider);  
    printk("%s DCoffsetValue = %d \n",__func__,DCoffsetValue);
    DCoffsetValue = (DCoffsetValue / DC1unit_in_uv);
    printk("%s DCoffsetValue = %d \n",__func__,DCoffsetValue);
    Dccompsentation = DCoffsetValue;
    RegValue = Dccompsentation;
    printk("%s RegValue = 0x%x\n",__func__,RegValue);
    Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, RegValue , 0xffff);
}

static void SetHplOffset(int OffsetTrimming)
{
    short Dccompsentation = 0;
    int DCoffsetValue = 0;
    unsigned short RegValue = 0;
    printk("%s OffsetTrimming = %d \n",__func__,OffsetTrimming);
    DCoffsetValue = OffsetTrimming *1000000;
    DCoffsetValue = (DCoffsetValue / DC1devider);  
    printk("%s DCoffsetValue = %d \n",__func__,DCoffsetValue);
    DCoffsetValue = (DCoffsetValue / DC1unit_in_uv);
    printk("%s DCoffsetValue = %d \n",__func__,DCoffsetValue);
    Dccompsentation = DCoffsetValue;
    RegValue = Dccompsentation;
    printk("%s RegValue = 0x%x\n",__func__,RegValue);
    Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, RegValue  , 0xffff);
}

static void EnableDcCompensation(bool bEnable)
{
    if(bEnable == true)
    {
        Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, bEnable , 0x1);
    }
    else
    {
        Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, bEnable , 0x1);
    }
}

static void SetHprOffsetTrim(void)
{
    int OffsetTrimming = mHprTrimOffset - TrimOffset;
    printk("%s mHprTrimOffset = %d TrimOffset = %d \n",__func__,mHprTrimOffset,TrimOffset);
    SetHprOffset(OffsetTrimming);
}

static void SetHpLOffsetTrim(void)
{
    int OffsetTrimming = mHplTrimOffset - TrimOffset;
    printk("%s mHprTrimOffset = %d TrimOffset = %d \n",__func__,mHplTrimOffset,TrimOffset);
    SetHplOffset(OffsetTrimming);
}

static void SetDcCompenSation(void)
{
    SetHprOffsetTrim();
    SetHpLOffsetTrim();
    EnableDcCompensation(true);
}

static void OpenClassH(void)
{
    Ana_Set_Reg(AFE_CLASSH_CFG7, 0xAD2D, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG8, 0x1313, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG9, 0x132d, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG10, 0x2d13, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG11, 0x1315, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG12, 0x6464, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG13, 0x2a2a, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG14, 0x009c, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG26, 0x9313, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG27, 0x1313, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG28, 0x1315, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG29, 0x1515, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG30, 0x1515, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG1, 0xBF04, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG2, 0x5fbf, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG3, 0x1104, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG4, 0x2412, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG5, 0x0201, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG6, 0x2800, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG14, 0x009c, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG21, 0x2108, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG22, 0x06db, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG23, 0xffff , 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG24, 0x0bd6, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG25, 0x1740, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd608, 0xffff); 
    udelay(100);
    Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd20d, 0xffff); 
    udelay(100);
}

static void OpenClassAB(void)
{
    Ana_Set_Reg(AFE_CLASSH_CFG7, 0x8909, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG8, 0x0909, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG9, 0x1309, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG10, 0x0909, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG11, 0x0915, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG12, 0x1414, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG13, 0x1414, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG14, 0x009c, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG26, 0x9313, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG27, 0x1313, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG28, 0x1315, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG29, 0x1515, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG30, 0x1515, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG1, 0x0024, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG2, 0x2f90, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG3, 0x1104, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG4, 0x2412, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG5, 0x0201, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG6, 0x2800, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG21, 0xa108, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG22, 0x06db, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG23, 0x0bd6, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG24, 0x1492, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG25, 0x1740, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd419, 0xffff); 
    Ana_Set_Reg(AFE_CLASSH_CFG1,   0x0021, 0xffff); 
}

static void SetDCcoupleNP(int ADCType, int mode)
{
    printk("%s ADCType= %d mode = %d\n", __func__, ADCType, mode);
#ifndef CONFIG_MTK_FPGA
    switch (mode)
    {
        case AUDIO_ANALOGUL_MODE_ACC:
        case AUDIO_ANALOGUL_MODE_DCC:
        case AUDIO_ANALOGUL_MODE_DMIC:
        {
            if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
            {
                mt6331_upmu_set_rg_audmicbias0dcswnen(false); 
                mt6331_upmu_set_rg_audmicbias0dcswpen(false); 
            }
            else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
            {
                mt6331_upmu_set_rg_audmicbias1dcswnen(false); 
                mt6331_upmu_set_rg_audmicbias1dcswpen(false); 
            }
        }
        break;
        case AUDIO_ANALOGUL_MODE_DCCECMDIFF:
        {
            if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
            {
                mt6331_upmu_set_rg_audmicbias0dcswnen(true); 
                mt6331_upmu_set_rg_audmicbias0dcswpen(true); 
            }
            else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
            {
                mt6331_upmu_set_rg_audmicbias1dcswnen(true); 
                mt6331_upmu_set_rg_audmicbias1dcswpen(true); 
            }
        }
        break;
        case AUDIO_ANALOGUL_MODE_DCCECMSINGLE:
        {
            if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
            {
                mt6331_upmu_set_rg_audmicbias0dcswnen(false); 
                mt6331_upmu_set_rg_audmicbias0dcswpen(true); 
            }
            else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
            {
                mt6331_upmu_set_rg_audmicbias1dcswnen(true); 
                mt6331_upmu_set_rg_audmicbias1dcswpen(false); 
            }
        }
        break;
        default:
            break;
    }
#endif
}

static void OpenMicbias3(bool bEnable)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, (bEnable << 7), (0x1 << 7));
}

static void OpenMicbias2(bool bEnable)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, (bEnable << 0), (0x1 << 0));
}

static void SetMicVref2(uint32_t vRef)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, vRef << 1, 0x7 << 1);
}

static void SetMicVref3(uint32_t vRef)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, vRef << 8, 0x7 << 8);
}

static void EnableMicBias(uint32_t Mic, bool bEnable)
{
    if (bEnable == true)
    {
        switch (Mic)
        {
            case AUDIO_ANALOG_DEVICE_IN_ADC1:
                OpenMicbias0(true);
                OpenMicbias1(true);
                break;
            case AUDIO_ANALOG_DEVICE_IN_ADC2:
                OpenMicbias0(true);
                OpenMicbias1(true);
                OpenMicbias2(true);
                break;
            case AUDIO_ANALOG_DEVICE_IN_ADC3:
            case AUDIO_ANALOG_DEVICE_IN_ADC4:
                OpenMicbias3(true);
                break;
        }
    }
    else
    {
        switch (Mic)
        {
            case AUDIO_ANALOG_DEVICE_IN_ADC1:
                OpenMicbias0(false);
                OpenMicbias1(false);
                break;
            case AUDIO_ANALOG_DEVICE_IN_ADC2:
                OpenMicbias0(false);
                OpenMicbias1(false);
                OpenMicbias2(false);
                break;
            case AUDIO_ANALOG_DEVICE_IN_ADC3:
            case AUDIO_ANALOG_DEVICE_IN_ADC4:
                OpenMicbias3(false);
                break;
        }
    }
}

static void SetMic2DCcoupleSwitch(bool internal)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, internal << 4, 0x1 << 4);
    Ana_Set_Reg(AUDMICBIAS_CFG1, internal << 5, 0x1 << 5);
}

static void SetMic3DCcoupleSwitch(bool internal)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, internal << 11, 0x1 << 4);
    Ana_Set_Reg(AUDMICBIAS_CFG1, internal << 12, 0x1 << 5);
}


static void SetMic2powermode(bool lowpower)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, lowpower << 6, 0x1 << 6);
}

static void SetMic3powermode(bool lowpower)
{
    Ana_Set_Reg(AUDMICBIAS_CFG1, lowpower << 13, 0x1 << 13);
}


static void OpenMicbias1(bool bEnable)
{
    printk("%s bEnable = %d \n", __func__, bEnable);
#ifndef CONFIG_MTK_FPGA
    if (bEnable == true)
    {
        mt6331_upmu_set_rg_audpwdbmicbias1(true); 
    }
    else
    {
        mt6331_upmu_set_rg_audmicbias1lowpen(true); 
        mt6331_upmu_set_rg_audpwdbmicbias1(false); 
    }
#endif
}

static void SetMicbias1lowpower(bool benable)
{
#ifndef CONFIG_MTK_FPGA
    mt6331_upmu_set_rg_audmicbias1lowpen(benable); 
#endif
}

static void OpenMicbias0(bool bEanble)
{
    printk("%s bEanble = %d\n", __func__, bEanble);
#ifndef CONFIG_MTK_FPGA
    if (bEanble == true)
    {
        mt6331_upmu_set_rg_audpwdbmicbias0(true); 
        mt6331_upmu_set_rg_audmicbias0vref(0x2); 
    }
    else
    {
        mt6331_upmu_set_rg_audmicbias0lowpen(true); 
        mt6331_upmu_set_rg_audpwdbmicbias0(false); 
    }
#endif
}

static void SetMicbias0lowpower(bool benable)
{
#ifndef CONFIG_MTK_FPGA
    mt6331_upmu_set_rg_audmicbias0lowpen(benable); 
#endif
}





static uint32 GetULNewIFFrequency(uint32 frequency)
{
    uint32 Reg_value = 0;
    switch (frequency)
    {
        case 8000:
        case 16000:
        case 32000:
            Reg_value = 1;
            break;
        case 48000:
            Reg_value = 3;
        default:
            break;
    }
    return Reg_value;
}

uint32 GetULFrequency(uint32 frequency)
{
    uint32 Reg_value = 0;
    printk("%s frequency =%d\n", __func__, frequency);
    switch (frequency)
    {
        case 8000:
        case 16000:
        case 32000:
            Reg_value = 0x0;
            break;
        case 48000:
            Reg_value = 0x1;
        default:
            break;

    }
    return Reg_value;
}


uint32 ULSampleRateTransform(uint32 SampleRate)
{
    switch (SampleRate)
    {
        case 8000:
            return 0;
        case 16000:
            return 1;
        case 32000:
            return 2;
        case 48000:
            return 3;
        default:
            break;
    }
    return 0;
}


static int mt63xx_codec_startup(struct snd_pcm_substream *substream , struct snd_soc_dai *Daiport)
{
    
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE && substream->runtime->rate)
    {
        
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] = substream->runtime->rate;

    }
    else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && substream->runtime->rate)
    {
        
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = substream->runtime->rate;
    }
    
    return 0;
}

static int mt63xx_codec_prepare(struct snd_pcm_substream *substream , struct snd_soc_dai *Daiport)
{
    
    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        printk("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_CAPTURE rate = %d\n", substream->runtime->rate);
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC] = substream->runtime->rate;

    }
    else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        printk("mt63xx_codec_prepare set up SNDRV_PCM_STREAM_PLAYBACK rate = %d\n", substream->runtime->rate);
        mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC] = substream->runtime->rate;
    }
    return 0;
}

static int mt6323_codec_trigger(struct snd_pcm_substream *substream , int command , struct snd_soc_dai *Daiport)
{
    switch (command)
    {
        case SNDRV_PCM_TRIGGER_START:
        case SNDRV_PCM_TRIGGER_RESUME:
        case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
            break;
    }

    
    return 0;
}

static const struct snd_soc_dai_ops mt6323_aif1_dai_ops =
{
    .startup    = mt63xx_codec_startup,
    .prepare   = mt63xx_codec_prepare,
    .trigger     = mt6323_codec_trigger,
};

static struct snd_soc_dai_driver mtk_6331_dai_codecs[] =
{
    {
        .name = MT_SOC_CODEC_TXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_DL1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_RXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .capture = {
            .stream_name = MT_SOC_UL1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_TDMRX_DAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .capture = {
            .stream_name = MT_SOC_TDM_CAPTURE_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_192000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_I2S0TXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_I2SDL1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_192000,
            .formats = SND_SOC_ADV_MT_FMTS,
        }
    },
    {
        .name = MT_SOC_CODEC_PCMTXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_PCM2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
        .capture = {
            .stream_name = MT_SOC_PCM2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_PCMRXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_PCM1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
        .capture = {
            .stream_name = MT_SOC_PCM1_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_VOICECALLEXTINTDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_VOICE_EXTINT_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
        .capture = {
            .stream_name = MT_SOC_VOICE_EXTINT_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
    {
        .name = MT_SOC_CODEC_FMI2S2RXDAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_FM_I2S2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
        .capture = {
            .stream_name = MT_SOC_FM_I2S2_RECORD_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_FMMRGTXDAI_DUMMY_DAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_FM_MRGTX_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_44100,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_ULDLLOOPBACK_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
        .capture = {
            .stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_STUB_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_ROUTING_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_RXDAI2_NAME,
        .capture = {
            .stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_MRGRX_DAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_MRGRX_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 8,
            .rates = SNDRV_PCM_RATE_8000_192000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
        .capture = {
            .stream_name = MT_SOC_MRGRX_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 8,
            .rates = SNDRV_PCM_RATE_8000_192000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
    {
        .name = MT_SOC_CODEC_HP_IMPEDANCE_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .playback = {
            .stream_name = MT_SOC_HP_IMPEDANCE_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
   {
        .name = MT_SOC_CODEC_MOD_DAI_NAME,
        .ops = &mt6323_aif1_dai_ops,
        .capture = {
            .stream_name = MT_SOC_MODDAI_STREAM_NAME,
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_48000,
            .formats = SND_SOC_ADV_MT_FMTS,
        },
    },
};


uint32 GetDLNewIFFrequency(unsigned int frequency)
{
    uint32 Reg_value = 0;
    
    switch (frequency)
    {
        case 8000:
            Reg_value = 0;
            break;
        case 11025:
            Reg_value = 1;
            break;
        case 12000:
            Reg_value = 2;
            break;
        case 16000:
            Reg_value = 3;
            break;
        case 22050:
            Reg_value = 4;
            break;
        case 24000:
            Reg_value = 5;
            break;
        case 32000:
            Reg_value = 6;
            break;
        case 44100:
            Reg_value = 7;
            break;
        case 48000:
            Reg_value = 8;
        default:
            printk("ApplyDLNewIFFrequency with frequency = %d", frequency);
    }
    return Reg_value;
}

static bool GetDLStatus(void)
{
    int i = 0;
    for (i = 0; i < AUDIO_ANALOG_DEVICE_2IN1_SPK ; i++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}

static void TurnOnDacPower(void)
{
    printk("TurnOnDacPower\n");
    audckbufEnable(true);
    ClsqEnable(true);
    Topck_Enable(true);
    NvregEnable(true);
    if ((GetAdcStatus() == false))
    {
        Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x003a, 0xffff);   
    }
    else
    {
        Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
    }
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffffffff);
    Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffffffff); 
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffffffff); 
    Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffffffff); 
    Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffffffff); 
    Ana_Set_Reg(AFE_UL_DL_CON0 , 0x0001, 0xffffffff); 

    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0 , GetDLNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC]) << 12 | 0x330 , 0xffffffff);
    Ana_Set_Reg(AFE_DL_SRC2_CON0_H , GetDLNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_OUT_DAC]) << 12 | 0x330 , 0xffffffff);

    Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1801 , 0xffffffff); 
    Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0000 , 0xffffffff); 

}

static void TurnOffDacPower(void)
{
    printk("TurnOffDacPower\n");

    #if 1 
    Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0000, 0xffff); 
    if (GetAdcStatus() == false)
    {
	    Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0x0001); 
    }
    udelay(250);

    Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0040, 0x0040); 
    #endif
    Ana_Set_Reg(AFE_CLASSH_CFG1, 0x24, 0xffff);
    Ana_Set_Reg(AFE_CLASSH_CFG0, 0xd518, 0xffff); 
    Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0, 0xffff); 
    ClsqEnable(false);
    Topck_Enable(false);
    NvregEnable(false);
    audckbufEnable(false);
}

static void HeadsetVoloumeRestore(void)
{
    int index = 0,oldindex = 0,offset =0,count =1;
    printk("%s\n", __func__);
    index =   8 ;
    oldindex = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
    if(index  > oldindex)
    {
        printk("index = %d oldindex = %d \n",index,oldindex);
        offset = index - oldindex;
        while(offset >0)
        {
            Ana_Set_Reg(ZCD_CON2, ((oldindex + count)<<7 )|(oldindex + count) , 0xf9f);
            offset--;
            count++;
            udelay(100);
        }
    }
    else
    {
        printk("index = %d oldindex = %d \n",index,oldindex);
        offset = oldindex - index;
        while(offset >0)
        {
            Ana_Set_Reg(ZCD_CON2, ((oldindex-count)<<7)|(oldindex-count) , 0xf9f);
            offset--;
            count++;
            udelay(100);
        }
    }
    Ana_Set_Reg(ZCD_CON2, 0x489, 0xf9f);
}

static void HeadsetVoloumeSet(void)
{
    int index = 0,oldindex = 0,offset =0, count =1;
    printk("%s\n", __func__);
    index =   mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
    oldindex = 8;
    if(index  > oldindex)
    {
        printk("index = %d oldindex = %d \n",index,oldindex);
        offset = index - oldindex;
        while(offset >0)
        {
            Ana_Set_Reg(ZCD_CON2, ((oldindex + count)<<7 )|(oldindex + count) , 0xf9f);
            offset--;
            count++;
            udelay(200);
        }
    }
    else
    {
        printk("index = %d oldindex = %d \n",index,oldindex);
        offset = oldindex - index;
        while(offset >0)
        {
            Ana_Set_Reg(ZCD_CON2, ((oldindex-count)<<7)|(oldindex-count) , 0xf9f);
            offset--;
            count++;
            udelay(200);
        }
    }
    Ana_Set_Reg(ZCD_CON2, (index << 7) | (index), 0xf9f);
}

static void Audio_Amp_Change(int channels , bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        
        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == false &&
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == false)
        {
            pr_info("%s: channels: %d, enable: %d\n", __func__, channels, enable);
#ifndef CONFIG_AMP_RT5506
            OpenClassH();

            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 

            Ana_Set_Reg(ZCD_CON0,   0x0000, 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,   0xE000, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x8068, 0xffff); 
            Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG5, 0x0007, 0x0007); 

            Ana_Set_Reg(ZCD_CON2,  0x0489 , 0xffff); 
            Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG1,  0x900 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,  0xE001 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG1,  0x940 , 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0, 0x000f , 0x000f); 
            udelay(100);
            Ana_Set_Reg(AUDBUF_CFG0, 0xE141 , 0xffff);
            Ana_Set_Reg(ZCD_CON2, 0x0489 , 0xffff); 
            SetDcCompenSation();
            udelay(100);
            
            Ana_Set_Reg(AUDBUF_CFG0, 0xe147 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0, 0xE14e , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG5, 0x0003, 0x0007); 
            Ana_Set_Reg(AUDBUF_CFG1, 0x0900 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2, 0x0020 , 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff);
#else
            printk("rt5506 amp on\n");
            htc_acoustic_hs_amp_ctrl(1, 0); 

            
            OpenClassAB();
            Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0028 , 0xffffffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0068 , 0xffffffff); 
            Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
            Ana_Set_Reg(ZCD_CON0  , 0x0700 , 0xffffffff); 
            Ana_Set_Reg(AUDBUF_CFG6  , 0x00C0 , 0xffffffff); 
            Ana_Set_Reg(IBIASDIST_CFG0  , 0x0092 , 0xffffffff); 
            Ana_Set_Reg(ZCD_CON1  , 0x0F9F , 0xffffffff); 
            Ana_Set_Reg(AUDBUF_CFG7  , 0x0102 , 0xffffffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5501 , 0xffffffff); 
            Ana_Set_Reg(AUDDAC_CFG0  , 0x000F , 0xffffffff); 
            SetDcCompenSation();
            Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); 
            Ana_Set_Reg(AUDBUF_CFG6  , 0x00EB , 0xffffffff); 
            Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); 
#endif

            
            HeadsetVoloumeSet();
        }
        else if (channels == AUDIO_ANALOG_CHANNELS_LEFT1)
        {
            

        }
        else if (channels == AUDIO_ANALOG_CHANNELS_RIGHT1)
        {
            
        }
    }
    else
    {

        if (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] == false &&
            mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] == false)
        {
            pr_info("Audio_Amp_Change off amp\n");
#ifndef CONFIG_AMP_RT5506
            HeadsetVoloumeRestore();
            
            Ana_Set_Reg(AUDBUF_CFG0, 0xE148, 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); 
            Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG5, 0x0000, 0x0003); 
            EnableDcCompensation(false);
#else
            printk("rt5506 amp off\n");
            htc_acoustic_hs_amp_ctrl(0, 0); 

            Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); 
            Ana_Set_Reg(AUDBUF_CFG6  , 0x0000 , 0xffffffff);
            Ana_Set_Reg(AUDBUF_CFG7  , 0x0900 , 0xffffffff);
            Ana_Set_Reg(AUDDAC_CFG0  , 0x0000 , 0xffffffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5500 , 0xffffffff); 
            Ana_Set_Reg(IBIASDIST_CFG0  , 0x0192 , 0xffffffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0028 , 0xffffffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0000 , 0xffffffff); 
#endif
        }
        else if (channels == AUDIO_ANALOG_CHANNELS_LEFT1)
        {
            
        }
        else if (channels == AUDIO_ANALOG_CHANNELS_RIGHT1)
        {
            
        }
        if (GetDLStatus() == false)
        {
            Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1800 , 0xffffffff);
            if (GetAdcStatus() == false)
            {
                Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
            }
            TurnOffDacPower();
        }
    }
}

static int Audio_AmpL_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpL_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL];
    return 0;
}

static int Audio_AmpL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mutex_lock(&Ana_Ctrl_Mutex);

    printk("%s() gain = %ld \n ", __func__, ucontrol->value.integer.value[0]);
    if ((ucontrol->value.integer.value[0]  == true) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL]  == false))
    {
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
    }
    else if ((ucontrol->value.integer.value[0]  == false) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL]  == true))
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_LEFT1 , false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}

static int Audio_AmpR_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR];
    return 0;
}

static int Audio_AmpR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mutex_lock(&Ana_Ctrl_Mutex);

    printk("%s()\n", __func__);
    if ((ucontrol->value.integer.value[0]  == true) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR]  == false))
    {
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
    }
    else if ((ucontrol->value.integer.value[0]  == false) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR]  == true))
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
        Audio_Amp_Change(AUDIO_ANALOG_CHANNELS_RIGHT1 , false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}


static void Voice_Amp_Change(bool enable)
{
    if (enable)
    {
        printk("turn on ampL\n");
        if (GetDLStatus() == false)
        {
            Ana_Set_Reg(0x0680, 0x0000, 0xffff); 
            Ana_Set_Reg(TOP_CKSEL_CON_CLR, 0x0001, 0x0001); 
            TurnOnDacPower();
            printk("Voice_Amp_Change on amp\n");
            OpenClassAB();
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
            Ana_Set_Reg(ZCD_CON0,   0x0700, 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
            Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
            Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0,  0x5501 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0,  0x0009 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,  0xE010 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,  0xE011 , 0xffff); 
            Ana_Set_Reg(ZCD_CON3,  0x0009 , 0xffff); 
        }
    }
    else
    {
        printk("turn off ampL\n");
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
            Ana_Set_Reg(AUDBUF_CFG0,  0xE010 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0,  0x0000 , 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0,  0x5500 , 0xffff); 
            Ana_Set_Reg(IBIASDIST_CFG0,  0x0192 , 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,  0x0028 , 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0,  0x0000 , 0xffff); 
        }
    }
}

static int Voice_Amp_Get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    printk("Voice_Amp_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL];
    return 0;
}

static int Voice_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mutex_lock(&Ana_Ctrl_Mutex);
    printk("%s()\n", __func__);
    if ((ucontrol->value.integer.value[0]  == true) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL]  == false))
    {
        Voice_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
    }
    else if ((ucontrol->value.integer.value[0]  == false) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL]  == true))
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
        Voice_Amp_Change(false);
    }
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}

static void Speaker_Amp_Change(bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        printk("turn on Speaker_Amp_Change \n");
        
        
        OpenClassAB();
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0028 , 0xffffffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0068 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        Ana_Set_Reg(ZCD_CON0  , 0x0700 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00C0 , 0xffffffff); 
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0092 , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON1  , 0x0F9F , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0102 , 0xffffffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5501 , 0xffffffff); 
        Ana_Set_Reg(AUDDAC_CFG0  , 0x000F , 0xffffffff); 
        SetDcCompenSation();
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00EB , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); 
#ifdef CONFIG_MTK_SPEAKER
        if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
        {
            Speaker_ClassD_Open();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
        {
            Speaker_ClassAB_Open();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
        {
#ifdef CONFIG_HTC_RECEIVER
            setReceiverSelect(true);
#else
            Speaker_ReveiverMode_Open();
#endif
        }
#endif
        Apply_Speaker_Gain();
    }
    else
    {
        printk("turn off Speaker_Amp_Change \n");
#ifdef CONFIG_MTK_SPEAKER
        if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
        {
            Speaker_ClassD_close();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
        {
            Speaker_ClassAB_close();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
        {
#ifdef CONFIG_HTC_RECEIVER
            setReceiverSelect(false);
#else
            Speaker_ReveiverMode_Close();
#endif
        }
#endif
        if (GetDLStatus() == false)
        {
            TurnOffDacPower();
        }
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG6  , 0x0000 , 0xffffffff);
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0900 , 0xffffffff);
        Ana_Set_Reg(AUDDAC_CFG0  , 0x0000 , 0xffffffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5500 , 0xffffffff); 
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0192 , 0xffffffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0028 , 0xffffffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0000 , 0xffffffff); 
    }
}

static int Speaker_Amp_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] ;
    return 0;
}

static int Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() value = %ld \n ", __func__, ucontrol->value.integer.value[0]);
	printk("Speaker_Amp_Set value = %ld \n ", ucontrol->value.integer.value[0]);
    if ((ucontrol->value.integer.value[0] == true) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL]  == false))
    {
        Speaker_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
    }
    else if ((ucontrol->value.integer.value[0] == false) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL]  == true))
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
        Speaker_Amp_Change(false);
    }
    return 0;
}

static void Headset_Speaker_Amp_Change(bool enable)
{
    if (enable)
    {
        if (GetDLStatus() == false)
        {
            TurnOnDacPower();
        }
        printk("turn on Speaker_Amp_Change \n");
        
        
        OpenClassAB();

        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0028 , 0xffffffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0 , 0x0068 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        Ana_Set_Reg(ZCD_CON0  , 0x0700 , 0xffffffff); 

        Ana_Set_Reg(AUDBUF_CFG0  , 0xE008 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00C0 , 0xffffffff); 

        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0092 , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON2  , 0x0489 , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON3 , 0x001F , 0xffffffff); 

        Ana_Set_Reg(AUDBUF_CFG1  , 0x0900 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0102 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0022 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE009 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG1  , 0x0940 , 0xffffffff); 
        msleep(1);

        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5501 , 0xffffffff); 
        Ana_Set_Reg(AUDDAC_CFG0  , 0x000F , 0xffffffff); 
        SetDcCompenSation();
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00E8 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG6  , 0x00EB , 0xffffffff); 
        Ana_Set_Reg(ZCD_CON1  , 0x0489 , 0xffffffff); 

        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0A9 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0AF , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG1  , 0x0900 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0020 , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE0AE , 0xffffffff); 
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0489 , 0xffffffff); 

#ifdef CONFIG_MTK_SPEAKER
        if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
        {
            Speaker_ClassD_Open();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
        {
            Speaker_ClassAB_Open();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
        {
            Speaker_ReveiverMode_Open();
        }
#endif
        HeadsetRVolumeSet();
        HeadsetLVolumeSet();
        Apply_Speaker_Gain();
    }
    else
    {

#ifdef CONFIG_MTK_SPEAKER
        if (Speaker_mode == AUDIO_SPEAKER_MODE_D)
        {
            Speaker_ClassD_close();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_AB)
        {
            Speaker_ClassAB_close();
        }
        else if (Speaker_mode == AUDIO_SPEAKER_MODE_RECEIVER)
        {
            Speaker_ReveiverMode_close();
        }
#endif

        Ana_Set_Reg(AUDBUF_CFG0  , 0xE149 , 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0  , 0x0000 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0  , 0xE148 , 0xffff);
        Ana_Set_Reg(AUDBUF_CFG2  , 0x0020 , 0xffff);
        Ana_Set_Reg(AUDBUF_CFG6  , 0x0000 , 0xffff);
        Ana_Set_Reg(AUDBUF_CFG7  , 0x0902 , 0xffff);
        Ana_Set_Reg(AUDCLKGEN_CFG0  , 0x5500 , 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0  , 0x0192 , 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0028 , 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0  , 0x0000 , 0xffff); 

        printk("turn off Speaker_Amp_Change \n");
        if (GetDLStatus() == false)
        {
            Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1800 , 0xffff);
            if (GetAdcStatus() == false)
            {
                Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
            }
            TurnOffDacPower();
        }
    }

}


static int Headset_Speaker_Amp_Get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] ;
    return 0;
}

static int Headset_Speaker_Amp_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    

    printk("%s() gain = %lu \n ", __func__, ucontrol->value.integer.value[0]);
    if ((ucontrol->value.integer.value[0]  == true) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R]  == false))
    {
        Headset_Speaker_Amp_Change(true);
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] = ucontrol->value.integer.value[0];
    }
    else if ((ucontrol->value.integer.value[0]  == false) && (mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R]  == true))
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_VOLUME_SPEAKER_HEADSET_R] = ucontrol->value.integer.value[0];
        Headset_Speaker_Amp_Change(false);
    }
    return 0;
}

static bool Ext_Speaker_Mode = false;
static int Audio_Ext_Speaker_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    Ext_Speaker_Mode = ucontrol->value.integer.value[0];
    printk("%s Ext_Speaker_Mode = %d",__func__,Ext_Speaker_Mode);
    if(Ext_Speaker_Mode == true) 
    {

    }
    else 
    {

    }
    return 0;
}

static int Audio_Ext_Speaker_Get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = Ext_Speaker_Mode ;
    return 0;
}

static const char *Ext_speaker_amp_function[] = {"Off", "On"};
static const struct soc_enum Audio_Ext_dev_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Ext_speaker_amp_function), Ext_speaker_amp_function),
};

static const struct snd_kcontrol_new mt_ext_dev_controls[] =
{
    SOC_ENUM_EXT("Ext_Speaker_Amp_Switch", Audio_Ext_dev_Enum[0], Audio_Ext_Speaker_Get, Audio_Ext_Speaker_Set),
};
#ifdef CONFIG_MTK_SPEAKER
static const char *speaker_amp_function[] = {"CALSSD", "CLASSAB", "RECEIVER"};
static const char *speaker_PGA_function[] = {"4Db", "5Db", "6Db", "7Db", "8Db", "9Db", "10Db",
                                             "11Db", "12Db", "13Db", "14Db", "15Db", "16Db", "17Db"
                                            };
static const char *speaker_OC_function[] = {"Off", "On"};
static const char *speaker_CS_function[] = {"Off", "On"};
static const char *speaker_CSPeakDetecReset_function[] = {"Off", "On"};

static int Audio_Speaker_Class_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	printk("Audio_Speaker_Class_Set\n");
    mutex_lock(&Ana_Ctrl_Mutex);
    Speaker_mode = ucontrol->value.integer.value[0];
    mutex_unlock(&Ana_Ctrl_Mutex);
    return 0;
}

static int Audio_Speaker_Class_Get(struct snd_kcontrol *kcontrol,
                                   struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = Speaker_mode ;
    return 0;
}

static int Audio_Speaker_Pga_Gain_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    Speaker_pga_gain = ucontrol->value.integer.value[0];
    Ana_Set_Reg(SPK_CON9,  (Speaker_pga_gain +1)<< 8, 0xf << 8);
    return 0;
}

static int Audio_Speaker_OcFlag_Get(struct snd_kcontrol *kcontrol,
                                    struct snd_ctl_elem_value *ucontrol)
{
    mSpeaker_Ocflag =  GetSpeakerOcFlag();
    ucontrol->value.integer.value[0] = mSpeaker_Ocflag ;
    return 0;
}

static int Audio_Speaker_OcFlag_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s is not support setting \n", __func__);
    return 0;
}

static int Audio_Speaker_Pga_Gain_Get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0] = Speaker_pga_gain ;
    return 0;
}

static int Audio_Speaker_Current_Sensing_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    static int dauxadc1_tempval = 0;

    if (ucontrol->value.integer.value[0])
    {
        printk("enable Audio_Speaker_Current_Sensing_Set\n");
        
        Ana_Set_Reg(SPK_CON16,  0x8000, 0x8000); 
        Ana_Set_Reg(SPK_CON14,  0x0050, 0x0050); 
        mEnableAuxAdc = 1;
        dauxadc1_tempval = Ana_Get_Reg(0x886a);
        Ana_Set_Reg(0x886a, 0x00b0, 0xFFFF);    
    }
    else
    {
        printk("disable Audio_Speaker_Current_Sensing_Set\n");
        
        Ana_Set_Reg(SPK_CON16,  0x0, 0x8000); 
        Ana_Set_Reg(SPK_CON14,  0x0, 0x0050); 
        Ana_Set_Reg(0x80be, 0x02, 0x02);
        Ana_Set_Reg(0x80c0, 0x02, 0x02);
        Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x8000); 
        Ana_Set_Reg(MT6332_AUXADC_CON0, 0x0000, 0x8000);
        Ana_Set_Reg(0x886a, dauxadc1_tempval, 0xFFFF);    
        mEnableAuxAdc = 0;
    }
    return 0;
}

static int Audio_Speaker_Current_Sensing_Get(struct snd_kcontrol *kcontrol,
                                             struct snd_ctl_elem_value *ucontrol)
{
    
    ucontrol->value.integer.value[0] = (Ana_Get_Reg(SPK_CON16) >> 15) & 0x01; 
    return 0;
}

static int Audio_Speaker_Current_Sensing_Peak_Detector_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 0;
}

static int Audio_Speaker_Current_Sensing_Peak_Detector_Get(struct snd_kcontrol *kcontrol,
                                                           struct snd_ctl_elem_value *ucontrol)
{
    return 0;
}


static const struct soc_enum Audio_Speaker_Enum[] =
{
    
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_amp_function), speaker_amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_PGA_function), speaker_PGA_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_OC_function), speaker_OC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_CS_function), speaker_CS_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(speaker_CSPeakDetecReset_function), speaker_CSPeakDetecReset_function),
};

static const struct snd_kcontrol_new mt6331_snd_Speaker_controls[] =
{
    SOC_ENUM_EXT("Audio_Speaker_class_Switch", Audio_Speaker_Enum[0], Audio_Speaker_Class_Get, Audio_Speaker_Class_Set),
    SOC_ENUM_EXT("Audio_Speaker_PGA_gain", Audio_Speaker_Enum[1], Audio_Speaker_Pga_Gain_Get, Audio_Speaker_Pga_Gain_Set),
    SOC_ENUM_EXT("Audio_Speaker_OC_Falg", Audio_Speaker_Enum[2], Audio_Speaker_OcFlag_Get, Audio_Speaker_OcFlag_Set),
    SOC_ENUM_EXT("Audio_Speaker_CurrentSensing", Audio_Speaker_Enum[3], Audio_Speaker_Current_Sensing_Get, Audio_Speaker_Current_Sensing_Set),
    SOC_ENUM_EXT("Audio_Speaker_CurrentPeakDetector", Audio_Speaker_Enum[4], Audio_Speaker_Current_Sensing_Peak_Detector_Get, Audio_Speaker_Current_Sensing_Peak_Detector_Set),
};

#define HW_BUFFER_LENGTH 21
const uint32 V_Buffer_Table[HW_BUFFER_LENGTH] =
{
    MT6332_AUXADC_ADC19,
    MT6332_AUXADC_ADC20,
    MT6332_AUXADC_ADC21,
    MT6332_AUXADC_ADC22,
    MT6332_AUXADC_ADC23,
    MT6332_AUXADC_ADC24,
    MT6332_AUXADC_ADC25,
    MT6332_AUXADC_ADC26,
    MT6332_AUXADC_ADC27,
    MT6332_AUXADC_ADC28,
    MT6332_AUXADC_ADC29,
    MT6332_AUXADC_ADC30,
    MT6332_AUXADC_ADC0,
    MT6332_AUXADC_ADC1,
    MT6332_AUXADC_ADC2,
    MT6332_AUXADC_ADC3,
    MT6332_AUXADC_ADC4,
    MT6332_AUXADC_ADC5,
    MT6332_AUXADC_ADC6,
    MT6332_AUXADC_ADC7,
    MT6332_AUXADC_ADC8
};

const uint32 I_Buffer_Table[HW_BUFFER_LENGTH] =
{
    MT6332_AUXADC_ADC31,
    MT6332_AUXADC_ADC32,
    MT6332_AUXADC_ADC33,
    MT6332_AUXADC_ADC34,
    MT6332_AUXADC_ADC35,
    MT6332_AUXADC_ADC36,
    MT6332_AUXADC_ADC37,
    MT6332_AUXADC_ADC38,
    MT6332_AUXADC_ADC39,
    MT6332_AUXADC_ADC40,
    MT6332_AUXADC_ADC41,
    MT6332_AUXADC_ADC42,
    MT6332_AUXADC_ADC9,
    MT6332_AUXADC_ADC10,
    MT6332_AUXADC_ADC11,
    MT6332_AUXADC_ADC12,
    MT6332_AUXADC_ADC13,
    MT6332_AUXADC_ADC14,
    MT6332_AUXADC_ADC15,
    MT6332_AUXADC_ADC16,
    MT6332_AUXADC_ADC17
};

int Audio_AuxAdcData_Get_ext(void)
{
    const int dRecCount = 1024;
    int dRecReadIndex = 0;
    int dValidCount = 0;
    int dCheckCount = 0;
    int cnt1, cnt2, iv_queue, v_cnt, i_cnt, cnt, i, hw_read_idx, iov_flag, iov_cnt;
    int dMax, dCurValue;
    int output_freq, freq_meter_data;

    printk("vibspk auxadc in+\n");

    if (mEnableAuxAdc == 0)
    {
        return 0;
    }

    if (mEnableAuxAdc == 1)
    {
        Ana_Set_Reg(0x80be, 0x02, 0x02);
        Ana_Set_Reg(0x80c0, 0x02, 0x02);
        Ana_Set_Reg(MT6332_AUXADC_CON0, 0x8000, 0x8000);
        Ana_Set_Reg(0x8094, 0, 0xFFFF);
        Ana_Set_Reg(0x809a, 0x6023, 0xFFFF);
        Ana_Set_Reg(0x80a0, 0x004a, 0xFFFF);
        Ana_Set_Reg(0x80b2, 0x01ff, 0xFFFF);
        Ana_Set_Reg(0x80c2, 0x0005, 0xFFFF);
        
    }

#if 1
    if (mEnableAuxAdc == 1)
    {
        Ana_Set_Reg(0x0680, 0x0000, 0xFFFF);
        Ana_Set_Reg(0x015A, 0x0001, 0xFFFF);
        Ana_Set_Reg(0x015C, 0x0003, 0xFFFF);
        Ana_Set_Reg(0x013C, 0x3000, 0xFFFF);
        Ana_Set_Reg(0x809A, 0x6025, 0xFFFF);
        Ana_Set_Reg(0x8552, 0x0004, 0xFFFF);
        Ana_Set_Reg(0x853E, 0x0021, 0xFFFF);
        Ana_Set_Reg(0x8534, 0x0300, 0xFFFF);
        Ana_Set_Reg(0x853A, 0x0000, 0xFFFF);
        Ana_Set_Reg(0x8542, 0x8542, 0xFFFF);
        Ana_Set_Reg(0x8094, 0x0000, 0xFFFF);
        Ana_Set_Reg(0x809E, 0x0004, 0xFFFF);
        while ((Ana_Get_Reg(0x809A) & 0x4) != 0)
        {
            printk("WAITING clock\n");
        }

        Ana_Set_Reg(0x809A, 0x6023, 0xFFFF); 
        
        Ana_Set_Reg(0x80A6, 0x0100, 0xFFFF);
        Ana_Set_Reg(0x80BC, 0x0040, 0xFFFF);
        Ana_Set_Reg(0x80BC, 0x0000, 0xFFFF);
        
        Ana_Set_Reg(0x8CD0, 175, 0xFFFF); 
        
        
        Ana_Set_Reg(0x8CCE, 0x8000, 0xFFFF); 
        msleep(2);
        while ((Ana_Get_Reg(0x8CCE) & 0x8) != 0)
        {
            printk("WAITING FQMTR_CON0[3]\n");
            
        }
        
        
        
        freq_meter_data = Ana_Get_Reg(0x8CD2); 
        
        output_freq = (32768 * freq_meter_data) / (175);
        
        
        printk("freq_meter_data %d %d\n", freq_meter_data, output_freq);
        output_freq = output_freq / 96000;
        printk("freq divider %d\n", output_freq);
    }
#endif
    dMax = dCurValue = 0;
    hw_read_idx = 0;
    if (mEnableAuxAdc == 1)
    {
        Ana_Set_Reg(MT6332_AUXADC_CON13, 20, 0x01FF); 
        Ana_Set_Reg(MT6332_AUXADC_CON12, 21, 0x007F); 
    }
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0x0040, 0x0040);
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0x0080, 0x0080);
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0x0100, 0x0100);
    msleep(1);
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x0040);
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x0080);
    Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x0100);
    if (mEnableAuxAdc == 1)
    {
        Ana_Set_Reg(MT6332_AUXADC_CON12, 21, 0x007F); 
        Ana_Set_Reg(MT6332_AUXADC_CON36, 0xAB, 0x01FF); 
        Ana_Set_Reg(MT6332_AUXADC_CON12, 0x8000, 0x8000); 
    }

    

    
    Ana_Set_Reg(MT6332_AUXADC_CON13, 0x0200, 0x0200);     

    

    do
    {
        iv_queue = Ana_Get_Reg(MT6332_AUXADC_CON33);
        v_cnt = (iv_queue >> 8) & 0x3F;
        i_cnt = iv_queue & 0x3F;
        
        
        {
            dCheckCount++;
        }
        

        if ( (i_cnt > 0))
        {
            dValidCount++;
            if (i_cnt > HW_BUFFER_LENGTH)
            {
                iov_cnt = i_cnt;
                iov_flag = 1;
                i_cnt = HW_BUFFER_LENGTH;
            }
            else
            {
                iov_flag = 0;
            }

            cnt = i_cnt;
            if (cnt + hw_read_idx >= HW_BUFFER_LENGTH)
            {
                cnt1 = HW_BUFFER_LENGTH - hw_read_idx;
                cnt2 = cnt - cnt1;
            }
            else
            {
                cnt1 = cnt;
                cnt2 = 0;
            }

            for (i = 0; i < cnt1; i++)
            {
                int i_tmp;

                i_tmp = Ana_Get_Reg(I_Buffer_Table[hw_read_idx]);
                
                if (hw_read_idx == 17 || hw_read_idx == 18 || hw_read_idx == 19)
                {
                    dCurValue = ((i_tmp >> 3) & 0xFFF); 
                }
                else
                {
                    dCurValue = (i_tmp & 0xFFF); 
                }

                
                
                
                
                
                if (dCurValue > dMax)
                {
                    dMax = dCurValue;
                }
                hw_read_idx++;

                if (hw_read_idx >= HW_BUFFER_LENGTH)
                {
                    hw_read_idx = 0;
                }
            }

            
            for (i = 0; i < cnt2; i++)
            {
                int i_tmp;

                i_tmp = Ana_Get_Reg(I_Buffer_Table[hw_read_idx]);
                
                if (hw_read_idx == 17 || hw_read_idx == 18 || hw_read_idx == 19)
                {
                    dCurValue = ((i_tmp >> 3) & 0xFFF); 
                }
                else
                {
                    dCurValue = (i_tmp & 0xFFF); 
                }
                if (dCurValue > dMax)
                {
                    dMax = dCurValue;
                }
                hw_read_idx++;
                if (hw_read_idx >= HW_BUFFER_LENGTH)
                {
                    hw_read_idx = 0;
                }
            }

            dRecReadIndex += cnt;

            if (iov_flag)
            {
                Ana_Set_Reg(MT6332_AUXADC_CON12, 0x0080, 0x0080);
                printk("vibspk auxadc skip io [%d] i [%d]\n", iov_flag, iov_cnt);
                Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x0080);
            }
        }


    }
    while (dRecCount > dRecReadIndex);

    
    
    
    printk("vibspk auxadc- : dMax = %d dRecReadIndex [%d] dValidCount [%d] dCheckCount [%d]\n", dMax, dRecReadIndex, dValidCount, dCheckCount);
#if 0
    Ana_Set_Reg(0x80be, 0x02, 0x02);
    Ana_Set_Reg(0x80c0, 0x02, 0x02);

    Ana_Set_Reg(MT6332_AUXADC_CON12, 0, 0x8000); 
    Ana_Set_Reg(MT6332_AUXADC_CON13, 0, 0x0200);     
    Ana_Set_Reg(MT6332_AUXADC_CON0, 0x0000, 0x8000);
#endif
    Ana_Set_Reg(MT6332_AUXADC_CON13, 0, 0x0200);     
    mEnableAuxAdc = 2;
    printk("vibspk auxadc-\n");
    return dMax;
}


#endif

static int Audio_AuxAdcData_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
#ifdef CONFIG_MTK_SPEAKER
    ucontrol->value.integer.value[0] = Audio_AuxAdcData_Get_ext();
#else
    ucontrol->value.integer.value[0] = 0;
#endif
    return 0;
}

static int Audio_AuxAdcData_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    dAuxAdcChannel = ucontrol->value.integer.value[0];
    printk("%s dAuxAdcChannel = 0x%x \n", __func__, dAuxAdcChannel);
    return 0;
}


static const struct snd_kcontrol_new Audio_snd_auxadc_controls[] =
{
    SOC_SINGLE_EXT("Audio AUXADC Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_AuxAdcData_Get, Audio_AuxAdcData_Set),
};


static const char *amp_function[] = {"Off", "On"};
static const char *aud_clk_buf_function[] = {"Off", "On"};

static const char *DAC_DL_PGA_Headset_GAIN[] = {"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
                                                "-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db" , "-40Db"
                                               };
static const char *DAC_DL_PGA_Handset_GAIN[] = {"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
                                                "-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db" , "-40Db"
                                               };

static const char *DAC_DL_PGA_Speaker_GAIN[] = {"8Db", "7Db", "6Db", "5Db", "4Db", "3Db", "2Db", "1Db", "0Db", "-1Db", "-2Db", "-3Db",
                                                "-4Db", "-5Db", "-6Db", "-7Db", "-8Db", "-9Db", "-10Db" , "-40Db"
                                               };


static int Lineout_PGAL_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("Speaker_PGA_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL];
    return 0;
}

static int Lineout_PGAL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN) - 1))
    {
        index = 0x1f;
    }
    Ana_Set_Reg(ZCD_CON1, index , 0x001f);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKL] = ucontrol->value.integer.value[0];
    return 0;
}

static int Lineout_PGAR_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("%s  = %d\n", __func__, mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR];
    return 0;
}

static int Lineout_PGAR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    int index = 0;
    printk("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN) - 1))
    {
        index = 0x1f;
    }
    Ana_Set_Reg(ZCD_CON1, index << 7 , 0x0f10);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_SPKR] = ucontrol->value.integer.value[0];
    return 0;
}

static int Handset_PGA_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("Handset_PGA_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL];
    return 0;
}

static int Handset_PGA_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    int index = 0;

    printk("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN) - 1))
    {
        index = 0x1f;
    }
    Ana_Set_Reg(ZCD_CON3, index , 0x001f);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HSOUTL] = ucontrol->value.integer.value[0];
    return 0;
}


static void HeadsetLVolumeSet(void)
{
    int index = 0;
    printk("%s\n", __func__);
    index =   mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL];
    Ana_Set_Reg(ZCD_CON2, index , 0x001f);
}

static int Headset_PGAL_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("Headset_PGAL_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL];
    return 0;
}

static int Headset_PGAL_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    int index = 0;
    int offset = 0 , step = 0;

    

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN) - 1))
    {
        index = 0x1f;
    }
    offset =  mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL] - ucontrol->value.integer.value[0];

    if (offset > 0)
    {
        step = -1;
    }
    else if (offset == 0)
    {
        printk("return for gain is the same");
        return 0;
    }
    else
    {
        step = 1;
    }
    
#if 0
    while (offset != 0)
    {
        mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL] += step;
        Ana_Set_Reg(ZCD_CON2, (mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL]), 0x001f);
        msleep(1);
        offset += step;
    }
#endif
    msleep(1);
    Ana_Set_Reg(ZCD_CON2, index , 0x001f);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTL] = ucontrol->value.integer.value[0];
    return 0;
}

static void HeadsetRVolumeSet(void)
{
    int index = 0;
    printk("%s\n", __func__);
    index =   mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
    Ana_Set_Reg(ZCD_CON2, index << 7, 0x0f80);
}

static int Headset_PGAR_Get(struct snd_kcontrol *kcontrol,
                            struct snd_ctl_elem_value *ucontrol)
{
    printk("Headset_PGAR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR];
    return 0;
}

static int Headset_PGAR_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    int index = 0;
    int offset = 0 , step = 0;

    printk("%s(), index = %d\n", __func__, ucontrol->value.enumerated.item[0]);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    if (ucontrol->value.enumerated.item[0] == (ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN) - 1))
    {
        index = 0x1f;
    }

    offset =  mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] - ucontrol->value.integer.value[0];
    if (offset > 0)
    {
        step = -1;
    }
    else if (offset == 0)
    {
        printk("return for gain is the same");
        return 0;
    }
    else
    {
        step = 1;
    }
#if 0
    while (offset != 0)
    {
        mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] += step;
        Ana_Set_Reg(ZCD_CON2, (mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR]) << 7, 0x0f80);
        msleep(1);
        offset += step;
    }
#endif
    msleep(1);
    Ana_Set_Reg(ZCD_CON2, index << 7, 0x0f80);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = ucontrol->value.integer.value[0];
    return 0;
}



static uint32 mHp_Impedance = 32;

static int Audio_Hp_Impedance_Get(struct snd_kcontrol *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_Hp_Impedance_Get = %d\n", mHp_Impedance);
    ucontrol->value.integer.value[0] = mHp_Impedance;
    return 0;

}

static int Audio_Hp_Impedance_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    mHp_Impedance = ucontrol->value.integer.value[0];
    printk("%s Audio_Hp_Impedance_Set = 0x%x \n", __func__, mHp_Impedance);
    return 0;
}

static int Aud_Clk_Buf_Get(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    printk("\%s n", __func__);
    ucontrol->value.integer.value[0] = audck_buf_Count;
    return 0;
}

static int Aud_Clk_Buf_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    
    printk("%s(), value = %d\n", __func__, ucontrol->value.enumerated.item[0]);
    if (ucontrol->value.integer.value[0])
    {
        audckbufEnable(true);
    }
    else
    {
        audckbufEnable(false);
    }
    return 0;
}


static const struct soc_enum Audio_DL_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(amp_function), amp_function),
    
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Headset_GAIN), DAC_DL_PGA_Headset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Handset_GAIN), DAC_DL_PGA_Handset_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN), DAC_DL_PGA_Speaker_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(DAC_DL_PGA_Speaker_GAIN), DAC_DL_PGA_Speaker_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aud_clk_buf_function), aud_clk_buf_function),
};

static const struct snd_kcontrol_new mt6331_snd_controls[] =
{
    SOC_ENUM_EXT("Audio_Amp_R_Switch", Audio_DL_Enum[0], Audio_AmpR_Get, Audio_AmpR_Set),
    SOC_ENUM_EXT("Audio_Amp_L_Switch", Audio_DL_Enum[1], Audio_AmpL_Get, Audio_AmpL_Set),
    SOC_ENUM_EXT("Voice_Amp_Switch", Audio_DL_Enum[2], Voice_Amp_Get, Voice_Amp_Set),
    SOC_ENUM_EXT("Speaker_Amp_Switch", Audio_DL_Enum[3], Speaker_Amp_Get, Speaker_Amp_Set),
    SOC_ENUM_EXT("Headset_Speaker_Amp_Switch", Audio_DL_Enum[4], Headset_Speaker_Amp_Get, Headset_Speaker_Amp_Set),
    SOC_ENUM_EXT("Headset_PGAL_GAIN", Audio_DL_Enum[5], Headset_PGAL_Get, Headset_PGAL_Set),
    SOC_ENUM_EXT("Headset_PGAR_GAIN", Audio_DL_Enum[6], Headset_PGAR_Get, Headset_PGAR_Set),
    SOC_ENUM_EXT("Handset_PGA_GAIN", Audio_DL_Enum[7], Handset_PGA_Get, Handset_PGA_Set),
    SOC_ENUM_EXT("Lineout_PGAR_GAIN", Audio_DL_Enum[8], Lineout_PGAR_Get, Lineout_PGAR_Set),
    SOC_ENUM_EXT("Lineout_PGAL_GAIN", Audio_DL_Enum[9], Lineout_PGAL_Get, Lineout_PGAL_Set),
    SOC_ENUM_EXT("AUD_CLK_BUF_Switch", Audio_DL_Enum[10], Aud_Clk_Buf_Get, Aud_Clk_Buf_Set),
    SOC_SINGLE_EXT("Audio HP Impedance", SND_SOC_NOPM, 0, 512, 0, Audio_Hp_Impedance_Get, Audio_Hp_Impedance_Set),
};

static const struct snd_kcontrol_new mt6331_Voice_Switch[] =
{
    
};

void SetMicPGAGain(void)
{
    int index = 0;
    index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index , 0x0007);
    index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 4, 0x0070);
    index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 8, 0x0700);
    index = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 12, 0x7000);
}

static bool GetAdcStatus(void)
{
    int i = 0;
    for (i = AUDIO_ANALOG_DEVICE_IN_ADC1 ; i < AUDIO_ANALOG_DEVICE_MAX ; i++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}

static bool GetDacStatus(void)
{
    int i = 0;
    for (i = AUDIO_ANALOG_DEVICE_OUT_EARPIECER ; i < AUDIO_ANALOG_DEVICE_2IN1_SPK ; i++)
    {
        if (mCodec_data->mAudio_Ana_DevicePower[i] == true)
        {
            return true;
        }
    }
    return false;
}


static bool TurnOnADcPowerACC(int ADCType, bool enable)
{
    printk("%s ADCType = %d enable = %d \n", __func__, ADCType, enable);
    if (enable)
    {
        
        uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
        uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2];
        if (GetAdcStatus() == false)
        {
            audckbufEnable(true);
            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0001, 0x0001);  
            }
            else
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0000, 0x0001);  
            }
            NvregEnable(true);
            ClsqAuxEnable(true);
            ClsqEnable(true);
            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x4400, 0xffff);      
            }
            else
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      
            }
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);

            
            
            if (mAdc_Power_Mode == false)
            {
                SetMicbias1lowpower(false);
                SetMicbias0lowpower(false);
            }
            else
            {
                SetMicbias1lowpower(true);
                SetMicbias0lowpower(true);
            }

            
            SetMicVref2(0x2);
            SetMicVref3(0x2); 
            SetMic2DCcoupleSwitch(false);
            SetMic3DCcoupleSwitch(false);
            if (mAdc_Power_Mode == false)
            {
                SetMic2powermode(false);
                SetMic3powermode(false);
            }
            else
            {
                SetMic2powermode(true);
                SetMic3powermode(true);
            }
            
            
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0007, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0000, 0x8888);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x000f, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x8888, 0x8888);   
            }
            
            SetMicPGAGain();
            
            

            
            Topck_Enable(true);
            AdcClockEnable(true);
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   
            if ((GetDacStatus() == false))
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);   
            }
            else
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
            }
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   
            Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);   

            Ana_Set_Reg(AFE_MIC_ARRAY_CFG, 0x44e4, 0xffff);   
            Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   

            Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x302F | (GetULNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]) << 10), 0xffff); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1) , 0x001f);
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0041, 0xffff);   
        }

        
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0051, 0x001f);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            AudioPreAmp1_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
            Audio_ADC1_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1]);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);
        }
        else   if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0040, 0x03c0);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            Audio_ADC2_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2]);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0700, 0x0f00);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0f00, 0x0f00);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0280, 0x0380);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0001, 0x000f);   
            Audio_ADC3_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3]);
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x7000, 0xf000);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0xf000, 0xf000);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x1400, 0x1c00);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0010, 0x00f0);   
            Audio_ADC4_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4]);
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else
        {
            printk("\n");
        }
    }
    else
    {
        if ((GetAdcStatus() == false))
        {
            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0xa000, 0xffff);   

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0000, 0xffff);   

            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2222, 0xffff);   
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0002, 0xffff);   

            
            SetMic2powermode(true);
            SetMic3powermode(true);

            Ana_Set_Reg(AUDADC_CFG0, 0x0000, 0xffff);   
            ClsqAuxEnable(false);
            ClsqEnable(false);
            NvregEnable(false);
            Topck_Enable(false);
            audckbufEnable(false);
            if (GetDLStatus() == false)
            {
                

            }
            else
            {

            }

        }
        
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {
        }
    }
    return true;
}

static bool TurnOnADcPowerDmic(int ADCType, bool enable)
{
    printk("%s ADCType = %d enable = %d \n", __func__, ADCType, enable);
    if (enable)
    {
        uint32 ULIndex = GetULFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]);
        uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
        uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2];
        if (GetAdcStatus() == false)
        {
            audckbufEnable(true);
            NvregEnable(true);
            ClsqAuxEnable(true);
            ClsqEnable(true);
            Ana_Set_Reg(AUDDIGMI_CFG0, 0x0041, 0xffff);    
            Ana_Set_Reg(AUDDIGMI_CFG1, 0x0041, 0xffff);    

            Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      
            
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, AUDIO_ANALOGUL_MODE_DMIC);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, AUDIO_ANALOGUL_MODE_DMIC);
            SetMicbias1lowpower(false);
            SetMicbias0lowpower(false);

            
            SetMicVref2(0x2);
            SetMicVref3(0x2); 
            SetMic2DCcoupleSwitch(false);
            SetMic3DCcoupleSwitch(false);
            SetMic2powermode(false);
            SetMic3powermode(false);


            
            Topck_Enable(true);
            AdcClockEnable(true);

            if ((GetDacStatus() == false))
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);   
            }
            else
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
            }

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   
            Ana_Set_Reg(PMIC_AFE_TOP_CON0, (ULIndex << 7) | (ULIndex << 6), 0xffff); 
            Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (1 << 7),(1<<7) ); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (1 << 5) | (1 << 6), (1 << 5) | (1 << 6)); 

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (1 << 5) | (1 << 6), (1 << 5) | (1 << 6)); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0043, 0xffff);

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0003, 0xffff);   

        }
        
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
        }
        else   if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {

        }
        else
        {
            printk("\n");
        }
    }
    else
    {
        if (GetAdcStatus() == false)
        {

            
            SetMic2powermode(true);
            SetMic3powermode(true);
            OpenMicbias3(false);
            OpenMicbias2(false);
            SetMic2powermode(true);
            SetMic3powermode(true);
            

            Ana_Set_Reg(AUDDIGMI_CFG0, 0x0040, 0xffff);    
            Ana_Set_Reg(AUDDIGMI_CFG1, 0x0040, 0xffff);    

            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (0 << 7),(1<<7) ); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (0 << 5) | (0 << 6), (1 << 5) | (1 << 6)); 

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, 0x0000, 0xffe0);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0xa000, 0xffff);   

            ClsqAuxEnable(false);
            ClsqEnable(false);
            NvregEnable(false);
            Topck_Enable(false);
            audckbufEnable(false);
            if (GetDLStatus() == false)
            {
                
                Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
            }
            else
            {

            }

        }
        
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {

        }
    }
    return true;
}

static bool TurnOnADcPowerDCC(int ADCType, bool enable)
{
    printk("%s ADCType = %d enable = %d \n", __func__, ADCType, enable);
    if (enable)
    {
        
        uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
        uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2];
        if (GetAdcStatus() == false)
        {
            audckbufEnable(true);
            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0001, 0x0001);  
            }
            else
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0000, 0x0001);  
            }
            NvregEnable(true);    
            ClsqAuxEnable(true);    
            ClsqEnable(true);

            
            Topck_Enable(true);
            AdcClockEnable(true);

            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x4400, 0xffff);      
            }
            else
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      
            }
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);
            
            
            
            if (mAdc_Power_Mode == false)
            {
                SetMicbias1lowpower(false);
                SetMicbias0lowpower(false);
            }
            else
            {
                SetMicbias1lowpower(true);
                SetMicbias0lowpower(true);
            }

            
            
            SetMicVref2(0x2);
            SetMicVref3(0x2); 
            SetMic2DCcoupleSwitch(false);
            SetMic3DCcoupleSwitch(false);
            if (mAdc_Power_Mode == false)
            {
                SetMic2powermode(false);
                SetMic3powermode(false);
            }
            else
            {
                SetMic2powermode(true);
                SetMic3powermode(true);
            }

            
            
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0007, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0000, 0x8888);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x000f, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x8888, 0x8888);   
            }

            Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2061, 0xffff);   
            SetMicPGAGain();

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x01c7, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x00d3, 0xffff);   

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   

            if ((GetDacStatus() == false))
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);   
            }
            else
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
            }

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   
            Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);   

            Ana_Set_Reg(AFE_MIC_ARRAY_CFG, 0x44e4, 0xffff);   
            Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   

            Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x302F | (GetULNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]) << 10), 0xffff); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1) , 0x001f); 

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0041, 0xffff);   

        }
        
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0007, 0x003f);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0013, 0x003f);   
            Audio_ADC1_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1]);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);
            AudioPreAmp1_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }

            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x01c0, 0x03c0);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x00c0, 0x03c0);   
            Audio_ADC2_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2]);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0700, 0x0f00);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0f00, 0x0f00);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0007, 0x000f);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0280, 0x0380);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0003, 0x000f);   
            Audio_ADC3_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3]);

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x7000, 0xf000);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0xf000, 0xf000);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0070, 0x00f0);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x1400, 0x1c00);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0030, 0x00f0);   
            Audio_ADC4_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4]);
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else
        {
            printk("\n");
        }
    }
    else
    {
        if ((GetAdcStatus() == false))
        {
            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0xa000, 0xffff);   

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_DCCLK_CFG0, 0x0, 0xffff);   

            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2222, 0xffff);   
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0002, 0xffff);   

            
            SetMic2powermode(true);
            SetMic3powermode(true);
            
            
            
            
            
            

            Ana_Set_Reg(AUDADC_CFG0, 0x0000, 0xffff);   

            ClsqAuxEnable(false);
            ClsqEnable(false);
            NvregEnable(false);
            Topck_Enable(false);
            audckbufEnable(false);
            if (GetDLStatus() == false)
            {
                Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
            }
            else
            {

            }

        }
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {

        }
    }
    return true;
}


static bool TurnOnADcPowerDCCECM(int ADCType, bool enable)
{
    printk("%s ADCType = %d enable = %d \n", __func__, ADCType, enable);
    if (enable)
    {
        
        uint32 SampleRate_VUL1 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC];
        uint32 SampleRate_VUL2 = mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC_2];
        if (GetAdcStatus() == false)
        {
            audckbufEnable(true);
            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0001, 0x0001);  
            }
            else
            {
                Ana_Set_Reg(AUDBUF_CFG4, 0x0000, 0x0001);  
            }
            NvregEnable(true);    
            ClsqAuxEnable(true);    
            ClsqEnable(true);

            
            Topck_Enable(true);
            AdcClockEnable(true);

            if (mAdc_Power_Mode == true)
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x4400, 0xffff);      
            }
            else
            {
                Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      
            }

            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);

            
            
            if (mAdc_Power_Mode == false)
            {
                SetMicbias1lowpower(false);
                SetMicbias0lowpower(false);
            }
            else
            {
                SetMicbias1lowpower(true);
                SetMicbias0lowpower(true);
            }
            


            
            
            SetMicVref2(0x2);
            SetMicVref3(0x2); 
            SetMic2DCcoupleSwitch(true);
            SetMic3DCcoupleSwitch(true);
            if (mAdc_Power_Mode == false)
            {
                SetMic2powermode(false);
                SetMic3powermode(false);
            }
            else
            {
                SetMic2powermode(true);
                SetMic3powermode(true);
            }

            
            

            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0007, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0000, 0x8888);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x000f, 0xffff);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x8888, 0x8888);   
            }

            Ana_Set_Reg(AFE_DCCLK_CFG0, 0x2061, 0xffff);   

            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x3333, 0xffff);   
            SetMicPGAGain();

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x01c7, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x00d3, 0xffff);   


            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   

            if ((GetDacStatus() == false))
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x005a, 0xffff);   
            }
            else
            {
                Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
            }

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);   
            Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);   

            Ana_Set_Reg(AFE_MIC_ARRAY_CFG, 0x44e4, 0xffff);   
            Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   

            Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x302F | (GetULNewIFFrequency(mBlockSampleRate[AUDIO_ANALOG_DEVICE_IN_ADC]) << 10), 0xffff); 
            Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (ULSampleRateTransform(SampleRate_VUL1) << 3 | ULSampleRateTransform(SampleRate_VUL1) << 1) , 0x001f); 

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);

            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0041, 0xffff);   

        }
        EnableMicBias(ADCType, enable);
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0007, 0x003f);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0013, 0x003f);   
            AudioPreAmp1_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
            Audio_ADC1_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1]);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC1, mAudio_Analog_Mic1_mode);

        }
        else   if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0007, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0070, 0x00f0);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x000f, 0x000f);   
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x00f0, 0x00f0);   
            }
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x01c0, 0x03c0);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0x007f);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x00c0, 0x03c0);   
            Audio_ADC2_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2]);
            SetDCcoupleNP(AUDIO_ANALOG_DEVICE_IN_ADC2, mAudio_Analog_Mic2_mode);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0700, 0x0f00);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x0f00, 0x0f00);   
            }

            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0007, 0x000f);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0280, 0x0380);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0003, 0x000f);   
            Audio_ADC3_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3]);

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {
            if (mAdc_Power_Mode == false)
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x7000, 0xf000);   
            }
            else
            {
                Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0xf000, 0xf000);   
            }

            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0070, 0x00f0);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x1400, 0x1c00);   
            Ana_Set_Reg(AUDPREAMP_CFG2, 0x0030, 0x00f0);   
            Audio_ADC4_Set_Input(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4]);

            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); 
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
        }
        else
        {
            printk("\n");
        }
    }
    else
    {
        if ((GetAdcStatus() == false))
        {
            Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0xa000, 0xffff);   

            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0000, 0xffff);   
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0000, 0xffff);   
            Ana_Set_Reg(AFE_DCCLK_CFG0, 0x0, 0xffff);   

            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2222, 0xffff);   
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0002, 0xffff);   

            
            SetMic2powermode(true);
            SetMic3powermode(true);
            
            
            

            
            
            

            Ana_Set_Reg(AUDADC_CFG0, 0x0000, 0xffff);   

            ClsqAuxEnable(false);
            ClsqEnable(false);
            NvregEnable(false);
            Topck_Enable(false);
            audckbufEnable(false);
            if (GetDLStatus() == false)
            {
                Ana_Set_Reg(AFE_UL_DL_CON0, 0x0000, 0xffff);   
            }
            else
            {

            }

        }
        EnableMicBias(ADCType, enable);
        
        if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC1)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC2)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC3)
        {

        }
        else if (ADCType == AUDIO_ANALOG_DEVICE_IN_ADC4)
        {

        }
    }
    return true;
}

static bool TurnOnVOWDigitalHW(bool enable)
{
    printk("%s enable = %d \n", __func__, enable);
    if (enable)
    {
        Ana_Set_Reg(AFE_VOW_TOP, 0x0810, 0xffff);   
    }
    else
    {
        Ana_Set_Reg(AFE_VOW_TOP, 0x0012, 0xffff);   
        Ana_Set_Reg(AFE_VOW_TOP, 0x8012, 0xffff);   
    }
    return true;
}

static bool TurnOnVOWADcPowerACC(int ADCType, bool enable)
{
    printk("%s ADCType = %d enable = %d \n", __func__, ADCType, enable);
    if (enable)
    {
#if defined(VOW_TONE_TEST)
        OpenAfeDigitaldl1(false);
        OpenAnalogHeadphone(false);
        EnableSideGenHw(Soc_Aud_InterConnectionOutput_O03, Soc_Aud_InterConnectionOutput_Num_Output, false);
        AudDrv_Clk_Off();
#endif
#if defined (MTK_VOW_SUPPORT)
        
        VowDrv_EnableHW(true);
#endif
        
        
        
        {
            NvregEnable(true); 
            
            Ana_Set_Reg(ANALDO_CON3, 0xc542, 0xffff);   
            Ana_Set_Reg(AUDBUF_CFG4, 0x1, 0xffff);   
            Ana_Set_Reg(AUDMICBIAS_CFG0, 0x004f, 0xffff);   
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x000f, 0xffff);   
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x222f, 0xffff);   

            Ana_Set_Reg(AUDVOWPLL_CFG2, 0x002b, 0xffff);   
            Ana_Set_Reg(AUDVOWPLL_CFG0, 0x00bd, 0xffff);   

            Ana_Set_Reg(AUDADC_CFG0, 0x3400, 0xffff);   

            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0002, 0xffff);   
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0011, 0xffff); 

            Ana_Set_Reg(AUDADC_CFG1, 0x6400, 0xffff);   

            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0005, 0xffff);   

            
            
            ClsqEnable(true);
            ClsqAuxEnable(true);
            


            
            LowPowerAdcClockEnable(true);
            
            
            
            
#if 1  
            Ana_Set_Reg(AFE_VOW_CFG0, reg_AFE_VOW_CFG0, 0xffff);   
            Ana_Set_Reg(AFE_VOW_CFG1, reg_AFE_VOW_CFG1, 0xffff);   
            Ana_Set_Reg(AFE_VOW_CFG2, reg_AFE_VOW_CFG2, 0xffff);   
            Ana_Set_Reg(AFE_VOW_CFG3, reg_AFE_VOW_CFG3, 0xffff);   
            Ana_Set_Reg(AFE_VOW_CFG4, reg_AFE_VOW_CFG4, 0xffff);   
            Ana_Set_Reg(AFE_VOW_CFG5, reg_AFE_VOW_CFG5, 0xffff);   
#endif
            
            
#if defined(VOW_TONE_TEST)
            
            AudDrv_Clk_On();
            OpenAfeDigitaldl1(true);
            OpenAnalogHeadphone(true);
#endif
        }

    }
    else
    {
#if defined (MTK_VOW_SUPPORT)
        
        VowDrv_EnableHW(false);
#endif
        
        {
            
            
            
            
            LowPowerAdcClockEnable(false);
            
            ClsqEnable(false);
            ClsqAuxEnable(false);
            
            Ana_Set_Reg(AUDPREAMP_CFG1, 0x0000, 0xffff);         
            Ana_Set_Reg(AUDADC_CFG1, 0x0400, 0xffff);            
            Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0xffff);         
            Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0000, 0xffff);     
            Ana_Set_Reg(AUDADC_CFG0, 0x0000, 0xffff);            
            Ana_Set_Reg(AUDVOWPLL_CFG0, 0x00BC, 0xffff);         
            Ana_Set_Reg(AUDVOWPLL_CFG2, 0x0023, 0xffff);         
            Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2222, 0xffff);      
            Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0002, 0xffff);      
            Ana_Set_Reg(AUDMICBIAS_CFG0, 0x0000, 0xffff);        
            Ana_Set_Reg(AUDBUF_CFG4, 0x0000, 0xffff);            
            Ana_Set_Reg(ANALDO_CON3, 0xC540, 0xffff);            
            
            
            NvregEnable(false);
            
            
        }
    }
    return true;
}



static const char *ADC_function[] = {"Off", "On"};
static const char *ADC_power_mode[] = {"normal", "lowpower"};
static const char *PreAmp_Mux_function[] = {"OPEN", "IN_ADC1", "IN_ADC2"};
static const char *ADC_UL_PGA_GAIN[] = { "0Db", "6Db", "12Db", "18Db", "24Db", "30Db"};
static const char *Pmic_Digital_Mux[] = { "ADC1", "ADC2", "ADC3", "ADC4"};
static const char *Adc_Input_Sel[] = { "idle", "AIN", "Preamp"};
static const char *Audio_AnalogMic_Mode[] = { "ACCMODE", "DCCMODE", "DMIC", "DCCECMDIFFMODE", "DCCECMSINGLEMODE"};
static const char *Audio_VOW_ADC_Function[] = {"Off", "On"};
static const char *Audio_VOW_Digital_Function[] = {"Off", "On"};

static const struct soc_enum Audio_UL_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_function), ADC_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(PreAmp_Mux_function), PreAmp_Mux_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Adc_Input_Sel), Adc_Input_Sel),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_UL_PGA_GAIN), ADC_UL_PGA_GAIN),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Digital_Mux), Pmic_Digital_Mux),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_AnalogMic_Mode), Audio_AnalogMic_Mode),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ADC_power_mode), ADC_power_mode),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_VOW_ADC_Function), Audio_VOW_ADC_Function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Audio_VOW_Digital_Function), Audio_VOW_Digital_Function),
};

static int Audio_ADC1_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_ADC1_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1];
    return 0;
}

static int Audio_ADC1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    mutex_lock(&Ana_Power_Mutex);
    if (ucontrol->value.integer.value[0])
    {
        if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1 , true);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1 , true);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC1, true);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC1 , true);
        }
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC1] = ucontrol->value.integer.value[0];
        if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1 , false);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC1 , false);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC1, false);
        }
        else if (mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic1_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC1, false);
        }
    }
    mutex_unlock(&Ana_Power_Mutex);
    return 0;
}

static int Audio_ADC2_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_ADC2_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2];
    return 0;
}

static int Audio_ADC2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    mutex_lock(&Ana_Power_Mutex);
    if (ucontrol->value.integer.value[0])
    {
        if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC2 , true);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2 , true);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC2 , true);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC2 , true);
        }
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC2] = ucontrol->value.integer.value[0];
        if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC2 , false);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC2 , false);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC2 , false);
        }
        else if (mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic2_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC2 , false);
        }
    }
    mutex_unlock(&Ana_Power_Mutex);
    return 0;
}

static int Audio_ADC3_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_ADC3_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3];
    return 0;
}

static int Audio_ADC3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    mutex_lock(&Ana_Power_Mutex);
    if (ucontrol->value.integer.value[0])
    {
        if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC3 , true);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC3 , true);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC3 , true);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC3 , true);
        }
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC3] = ucontrol->value.integer.value[0];

        if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC3 , false);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC3 , false);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC3 , false);
        }
        else if (mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic3_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC3 , false);
        }
    }
    mutex_unlock(&Ana_Power_Mutex);
    return 0;
}

static int Audio_ADC4_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_ADC4_Get = %d\n", mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4];
    return 0;
}

static int Audio_ADC4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    mutex_lock(&Ana_Power_Mutex);
    if (ucontrol->value.integer.value[0])
    {
        if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC4 , true);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC4 , true);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC4 , true);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC4 , true);
        }
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4] = ucontrol->value.integer.value[0];
    }
    else
    {
        mCodec_data->mAudio_Ana_DevicePower[AUDIO_ANALOG_DEVICE_IN_ADC4] = ucontrol->value.integer.value[0];
        if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_ACC)
        {
            TurnOnADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC4 , false);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCC)
        {
            TurnOnADcPowerDCC(AUDIO_ANALOG_DEVICE_IN_ADC4 , false);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DMIC)
        {
            TurnOnADcPowerDmic(AUDIO_ANALOG_DEVICE_IN_ADC4 , false);
        }
        else if (mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCCECMDIFF || mAudio_Analog_Mic4_mode == AUDIO_ANALOGUL_MODE_DCCECMSINGLE)
        {
            TurnOnADcPowerDCCECM(AUDIO_ANALOG_DEVICE_IN_ADC4 , false);
        }
    }
    mutex_unlock(&Ana_Power_Mutex);
    return 0;
}

static int Audio_ADC1_Sel_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1];
    return 0;
}

static void Audio_ADC1_Set_Input(int Input)
{
    if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_OPEN)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0000 << 1), 0x0006);  
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_ADC)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0001 << 1), 0x0006);
    }
    
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0002 << 1), 0x0006);
    }
    else
    {
        printk("%s() warning \n ", __func__);
    }
    printk("%s() done \n", __func__);
}

static int Audio_ADC1_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int input = ucontrol->value.integer.value[0] ;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    Audio_ADC1_Set_Input(input);
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_ADC2_Sel_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2];
    return 0;
}

static void Audio_ADC2_Set_Input(int Input)
{
    if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_OPEN)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0000 << 5), 0x0060);  
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_ADC)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0001 << 5), 0x0060);
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0002 << 5), 0x0060);
    }
    else
    {
        printk("%s() warning \n ", __func__);
    }
}


static int Audio_ADC2_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int input = ucontrol->value.integer.value[0];
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    Audio_ADC2_Set_Input(input);
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2] = ucontrol->value.integer.value[0];
    return 0;
}

static void Audio_ADC3_Set_Input(int Input)
{
    if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_OPEN)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0000 << 8), 0x0300);  
    }
    else if (Input  == AUDIO_ANALOG_AUDIOANALOG_INPUT_ADC)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0001 << 8), 0x0300);
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0002 << 8), 0x0300);
    }
    else
    {
        printk("%s() warning \n ", __func__);
    }
}

static int Audio_ADC3_Sel_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3];
    return 0;
}

static int Audio_ADC3_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int input = ucontrol->value.integer.value[0];
    printk("%s()\n", __func__);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    Audio_ADC3_Set_Input(input);
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_ADC4_Sel_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4];
    return 0;
}

static void Audio_ADC4_Set_Input(int Input)
{
    if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_OPEN)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0000 << 11), 0x1800);  
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_ADC)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0001 << 11), 0x1800);
    }
    else if (Input == AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP)
    {
        Ana_Set_Reg(AUDPREAMP_CFG1, (0x0002 << 11), 0x1800);
    }
    else
    {
        printk("%s() warning \n ", __func__);
    }
}


static int Audio_ADC4_Sel_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int input = ucontrol->value.integer.value[0];
    printk("%s()\n", __func__);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Adc_Input_Sel))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    Audio_ADC4_Set_Input(input);
    printk("%s() done \n", __func__);
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4] = ucontrol->value.integer.value[0];
    return 0;
}

static bool AudioPreAmp1_Sel(int Mul_Sel)
{
    printk("%s Mul_Sel = %d ", __func__, Mul_Sel);
    if (Mul_Sel == 0)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0000, 0x0030);    
    }
    else if (Mul_Sel == 1)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0010, 0x0030);       
    }
    
    else if (Mul_Sel == 2)
    {
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0020, 0x0030);    
    }
    else
    {
        printk("AnalogSetMux warning");
    }

    return true;
}


static int Audio_PreAmp1_Get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]; = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1];
    return 0;
}

static int Audio_PreAmp1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);

    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(PreAmp_Mux_function))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1] = ucontrol->value.integer.value[0];
    AudioPreAmp1_Sel(mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_PREAMP_1]);
    printk("%s() done \n", __func__);
    return 0;
}

static int Audio_PGA1_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1];
    return 0;
}

static int Audio_PGA1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index , 0x0007);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_PGA2_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_PGA2_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2];
    return 0;
}

static int Audio_PGA2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 4, 0x0070);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_PGA3_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d\n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3];
    return 0;
}

static int Audio_PGA3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 8, 0x0700);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_PGA4_Get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_AmpR_Get = %d \n", mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4];
    return 0;
}

static int Audio_PGA4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_UL_PGA_GAIN))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    Ana_Set_Reg(AUDPREAMPGAIN_CFG0, index << 12, 0x7000);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource1_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    printk("Audio_MicSource1_Get = %d\n", mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1];
    return 0;
}

static int Audio_MicSource1_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() index = %d done \n", __func__, index);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index | index << 8, 0x0303);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_1] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource2_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2];
    return 0;
}

static int Audio_MicSource2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index << 2 | index << 10, 0x0c0c);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_2] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_MicSource3_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3];
    return 0;
}

static int Audio_MicSource3_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index << 4 | index << 12, 0x3030);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_3] = ucontrol->value.integer.value[0];
    return 0;
}


static int Audio_MicSource4_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() = %d\n", __func__, mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4]);
    ucontrol->value.integer.value[0] = mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4];
    return 0;
}

static int Audio_MicSource4_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int index = 0;
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Pmic_Digital_Mux))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    index = ucontrol->value.integer.value[0];
    printk("%s() done \n", __func__);
    Ana_Set_Reg(AFE_MIC_ARRAY_CFG, index << 6 | index << 14, 0xc0c0);
    mCodec_data->mAudio_Ana_Mux[AUDIO_MICSOURCE_MUX_IN_4] = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Mic1_Mode_Select_Get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
    printk("%s() mAudio_Analog_Mic1_mode = %d\n", __func__, mAudio_Analog_Mic1_mode);
    ucontrol->value.integer.value[0] = mAudio_Analog_Mic1_mode;
    return 0;
}

static int Audio_Mic1_Mode_Select_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mAudio_Analog_Mic1_mode = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Analog_Mic1_mode);
    return 0;
}

static int Audio_Mic2_Mode_Select_Get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAudio_Analog_Mic2_mode);
    ucontrol->value.integer.value[0] = mAudio_Analog_Mic2_mode;
    return 0;
}

static int Audio_Mic2_Mode_Select_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mAudio_Analog_Mic2_mode = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Analog_Mic2_mode);
    return 0;
}


static int Audio_Mic3_Mode_Select_Get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAudio_Analog_Mic3_mode);
    ucontrol->value.integer.value[0] = mAudio_Analog_Mic3_mode;
    return 0;
}

static int Audio_Mic3_Mode_Select_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mAudio_Analog_Mic3_mode = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Analog_Mic3_mode);
    return 0;
}

static int Audio_Mic4_Mode_Select_Get(struct snd_kcontrol *kcontrol,
                                      struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAudio_Analog_Mic4_mode);
    ucontrol->value.integer.value[0] = mAudio_Analog_Mic4_mode;
    return 0;
}

static int Audio_Mic4_Mode_Select_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_AnalogMic_Mode))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mAudio_Analog_Mic4_mode = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Analog_Mic4_mode);
    return 0;
}

static int Audio_Adc_Power_Mode_Get(struct snd_kcontrol *kcontrol,
                                    struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAdc_Power_Mode);
    ucontrol->value.integer.value[0] = mAdc_Power_Mode;
    return 0;
}

static int Audio_Adc_Power_Mode_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(ADC_power_mode))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }
    mAdc_Power_Mode = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAdc_Power_Mode);
    return 0;
}


static int Audio_Vow_ADC_Func_Switch_Get(struct snd_kcontrol *kcontrol,
                                         struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAudio_Vow_Analog_Func_Enable);
    ucontrol->value.integer.value[0] = mAudio_Vow_Analog_Func_Enable;
    return 0;
}

static int Audio_Vow_ADC_Func_Switch_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_VOW_ADC_Function))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }

    if (ucontrol->value.integer.value[0])
    {
        TurnOnVOWADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1, true);
    }
    else
    {
        TurnOnVOWADcPowerACC(AUDIO_ANALOG_DEVICE_IN_ADC1, false);
    }

    mAudio_Vow_Analog_Func_Enable = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Vow_Analog_Func_Enable);
    return 0;
}

static int Audio_Vow_Digital_Func_Switch_Get(struct snd_kcontrol *kcontrol,
                                             struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %d\n", __func__, mAudio_Vow_Digital_Func_Enable);
    ucontrol->value.integer.value[0] = mAudio_Vow_Digital_Func_Enable;
    return 0;
}

static int Audio_Vow_Digital_Func_Switch_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.enumerated.item[0] > ARRAY_SIZE(Audio_VOW_Digital_Function))
    {
        printk("return -EINVAL\n");
        return -EINVAL;
    }

    if (ucontrol->value.integer.value[0])
    {
        TurnOnVOWDigitalHW(true);
    }
    else
    {
        TurnOnVOWDigitalHW(false);
    }

    mAudio_Vow_Digital_Func_Enable = ucontrol->value.integer.value[0];
    printk("%s() mAudio_Analog_Mic1_mode = %d \n", __func__, mAudio_Vow_Digital_Func_Enable);
    return 0;
}


static int Audio_Vow_Cfg0_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG0;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg0_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG0 = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Vow_Cfg1_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG1;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg1_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG1 = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Vow_Cfg2_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG2;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg2_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG2 = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Vow_Cfg3_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG3;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg3_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG3 = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Vow_Cfg4_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG4;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg4_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG4 = ucontrol->value.integer.value[0];
    return 0;
}

static int Audio_Vow_Cfg5_Get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    int value = reg_AFE_VOW_CFG5;
    printk("%s()  = %d\n", __func__, value);
    ucontrol->value.integer.value[0] = value;
    return 0;
}

static int Audio_Vow_Cfg5_Set(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()  = %ld\n", __func__, ucontrol->value.integer.value[0]);
    
    reg_AFE_VOW_CFG5 = ucontrol->value.integer.value[0];
    return 0;
}


static bool SineTable_DAC_HP_flag = false;
static bool SineTable_UL2_flag = false;

static int SineTable_UL2_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if (ucontrol->value.integer.value[0])
    {
        Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0002 , 0x2); 
        Ana_Set_Reg(AFE_SGEN_CFG0 , 0x0080 , 0xffffffff);
        Ana_Set_Reg(AFE_SGEN_CFG1 , 0x0101 , 0xffffffff);
    }
    else
    {
        Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0002 , 0x2); 
        Ana_Set_Reg(AFE_SGEN_CFG0 , 0x0000 , 0xffffffff);
        Ana_Set_Reg(AFE_SGEN_CFG1 , 0x0101 , 0xffffffff);
    }
    return 0;
}

static int SineTable_UL2_Get(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = SineTable_UL2_flag;
    return 0;
}

static int SineTable_DAC_HP_Get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = SineTable_DAC_HP_flag;
    return 0;
}

static int SineTable_DAC_HP_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        SineTable_DAC_HP_flag = ucontrol->value.integer.value[0];
        printk("TurnOnDacPower\n");
        audckbufEnable(true);
        ClsqEnable(true);
        Topck_Enable(true);
        NvregEnable(true);
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffffffff);
        Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffffffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffffffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffffffff); 
        Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffffffff); 
        Ana_Set_Reg(AFE_UL_DL_CON0 , 0x0001, 0xffffffff); 

        Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0 , 0x8330 , 0xffffffff);
        Ana_Set_Reg(AFE_DL_SRC2_CON0_H , 0x8330, 0xffff000f);

        Ana_Set_Reg(AFE_DL_SRC2_CON0_L , 0x1801 , 0xffffffff); 
        Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0001 , 0xffffffff); 
        Ana_Set_Reg(AFE_SGEN_CFG0 , 0x0080 , 0xffffffff);
        Ana_Set_Reg(AFE_SGEN_CFG1 , 0x0101 , 0xffffffff);

        Ana_Set_Reg(0x0680, 0x0000, 0xffff); 
        OpenClassAB();
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        Ana_Set_Reg(ZCD_CON0,   0x0700, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
        Ana_Set_Reg(ZCD_CON2,  0x0F9F , 0xffff); 
        Ana_Set_Reg(ZCD_CON3,  0x001F , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); 
        msleep(1);
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000f , 0xffff); 
        SetDcCompenSation();

        Ana_Set_Reg(AUDBUF_CFG0, 0xE149 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0, 0xE14F , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1, 0x0900 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG2, 0x0020 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0, 0xE14E , 0xffff); 
        Ana_Set_Reg(ZCD_CON2,       0x0489 , 0xffff); 

    }
    else
    {
        SineTable_DAC_HP_flag = ucontrol->value.integer.value[0];
        if (GetDLStatus() == false)
        {
            Ana_Set_Reg(AUDBUF_CFG0, 0xE149, 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0, 0x0000, 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5500, 0xffff); 
            Ana_Set_Reg(IBIASDIST_CFG0, 0x0192, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0028, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0000, 0xffff); 
            Ana_Set_Reg(AFE_CLASSH_CFG0, 0xd518, 0xffff); 
            Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0518, 0xffff); 
            Ana_Set_Reg(PMIC_AFE_TOP_CON0 , 0x0000 , 0xffffffff); 
        }
    }
    return 0;
}

static void ADC_LOOP_DAC_Func(int command)
{
    if (command == AUDIO_ANALOG_DAC_LOOP_DAC_HS_ON || command == AUDIO_ANALOG_DAC_LOOP_DAC_HP_ON)
    {
        audckbufEnable(true);
        ClsqEnable(true);
        Topck_Enable(true);
        NvregEnable(true);
        Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   
        Ana_Set_Reg(AUDADC_CFG0, 0x0400, 0xffff);      

        
        OpenMicbias1(true);
        SetMicbias1lowpower(false);
        OpenMicbias0(true);
        SetMicbias0lowpower(false);

        Ana_Set_Reg(AUDMICBIAS_CFG1, 0x285, 0xffff);   
        Ana_Set_Reg(AUDLDO_NVREG_CFG1, 0x0007, 0xffff);   
        Ana_Set_Reg(AUDLDO_NVREG_CFG2, 0x2277, 0xffff);   
        Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x0022, 0xffff);   
        SetMicPGAGain();
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x0051, 0xffff);   
        Ana_Set_Reg(AUDPREAMP_CFG1, 0x0055, 0xffff);   

        Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffffffff); 
        Ana_Set_Reg(AFE_TOP_CON0, 0x4000, 0xffffffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffffffff);
        Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffffffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffffffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffffffff); 
        Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffffffff); 
        Ana_Set_Reg(AFE_UL_DL_CON0 , 0x0001, 0xffffffff); 

        Ana_Set_Reg(AFE_UL_SRC0_CON0_H, 0x0000 , 0x0010); 

        Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0001, 0xffff);   
        Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0, 0x0380, 0xffff); 
        Ana_Set_Reg(AFE_DL_SRC2_CON0_H, 0x0800, 0xffff);   
        Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x0001, 0xffff); 

        
        
        OpenClassAB();

        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        Ana_Set_Reg(ZCD_CON0,   0x0700, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 
        if (command == AUDIO_ANALOG_DAC_LOOP_DAC_HS_ON)
        {
            Ana_Set_Reg(ZCD_CON3,  0x001f , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0, 0x0009 , 0xffff); 
            SetDcCompenSation();

            Ana_Set_Reg(AUDBUF_CFG0, 0xE010 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0, 0xE011 , 0xffff); 
            Ana_Set_Reg(ZCD_CON3,  0x0009 , 0xffff); 
        }
        else if (command == AUDIO_ANALOG_DAC_LOOP_DAC_HP_ON)
        {
            Ana_Set_Reg(ZCD_CON2,  0x0F9F , 0xffff); 
            Ana_Set_Reg(ZCD_CON3,  0x001f , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); 
            Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0, 0x000F , 0xffff); 
            SetDcCompenSation();

            Ana_Set_Reg(AUDBUF_CFG0, 0xE149 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0, 0xE14F , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG1, 0x0900 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG2, 0x0020 , 0xffff); 
            Ana_Set_Reg(AUDBUF_CFG0, 0xE14E , 0xffff); 
            Ana_Set_Reg(ZCD_CON2,  0x0489 , 0xffff); 
        }
    }
    else
    {
        if (command == AUDIO_ANALOG_DAC_LOOP_DAC_HS_ON)
        {
            Ana_Set_Reg(AUDBUF_CFG0,  0xe010 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0,  0x0000, 0xffff); 
        }
        else if (command == AUDIO_ANALOG_DAC_LOOP_DAC_HP_ON)
        {
            Ana_Set_Reg(AUDBUF_CFG0,  0xE149 , 0xffff); 
            Ana_Set_Reg(AUDDAC_CFG0,  0x0000, 0xffff); 
        }
        Ana_Set_Reg(AUDCLKGEN_CFG0,  0x5500, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,  0x0192, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,  0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,  0x0000, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG0, 0xd518, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0, 0x0518, 0xffff); 
    }
}

static bool DAC_LOOP_DAC_HS_flag = false;
static int ADC_LOOP_DAC_HS_Get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = DAC_LOOP_DAC_HS_flag;
    return 0;
}

static int ADC_LOOP_DAC_HS_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        DAC_LOOP_DAC_HS_flag = ucontrol->value.integer.value[0];
        ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HS_ON);
    }
    else
    {
        DAC_LOOP_DAC_HS_flag = ucontrol->value.integer.value[0];
        ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HS_OFF);
    }
    return 0;
}

static bool DAC_LOOP_DAC_HP_flag = false;
static int ADC_LOOP_DAC_HP_Get(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = DAC_LOOP_DAC_HP_flag;
    return 0;
}

static int ADC_LOOP_DAC_HP_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        DAC_LOOP_DAC_HP_flag = ucontrol->value.integer.value[0];
        ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HP_ON);
    }
    else
    {
        DAC_LOOP_DAC_HP_flag = ucontrol->value.integer.value[0];
        ADC_LOOP_DAC_Func(AUDIO_ANALOG_DAC_LOOP_DAC_HP_OFF);
    }
    return 0;
}

static bool Voice_Call_DAC_DAC_HS_flag = false;
static int Voice_Call_DAC_DAC_HS_Get(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    ucontrol->value.integer.value[0] = Voice_Call_DAC_DAC_HS_flag;
    return 0;
}

static int Voice_Call_DAC_DAC_HS_Set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    printk("%s()\n", __func__);
    if (ucontrol->value.integer.value[0])
    {
        Voice_Call_DAC_DAC_HS_flag = ucontrol->value.integer.value[0];
        
        Ana_Set_Reg(AUDNVREGGLB_CFG0,  0x0000 , 0xffff); 
        Ana_Set_Reg(TOP_CLKSQ,  0x0001, 0xffff); 
        Ana_Set_Reg(AUDADC_CFG0,  0x0400, 0xffff); 
        
        OpenMicbias1(true);
        SetMicbias1lowpower(false);
        OpenMicbias0(true);
        SetMicbias0lowpower(false);

        Ana_Set_Reg(AUDMICBIAS_CFG1,  0x285, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG1,  0x0007, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG2,  0x2277, 0xffff); 
        Ana_Set_Reg(AUDPREAMPGAIN_CFG0, 0x033, 0xffff); 
        SetMicPGAGain();
        Ana_Set_Reg(AUDPREAMP_CFG0, 0x051, 0xffff); 
        Ana_Set_Reg(AUDPREAMP_CFG1, 0x055, 0xffff); 

        Ana_Set_Reg(TOP_CLKSQ_SET, 0x0003, 0xffff); 
        Ana_Set_Reg(TOP_CKPDN_CON0_CLR, 0x3000, 0xffff); 
        Ana_Set_Reg(TOP_CKSEL_CON_CLR, 0x0001, 0x0001); 

        Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   

        Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0006, 0xffff); 
        Ana_Set_Reg(AFUNC_AUD_CON0, 0xc3a1, 0xffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x0003, 0xffff); 
        Ana_Set_Reg(AFUNC_AUD_CON2, 0x000b, 0xffff); 
        Ana_Set_Reg(AFE_DL_SDM_CON1, 0x001e, 0xffff); 
        Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff); 
        Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0, 0x3330, 0xffff); 
        Ana_Set_Reg(AFE_DL_SRC2_CON0_H, 0x3330, 0xffff); 
        Ana_Set_Reg(AFE_DL_SRC2_CON0_L, 0x1801, 0xffff); 
        Ana_Set_Reg(AFE_UL_SRC0_CON0_H, 0x000a, 0xffff); 
        Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0001, 0xffff); 

        
        Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff); 
        Ana_Set_Reg(AFE_SGEN_CFG0, 0x0080, 0xffff); 
        Ana_Set_Reg(AFE_SGEN_CFG1, 0x0101, 0xffff); 

        
        Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0000, 0xffff); 

        Ana_Set_Reg(AFE_CLASSH_CFG7, 0x8909, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG8, 0x0d0d, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG9, 0x0d10, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG10, 0x1010, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG11, 0x1010, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG12, 0x0000, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG13, 0x0000, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG14, 0x009c, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG26, 0x8d0d, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG27, 0x0d0d, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG28, 0x0d10, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG29, 0x1010, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG30, 0x1010, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG1, 0x0024, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG2, 0x2f90, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG3, 0x1104, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG4, 0x2412, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG5, 0x0201, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG6, 0x2800, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG21, 0xa108, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG22, 0x06db, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG23, 0x0bd6, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG24, 0x1492, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG25, 0x1740, 0xffff); 
        Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd518, 0xffff); 
        msleep(1);
        Ana_Set_Reg(AFE_CLASSH_CFG0,   0xd419, 0xffff); 
        msleep(1);
        Ana_Set_Reg(AFE_CLASSH_CFG1,   0x0021, 0xffff); 
        msleep(1);

        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0028, 0xffff); 
        Ana_Set_Reg(AUDLDO_NVREG_CFG0,   0x0068, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG5, 0x001f, 0xffff); 
        Ana_Set_Reg(ZCD_CON0,   0x0700, 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,   0xE008, 0xffff); 
        Ana_Set_Reg(IBIASDIST_CFG0,   0x0092, 0xffff); 

        Ana_Set_Reg(ZCD_CON2,  0x0F9F , 0xffff); 
        Ana_Set_Reg(ZCD_CON3,  0x001f , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0900 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG2,  0x0022 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0,  0xE009 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG1,  0x0940 , 0xffff); 
        Ana_Set_Reg(AUDCLKGEN_CFG0, 0x5501 , 0xffff); 
        Ana_Set_Reg(AUDDAC_CFG0, 0x000F , 0xffff); 
        SetDcCompenSation();

        Ana_Set_Reg(AUDBUF_CFG0, 0xE010 , 0xffff); 
        Ana_Set_Reg(AUDBUF_CFG0, 0xE011 , 0xffff);
        Ana_Set_Reg(AUDBUF_CFG1, 0x0900 , 0xffff);
        Ana_Set_Reg(AUDBUF_CFG2, 0x0020 , 0xffff);
        
        Ana_Set_Reg(ZCD_CON2,  0x0489 , 0xffff); 
        Ana_Set_Reg(ZCD_CON3,  0x0489 , 0xffff); 

        

    }
    else
    {
        Voice_Call_DAC_DAC_HS_flag = ucontrol->value.integer.value[0];
    }
    return 0;
}




static const char *Pmic_Test_function[] = {"Off", "On"};

static const struct soc_enum Pmic_Test_Enum[] =
{
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(Pmic_Test_function), Pmic_Test_function),
};

static const struct snd_kcontrol_new mt6331_pmic_Test_controls[] =
{
    SOC_ENUM_EXT("SineTable_DAC_HP", Pmic_Test_Enum[0], SineTable_DAC_HP_Get, SineTable_DAC_HP_Set),
    SOC_ENUM_EXT("DAC_LOOP_DAC_HS", Pmic_Test_Enum[1], ADC_LOOP_DAC_HS_Get, ADC_LOOP_DAC_HS_Set),
    SOC_ENUM_EXT("DAC_LOOP_DAC_HP", Pmic_Test_Enum[2], ADC_LOOP_DAC_HP_Get, ADC_LOOP_DAC_HP_Set),
    SOC_ENUM_EXT("Voice_Call_DAC_DAC_HS", Pmic_Test_Enum[3], Voice_Call_DAC_DAC_HS_Get, Voice_Call_DAC_DAC_HS_Set),
    SOC_ENUM_EXT("SineTable_UL2", Pmic_Test_Enum[4], SineTable_UL2_Get, SineTable_UL2_Set),
};

static const struct snd_kcontrol_new mt6331_UL_Codec_controls[] =
{
    SOC_ENUM_EXT("Audio_ADC_1_Switch", Audio_UL_Enum[0], Audio_ADC1_Get, Audio_ADC1_Set),
    SOC_ENUM_EXT("Audio_ADC_2_Switch", Audio_UL_Enum[1], Audio_ADC2_Get, Audio_ADC2_Set),
    SOC_ENUM_EXT("Audio_ADC_3_Switch", Audio_UL_Enum[2], Audio_ADC3_Get, Audio_ADC3_Set),
    SOC_ENUM_EXT("Audio_ADC_4_Switch", Audio_UL_Enum[3], Audio_ADC4_Get, Audio_ADC4_Set),
    SOC_ENUM_EXT("Audio_Preamp1_Switch", Audio_UL_Enum[4], Audio_PreAmp1_Get, Audio_PreAmp1_Set),
    SOC_ENUM_EXT("Audio_ADC_1_Sel", Audio_UL_Enum[5], Audio_ADC1_Sel_Get, Audio_ADC1_Sel_Set),
    SOC_ENUM_EXT("Audio_ADC_2_Sel", Audio_UL_Enum[6], Audio_ADC2_Sel_Get, Audio_ADC2_Sel_Set),
    SOC_ENUM_EXT("Audio_ADC_3_Sel", Audio_UL_Enum[7], Audio_ADC3_Sel_Get, Audio_ADC3_Sel_Set),
    SOC_ENUM_EXT("Audio_ADC_4_Sel", Audio_UL_Enum[8], Audio_ADC4_Sel_Get, Audio_ADC4_Sel_Set),
    SOC_ENUM_EXT("Audio_PGA1_Setting", Audio_UL_Enum[9], Audio_PGA1_Get, Audio_PGA1_Set),
    SOC_ENUM_EXT("Audio_PGA2_Setting", Audio_UL_Enum[10], Audio_PGA2_Get, Audio_PGA2_Set),
    SOC_ENUM_EXT("Audio_PGA3_Setting", Audio_UL_Enum[11], Audio_PGA3_Get, Audio_PGA3_Set),
    SOC_ENUM_EXT("Audio_PGA4_Setting", Audio_UL_Enum[12], Audio_PGA4_Get, Audio_PGA4_Set),
    SOC_ENUM_EXT("Audio_MicSource1_Setting", Audio_UL_Enum[13], Audio_MicSource1_Get, Audio_MicSource1_Set),
    SOC_ENUM_EXT("Audio_MicSource2_Setting", Audio_UL_Enum[14], Audio_MicSource2_Get, Audio_MicSource2_Set),
    SOC_ENUM_EXT("Audio_MicSource3_Setting", Audio_UL_Enum[15], Audio_MicSource3_Get, Audio_MicSource3_Set),
    SOC_ENUM_EXT("Audio_MicSource4_Setting", Audio_UL_Enum[16], Audio_MicSource4_Get, Audio_MicSource4_Set),
    SOC_ENUM_EXT("Audio_MIC1_Mode_Select", Audio_UL_Enum[17], Audio_Mic1_Mode_Select_Get, Audio_Mic1_Mode_Select_Set),
    SOC_ENUM_EXT("Audio_MIC2_Mode_Select", Audio_UL_Enum[18], Audio_Mic2_Mode_Select_Get, Audio_Mic2_Mode_Select_Set),
    SOC_ENUM_EXT("Audio_MIC3_Mode_Select", Audio_UL_Enum[19], Audio_Mic3_Mode_Select_Get, Audio_Mic3_Mode_Select_Set),
    SOC_ENUM_EXT("Audio_MIC4_Mode_Select", Audio_UL_Enum[20], Audio_Mic4_Mode_Select_Get, Audio_Mic4_Mode_Select_Set),
    SOC_ENUM_EXT("Audio_Mic_Power_Mode", Audio_UL_Enum[21], Audio_Adc_Power_Mode_Get, Audio_Adc_Power_Mode_Set),
    SOC_ENUM_EXT("Audio_Vow_ADC_Func_Switch", Audio_UL_Enum[22], Audio_Vow_ADC_Func_Switch_Get, Audio_Vow_ADC_Func_Switch_Set),
    SOC_ENUM_EXT("Audio_Vow_Digital_Func_Switch", Audio_UL_Enum[23], Audio_Vow_Digital_Func_Switch_Get, Audio_Vow_Digital_Func_Switch_Set),
    SOC_SINGLE_EXT("Audio VOWCFG0 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg0_Get, Audio_Vow_Cfg0_Set),
    SOC_SINGLE_EXT("Audio VOWCFG1 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg1_Get, Audio_Vow_Cfg1_Set),
    SOC_SINGLE_EXT("Audio VOWCFG2 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg2_Get, Audio_Vow_Cfg2_Set),
    SOC_SINGLE_EXT("Audio VOWCFG3 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg3_Get, Audio_Vow_Cfg3_Set),
    SOC_SINGLE_EXT("Audio VOWCFG4 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg4_Get, Audio_Vow_Cfg4_Set),
    SOC_SINGLE_EXT("Audio VOWCFG5 Data", SND_SOC_NOPM, 0, 0x80000, 0, Audio_Vow_Cfg5_Get, Audio_Vow_Cfg5_Set),
};




static const struct snd_soc_dapm_widget mt6331_dapm_widgets[] =
{
    
    SND_SOC_DAPM_OUTPUT("EARPIECE"),
    SND_SOC_DAPM_OUTPUT("HEADSET"),
    SND_SOC_DAPM_OUTPUT("SPEAKER"),

};

static const struct snd_soc_dapm_route mtk_audio_map[] =
{
    {"VOICE_Mux_E", "Voice Mux", "SPEAKER PGA"},
};

static void mt6331_codec_init_reg(struct snd_soc_codec *codec)
{
    printk("%s  \n", __func__);
    Ana_Set_Reg(TOP_CLKSQ, 0x0 , 0xffff);
    Ana_Set_Reg(AUDNVREGGLB_CFG0, 0x0001, 0xffff);
    Ana_Set_Reg(TOP_CKPDN_CON0_SET, 0x3000, 0x3000);
    Ana_Set_Reg(AUDBUF_CFG0,  0xE000 , 0xe000); 
    
#ifndef CONFIG_MTK_FPGA
    mt6331_upmu_set_rg_audmicbias1lowpen(true); 
    mt6331_upmu_set_rg_audmicbias0lowpen(true); 
#endif
    Ana_Set_Reg(AUDMICBIAS_CFG1, 0x2020, 0xffff);   
    Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0xA000, 0xffff);   
}

void InitCodecDefault(void)
{
    printk("%s\n", __func__);
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP1] = 3;
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP2] = 3;
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP3] = 3;
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_MICAMP4] = 3;
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = 8;
    mCodec_data->mAudio_Ana_Volume[AUDIO_ANALOG_VOLUME_HPOUTR] = 8;

    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC1] = AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC2] = AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC3] = AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
    mCodec_data->mAudio_Ana_Mux[AUDIO_ANALOG_MUX_IN_MIC4] = AUDIO_ANALOG_AUDIOANALOG_INPUT_PREAMP;
}

static int mt6331_codec_probe(struct snd_soc_codec *codec)
{
    struct snd_soc_dapm_context *dapm = &codec->dapm;
    printk("%s()\n", __func__);
    if (mInitCodec == true)
    {
        return 0;
    }

    snd_soc_dapm_new_controls(dapm, mt6331_dapm_widgets,
                              ARRAY_SIZE(mt6331_dapm_widgets));
    snd_soc_dapm_add_routes(dapm, mtk_audio_map,
                            ARRAY_SIZE(mtk_audio_map));

    
    snd_soc_add_codec_controls(codec, mt6331_snd_controls,
                               ARRAY_SIZE(mt6331_snd_controls));
    snd_soc_add_codec_controls(codec, mt6331_UL_Codec_controls,
                               ARRAY_SIZE(mt6331_UL_Codec_controls));
    snd_soc_add_codec_controls(codec, mt6331_Voice_Switch,
                               ARRAY_SIZE(mt6331_Voice_Switch));
    snd_soc_add_codec_controls(codec, mt6331_pmic_Test_controls,
                               ARRAY_SIZE(mt6331_pmic_Test_controls));

#ifdef CONFIG_MTK_SPEAKER
    snd_soc_add_codec_controls(codec, mt6331_snd_Speaker_controls,
                               ARRAY_SIZE(mt6331_snd_Speaker_controls));
#endif
    snd_soc_add_codec_controls(codec,mt_ext_dev_controls,
                               ARRAY_SIZE(mt_ext_dev_controls));

    snd_soc_add_codec_controls(codec, Audio_snd_auxadc_controls,
                               ARRAY_SIZE(Audio_snd_auxadc_controls));

    
    mCodec_data = kzalloc(sizeof(mt6331_Codec_Data_Priv), GFP_KERNEL);
    if (!mCodec_data)
    {
        printk("Failed to allocate private data\n");
        return -ENOMEM;
    }
    snd_soc_codec_set_drvdata(codec, mCodec_data);

    memset((void *)mCodec_data , 0 , sizeof(mt6331_Codec_Data_Priv));
    mt6331_codec_init_reg(codec);
    InitCodecDefault();
    mInitCodec = true;

    return 0;
}

static int mt6331_codec_remove(struct snd_soc_codec *codec)
{
    printk("%s()\n", __func__);
    return 0;
}

static unsigned int mt6331_read(struct snd_soc_codec *codec,
                                unsigned int reg)
{
    printk("mt6331_read reg = 0x%x", reg);
    Ana_Get_Reg(reg);
    return 0;
}

static int mt6331_write(struct snd_soc_codec *codec, unsigned int reg,
                        unsigned int value)
{
    printk("mt6331_write reg = 0x%x  value= 0x%x\n", reg, value);
    Ana_Set_Reg(reg , value , 0xffffffff);
    return 0;
}

static int mt6331_Readable_registers(struct snd_soc_codec *codec,
                                     unsigned int reg)
{
    return 1;
}

static int mt6331_volatile_registers(struct snd_soc_codec *codec,
                                     unsigned int reg)
{
    return 1;
}

static struct snd_soc_codec_driver soc_mtk_codec =
{
    .probe    = mt6331_codec_probe,
    .remove = mt6331_codec_remove,

    .read = mt6331_read,
    .write = mt6331_write,

    .readable_register = mt6331_Readable_registers,
    .volatile_register = mt6331_volatile_registers,

    
    
    

    .dapm_widgets = mt6331_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(mt6331_dapm_widgets),
    .dapm_routes = mtk_audio_map,
    .num_dapm_routes = ARRAY_SIZE(mtk_audio_map),

};

static int mtk_mt6331_codec_dev_probe(struct platform_device *pdev)
{
    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
    if (pdev->dev.dma_mask == NULL)
    {
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_CODEC_NAME);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_codec(&pdev->dev,
                                  &soc_mtk_codec, mtk_6331_dai_codecs, ARRAY_SIZE(mtk_6331_dai_codecs));
}

static int mtk_mt6331_codec_dev_remove(struct platform_device *pdev)
{
    printk("%s:\n", __func__);

    snd_soc_unregister_codec(&pdev->dev);
    return 0;

}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_codec_63xx_of_ids[] =
{
    { .compatible = "mediatek,mt_soc_codec_63xx", },
    {}
};
#endif

static struct platform_driver mtk_codec_6331_driver =
{
    .driver = {
        .name = MT_SOC_CODEC_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = mt_soc_codec_63xx_of_ids,
#endif
    },
    .probe  = mtk_mt6331_codec_dev_probe,
    .remove = mtk_mt6331_codec_dev_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtk_codec6331_dev;
#endif

static int __init mtk_mt6331_codec_init(void)
{
    int ret = 0;
    printk("%s:\n", __func__);
#ifndef CONFIG_OF
    soc_mtk_codec6331_dev = platform_device_alloc(MT_SOC_CODEC_NAME, -1);
    if (!soc_mtk_codec6331_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtk_codec6331_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtk_codec6331_dev);
        return ret;
    }
#endif
#ifdef CONFIG_HTC_RECEIVER
    htc_receiver_init();
#endif
    ret = platform_driver_register(&mtk_codec_6331_driver);

    return ret;
}
module_init(mtk_mt6331_codec_init);

static void __exit mtk_mt6331_codec_exit(void)
{
    printk("%s:\n", __func__);

    platform_driver_unregister(&mtk_codec_6331_driver);
#ifdef CONFIG_HTC_RECEIVER
    htc_receiver_release();
#endif
}

module_exit(mtk_mt6331_codec_exit);

MODULE_DESCRIPTION("MTK  codec driver");
MODULE_LICENSE("GPL v2");

