#include <linux/mu3phy/mtk-phy.h>

#ifdef CONFIG_PROJECT_PHY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_clkmgr.h>
#include <asm/io.h>
#include <linux/mu3phy/mtk-phy-asic.h>
#include <linux/mu3d/hal/mu3d_hal_osal.h>
#ifdef CONFIG_MTK_UART_USB_SWITCH
#include <linux/mu3d/hal/mu3d_hal_usb_drv.h>
extern void __iomem *ap_uart0_base;
#endif

#ifdef FOR_BRING_UP
#define enable_clock(x, y)
#define disable_clock(x, y)
#define hwPowerOn(x, y, z)
#define hwPowerDown(x, y)
#define set_ada_ssusb_xtal_ck(x)
#endif

#ifdef NEVER
void enable_ssusb_xtal_clock(bool enable)
{
    if (enable) {
		writel(readl((void __iomem *)AP_PLL_CON0)|(0x00000001),
		    (void __iomem *)AP_PLL_CON0);
		
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON0)|(0x00000002),
		    (void __iomem *)AP_PLL_CON0);

		writel(readl((void __iomem *)AP_PLL_CON2)|(0x00000001),
		    (void __iomem *)AP_PLL_CON2);

		
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON2)|(0x00000002),
		    (void __iomem *)AP_PLL_CON2);

		writel(readl((void __iomem *)AP_PLL_CON2)|(0x00000004),
		    (void __iomem *)AP_PLL_CON2);
    } else {
		
		
    }
}

void enable_ssusb26m_ck(bool enable)
{
    if (enable) {
		writel(readl((void __iomem *)AP_PLL_CON0)|(0x00000001),
		    (void __iomem *)AP_PLL_CON0);
		
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON0)|(0x00000002),
		(void __iomem *)AP_PLL_CON0);

    } else {
		
		
    }
}
#endif 

void usb20_pll_settings(bool host, bool forceOn)
{
	if (host) {
		if (forceOn) {
			os_printk(K_INFO, "%s-%d - Set USBPLL_FORCE_ON.\n", __func__, __LINE__);
			U3PhyWriteField32(U3D_USBPHYACR0, E60802_RG_USB20_USBPLL_FORCE_ON_OFST, E60802_RG_USB20_USBPLL_FORCE_ON, 0x1);
		} else {
			os_printk(K_INFO, "%s-%d - Clear USBPLL_FORCE_ON.\n", __func__, __LINE__);
			U3PhyWriteField32(U3D_USBPHYACR0, E60802_RG_USB20_USBPLL_FORCE_ON_OFST, E60802_RG_USB20_USBPLL_FORCE_ON, 0x0);
			return;
		}
	}

	os_printk(K_INFO, "%s-%d - Set PLL_FORCE_MODE and SIFSLV PLL_FORCE_ON.\n", __func__, __LINE__);
	U3PhyWriteField32(U3D_USBPHYACR2_0, E60802_RG_SIFSLV_USB20_PLL_FORCE_MODE_OFST, E60802_RG_SIFSLV_USB20_PLL_FORCE_MODE, 0x1);
	U3PhyWriteField32(U3D_U2PHYDCR0, E60802_RG_SIFSLV_USB20_PLL_FORCE_ON_OFST, E60802_RG_SIFSLV_USB20_PLL_FORCE_ON, 0x0);
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool in_uart_mode = false;
void uart_usb_switch_dump_register(void)
{
	
	set_ada_ssusb_xtal_ck(1);
    
	enable_clock(MT_CG_PERI_USB0, "USB30");
	udelay(50);

#ifdef CONFIG_MTK_FPGA
	printk("[MUSB]addr: 0x6B, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0+0x3));
	printk("[MUSB]addr: 0x6E, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM1+0x2));
	printk("[MUSB]addr: 0x22, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYACR4+0x2));
	printk("[MUSB]addr: 0x68, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0));
	printk("[MUSB]addr: 0x6A, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0+0x2));
	printk("[MUSB]addr: 0x1A, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYACR6+0x2));
#else
    #if 0
	os_printk(K_INFO, "[MUSB]addr: 0x6B, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0+0x3));
	os_printk(K_INFO, "[MUSB]addr: 0x6E, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM1+0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x22, value: %x\n", U3PhyReadReg8(U3D_U2PHYACR4+0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x68, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0));
	os_printk(K_INFO, "[MUSB]addr: 0x6A, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0+0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x1A, value: %x\n", U3PhyReadReg8(U3D_USBPHYACR6+0x2));
    #else
	os_printk(K_INFO, "[MUSB]addr: 0x18, value: 0x%x\n", U3PhyReadReg32(U3D_USBPHYACR6));
	os_printk(K_INFO, "[MUSB]addr: 0x20, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYACR4));
	os_printk(K_INFO, "[MUSB]addr: 0x68, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYDTM0));
	os_printk(K_INFO, "[MUSB]addr: 0x6C, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYDTM1));
    #endif
#endif

	
	set_ada_ssusb_xtal_ck(0);
    
    disable_clock(MT_CG_PERI_USB0, "USB30");

	os_printk(K_INFO, "[MUSB]addr: 0x110020B0 (UART0), value: %x\n\n", DRV_Reg8(ap_uart0_base + 0xB0));
}

bool usb_phy_check_in_uart_mode(void)
{
	PHY_INT32 usb_port_mode;

	
	set_ada_ssusb_xtal_ck(1);
    
	enable_clock(MT_CG_PERI_USB0, "USB30");

    udelay(50);
    usb_port_mode = U3PhyReadReg32(U3D_U2PHYDTM0) >> E60802_RG_UART_MODE_OFST ;

	
	set_ada_ssusb_xtal_ck(0);
    
    disable_clock(MT_CG_PERI_USB0, "USB30");
    os_printk(K_INFO, "%s+ usb_port_mode = %d\n", __func__, usb_port_mode);

	if (usb_port_mode == 0x1)
		return true;
	else
		return false;
}

void usb_phy_switch_to_uart(void)
{
	if (usb_phy_check_in_uart_mode()) {
	    os_printk(K_INFO, "%s+ UART_MODE\n", __func__);
		return;
	} else {
	    os_printk(K_INFO, "%s+ USB_MODE\n", __func__);
	}

	
	
	hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");

	
	hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");

	
	set_ada_ssusb_xtal_ck(1);

	
	enable_clock(MT_CG_PERI_USB0, "USB30");
	udelay(50);

	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);

	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);

	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 1);

	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_UART_MODE_OFST, E60802_RG_UART_MODE, 1);

	
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_TX_OE_OFST, E60802_FORCE_UART_TX_OE, 1);

	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_BIAS_EN_OFST, E60802_FORCE_UART_BIAS_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_TX_OE_OFST, E60802_RG_UART_TX_OE, 1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_BIAS_EN_OFST, E60802_RG_UART_BIAS_EN, 1);

	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DM_100K_EN_OFST, E60802_RG_USB20_DM_100K_EN, 1);

	
	set_ada_ssusb_xtal_ck(0);
    
    disable_clock(MT_CG_PERI_USB0, "USB30");

	
	DRV_WriteReg32(ap_uart0_base + 0xB0, 0x1);

	in_uart_mode = true;
}


void usb_phy_switch_to_usb(void)
{
    in_uart_mode = false;
	
	DRV_WriteReg32(ap_uart0_base + 0xB0, 0x0);	

	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);

    phy_init_soc(u3phy);

	
	
	set_ada_ssusb_xtal_ck(0);
    
    disable_clock(MT_CG_PERI_USB0, "USB30");
}
#endif

#define RG_SSUSB_VUSB10_ON (1<<5)
#define RG_SSUSB_VUSB10_ON_OFST (5)

PHY_INT32 phy_init_soc(struct u3phy_info *info)
{
	os_printk(K_INFO, "%s+\n", __func__);

	

	
	
	hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");

	
	hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");

	
	
	set_ada_ssusb_xtal_ck(1);

	
	
	

	
	enable_clock(MT_CG_PERI_USB0, "USB30");

	
	

	
	udelay(50);

	
	U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 1);

	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_ISO_EN_OFST, E60802_RG_USB20_ISO_EN, 0);

	#ifdef CONFIG_MTK_UART_USB_SWITCH
    if (!in_uart_mode) {
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_UART_MODE_OFST, E60802_RG_UART_MODE, 0);
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DM_100K_EN_OFST, E60802_RG_USB20_DM_100K_EN, 0);
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
    }
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DP_100K_MODE_OFST, E60802_RG_USB20_DP_100K_MODE, 1);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_DP_100K_EN_OFST, E60802_USB20_DP_100K_EN, 0);
#if defined(MTK_HDMI_SUPPORT) || defined(MTK_USB_MODE1)
    os_printk(K_INFO, "%s- USB PHY Driving Tuning Mode 1 Settings(7,7).\n", __func__);
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 0);
	U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST, E60802_RG_USB20_VRT_VREF_SEL, 7);
	U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST, E60802_RG_USB20_TERM_VREF_SEL, 7);
#else
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 1);
#endif
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST, E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
        
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x6);
    #else
    
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DP_100K_MODE_OFST, E60802_RG_USB20_DP_100K_MODE, 1);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_DP_100K_EN_OFST, E60802_USB20_DP_100K_EN, 0);
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DM_100K_EN_OFST, E60802_RG_USB20_DM_100K_EN, 0);
#if !defined(CONFIG_MTK_HDMI_SUPPORT) && !defined(MTK_USB_MODE1) 
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 1);
#endif
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST, E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
        
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x6);
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
    #endif

	
	udelay(800);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_VBUSVALID_OFST, E60802_FORCE_VBUSVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_AVALID_OFST, E60802_FORCE_AVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_SESSEND_OFST, E60802_FORCE_SESSEND, 1);

	
	usb20_pll_settings(false, false);

	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_DISCTH_OFST, E60802_RG_USB20_DISCTH, 0xf);
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);

	PHY_INT32 val1 = 0, val2 = 0;
	val1 = U3PhyReadField32(U3D_USBPHYACR6, E60802_RG_USB20_DISCTH_OFST, E60802_RG_USB20_DISCTH);
	val2 = U3PhyReadField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH);
	os_printk(K_INFO, "%s: Threshold=%x, rx-sensitivity=%xn", __func__, val1, val2);

	os_printk(K_INFO, "%s-\n", __func__);

	return PHY_TRUE;
}

PHY_INT32 u2_slew_rate_calibration(struct u3phy_info *info)
{
	PHY_INT32 i=0;
	PHY_INT32 fgRet = 0;
	PHY_INT32 u4FmOut = 0;
	PHY_INT32 u4Tmp = 0;

	os_printk(K_INFO, "%s\n", __func__);

	
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCAL_EN_OFST, E60802_RG_USB20_HSTX_SRCAL_EN, 1);

	
	udelay(1);

	
	
	U3PhyWriteField32((u3_sif_base + 0x110)
		, E60802_RG_FRCK_EN_OFST, E60802_RG_FRCK_EN, 0x1);

	
	
	U3PhyWriteField32((u3_sif_base + 0x100)
		, E60802_RG_CYCLECNT_OFST, E60802_RG_CYCLECNT, 0x400);

	
	
	U3PhyWriteField32((u3_sif_base + 0x100)
		, E60802_RG_FREQDET_EN_OFST, E60802_RG_FREQDET_EN, 0x1);

	os_printk(K_INFO, "Freq_Valid=(0x%08X)\n", U3PhyReadReg32(u3_sif_base + 0x110));

	mdelay(1);

	
	for(i=0; i<10; i++){
		
		
		u4FmOut = U3PhyReadReg32(u3_sif_base + 0x10C);
		os_printk(K_INFO, "FM_OUT value: u4FmOut = %d(0x%08X)\n", u4FmOut, u4FmOut);

		
		if (u4FmOut != 0) {
			fgRet = 0;
			os_printk(K_INFO, "FM detection done! loop = %d\n", i);
			break;
		}
		fgRet = 1;
		mdelay(1);
	}
	
	
	U3PhyWriteField32((u3_sif_base + 0x100)
		, E60802_RG_FREQDET_EN_OFST, E60802_RG_FREQDET_EN, 0);

	
	
	U3PhyWriteField32((u3_sif_base + 0x110)
		, E60802_RG_FRCK_EN_OFST, E60802_RG_FRCK_EN, 0);

	
	if(u4FmOut == 0){
		U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCTRL_OFST, E60802_RG_USB20_HSTX_SRCTRL, 0x4);
		fgRet = 1;
	}
	else{
		
		
		u4Tmp = (1024 * REF_CK * U2_SR_COEF_E60802) / (u4FmOut * 1000);
		os_printk(K_INFO, "SR calibration value u1SrCalVal = %d\n", (PHY_UINT8)u4Tmp);
		U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCTRL_OFST, E60802_RG_USB20_HSTX_SRCTRL, u4Tmp);
	}

	
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCAL_EN_OFST, E60802_RG_USB20_HSTX_SRCAL_EN, 0);

	return fgRet;
}

void usb_phy_savecurrent(unsigned int clk_on)
{
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);

    #ifdef CONFIG_MTK_UART_USB_SWITCH
    if (!in_uart_mode) {
	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 1);
    }
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
    #else
	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 1);
    #endif
	
	
	udelay(2000);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DPPULLDOWN_OFST, E60802_RG_DPPULLDOWN, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DMPULLDOWN_OFST, E60802_RG_DMPULLDOWN, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_XCVRSEL_OFST, E60802_RG_XCVRSEL, 0x1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_TERMSEL_OFST, E60802_RG_TERMSEL, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DATAIN_OFST, E60802_RG_DATAIN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DP_PULLDOWN_OFST, E60802_FORCE_DP_PULLDOWN, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DM_PULLDOWN_OFST, E60802_FORCE_DM_PULLDOWN, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_XCVRSEL_OFST, E60802_FORCE_XCVRSEL, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_TERMSEL_OFST, E60802_FORCE_TERMSEL, 1);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DATAIN_OFST, E60802_FORCE_DATAIN, 1);

	
	
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);

	
	
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST, E60802_RG_USB20_OTG_VBUSCMP_EN, 0);

	
	
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 0);

	
	udelay(800);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 0);

	
	udelay(1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_VBUSVALID_OFST, E60802_RG_VBUSVALID, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_AVALID_OFST, E60802_RG_AVALID, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_SESSEND_OFST, E60802_RG_SESSEND, 1);

	
	usb20_pll_settings(false, false);

	if (clk_on) {
		
		
		U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 0);

		
		udelay(10);

		
		disable_clock(MT_CG_PERI_USB0, "USB30");

		
		set_ada_ssusb_xtal_ck(0);

		
		
		hwPowerDown(MT6331_POWER_LDO_VUSB10, "VDD10_USB_P0");
	}

	os_printk(K_INFO, "%s-\n", __func__);
}

void usb_phy_recover(unsigned int clk_on)
{
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);

	if (!clk_on) {
		
		
		hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");

		
		hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");

		
		
		set_ada_ssusb_xtal_ck(1);

		
		
		

		
		enable_clock(MT_CG_PERI_USB0, "USB30");

		
		

		
		udelay(50);

		
		U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 1);
	}
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_DISCTH_OFST, E60802_RG_USB20_DISCTH, 0xf);
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);

	PHY_INT32 val1 = 0, val2 = 0;
	val1 = U3PhyReadField32(U3D_USBPHYACR6, E60802_RG_USB20_DISCTH_OFST, E60802_RG_USB20_DISCTH);
	val2 = U3PhyReadField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH);
	os_printk(K_INFO, "%s: Threshold=%x, rx-sensitivity=%xn", __func__, val1, val2);

	
	
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_ISO_EN_OFST, E60802_RG_USB20_ISO_EN, 0);

#ifdef CONFIG_MTK_UART_USB_SWITCH
    if (!in_uart_mode) {
	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
    }

	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
#else
	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL, 0);
	
	
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);

	
	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
#endif

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DPPULLDOWN_OFST, E60802_RG_DPPULLDOWN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DMPULLDOWN_OFST, E60802_RG_DMPULLDOWN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_XCVRSEL_OFST, E60802_RG_XCVRSEL, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_TERMSEL_OFST, E60802_RG_TERMSEL, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DATAIN_OFST, E60802_RG_DATAIN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DP_PULLDOWN_OFST, E60802_FORCE_DP_PULLDOWN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DM_PULLDOWN_OFST, E60802_FORCE_DM_PULLDOWN, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_XCVRSEL_OFST, E60802_FORCE_XCVRSEL, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_TERMSEL_OFST, E60802_FORCE_TERMSEL, 0);

	
	
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DATAIN_OFST, E60802_FORCE_DATAIN, 0);

	
	
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);

	
	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST, E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
        
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);

#if defined(CONFIG_MTK_HDMI_SUPPORT) || defined(MTK_USB_MODE1)
    os_printk(K_INFO, "%s- USB PHY Driving Tuning Mode 1 Settings.\n", __func__);
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 0);
#else
	
	
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST, E60802_RG_USB20_HS_100U_U3_EN, 1);
#endif

	U3PhyWriteField32(U3D_USB30_PHYA_REG6, E60802_RG_SSUSB_TX_EIDLE_CM_OFST, E60802_RG_SSUSB_TX_EIDLE_CM, 0xE);
	U3PhyWriteField32(U3D_PHYD_CDR1, E60802_RG_SSUSB_CDR_BIR_LTD0_OFST, E60802_RG_SSUSB_CDR_BIR_LTD0, 0xC);
	U3PhyWriteField32(U3D_PHYD_CDR1, E60802_RG_SSUSB_CDR_BIR_LTD1_OFST, E60802_RG_SSUSB_CDR_BIR_LTD1, 0x3);

	U3PhyWriteField32(U3D_U3PHYA_DA_REG0, E60802_RG_SSUSB_XTAL_EXT_EN_U3_OFST, E60802_RG_SSUSB_XTAL_EXT_EN_U3, 2);
	U3PhyWriteField32(U3D_SPLLC_XTALCTL3, E60802_RG_SSUSB_XTAL_RX_PWD_OFST, E60802_RG_SSUSB_XTAL_RX_PWD, 1);

	
	udelay(800);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_VBUSVALID_OFST, E60802_RG_VBUSVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_AVALID_OFST, E60802_RG_AVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_SESSEND_OFST, E60802_RG_SESSEND, 0);

#ifdef CONFIG_MTK_UART_USB_SWITCH
    if (in_uart_mode) {
	    os_printk(K_INFO, "%s- Switch to UART mode when UART cable in inserted before boot.\n", __func__);
        usb_phy_switch_to_uart();
    }
#endif
	
	usb20_pll_settings(false, false);

	os_printk(K_INFO, "%s-\n", __func__);
}

void usb_fake_powerdown(unsigned int clk_on)
{
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);

	if (clk_on) {
		
		
		disable_clock(MT_CG_PERI_USB0, "USB30");

		
		
		hwPowerDown(MT6331_POWER_LDO_VUSB10, "VDD10_USB_P0");
	}

	os_printk(K_INFO, "%s-\n", __func__);
}

#ifdef CONFIG_USBIF_COMPLIANCE
static bool charger_det_en = true ;

void Charger_Detect_En(bool enable)
{
	charger_det_en = enable ;
}
#endif


void Charger_Detect_Init(void)
{
	os_printk(K_INFO, "%s+\n", __func__);

#ifdef CONFIG_USBIF_COMPLIANCE
	if (charger_det_en == true){
#endif
	
	enable_clock(MT_CG_PERI_USB0, "USB30");

	
	udelay(50);

	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 1);

	udelay(1);

	
	disable_clock(MT_CG_PERI_USB0, "USB30");

#ifdef CONFIG_USBIF_COMPLIANCE
	}else{
		os_printk(K_INFO, "%s do not init detection as charger_det_en is false\n", __func__);
	}
#endif

	os_printk(K_INFO, "%s-\n", __func__);
}

void Charger_Detect_Release(void)
{
	os_printk(K_INFO, "%s+\n", __func__);

#ifdef CONFIG_USBIF_COMPLIANCE
	if (charger_det_en == true){
#endif
	
	enable_clock(MT_CG_PERI_USB0, "USB30");

	
	udelay(50);

	
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);

	udelay(1);

	
	disable_clock(MT_CG_PERI_USB0, "USB30");

#ifdef CONFIG_USBIF_COMPLIANCE
	}else{
		os_printk(K_INFO, "%s do not release detection as charger_det_en is false\n", __func__);
	}
#endif

	os_printk(K_INFO, "%s-\n", __func__);
}


#endif
