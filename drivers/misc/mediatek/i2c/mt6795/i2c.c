#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <asm/scatterlist.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_i2c.h>
#endif
#include <asm/io.h>
#include <mach/mt_gpio.h>
#include <mach/mt_clkmgr.h>
#include <mt_i2c.h>
#include <mach/sync_write.h>
#include "mach/memory.h"
#include "cust_gpio_usage.h"
#include <mach/mt_pm_ldo.h>

#include <mach/i2c.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/upmu_common.h>
#include <linux/aee.h>

#define TAG     "MT_I2C"

extern U32 pmic_read_interface (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT);
#define DMA_LOG_LEN 7
static struct i2c_dma_info g_dma_data[DMA_LOG_LEN];

#define DEBUG_9210_READ


void i2c_writel(mt_i2c * i2c, U8 offset, U16 value)
{
  
  mt_reg_sync_writel(value, (i2c->base) + (offset));
}

U32 i2c_readl(mt_i2c * i2c, U8 offset)
{
  return __raw_readl((void*)((i2c->base) + (offset)));
}
static void mt_i2c_clock_enable(mt_i2c *i2c);
static void mt_i2c_clock_disable(mt_i2c *i2c);

#ifdef CONFIG_OF
static void __iomem *ap_dma_base;
#endif
#ifdef I2C_DRIVER_IN_KERNEL
static struct mt_i2c_msg g_msg[2];
static mt_i2c  *g_i2c[2];
#define I2C_DRV_NAME        "mt-i2c"
#endif
#ifdef I2C_DEBUG_FS
  #define PORT_COUNT 7
  #define MESSAGE_COUNT 16
  #define I2C_T_DMA 1
  #define I2C_T_TRANSFERFLOW 2
  #define I2C_T_SPEED 3
  
  U8 i2c_port[ PORT_COUNT ][ MESSAGE_COUNT ];
#if 0
  #define I2CINFO( type, format, arg...) do { \
    if ( type < MESSAGE_COUNT && type >= 0 ) { \
      if ( i2c_port[i2c->id][0] != 0 && ( i2c_port[i2c->id][type] != 0 || i2c_port[i2c->id][MESSAGE_COUNT - 1] != 0) ) { \
        I2CLOG( format, ## arg); \
      } \
    } \
  } while (0)
#endif
#define I2CINFO( type, format, arg...) do { \
        I2CLOG( format, ## arg); \
  } while (0)

  #ifdef I2C_DRIVER_IN_KERNEL
    static ssize_t show_config(struct device *dev, struct device_attribute *attr, char *buff)
    {
      S32 i = 0;
      S32 j = 0;
      char *buf = buff;
      for ( i =0; i < PORT_COUNT; i++){
        for ( j=0;j < MESSAGE_COUNT; j++) i2c_port[i][j] += '0';
        strncpy(buf, (char *)i2c_port[i], MESSAGE_COUNT);
        buf += MESSAGE_COUNT;
        *buf = '\n';
        buf++;
        for ( j=0;j < MESSAGE_COUNT; j++) i2c_port[i][j] -= '0';
      }
      return (buf - buff);
    }

    static ssize_t set_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
    {
      S32 port,type,status;

      if ( sscanf(buf, "%d %d %d", &port, &type, &status) ) {
        if ( port >= PORT_COUNT || port < 0 || type >= MESSAGE_COUNT || type < 0 ) {
          
          I2CERR("i2c debug system: Parameter overflowed!\n");
        } else {
          if ( status != 0 )
            i2c_port[port][type] = 1;
          else
            i2c_port[port][type] = 0;

          I2CLOG("port:%d type:%d status:%s\ni2c debug system: Parameter accepted!\n", port, type, status?"on":"off");
        }
      } else {
        
        I2CERR("i2c debug system: Parameter invalid!\n");
      }
      return count;
    }

    static DEVICE_ATTR(debug, S_IRUGO|S_IWUGO, show_config, set_config);
  #endif
#else
  #define I2CINFO(type, format, arg...)
#endif
static S32 i2c_set_speed(mt_i2c *i2c)
{
  S32 ret = 0;
  S32 mode = 0;
  U32 khz = 0;
  
  
  U16 step_cnt_div = 0;
  U16 sample_cnt_div = 0;
  U32 tmp, sclk, hclk = i2c->clk;
  U16 max_step_cnt_div = 0;
  U32 diff, min_diff = i2c->clk;
  U16 sample_div = MAX_SAMPLE_CNT_DIV;
  U16 step_div = 0;
  
  
  
  
  if((i2c->speed == i2c->last_speed) && (i2c->mode == i2c->last_mode))
  {
  	
	
  	ret = 0;
	goto end;
  }
  
  mode=i2c->mode;
  khz = i2c->speed;

  max_step_cnt_div = (mode == HS_MODE) ? MAX_HS_STEP_CNT_DIV : MAX_STEP_CNT_DIV;
  step_div = max_step_cnt_div;

  if((mode == FS_MODE && khz > MAX_FS_MODE_SPEED) || (mode == HS_MODE && khz > MAX_HS_MODE_SPEED)){
    I2CERR(" the speed is too fast for this mode.\n");
    I2C_BUG_ON((mode == FS_MODE && khz > MAX_FS_MODE_SPEED) || (mode == HS_MODE && khz > MAX_HS_MODE_SPEED));
    ret = -EINVAL_I2C;
    goto end;
  }
  
  for (sample_cnt_div = 1; sample_cnt_div <= MAX_SAMPLE_CNT_DIV; sample_cnt_div++) {
      for (step_cnt_div = 1; step_cnt_div <= max_step_cnt_div; step_cnt_div++) {
        sclk = (hclk >> 1) / (sample_cnt_div * step_cnt_div);
        if (sclk > khz)
          continue;
        diff = khz - sclk;
        if (diff < min_diff) {
          min_diff = diff;
          sample_div = sample_cnt_div;
          step_div   = step_cnt_div;
        }
      }
    }
    sample_cnt_div = sample_div;
    step_cnt_div   = step_div;
  sclk = hclk / (2 * sample_cnt_div * step_cnt_div);
  
  if (sclk > khz) {
    I2CERR("%s mode: unsupported speed (%ldkhz)\n",(mode == HS_MODE) ? "HS" : "ST/FT", (long int)khz);
    I2CLOG("i2c->clk=%d,sclk=%d khz=%d,i2c->speed=%d hclk=%d sample_cnt_div=%d,step_cnt_div=%d.\n",i2c->clk,sclk,khz,i2c->speed,hclk,sample_cnt_div,step_cnt_div);
    I2C_BUG_ON(sclk > khz);
    ret = -ENOTSUPP_I2C;
    goto end;
  }

  step_cnt_div--;
  sample_cnt_div--;

  

  if (mode == HS_MODE) {

    
    tmp = i2c_readl(i2c, OFFSET_TIMING) & ~((0x7 << 8) | (0x3f << 0));
    tmp = (0 & 0x7) << 8 | (16 & 0x3f) << 0 | tmp;
    i2c->timing_reg=tmp;
	if(0 == i2c->timing_reg)
	{
		I2CLOG("hs base address 0x%p,tmp =0x%x\n",i2c->base,tmp);
		
	}
    
    

    
    tmp = i2c_readl(i2c, OFFSET_HS) & ~((0x7 << 12) | (0x7 << 8));
    tmp = (sample_cnt_div & 0x7) << 12 | (step_cnt_div & 0x7) << 8 | tmp;
    
    tmp |= 0x0001;
    i2c->high_speed_reg=tmp;
    
    
  }
  else {
    
    tmp  = i2c_readl(i2c, OFFSET_TIMING) & ~((0x7 << 8) | (0x3f << 0));
    tmp  = (sample_cnt_div & 0x7) << 8 | (step_cnt_div & 0x3f) << 0 | tmp;
    i2c->timing_reg=tmp;
	if(0 == i2c->timing_reg)
	{
		I2CLOG("n-hs base address 0x%p, tmp=0x%x, sample_cnt_div=0x%x, step_cnt_div=0x%x\n",i2c->base,tmp,sample_cnt_div,step_cnt_div);
		
	}
    
    
    
    
    tmp = i2c_readl(i2c, OFFSET_HS) & ~(0x0001);
    
    i2c->high_speed_reg=tmp;
    
    
    
  }
  
  I2CINFO( I2C_T_SPEED, " set sclk to %ldkhz(orig:%ldkhz), sample=%d,step=%d\n", sclk, khz, sample_cnt_div, step_cnt_div);
end:
  
  i2c->last_speed = i2c->speed;
  i2c->last_mode = i2c->mode;
  return ret;
}

void _i2c_dump_info(mt_i2c *i2c)
{
  
  int val=0; 
  I2CLOG("I2C(%d) dump info++++++++++++++++++++++\n",i2c->id);
  I2CLOG("I2C structure:\n"
    I2CTAG"Clk=%d,Id=%d,Speed mode=%x,St_rs=%x,Dma_en=%x,Op=%x,Poll_en=%x,Irq_stat=%x\n"
    I2CTAG"Trans_len=%x,Trans_num=%x,Trans_auxlen=%x,Data_size=%x,speed=%d\n"
    I2CTAG"Trans_stop=%u,Trans_comp=%u,Trans_error=%u\n",
    i2c->clk,i2c->id,i2c->mode,i2c->st_rs,i2c->dma_en,i2c->op,i2c->poll_en,i2c->irq_stat,\
    i2c->trans_data.trans_len,i2c->trans_data.trans_num,i2c->trans_data.trans_auxlen,i2c->trans_data.data_size,i2c->speed,\
    atomic_read(&i2c->trans_stop),atomic_read(&i2c->trans_comp),atomic_read(&i2c->trans_err));

  I2CLOG("base address 0x%p\n",i2c->base);
  I2CLOG("I2C register:\n"
    I2CTAG"SLAVE_ADDR=%x,INTR_MASK=%x,INTR_STAT=%x,CONTROL=%x,TRANSFER_LEN=%x\n"
    I2CTAG"TRANSAC_LEN=%x,DELAY_LEN=%x,TIMING=%x,START=%x,FIFO_STAT=%x\n"
    I2CTAG"IO_CONFIG=%x,HS=%x,DCM_EN=%x,DEBUGSTAT=%x,EXT_CONF=%x,TRANSFER_LEN_AUX=%x\n",
    (i2c_readl(i2c, OFFSET_SLAVE_ADDR)),
    (i2c_readl(i2c, OFFSET_INTR_MASK)),
    (i2c_readl(i2c, OFFSET_INTR_STAT)),
    (i2c_readl(i2c, OFFSET_CONTROL)),
    (i2c_readl(i2c, OFFSET_TRANSFER_LEN)),
    (i2c_readl(i2c, OFFSET_TRANSAC_LEN)),
    (i2c_readl(i2c, OFFSET_DELAY_LEN)),
    (i2c_readl(i2c, OFFSET_TIMING)),
    (i2c_readl(i2c, OFFSET_START)),
    (i2c_readl(i2c, OFFSET_FIFO_STAT)),
    (i2c_readl(i2c, OFFSET_IO_CONFIG)),
    (i2c_readl(i2c, OFFSET_HS)),
    (i2c_readl(i2c, OFFSET_DCM_EN)),
    (i2c_readl(i2c, OFFSET_DEBUGSTAT)),
    (i2c_readl(i2c, OFFSET_EXT_CONF)),
    (i2c_readl(i2c, OFFSET_TRANSFER_LEN_AUX)));

  I2CLOG("before enable DMA register(0x%ld):\n"
  	I2CTAG"INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,\n"
  	I2CTAG"STOP=%x,FLUSH=%x,CON=%x,TX_MEM_ADDR=%x, RX_MEM_ADDR=%x\n"
  	I2CTAG"TX_LEN=%x,RX_LEN=%x,INT_BUF_SIZE=%x,DEBUG_STATUS=%x\n"
  	I2CTAG"TX_4G_SURPPORT=%x,RX_4G_SURPPORT=%x\n",
	  g_dma_data[i2c->id].base,
      g_dma_data[i2c->id].int_flag,
      g_dma_data[i2c->id].int_en,
      g_dma_data[i2c->id].en,
      g_dma_data[i2c->id].rst,

  	  g_dma_data[i2c->id].stop,
  	  g_dma_data[i2c->id].flush,
      g_dma_data[i2c->id].con,
      g_dma_data[i2c->id].tx_mem_addr,
      g_dma_data[i2c->id].tx_mem_addr,
      
      g_dma_data[i2c->id].tx_len,
      g_dma_data[i2c->id].rx_len,
      g_dma_data[i2c->id].int_buf_size,
      g_dma_data[i2c->id].debug_sta,

	  g_dma_data[i2c->id].tx_4g_support,
	  g_dma_data[i2c->id].rx_4g_support);
  
    I2CLOG("DMA register(0x%p):\n"
  	I2CTAG"INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,\n"
  	I2CTAG"STOP=%x,FLUSH=%x,CON=%x,TX_MEM_ADDR=%x, RX_MEM_ADDR=%x\n"
  	I2CTAG"TX_LEN=%x,RX_LEN=%x,INT_BUF_SIZE=%x,DEBUG_STATUS=%x\n"
  	I2CTAG"TX_4G_SURPPORT=%x,RX_4G_SURPPORT=%x\n",
	  i2c->pdmabase,
      (__raw_readl((void*)i2c->pdmabase+OFFSET_INT_FLAG)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_INT_EN)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_EN)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_RST)),

  	  (__raw_readl((void*)i2c->pdmabase+OFFSET_STOP)),
  	  (__raw_readl((void*)i2c->pdmabase+OFFSET_FLUSH)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_CON)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_TX_MEM_ADDR)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_RX_MEM_ADDR)),
      
      (__raw_readl((void*)i2c->pdmabase+OFFSET_TX_LEN)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_RX_LEN)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_INT_BUF_SIZE)),
      (__raw_readl((void*)i2c->pdmabase+OFFSET_DEBUG_STA)),

	  (__raw_readl((void*)i2c->pdmabase+OFFSET_TX_4G_SUPPORT)),
	  (__raw_readl((void*)i2c->pdmabase+OFFSET_RX_4G_SUPPORT)));
	  
   
   
#ifndef CONFIG_MT_I2C_FPGA_ENABLE
  pmic_read_interface(MT6331_SYSLDO_CON3,&val,MT6331_PMIC_RG_VCAM_IO_EN_MASK,MT6331_PMIC_RG_VCAM_IO_EN_SHIFT);
  I2CLOG("i2c(%d) power=%d \n",i2c->id,val);
#endif
  
#ifndef CONFIG_MT_I2C_FPGA_ENABLE
#ifdef GPIO_I2C0_SDA_PIN
  I2CLOG("mt_get_gpio_in I2C0_SDA=%d,I2C0_SCA=%d,I2C1_SDA=%d,I2C1_SCA=%d",\
		  mt_get_gpio_in(GPIO_I2C0_SDA_PIN),mt_get_gpio_in(GPIO_I2C0_SCA_PIN),mt_get_gpio_in(GPIO_I2C1_SDA_PIN),mt_get_gpio_in(GPIO_I2C1_SCA_PIN));
#endif
#endif
  I2CLOG("I2C(%d) dump info------------------------------\n",i2c->id);
  return;
}
static int dma_busy_wait_ready(mt_i2c *i2c)
{
	long dma_tmo_poll=10;
	int res=0;
	if(NULL == i2c){

		I2CERR("dma_busy_wait_ready NULL pointer err \n" );
		return -1;
	}
	while(1 == (__raw_readl((void*)i2c->pdmabase+OFFSET_EN)))
	{
		I2CERR("wait dma transfer complet,dma_tmo_poll=%ld\n",dma_tmo_poll );
		udelay(5);
		dma_tmo_poll --;
		if(dma_tmo_poll == 0) {
			res=-1;
			break;
		}
	}
	return res;

}

static int dma_reset(mt_i2c *i2c)
{
	if(NULL == i2c){

		I2CERR("dma_reset NULL pointer err \n" );
		return -1;
	}
	
	mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_RST);
	
	udelay(50);
	
	mt_reg_sync_writel(1<<1, i2c->pdmabase + OFFSET_RST);
	udelay(5);
	mt_reg_sync_writel(0x0, i2c->pdmabase + OFFSET_RST);
	udelay(5);
	I2CERR("id=%d,addr: %x, dma HW reset\n",i2c->id, i2c->addr);
	return 0;
}

static S32 _i2c_deal_result(mt_i2c *i2c)
{
#ifdef I2C_DRIVER_IN_KERNEL
  long tmo = i2c->adap.timeout;
#else
  long tmo = 1;
#endif
  #ifdef DEBUG_9210_READ
  int dump_flag =0;
  #endif
  U16 data_size = 0;
  U8 *ptr = i2c->msg_buf;
  S32 ret = i2c->msg_len;
  long tmo_poll = 0xffff;
  int dma_err=0;
  
  

  if(i2c->poll_en)
  { 
    for (;;)
    { 
      i2c->irq_stat = i2c_readl(i2c, OFFSET_INTR_STAT);
      
      if(i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP))
      {
        atomic_set(&i2c->trans_stop, 1);
        spin_lock(&i2c->lock);
        
        i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
        spin_unlock(&i2c->lock);
        break;
      }
        tmo_poll --;
        if(tmo_poll == 0) {
        tmo = 0;
        break;
      }
    }
  } else { 
    tmo = wait_event_timeout(i2c->wait,atomic_read(&i2c->trans_stop), tmo);
  }

  
  #ifdef I2C_DRIVER_IN_KERNEL
  if (i2c->irq_stat & I2C_TRANSAC_COMP) {
    atomic_set(&i2c->trans_err, 0);
    atomic_set(&i2c->trans_comp, 1);
  }
  atomic_set(&i2c->trans_err, i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR));
  #endif

  
  if (!(tmo == 0 || atomic_read(&i2c->trans_err)) )
  {
    
    if((!i2c->dma_en) && (i2c->op == I2C_MASTER_RD || i2c->op == I2C_MASTER_WRRD) )
    { 
      data_size = (i2c_readl(i2c, OFFSET_FIFO_STAT) >> 4) & 0x000F;
	  BUG_ON(data_size > i2c->msg_len);
    
      #ifdef DEBUG_9210_READ
      if(i2c->addr == 0x68 && (*(i2c->msg_buf) == 0xd8 ))
      {
      	dump_flag =1;
      }
      #endif
      while (data_size--)
      {
        *ptr = i2c_readl(i2c, OFFSET_DATA_PORT);
        
        #ifdef DEBUG_9210_READ
        if(1 == dump_flag)
        {
        	if(0 == *ptr)
        	{
        		printk(" 9210 read 0xd8 fifo: slave(0x%x) fifo=0x%x, pid(%d)\n",i2c->addr,*ptr,current->pid);
				_i2c_dump_info(i2c);
        	}
        }
		#endif
        ptr++;
      }
    }
	if(i2c->dma_en)
	{
		dma_err=dma_busy_wait_ready(i2c);
		if(dma_err){
			I2CERR("i2c ok wait dma ready err \n" );
			_i2c_dump_info(i2c);
			dma_reset(i2c);
		}
	   
	}
    #ifdef I2C_DEBUG_FS
      _i2c_dump_info(i2c);
    #endif
  }else
  {
    
    if ( tmo == 0 ){
      I2CERR("id=%d,addr: %x, transfer timeout\n",i2c->id, i2c->addr);
      ret = -ETIMEDOUT_I2C;
    } else
    {
      I2CERR("id=%d,addr: %x, transfer error\n",i2c->id,i2c->addr);
      ret = -EREMOTEIO_I2C;
    }
    if (i2c->irq_stat & I2C_HS_NACKERR)
      I2CERR("I2C_HS_NACKERR\n");
    if (i2c->irq_stat & I2C_ACKERR)
      I2CERR("I2C_ACKERR\n");
    if (i2c->filter_msg==FALSE) 
    {
      _i2c_dump_info(i2c);
   }

    spin_lock(&i2c->lock);
    
    i2c_writel(i2c, OFFSET_SOFTRESET, 0x0001);
    
    i2c_writel( i2c, OFFSET_SLAVE_ADDR, 0x0000 );
    
    i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR|I2C_ACKERR|I2C_TRANSAC_COMP));
    
    i2c_writel(i2c, OFFSET_FIFO_ADDR_CLR, 0x0001);
    
	i2c->last_mode = -1;
	i2c->last_speed = 0;
    spin_unlock(&i2c->lock);
	if(i2c->dma_en){
		
		dma_reset(i2c);
	}
	
  }
  return ret;
}

static void record_i2c_dma_info(mt_i2c *i2c)
{
	if(i2c->id >= DMA_LOG_LEN)
	{
		I2CERR(" no space to record i2c dma log \n");
		return;
	}
	g_dma_data[i2c->id].base = (unsigned long)i2c->pdmabase;
	g_dma_data[i2c->id].int_flag = (__raw_readl((void *)i2c->pdmabase + OFFSET_INT_FLAG));
	g_dma_data[i2c->id].int_en = (__raw_readl((void *)i2c->pdmabase + OFFSET_INT_EN));
	g_dma_data[i2c->id].en = (__raw_readl((void *)i2c->pdmabase + OFFSET_EN));
	g_dma_data[i2c->id].rst = (__raw_readl((void *)i2c->pdmabase + OFFSET_RST));
	g_dma_data[i2c->id].stop = (__raw_readl((void *)i2c->pdmabase + OFFSET_STOP));
	g_dma_data[i2c->id].flush = (__raw_readl((void *)i2c->pdmabase + OFFSET_FLUSH));
	g_dma_data[i2c->id].con = (__raw_readl((void *)i2c->pdmabase + OFFSET_CON));
	g_dma_data[i2c->id].tx_mem_addr = (__raw_readl((void *)i2c->pdmabase + OFFSET_TX_MEM_ADDR));
	g_dma_data[i2c->id].rx_mem_addr = (__raw_readl((void *)i2c->pdmabase + OFFSET_RX_MEM_ADDR));
	g_dma_data[i2c->id].tx_len = (__raw_readl((void *)i2c->pdmabase + OFFSET_TX_LEN));
	g_dma_data[i2c->id].rx_len = (__raw_readl((void *)i2c->pdmabase + OFFSET_RX_LEN));
	g_dma_data[i2c->id].int_buf_size =
	    (__raw_readl((void *)i2c->pdmabase + OFFSET_INT_BUF_SIZE));
	g_dma_data[i2c->id].debug_sta = (__raw_readl((void *)i2c->pdmabase + OFFSET_DEBUG_STA));
	g_dma_data[i2c->id].tx_4g_support =
	    (__raw_readl((void *)i2c->pdmabase + OFFSET_TX_4G_SUPPORT));
	g_dma_data[i2c->id].rx_4g_support =
	    (__raw_readl((void *)i2c->pdmabase + OFFSET_RX_4G_SUPPORT));
}
static void _i2c_write_reg(mt_i2c *i2c)
{
  U8 *ptr = i2c->msg_buf;
  U32 data_size=i2c->trans_data.data_size;
  U32 addr_reg=0;
  

  i2c_writel(i2c, OFFSET_CONTROL, i2c->control_reg);

  
  if(i2c->speed <= 100){
    i2c_writel(i2c,OFFSET_EXT_CONF, 0x8001);
  } else {
	i2c_writel(i2c,OFFSET_EXT_CONF, 0x1800);
  }
  
  i2c_writel(i2c, OFFSET_TIMING, i2c->timing_reg);
  i2c_writel(i2c, OFFSET_HS, i2c->high_speed_reg);

  if(0 == i2c->delay_len)
    i2c->delay_len = 2;
  if(~i2c->control_reg & I2C_CONTROL_RS){  
    i2c_writel(i2c, OFFSET_DELAY_LEN, i2c->delay_len);
  }

  
  if (i2c->pushpull) {
      i2c_writel(i2c, OFFSET_IO_CONFIG, 0x0000);
  } else {
      i2c_writel(i2c, OFFSET_IO_CONFIG, 0x0003);
  }

  

  addr_reg = i2c->read_flag ? ((i2c->addr << 1) | 0x1) : ((i2c->addr << 1) & ~0x1);
  i2c_writel(i2c, OFFSET_SLAVE_ADDR, addr_reg);
  
  i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
  
  i2c_writel(i2c, OFFSET_FIFO_ADDR_CLR, 0x0001);
  
  if(i2c->poll_en)
    i2c_writel(i2c, OFFSET_INTR_MASK, i2c_readl(i2c, OFFSET_INTR_MASK) & ~(I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP)); 
  else
    i2c_writel(i2c, OFFSET_INTR_MASK, i2c_readl(i2c, OFFSET_INTR_MASK) | (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP)); 
  
  i2c_writel(i2c, OFFSET_TRANSFER_LEN, i2c->trans_data.trans_len & 0xFFFF);
  i2c_writel(i2c, OFFSET_TRANSFER_LEN_AUX, i2c->trans_data.trans_auxlen & 0xFFFF);
  
  i2c_writel(i2c, OFFSET_TRANSAC_LEN, i2c->trans_data.trans_num & 0xFF);

  

  if(i2c->dma_en){
    if(1 == (__raw_readl((void*)i2c->pdmabase+OFFSET_EN))){
    	   
    	   I2CERR(" dma not ready ,need reset\n");
    	   dma_reset(i2c);	
    }
    if (I2C_MASTER_RD == i2c->op) {
      mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
      mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_CON);
      mt_reg_sync_writel((u32)((long)i2c->msg_buf), i2c->pdmabase + OFFSET_RX_MEM_ADDR);
      mt_reg_sync_writel(i2c->trans_data.data_size, i2c->pdmabase + OFFSET_RX_LEN);
    } else if (I2C_MASTER_WR == i2c->op) {
      mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
      mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_CON);
      mt_reg_sync_writel((u32)((long)i2c->msg_buf), i2c->pdmabase + OFFSET_TX_MEM_ADDR);
      mt_reg_sync_writel(i2c->trans_data.data_size, i2c->pdmabase + OFFSET_TX_LEN);
    } else {
      mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
      mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_CON);
      mt_reg_sync_writel((u32)((long)i2c->msg_buf), i2c->pdmabase + OFFSET_TX_MEM_ADDR);
      mt_reg_sync_writel((u32)((long)i2c->msg_buf), i2c->pdmabase + OFFSET_RX_MEM_ADDR);
      mt_reg_sync_writel(i2c->trans_data.trans_len, i2c->pdmabase + OFFSET_TX_LEN);
      mt_reg_sync_writel(i2c->trans_data.trans_auxlen, i2c->pdmabase + OFFSET_RX_LEN);
    }
	    
	    record_i2c_dma_info(i2c);
    I2C_MB();
    mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_EN);

    I2CINFO( I2C_T_DMA, "addr %.2x dma %.2X byte\n", i2c->addr, i2c->trans_data.data_size);
    I2CINFO( I2C_T_DMA, "DMA Register:INT_FLAG:0x%x,CON:0x%x,TX_MEM_ADDR:0x%x, \
                 RX_MEM_ADDR:0x%x,TX_LEN:0x%x,RX_LEN:0x%x,EN:0x%x\n",\
                  readl(i2c->pdmabase + OFFSET_INT_FLAG),\
                  readl(i2c->pdmabase + OFFSET_CON),\
                  readl(i2c->pdmabase + OFFSET_TX_MEM_ADDR),\
                  readl(i2c->pdmabase + OFFSET_RX_MEM_ADDR),\
                  readl(i2c->pdmabase + OFFSET_TX_LEN),\
                  readl(i2c->pdmabase + OFFSET_RX_LEN),\
                  readl(i2c->pdmabase + OFFSET_EN));

  }
  else
  {
    
    if (I2C_MASTER_RD == i2c->op)
    {
      
    }else
    { 
      while (data_size--)
      {
        i2c_writel(i2c, OFFSET_DATA_PORT, *ptr);
        
        ptr++;
      }
    }
  }
  
  i2c->trans_data.data_size = data_size;
  
  if(0x0 == (i2c_readl(i2c, OFFSET_TIMING)))
  {
  	 
  	i2c_writel(i2c, OFFSET_TIMING, 0x1410);
	aee_kernel_warning(TAG, "@%s():%d, \n", __func__, __LINE__);
  	
  }
	i2c_writel(i2c,OFFSET_DCM_EN, 0x0);
}
static S32 _i2c_get_transfer_len(mt_i2c *i2c)
{
  S32 ret = I2C_OK;
  u16 trans_num = 0;
  u16 data_size = 0;
  u16 trans_len = 0;
  u16 trans_auxlen = 0;
  
  
  if(FALSE == i2c->dma_en)
  { 
    if(I2C_MASTER_WRRD != i2c->op)
    {
      trans_len = (i2c->msg_len) & 0xFFFF;
      trans_num = (i2c->msg_len >> 16) & 0xFF;
      if(0 == trans_num)
        trans_num = 1;
      trans_auxlen = 0;
      data_size = trans_len*trans_num;

      if(!trans_len || !trans_num || trans_len*trans_num > 8)
      {
        I2CERR(" non-WRRD transfer length is not right. trans_len=%x, tans_num=%x, trans_auxlen=%x\n", trans_len, trans_num, trans_auxlen);
        I2C_BUG_ON(!trans_len || !trans_num || trans_len*trans_num > 8);
        ret = -EINVAL_I2C;
      }
    } else
    {
      trans_len = (i2c->msg_len) & 0xFF;
      trans_auxlen = (i2c->msg_len >> 8) & 0xFF;
      trans_num = 2;
      data_size = trans_len;
      if(!trans_len || !trans_auxlen || trans_len > 8 || trans_auxlen > 8)
      {
        I2CERR(" WRRD transfer length is not right. trans_len=%x, tans_num=%x, trans_auxlen=%x\n", trans_len, trans_num, trans_auxlen);
        I2C_BUG_ON(!trans_len || !trans_auxlen || trans_len > 8 || trans_auxlen > 8);
        ret = -EINVAL_I2C;
      }
    }
  }
  else
  { 
    if(I2C_MASTER_WRRD != i2c->op)
    {
      trans_len = (i2c->msg_len) & 0xFFFF;
      trans_num = (i2c->msg_len >> 16) & 0xFF;
      if(0 == trans_num)
        trans_num = 1;
      trans_auxlen = 0;
      data_size = trans_len*trans_num;

      if(!trans_len || !trans_num || trans_len > MAX_DMA_TRANS_SIZE || trans_num > MAX_DMA_TRANS_NUM)
      {
        I2CERR(" DMA non-WRRD transfer length is not right. trans_len=%x, tans_num=%x, trans_auxlen=%x\n", trans_len, trans_num, trans_auxlen);
        I2C_BUG_ON(!trans_len || !trans_num || trans_len > MAX_DMA_TRANS_SIZE || trans_num > MAX_DMA_TRANS_NUM);
        ret = -EINVAL_I2C;
      }
      I2CINFO(I2C_T_DMA, "DMA non-WRRD mode!trans_len=%x, tans_num=%x, trans_auxlen=%x\n",trans_len, trans_num, trans_auxlen);
    } else
    {
      trans_len = (i2c->msg_len) & 0xFF;
      trans_auxlen = (i2c->msg_len >> 8) & 0xFF;
      trans_num = 2;
      data_size = trans_len;
      if(!trans_len || !trans_auxlen || trans_len > MAX_DMA_TRANS_SIZE || trans_auxlen > MAX_DMA_TRANS_NUM)
      {
        I2CERR(" DMA WRRD transfer length is not right. trans_len=%x, tans_num=%x, trans_auxlen=%x\n", trans_len, trans_num, trans_auxlen);
        I2C_BUG_ON(!trans_len || !trans_auxlen || trans_len > MAX_DMA_TRANS_SIZE || trans_auxlen > MAX_DMA_TRANS_NUM);
        ret = -EINVAL_I2C;
      }
      I2CINFO(I2C_T_DMA, "DMA WRRD mode!trans_len=%x, tans_num=%x, trans_auxlen=%x\n",trans_len, trans_num, trans_auxlen);
    }
  }

  i2c->trans_data.trans_num = trans_num;
  i2c->trans_data.trans_len = trans_len;
  i2c->trans_data.data_size = data_size;
  i2c->trans_data.trans_auxlen = trans_auxlen;

  return ret;
}
static S32 _i2c_transfer_interface(mt_i2c *i2c)
{
  S32 return_value=0;
  S32 ret=0;
  U8 *ptr = i2c->msg_buf;
  

  if(i2c->dma_en)
  {
    I2CINFO( I2C_T_DMA, "DMA Transfer mode!\n");
    if (i2c->pdmabase == 0) {
      I2CERR(" I2C%d doesnot support DMA mode!\n",i2c->id);
      I2C_BUG_ON(i2c->pdmabase == NULL);
      ret = -EINVAL_I2C;
      goto err;
    }
    
    if (virt_addr_valid(ptr)) {
      I2CERR(" DMA mode should use physical buffer address!\n");
      I2C_BUG_ON(virt_addr_valid(ptr));
      ret = -EINVAL_I2C;
      goto err;
    }
  }
#ifdef I2C_DRIVER_IN_KERNEL
  atomic_set(&i2c->trans_stop, 0);
  atomic_set(&i2c->trans_comp, 0);
  atomic_set(&i2c->trans_err, 0);
#endif
  i2c->irq_stat = 0;

  return_value=_i2c_get_transfer_len(i2c);
  if ( return_value < 0 ){
    I2CERR("_i2c_get_transfer_len fail,return_value=%d\n",return_value);
    ret =-EINVAL_I2C;
    goto err;
  }
  
  #ifdef CONFIG_MT_I2C_FPGA_ENABLE
    i2c->clk  = I2C_CLK_RATE;
  #else
    
  	i2c->clk = 17062; 
  #endif

  return_value=i2c_set_speed(i2c);
  if ( return_value < 0 ){
    I2CERR("i2c_set_speed fail,return_value=%d\n",return_value);
    ret =-EINVAL_I2C;
    goto err;
  }
  
  i2c->control_reg = I2C_CONTROL_ACKERR_DET_EN | I2C_CONTROL_CLK_EXT_EN;
  if(i2c->dma_en) {
    i2c->control_reg |= I2C_CONTROL_DMA_EN;
  }
  if(I2C_MASTER_WRRD == i2c->op)
    i2c->control_reg |= I2C_CONTROL_DIR_CHANGE;

  if(HS_MODE == i2c->mode || (i2c->trans_data.trans_num > 1 && I2C_TRANS_REPEATED_START == i2c->st_rs)) {
    i2c->control_reg |= I2C_CONTROL_RS;
  }

  spin_lock(&i2c->lock);
  _i2c_write_reg(i2c);

  
  I2C_MB();
#ifdef I2C_DRIVER_IN_KERNEL
  
  if (i2c->i2c_3dcamera_flag)
  {
    spin_unlock(&i2c->lock);
    if (g_i2c[0] == NULL)
    g_i2c[0] = i2c;
    else
    g_i2c[1] = i2c;

    goto end;
  }
#endif
  I2CINFO( I2C_T_TRANSFERFLOW, "Before start .....\n");
  #ifdef I2C_DEBUG_FS
#ifdef GPIO_I2C0_SDA_PIN
    I2CLOG("mt_get_gpio_in I2C0_SDA=%d,I2C0_SCA=%d,I2C1_SDA=%d,I2C1_SCA=%d",\
      mt_get_gpio_in(GPIO_I2C0_SDA_PIN),mt_get_gpio_in(GPIO_I2C0_SCA_PIN),mt_get_gpio_in(GPIO_I2C1_SDA_PIN),mt_get_gpio_in(GPIO_I2C1_SCA_PIN));
#endif
  #endif

  
  i2c_writel(i2c, OFFSET_START, 0x0001);
  spin_unlock(&i2c->lock);
  ret = _i2c_deal_result(i2c);
  I2CINFO(I2C_T_TRANSFERFLOW, "After i2c transfer .....\n");
err:
end:
    return ret;
}
static void _i2c_translate_msg(mt_i2c *i2c,struct mt_i2c_msg *msg)
{
  
  if(msg->addr & 0xFF00){
    msg->ext_flag |= msg->addr & 0xFF00;
  }
  I2CINFO( I2C_T_TRANSFERFLOW, "Before i2c transfer .....\n");

  i2c->msg_buf = msg->buf;
  i2c->msg_len = msg->len;
  if(msg->ext_flag & I2C_RS_FLAG)
    i2c->st_rs = I2C_TRANS_REPEATED_START;
  else
    i2c->st_rs = I2C_TRANS_STOP;

  if(msg->ext_flag & I2C_DMA_FLAG)
    i2c->dma_en = TRUE;
  else
    i2c->dma_en = FALSE;

  if (msg->ext_flag & I2C_WR_FLAG)
    i2c->op = I2C_MASTER_WRRD;
  else
  {
    if(msg->flags & I2C_M_RD)
      i2c->op = I2C_MASTER_RD;
    else
      i2c->op = I2C_MASTER_WR;
  }
  if(msg->ext_flag & I2C_POLLING_FLAG)
    i2c->poll_en = TRUE;
  else
    i2c->poll_en = FALSE;
  if(msg->ext_flag & I2C_A_FILTER_MSG)
    i2c->filter_msg = TRUE;
  else
    i2c->filter_msg = FALSE;
  i2c->delay_len = (msg->timing & 0xff0000) >> 16;

  
  if(0 == (msg->timing & 0xFFFF)){
    i2c->mode  = ST_MODE;
    i2c->speed = MAX_ST_MODE_SPEED;
  }
  else
  {
    if (msg->ext_flag & I2C_HS_FLAG)
      i2c->mode  = HS_MODE;
    else
      i2c->mode  = FS_MODE;

    i2c->speed= msg->timing & 0xFFFF;
  }

  
  if (msg->ext_flag & I2C_PUSHPULL_FLAG)
    i2c->pushpull=TRUE;
  else
    i2c->pushpull=FALSE;

  if (msg->ext_flag & I2C_3DCAMERA_FLAG)
    i2c->i2c_3dcamera_flag=TRUE;
  else
    i2c->i2c_3dcamera_flag=FALSE;

}

static S32 mt_i2c_start_xfer(mt_i2c *i2c, struct mt_i2c_msg *msg)
{
  S32 return_value = 0;
  S32 ret = msg->len;
  
  
  
  i2c->read_flag=(msg->flags & I2C_M_RD);
  i2c->addr=msg->addr;
  if(i2c->addr == 0){
    I2CERR(" addr is invalid.\n");
    I2C_BUG_ON(i2c->addr == NULL);
    ret = -EINVAL_I2C;
    goto err;
  }

  if(msg->buf == NULL){
    I2CERR(" data buffer is NULL.\n");
    I2C_BUG_ON(msg->buf == NULL);
    ret = -EINVAL_I2C;
    goto err;
  }
  if (g_i2c[0] == i2c || g_i2c[1] == i2c) {
    I2CERR("mt-i2c%d: Current I2C Adapter is busy.\n", i2c->id);
    ret = -EINVAL_I2C;
    goto err;
  }
  
  _i2c_translate_msg(i2c,msg);
  
  #ifdef I2C_DRIVER_IN_KERNEL
    if (i2c->i2c_3dcamera_flag)
    {
      if (g_msg[0].buf == NULL)
        memcpy((void *)&g_msg[0], msg, sizeof(struct mt_i2c_msg));
      else
        memcpy((void *)&g_msg[1], msg, sizeof(struct mt_i2c_msg));
    }
  #endif
  
  mt_i2c_clock_enable(i2c);
  return_value =_i2c_transfer_interface(i2c);
  if (!(msg->ext_flag & I2C_3DCAMERA_FLAG))
	mt_i2c_clock_disable(i2c);
  if ( return_value < 0 )
  {
    ret =-EINVAL_I2C;
    goto err;
  }
err:
  return ret;
}

static S32 mt_i2c_do_transfer(mt_i2c *i2c, struct mt_i2c_msg *msgs, S32 num)
{
  S32 ret = 0;
  S32 left_num = num;

  while (left_num--) {
      ret = mt_i2c_start_xfer(i2c, msgs++);
      if ( ret < 0 ){
        if ( ret != -EINVAL_I2C ) 
          return -EAGAIN;
        else
          return -EINVAL_I2C;
      }
  }
  
  return num;
}

extern char *mt_i2c_bus_to_virt(unsigned long address);

static S32 standard_i2c_start_xfer(mt_i2c *i2c, struct i2c_msg *msg)
{
  S32 return_value = 0;
  S32 ret = msg->len;
  u8 *temp_for_dma = 0;
  bool dma_need_copy_back = false;
  struct mt_i2c_msg msg_ext;
  
  
  
  
  
  msg_ext.addr  = msg->addr;
  msg_ext.flags = msg->flags;
  msg_ext.len   = msg->len;
  msg_ext.buf   = msg->buf;
  #ifdef COMPATIBLE_WITH_AOSP
  msg_ext.ext_flag = msg->ext_flag;
  if(msg->timing<=0)
  {
  	msg_ext.timing = i2c->defaul_speed;
	
  }
  else
  {
  	msg_ext.timing = msg->timing;
	
  }
  #else
  msg_ext.ext_flag = 0;
  msg_ext.timing = i2c->defaul_speed;
  #endif
  
  #ifdef COMPATIBLE_WITH_AOSP
  if(((msg->ext_flag&0xffff) & I2C_DMA_FLAG) && (msg->ext_flag>>16 == 0xdead))
  {
  	
  	
  }
  else
  {
  	  
  	if(msg->len > I2C_FIFO_SIZE)
  	{
  		
  		
  		dma_need_copy_back = true;
		msg_ext.ext_flag |= I2C_DMA_FLAG;
		temp_for_dma = msg_ext.buf;
		memcpy(i2c->dma_buf.vaddr, (msg_ext.buf), (msg_ext.len & 0x00FF));
		msg_ext.buf = (u8 *) i2c->dma_buf.paddr;
		
  	}
  }
  #else
  
  if(msg->len > I2C_FIFO_SIZE)
  {
  	
  	
  	dma_need_copy_back = true;
	msg_ext.ext_flag |= I2C_DMA_FLAG;
	temp_for_dma = msg_ext.buf;
	memcpy(i2c->dma_buf.vaddr, (msg_ext.buf), (msg_ext.len & 0x00FF));
	msg_ext.buf = (u8 *) mt_i2c_bus_to_virt(i2c->dma_buf.paddr);
	
  }
  #endif
  
  
  i2c->read_flag=(msg_ext.flags & I2C_M_RD);
  i2c->addr=msg_ext.addr;
  
  if(i2c->addr == 0){
    I2CERR(" addr is invalid.\n");
    I2C_BUG_ON(i2c->addr == NULL);
    ret = -EINVAL_I2C;
    goto err;
  }
  

  if(msg->buf == NULL){
    I2CERR(" data buffer is NULL.\n");
    I2C_BUG_ON(msg->buf == NULL);
    ret = -EINVAL_I2C;
    goto err;
  }
 
  if (g_i2c[0] == i2c || g_i2c[1] == i2c) {
    I2CERR("mt-i2c%d: Current I2C Adapter is busy.\n", i2c->id);
    ret = -EINVAL_I2C;
    goto err;
  }
  
  
  _i2c_translate_msg(i2c,&msg_ext);
  
  mt_i2c_clock_enable(i2c);
  return_value =_i2c_transfer_interface(i2c);
 
  if(true == dma_need_copy_back)
  {
  	
  	memcpy(temp_for_dma, i2c->dma_buf.vaddr, msg->len & 0xFF);
	msg->buf = temp_for_dma;
	
  }
  
  if (!(msg_ext.ext_flag & I2C_3DCAMERA_FLAG))
	mt_i2c_clock_disable(i2c);
  if ( return_value < 0 )
  {
    ret =-EINVAL_I2C;
    goto err;
  }
err:
  return ret;
}

static S32 standard_i2c_do_transfer(mt_i2c *i2c, struct i2c_msg *msgs, S32 num)
{
  S32 ret = 0;
  S32 left_num = num;

  while (left_num--) {
      ret = standard_i2c_start_xfer(i2c, msgs++);
      if ( ret < 0 ){
        if ( ret != -EINVAL_I2C ) 
          return -EAGAIN;
        else
          return -EINVAL_I2C;
      }
  }
  
  return num;
}
static S32 standard_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg msgs[], S32 num)
{
	S32 ret = 0;
  	S32 retry;
	
	
  	mt_i2c *i2c = i2c_get_adapdata(adap);
  	for (retry = 0; retry < adap->retries; retry++)
  	{
    	ret = standard_i2c_do_transfer(i2c, msgs, num);
    	if (ret != -EAGAIN) {
      	break;
    	}
    if ( retry < adap->retries - 1 )
      udelay(100);
  	}
	
  	if (ret != -EAGAIN)
    	return ret;
  	else
    	return -EREMOTEIO;
}

S32 mtk_i2c_transfer(struct i2c_adapter *adap, struct mt_i2c_msg msgs[], S32 num)
{
  S32 ret = 0;
  S32 retry;
  
  
  mt_i2c *i2c = i2c_get_adapdata(adap);
  for (retry = 0; retry < adap->retries; retry++)
  {
    ret = mt_i2c_do_transfer(i2c, msgs, num);
    if (ret != -EAGAIN) {
      break;
    }
    if ( retry < adap->retries - 1 )
      udelay(100);
  }

	if (ret != -EAGAIN)
		return ret;
	else
		return -EREMOTEIO;
}

int mtk_i2c_master_send(const struct i2c_client *client,
	const char *buf, int count, u32 ext_flag,u32 timing)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct mt_i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.timing = timing;
	msg.buf = (char *)buf;
	msg.ext_flag = ext_flag;
	ret = mtk_i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(mtk_i2c_master_send);

int mtk_i2c_master_recv(const struct i2c_client *client,
	char *buf, int count, u32 ext_flag,u32 timing)
{
	struct i2c_adapter *adap = client->adapter;
	struct mt_i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.timing = timing;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;
	msg.ext_flag = ext_flag;
	ret = mtk_i2c_transfer(adap, &msg, 1);

	return (ret == 1) ? count : ret;
}
EXPORT_SYMBOL(mtk_i2c_master_recv);
#ifdef I2C_DRIVER_IN_KERNEL
static S32 _i2c_deal_result_3dcamera(mt_i2c *i2c, struct mt_i2c_msg *msg)
{
  U16 addr = msg->addr;
  U16 read = (msg->flags & I2C_M_RD);
  i2c->msg_buf=msg->buf;
  i2c->msg_len=msg->len;
  i2c->addr = read ? ((addr << 1) | 0x1) : ((addr << 1) & ~0x1);
  return _i2c_deal_result(i2c);
}
#endif

static void mt_i2c_clock_enable(mt_i2c *i2c)
{
  #if (!defined(CONFIG_MT_I2C_FPGA_ENABLE))
	if (i2c->dma_en){
		I2CINFO( I2C_T_TRANSFERFLOW, "Before dma clock enable .....\n");
		enable_clock(MT_CG_PERI_AP_DMA, "i2c");
	}
    I2CINFO( I2C_T_TRANSFERFLOW, "Before i2c clock enable .....\n");
    enable_clock(i2c->pdn, "i2c");
    I2CINFO( I2C_T_TRANSFERFLOW, "clock enable done.....\n");
  #endif
  return;
}
static void mt_i2c_clock_disable(mt_i2c *i2c)
{
  #if (!defined(CONFIG_MT_I2C_FPGA_ENABLE))
	if (i2c->dma_en){
		I2CINFO( I2C_T_TRANSFERFLOW, "Before dma clock disable .....\n");
		disable_clock(MT_CG_PERI_AP_DMA, "i2c");
	}
    I2CINFO( I2C_T_TRANSFERFLOW, "Before i2c clock disable .....\n");
    disable_clock(i2c->pdn, "i2c");
    I2CINFO( I2C_T_TRANSFERFLOW, "clock disable done .....\n");
  #endif
  return;
}

static irqreturn_t mt_i2c_irq(S32 irqno, void *dev_id)
{
  mt_i2c *i2c = (mt_i2c *)dev_id;
  

  I2CINFO( I2C_T_TRANSFERFLOW, "i2c interrupt coming.....\n");
  
  
  i2c_writel(i2c,OFFSET_INTR_MASK,i2c_readl(i2c,OFFSET_INTR_MASK) & ~(I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
  
  i2c->irq_stat = i2c_readl(i2c, OFFSET_INTR_STAT);
  
  i2c_writel(i2c,OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
  

  
  atomic_set(&i2c->trans_stop, 1);
  wake_up(&i2c->wait);
  return IRQ_HANDLED;
}


S32 mt_wait4_i2c_complete(void)
{
  mt_i2c *i2c0 = g_i2c[0];
  mt_i2c *i2c1 = g_i2c[1];
  S32 result0, result1;
  S32 ret = 0;

  if ((i2c0 == NULL) ||(i2c1 == NULL)) {
    
    ret = -EINVAL_I2C;
    goto end;
  }

  result0 = _i2c_deal_result_3dcamera(i2c0, &g_msg[0]);
  result1 = _i2c_deal_result_3dcamera(i2c1, &g_msg[1]);

  if (result0 < 0 || result1 < 0) {
    ret = -EINVAL_I2C;
  }
  if(NULL != i2c0) mt_i2c_clock_disable(i2c0);
  if(NULL != i2c1) mt_i2c_clock_disable(i2c1);

end:
  g_i2c[0] = NULL;
  g_i2c[1] = NULL;

  g_msg[0].buf = NULL;
  g_msg[1].buf = NULL;

  return ret;
}

static U32 mt_i2c_functionality(struct i2c_adapter *adap)
{
  return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}
static struct i2c_algorithm mt_i2c_algorithm = {
#ifdef USE_I2C_MTK_EXT
	#ifdef COMPATIBLE_WITH_AOSP
	.master_xfer   = standard_i2c_transfer,
	#else
  	.master_xfer   = mtk_i2c_transfer,
  	#endif
#else
  .master_xfer   = standard_i2c_transfer,
#endif
  .smbus_xfer    = NULL,
  .functionality = mt_i2c_functionality,
};


static void mt_i2c_power_ctl(int id,int val)
{
#ifndef CONFIG_MT_I2C_FPGA_ENABLE
	printk("mt_i2c(%d)_power_ctl (%d) \n ",id,val);
	switch(id)
	{
		case 0:
			if(0 == val)
			{
				printk("mt_i2c(%d)_power_ctl disable \n ",id);
				
				
				hwPowerDown(MT6331_POWER_LDO_VCAM_IO,"I2C_DRV"); 
			}
			else if(1 == val)
			{
				printk("mt_i2c(%d)_power_ctl enable \n ",id);
				
				
				hwPowerOn(MT6331_POWER_LDO_VCAM_IO,VOL_DEFAULT,"I2C_DRV");
			}
			else
			{
				printk("mt_i2c_power_ctl err\n");
			}
			break;
		case 1:
			printk("hw always power on\n");
			break;
		case 2:
			printk("same power domain with i2c0\n");
			break;
		case 3:
			printk("hw always power on\n");
			break;
		case 4:
			printk("hw always power on\n");
			break;
		default:
			printk("id err\n");
			break;
			
	}
#endif	
}

static inline void mt_i2c_init_hw(mt_i2c *i2c)
{

  
  mt_i2c_power_ctl(i2c->id,1);
  
  i2c_writel(i2c,OFFSET_SOFTRESET, 0x0001);
  i2c_writel(i2c,OFFSET_DCM_EN, 0x0);
  
}

static void mt_i2c_free(mt_i2c *i2c)
{
  if (!i2c)
    return;

  free_irq(i2c->irqnr, i2c);
  i2c_del_adapter(&i2c->adap);
  kfree(i2c);
}

static S32 mt_i2c_probe(struct platform_device *pdev)
{
  S32 ret, irq;
  mt_i2c *i2c = NULL;
  struct resource *res;

    printk("mtk i2c probe [Remove mutex in i2c_transfer]+++++\n");
  
  res   = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (res == NULL)
       return -ENODEV;

  
  if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
    return -ENOMEM;
  }

  if (NULL == (i2c = kzalloc(sizeof(mt_i2c), GFP_KERNEL)))
    return -ENOMEM;

#ifdef CONFIG_OF
  i2c->base = of_iomap(pdev->dev.of_node, 0);
  if (!i2c->base) {
    I2CERR("I2C iomap failed\n");
    return -ENODEV;
  }

  if (of_property_read_u32(pdev->dev.of_node, "cell-index", &pdev->id)) {
    I2CERR("I2C get cell-index failed\n");
    return -ENODEV;
  }
  i2c->id = pdev->id;
  irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
  if (!irq) {
    I2CERR("I2C get irq failed\n");
    return -ENODEV;
  }
  
  if (of_property_read_u32(pdev->dev.of_node, "def_speed", &pdev->id)) {
    I2CERR("I2C get def_speed failed\n");
	i2c->defaul_speed  = 100;
  }

  i2c->defaul_speed = pdev->id;
#else
  irq = platform_get_irq(pdev, 0);
  if (irq < 0)
       return -ENODEV;
  
  i2c->id   = pdev->id;
  i2c->base = IO_PHYS_TO_VIRT(res->start);
  
#endif
  i2c->irqnr  = irq;
  #if (defined(CONFIG_MT_I2C_FPGA_ENABLE))
    i2c->clk  = I2C_CLK_RATE;
  #else
    i2c->clk  = mt_get_bus_freq();

    switch(i2c->id){
      case 0:
        i2c->pdn = MT_CG_PERI_I2C0;
        break;
      case 1:
        i2c->pdn = MT_CG_PERI_I2C1;
        break;
	  case 2:
		i2c->pdn = MT_CG_PERI_I2C2;
		break;
      case 3:
		i2c->pdn = MT_CG_PERI_I2C3;
		break;
	  case 4:
		i2c->pdn = MT_CG_PERI_I2C4;
		break;
	  default:
		dev_err(&pdev->dev, "Error id %d\n", i2c->id);
		break;
	}
#endif
	i2c->dev  = &i2c->adap.dev;

  i2c->adap.dev.parent  = &pdev->dev;
#ifdef CONFIG_OF
  i2c->adap.dev.of_node  = pdev->dev.of_node;
#endif
  i2c->adap.nr      = i2c->id;
  i2c->adap.owner     = THIS_MODULE;
  i2c->adap.algo      = &mt_i2c_algorithm;
  i2c->adap.algo_data   = NULL;
  i2c->adap.timeout   = 2 * HZ; 
  i2c->adap.retries   = 1; 
  
  i2c->dma_buf.vaddr =
		dma_alloc_coherent(&pdev->dev, MAX_DMA_TRANS_NUM,
			  &i2c->dma_buf.paddr, GFP_KERNEL|GFP_DMA32);

  
    if(!i2c->dma_buf.vaddr)
    {
        printk("mt-i2c:[Error] Allocate DMA I2C Buffer failed!\n");
    }
    memset(i2c->dma_buf.vaddr, 0, MAX_DMA_TRANS_NUM);
  snprintf(i2c->adap.name, sizeof(i2c->adap.name), I2C_DRV_NAME);


 #ifdef CONFIG_OF
  i2c->pdmabase = DMA_I2C_BASE(i2c->id, ap_dma_base);
 #else
  i2c->pdmabase = AP_DMA_BASE + 0x100 + (0x80*(i2c->id));
 #endif 
  I2CLOG(" id: %d, reg: 0x%p, dma_reg: 0x%p, irq: %d\n", i2c->id, i2c->base, i2c->pdmabase, i2c->irqnr);
  spin_lock_init(&i2c->lock);
  init_waitqueue_head(&i2c->wait);

  ret = request_irq(irq, mt_i2c_irq, IRQF_TRIGGER_LOW, I2C_DRV_NAME, i2c);

  if (ret){
    dev_err(&pdev->dev, "Can Not request I2C IRQ %d\n", irq);
    goto free;
  }
   
    printk("i2c-bus%d speed is %dKhz \n",i2c->id, i2c->defaul_speed);

  mt_i2c_init_hw(i2c);
  i2c_set_adapdata(&i2c->adap, i2c);
  ret = i2c_add_numbered_adapter(&i2c->adap);
  if (ret){
    dev_err(&pdev->dev, "failed to add i2c bus to i2c core\n");
    goto free;
  }
  platform_set_drvdata(pdev, i2c);

#ifdef CONFIG_OF
  of_i2c_register_devices(&i2c->adap);
#endif

#ifdef I2C_DEBUG_FS
  ret = device_create_file(i2c->dev, &dev_attr_debug);
  if ( ret ){
    
  }
#endif
  printk("i2c-%d: base(0x%p),dmabase(0x%p),irq(0x%d)",i2c->id,i2c->base,i2c->pdmabase,i2c->irqnr);
    printk("mtk i2c probe-----\n");
  return ret;

free:
  mt_i2c_free(i2c);
    printk("mtk i2c probe----- error\n");
  return ret;
}


static S32 mt_i2c_remove(struct platform_device *pdev)
{
  mt_i2c *i2c = platform_get_drvdata(pdev);
  if (i2c) {
    platform_set_drvdata(pdev, NULL);
    mt_i2c_free(i2c);
  }
  return 0;
}

#ifdef CONFIG_PM
  static S32 mt_i2c_suspend(struct platform_device *pdev, pm_message_t state)
  {
     mt_i2c *i2c = platform_get_drvdata(pdev);
     mt_i2c_power_ctl(0,0);
	 
	 printk("[I2C %d] Suspend!\n", i2c->id);
	 
     return 0;
  }

  static S32 mt_i2c_resume(struct platform_device *pdev)
  {
     mt_i2c *i2c = platform_get_drvdata(pdev);
     mt_i2c_power_ctl(0,1);
	 
	 printk("[I2C %d] Resume!\n", i2c->id);
     return 0;
  }
#else
  #define mt_i2c_suspend  NULL
  #define mt_i2c_resume NULL
#endif
static const struct of_device_id mt_i2c_of_match[] = {
	{ .compatible = "mediatek,I2C0", },
	{ .compatible = "mediatek,I2C1", },
	{ .compatible = "mediatek,I2C2", },
	{ .compatible = "mediatek,I2C3", },
	{ .compatible = "mediatek,I2C4", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_i2c_of_match);

static struct platform_driver mt_i2c_driver = {
  .probe   = mt_i2c_probe,
  .remove  = mt_i2c_remove,
  .suspend = mt_i2c_suspend,
  .resume  = mt_i2c_resume,
  .driver  = {
        .name  = I2C_DRV_NAME,
        .owner = THIS_MODULE,
		#ifdef CONFIG_OF
		.of_match_table = mt_i2c_of_match,
		#endif
    },
};

static S32 __init mt_i2c_init(void)
{
#ifdef CONFIG_OF
	struct device_node *ap_dma_node;

	
	ap_dma_node = of_find_compatible_node(NULL, NULL, "mediatek,AP_DMA");
	if (!ap_dma_node) {
		I2CERR("Cannot find AP_DMA node\n");
		return -ENODEV;
	}

	ap_dma_base = of_iomap(ap_dma_node, 0);
	if (!ap_dma_base) {
		I2CERR("AP_DMA iomap failed\n");
		return -ENOMEM;
	}
#endif
  return platform_driver_register(&mt_i2c_driver);
}

static void __exit mt_i2c_exit(void)
{
  platform_driver_unregister(&mt_i2c_driver);
}

module_init(mt_i2c_init);
module_exit(mt_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek I2C Bus Driver");
MODULE_AUTHOR("Infinity Chen <infinity.chen@mediatek.com>");
