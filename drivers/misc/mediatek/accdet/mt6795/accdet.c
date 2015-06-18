
#include "accdet.h"

#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/upmu_common.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include "accdet_htc.h"
#ifdef CONFIG_AMP_RT5506
#include "../../../../drivers/i2c/chips/rt5506.h"
#endif

#define SW_WORK_AROUND_ACCDET_REMOTE_BUTTON_ISSUE
#define DEBUG_THREAD 1


#define REGISTER_VALUE(x)   (x - 1)

#define HS_DELAY_BUTTON 500
#define HS_JIFFIES_BUTTON msecs_to_jiffies(HS_DELAY_BUTTON)
#define BUTTON_WAKE_LOCK_TIMEOUT (2*HZ)
static volatile int cur_key_flag = 0;
static volatile int cur_key_report = 0;
struct button_work {
	struct delayed_work key_work;
	int key_code;
	int key_flag;
	int key_force_report;
};
static struct workqueue_struct *button_wq = NULL;
struct wake_lock accdet_button_wake_lock;
static void button_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(button_work, button_work_func);

static int button_press_debounce = 0x400;

static int debug_enable = 1;
int cur_key = 0;

int accdet_irq;
unsigned int gpiopin,headsetdebounce;
struct headset_mode_settings *cust_headset_settings = NULL;

#define ACCDET_DEBUG(format, args...) do{ \
	if(debug_enable) \
	{\
		printk(KERN_WARNING format,##args);\
	}\
}while(0)

static struct switch_dev accdet_data;
static struct input_dev *kpd_accdet_dev;
static struct cdev *accdet_cdev;
static struct class *accdet_class = NULL;
static struct device *accdet_nor_device = NULL;

static dev_t accdet_devno;

static int pre_status = 0;
static int pre_state_swctrl = 0;
static volatile int accdet_status = PLUG_OUT;
static int cable_type = 0;
#ifdef ACCDET_PIN_RECOGNIZATION
static int cable_pin_recognition = 0;
static int show_icon_delay = 0;
#endif

static int eint_accdet_sync_flag = 0;

static s64 long_press_time_ns = 0 ;

static int g_accdet_first = 1;
static bool IRQ_CLR_FLAG = FALSE;
static volatile int call_status =0;
static volatile int button_status = 0;


struct wake_lock accdet_suspend_lock; 
struct wake_lock accdet_irq_lock;
struct wake_lock accdet_key_lock;
struct wake_lock accdet_timer_lock;


static struct work_struct accdet_work;
static struct workqueue_struct * accdet_workqueue = NULL;

static int long_press_time;

static DEFINE_MUTEX(accdet_eint_irq_sync_mutex);

static inline void clear_accdet_interrupt(void);
static inline void clear_accdet_eint_interrupt(void);

#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
static struct work_struct accdet_eint_work;
static struct workqueue_struct * accdet_eint_workqueue = NULL;
static void send_key_event(int, int);
static inline void accdet_init(void);
#define MICBIAS_DISABLE_TIMER   (6 *HZ)         
struct timer_list micbias_timer;
static void disable_micbias(unsigned long a);
#define EINT_PIN_PLUG_IN        (1)
#define EINT_PIN_PLUG_OUT       (0)
volatile int cur_eint_state = EINT_PIN_PLUG_OUT;
static struct work_struct accdet_disable_work;
static struct workqueue_struct * accdet_disable_workqueue = NULL;
#else
static int g_accdet_working_in_suspend =0;
#endif

extern S32 pwrap_read( U32  adr, U32 *rdata );
extern S32 pwrap_write( U32  adr, U32  wdata );
extern struct headset_mode_settings* get_cust_headset_settings(void);
extern struct headset_key_custom* get_headset_key_custom_setting(void);
extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
extern struct file_operations *accdet_get_fops(void);
extern struct platform_driver accdet_driver_func(void);

#ifdef DEBUG_THREAD
extern  void accdet_create_attr_func(void); 
#endif
static U32 pmic_pwrap_read(U32 addr);
static void pmic_pwrap_write(U32 addr, unsigned int wdata);

static struct htc_headset_info *hi;


void accdet_detect(void)
{
	int ret = 0 ;
    
	ACCDET_DEBUG("[Accdet]accdet_detect\n");
    
	accdet_status = PLUG_OUT;
    ret = queue_work(accdet_workqueue, &accdet_work);	
    if(!ret)
    {
  		ACCDET_DEBUG("[Accdet]accdet_detect:accdet_work return:%d!\n", ret);  		
    }

	return;
}
EXPORT_SYMBOL(accdet_detect);

void accdet_state_reset(void)
{
    
	ACCDET_DEBUG("[Accdet]accdet_state_reset\n");
    
	accdet_status = PLUG_OUT;
    cable_type = NO_DEVICE;
        
	return;
}
EXPORT_SYMBOL(accdet_state_reset);

int accdet_get_cable_type(void)
{
	return cable_type;
}
void accdet_auxadc_switch(int enable)
{
	if(enable) { 
		 pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_BF_ON);
		 
	}else {
		 pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)&~(ACCDET_BF_ON));
		 
	}
}

static U64 accdet_get_current_time(void)
{
	return sched_clock(); 
}
static BOOL accdet_timeout_ns (U64 start_time_ns, U64 timeout_time_ns)
{
	U64 cur_time=0;
	U64 elapse_time=0;

	
	cur_time = accdet_get_current_time();
	if(cur_time < start_time_ns){
		ACCDET_DEBUG("@@@@Timer overflow! start%lld cur timer%lld\n",start_time_ns,cur_time);
		start_time_ns=cur_time;
		timeout_time_ns=400*1000; 
		ACCDET_DEBUG("@@@@reset timer! start%lld setting%lld\n",start_time_ns,timeout_time_ns);
	}
	elapse_time=cur_time-start_time_ns;

	
	if (timeout_time_ns <= elapse_time)
	{
		
		ACCDET_DEBUG("@@@@ACCDET IRQ clear Timeout\n");
		return FALSE;
	}
	return TRUE;
}

static U32 pmic_pwrap_read(U32 addr)
{
	U32 val =0;
	pwrap_read(addr, &val);
	
	return val;
}

static void pmic_pwrap_write(unsigned int addr, unsigned int wdata)
{
    pwrap_write(addr, wdata);
	
}
static int Accdet_PMIC_IMM_GetOneChannelValue(int deCount)
{
	unsigned int vol_val = 0;
	pmic_pwrap_write(ACCDET_AUXADC_CTL_SET, ACCDET_CH_REQ_EN);
	mdelay(3);
	while((pmic_pwrap_read(ACCDET_AUXADC_REG)&ACCDET_DATA_READY)!=ACCDET_DATA_READY) {} 
	vol_val = (pmic_pwrap_read(ACCDET_AUXADC_REG) & ACCDET_DATA_MASK);
	
	vol_val = (vol_val*3200)/4096; 
	ACCDET_DEBUG("ACCDET read Voltage: %d mv!! \n\r", vol_val);
	return vol_val;
}

#if !defined ACCDET_EINT && !defined ACCDET_EINT_IRQ
static bool is_long_press(void)
{
	int current_status = 0;
	int index = 0;
	int count = long_press_time / 100;
	while(index++ < count)
	{ 
		current_status = ((pmic_pwrap_read(ACCDET_STATE_RG) & 0xc0)>>6);
		if(current_status != 0)
		{
			return false;
		}
			
		msleep(100);
	}
	
	return true;
}
#endif

#ifdef ACCDET_PIN_SWAP

static void accdet_FSA8049_enable(void)
{
	mt_set_gpio_mode(GPIO_FSA8049_PIN, GPIO_FSA8049_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_FSA8049_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FSA8049_PIN, GPIO_OUT_ONE);
}

static void accdet_FSA8049_disable(void)
{
	mt_set_gpio_mode(GPIO_FSA8049_PIN, GPIO_FSA8049_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_FSA8049_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_FSA8049_PIN, GPIO_OUT_ZERO);
}


#endif
static void inline headset_plug_out(void) 
{
        accdet_status = PLUG_OUT;
        cable_type = NO_DEVICE;
        
        switch_set_state((struct switch_dev *)&accdet_data, cable_type);
        ACCDET_DEBUG( " [accdet] set state in cable_type = NO_DEVICE\n");
        
}

static void inline enable_accdet(u32 state_swctrl)
{
   
   ACCDET_DEBUG("accdet: enable_accdet\n");
   
   pmic_pwrap_write(TOP_CKPDN_CLR, RG_ACCDET_CLK_CLR); 
   
   pmic_pwrap_write(ACCDET_STATE_SWCTRL, pmic_pwrap_read(ACCDET_STATE_SWCTRL)|state_swctrl);
   pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)|ACCDET_ENABLE);
  

}

static void inline disable_accdet(void)
{
	int irq_temp = 0;
	
	
	pmic_pwrap_write(INT_CON_ACCDET_CLR, RG_ACCDET_IRQ_CLR);
	clear_accdet_interrupt();
	udelay(200);
	mutex_lock(&accdet_eint_irq_sync_mutex);
	while(pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT)
	{
		ACCDET_DEBUG("[Accdet]check_cable_type: Clear interrupt on-going....\n");
		msleep(5);
	}
	irq_temp = pmic_pwrap_read(ACCDET_IRQ_STS);
	irq_temp = irq_temp & (~IRQ_CLR_BIT);
	pmic_pwrap_write(ACCDET_IRQ_STS, irq_temp);
	ACCDET_DEBUG("[Accdet]disable_accdet:Clear interrupt:Done[0x%x]!\n", pmic_pwrap_read(ACCDET_IRQ_STS));	
	mutex_unlock(&accdet_eint_irq_sync_mutex);
   
   ACCDET_DEBUG("accdet: disable_accdet\n");
   pre_state_swctrl = pmic_pwrap_read(ACCDET_STATE_SWCTRL);
   #ifdef ACCDET_EINT
   pmic_pwrap_write(ACCDET_STATE_SWCTRL, 0);
   pmic_pwrap_write(ACCDET_CTRL, ACCDET_DISABLE);
   
   
   pmic_pwrap_write(TOP_CKPDN_SET, RG_ACCDET_CLK_SET); 
   #endif
   #ifdef ACCDET_EINT_IRQ
   pmic_pwrap_write(ACCDET_STATE_SWCTRL, pmic_pwrap_read(ACCDET_STATE_SWCTRL)&(~ACCDET_SWCTRL_EN));
   pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)&(~(ACCDET_ENABLE)));
   #endif

}

#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
static void disable_micbias(unsigned long a)
{
	  int ret = 0;
      ret = queue_work(accdet_disable_workqueue, &accdet_disable_work);	
      if(!ret)
      {
  	    ACCDET_DEBUG("[Accdet]disable_micbias:accdet_work return:%d!\n", ret);  		
      }
}
static void disable_micbias_callback(struct work_struct *work)
{
	
        if(cable_type == HEADSET_NO_MIC) {
			#ifdef ACCDET_PIN_RECOGNIZATION
			   show_icon_delay = 0;
			   cable_pin_recognition = 0;
			   ACCDET_DEBUG("[Accdet] cable_pin_recognition = %d\n", cable_pin_recognition);
				pmic_pwrap_write(ACCDET_PWM_WIDTH, cust_headset_settings->pwm_width);
    			pmic_pwrap_write(ACCDET_PWM_THRESH, cust_headset_settings->pwm_thresh);
			#endif
            
   			pmic_pwrap_write(ACCDET_STATE_SWCTRL, pmic_pwrap_read(ACCDET_STATE_SWCTRL)&~ACCDET_SWCTRL_IDLE_EN);
			#ifdef ACCDET_PIN_SWAP
		    	
		    	
			#endif
                disable_accdet();
                ACCDET_DEBUG("[Accdet] more than 5s MICBIAS : Disabled\n");
        }
	#ifdef ACCDET_PIN_RECOGNIZATION
		else if(cable_type == HEADSET_MIC) {       
			pmic_pwrap_write(ACCDET_PWM_WIDTH, cust_headset_settings->pwm_width);
    		pmic_pwrap_write(ACCDET_PWM_THRESH, cust_headset_settings->pwm_thresh);
			ACCDET_DEBUG("[Accdet]pin recog after 5s recover micbias polling!\n");
		}
	#endif
}

static void accdet_eint_work_callback(struct work_struct *work)
{
	struct button_work *btn_work;

#ifdef ACCDET_EINT_IRQ
	int irq_temp = 0;
	if (cur_eint_state == EINT_PIN_PLUG_IN) {
		ACCDET_DEBUG("[Accdet]EINT func :plug-in\n");
		mutex_lock(&accdet_eint_irq_sync_mutex);
		eint_accdet_sync_flag = 1;
		mutex_unlock(&accdet_eint_irq_sync_mutex);
		wake_lock_timeout(&accdet_timer_lock, 7*HZ);
		#ifdef ACCDET_PIN_SWAP
			
			msleep(800);
		    accdet_FSA8049_enable();  
		    ACCDET_DEBUG("[Accdet] FSA8049 enable!\n");
			msleep(250); 
		#endif
		
			accdet_init();
		
		#ifdef ACCDET_PIN_RECOGNIZATION
		  show_icon_delay = 1;
		
		  pmic_pwrap_write(ACCDET_PWM_WIDTH, cust_headset_settings->pwm_width);
    	  pmic_pwrap_write(ACCDET_PWM_THRESH, cust_headset_settings->pwm_width);
		  ACCDET_DEBUG("[Accdet]pin recog start!  micbias always on!\n");
		#endif
		
			pmic_pwrap_write(ACCDET_STATE_SWCTRL, (pmic_pwrap_read(ACCDET_STATE_SWCTRL)|ACCDET_SWCTRL_IDLE_EN));
		
			enable_accdet(ACCDET_SWCTRL_EN); 
    } else {
		ACCDET_DEBUG("[Accdet]EINT func :plug-out\n");
		mutex_lock(&accdet_eint_irq_sync_mutex);
		eint_accdet_sync_flag = 0;
		mutex_unlock(&accdet_eint_irq_sync_mutex);
		del_timer_sync(&micbias_timer);
		#ifdef ACCDET_PIN_RECOGNIZATION
		  show_icon_delay = 0;
		  cable_pin_recognition = 0;
		#endif
		#ifdef ACCDET_PIN_SWAP
			
		    accdet_FSA8049_disable();  
		    ACCDET_DEBUG("[Accdet] FSA8049 disable!\n");
		#endif
			accdet_auxadc_switch(0);
			disable_accdet();			   
			headset_plug_out();
		 
		 
		 irq_temp = pmic_pwrap_read(ACCDET_IRQ_STS);
		 irq_temp = irq_temp & (~IRQ_EINT_CLR_BIT);
		 pmic_pwrap_write(ACCDET_IRQ_STS, irq_temp); 
    }
#else
   
#ifdef CONFIG_OF
    disable_irq(accdet_irq);
#else
    mt_eint_mask(CUST_EINT_ACCDET_NUM);
#endif
	
   
    if (cur_eint_state == EINT_PIN_PLUG_IN) {
#ifdef CONFIG_AMP_RT5506
		pr_info("%s: rt5506_headset_detect: true\n", __func__);
		rt5506_headset_detect(NULL, true);
#endif
		ACCDET_DEBUG("[Accdet]EINT func :plug-in\n");
		mutex_lock(&accdet_eint_irq_sync_mutex);
		eint_accdet_sync_flag = 1;
		mutex_unlock(&accdet_eint_irq_sync_mutex);
		wake_lock_timeout(&accdet_timer_lock, 7*HZ);
		#ifdef ACCDET_PIN_SWAP
			
			msleep(800);
		    accdet_FSA8049_enable();  
		    ACCDET_DEBUG("[Accdet] FSA8049 enable!\n");
			msleep(250); 
		#endif
		
			accdet_init();
		
		#ifdef ACCDET_PIN_RECOGNIZATION
		  show_icon_delay = 1;
		
		  pmic_pwrap_write(ACCDET_PWM_WIDTH, cust_headset_settings->pwm_width);
    	  pmic_pwrap_write(ACCDET_PWM_THRESH, cust_headset_settings->pwm_width);
		  ACCDET_DEBUG("[Accdet]pin recog start!  micbias always on!\n");
		#endif
		
		pmic_pwrap_write(ACCDET_STATE_SWCTRL, (pmic_pwrap_read(ACCDET_STATE_SWCTRL)|ACCDET_SWCTRL_IDLE_EN));
		
		enable_accdet(ACCDET_SWCTRL_EN); 
    } else {
#ifdef CONFIG_AMP_RT5506
		pr_info("%s: rt5506_headset_detect: false\n", __func__);
		rt5506_headset_detect(NULL, false);
		if(hi) hi->hs_35mm_type = HTC_HEADSET_UNPLUG;
#endif
		ACCDET_DEBUG("[Accdet]EINT func :plug-out\n");
		mutex_lock(&accdet_eint_irq_sync_mutex);
		eint_accdet_sync_flag = 0;
		mutex_unlock(&accdet_eint_irq_sync_mutex);
		del_timer_sync(&micbias_timer);
		#ifdef ACCDET_PIN_RECOGNIZATION
		  show_icon_delay = 0;
		  cable_pin_recognition = 0;
		#endif
		#ifdef ACCDET_PIN_SWAP
			
		    accdet_FSA8049_disable();  
		    ACCDET_DEBUG("[Accdet] FSA8049 disable!\n");
		#endif

		accdet_auxadc_switch(0);
		disable_accdet();
		headset_plug_out();
		
#if 1
		btn_work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
		if (!btn_work) {
			ACCDET_DEBUG("[Accdet] %s: Failed to allocate button memory\n", __func__);
		} else {
			btn_work->key_code = cur_key_report;
			btn_work->key_flag = !cur_key_flag;
			btn_work->key_force_report = 1;
			ACCDET_DEBUG("[Accdet] %s: delay wq with key [%s] flag %d force_report %d\n",
				__func__, button_string[btn_work->key_code], btn_work->key_flag, btn_work->key_force_report);
			wake_lock_timeout(&accdet_button_wake_lock, BUTTON_WAKE_LOCK_TIMEOUT);
			INIT_DELAYED_WORK(&btn_work->key_work, button_work_func);
			queue_delayed_work(button_wq, &btn_work->key_work, HS_JIFFIES_BUTTON);
		}
#else
		send_key_event(cur_key, !key_flag);
#endif
    }
    
    
#ifdef CONFIG_OF
	enable_irq(accdet_irq);
#else
    mt_eint_unmask(CUST_EINT_ACCDET_NUM);
#endif
    ACCDET_DEBUG("[Accdet]enable_irq  !!!!!!\n");
#endif   
}

#ifdef CONFIG_OF
static irqreturn_t accdet_eint_func(int irq,void *data)
{
	int ret=0;
	if(cur_eint_state ==  EINT_PIN_PLUG_IN ) 
	{
	#ifndef ACCDET_EINT_IRQ
		if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
		    irq_set_irq_type(accdet_irq,IRQ_TYPE_LEVEL_HIGH);
		}else{
		    irq_set_irq_type(accdet_irq,IRQ_TYPE_LEVEL_LOW);
		}
	#endif
	#ifdef ACCDET_EINT_IRQ
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)&(~(7<<4)));
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)|EINT_IRQ_DE_IN);
	#else
		mt_gpio_set_debounce(gpiopin,headsetdebounce);
	#endif

		
		cur_eint_state = EINT_PIN_PLUG_OUT;
	} 
	else 
	{
	#ifndef ACCDET_EINT_IRQ
		if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
		    irq_set_irq_type(accdet_irq,IRQ_TYPE_LEVEL_LOW);
		}else{
		    irq_set_irq_type(accdet_irq,IRQ_TYPE_LEVEL_HIGH);
		}
	#endif
	
	#ifdef ACCDET_EINT_IRQ
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)&(~(7<<4)));
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)|EINT_IRQ_DE_OUT);
	#else
		mt_gpio_set_debounce(gpiopin,ACCDET_SHORT_PLUGOUT_DEBOUNCE_CN*1000);
	#endif  
		
		cur_eint_state = EINT_PIN_PLUG_IN;

		
		init_timer(&micbias_timer);
		micbias_timer.expires = jiffies + MICBIAS_DISABLE_TIMER;
		micbias_timer.function = &disable_micbias;
		micbias_timer.data = ((unsigned long) 0 );
		add_timer(&micbias_timer);
	}

	ret = queue_work(accdet_eint_workqueue, &accdet_eint_work);	
	return IRQ_HANDLED;
}
#else
static void accdet_eint_func(void)
{
	int ret=0;
	if(cur_eint_state ==  EINT_PIN_PLUG_IN ) 
	{
	#ifndef ACCDET_EINT_IRQ
		if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
            mt_eint_set_polarity(CUST_EINT_ACCDET_NUM, (1));
		}else{
		    mt_eint_set_polarity(CUST_EINT_ACCDET_NUM, (0));
		}
	#endif
	#ifdef ACCDET_EINT_IRQ
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)&(~(7<<4)));
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)|EINT_IRQ_DE_IN);
	#else
		mt_eint_set_hw_debounce(CUST_EINT_ACCDET_NUM, CUST_EINT_ACCDET_DEBOUNCE_CN);
	#endif

		
		cur_eint_state = EINT_PIN_PLUG_OUT;
	} 
	else 
	{
	#ifndef ACCDET_EINT_IRQ
		if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
		    mt_eint_set_polarity(CUST_EINT_ACCDET_NUM, !(1));
		}else{
		    mt_eint_set_polarity(CUST_EINT_ACCDET_NUM, !(0));
		}
	#endif
	
	#ifdef ACCDET_EINT_IRQ
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)&(~(7<<4)));
		pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)|EINT_IRQ_DE_OUT);
	#else
	    mt_eint_set_hw_debounce(CUST_EINT_ACCDET_NUM, ACCDET_SHORT_PLUGOUT_DEBOUNCE_CN);
	#endif  
		
		cur_eint_state = EINT_PIN_PLUG_IN;

		
					
		init_timer(&micbias_timer);
		micbias_timer.expires = jiffies + MICBIAS_DISABLE_TIMER;
		micbias_timer.function = &disable_micbias;
		micbias_timer.data = ((unsigned long) 0 );
		add_timer(&micbias_timer);
	}

	ret = queue_work(accdet_eint_workqueue, &accdet_eint_work);	
}
#endif
#ifndef ACCDET_EINT_IRQ
static inline int accdet_setup_eint(void) 
{
	int ret;
#ifdef CONFIG_OF
	u32 ints[2]={0,0};
	struct device_node *node;
#endif
	
    ACCDET_DEBUG("[Accdet]accdet_setup_eint\n");
	mt_set_gpio_mode(GPIO_ACCDET_EINT_PIN, GPIO_ACCDET_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ACCDET_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ACCDET_EINT_PIN, GPIO_PULL_DISABLE); 

#ifdef CONFIG_OF
    node = of_find_compatible_node(NULL,NULL,"mediatek, ACCDET-eint");
	if(node) {
        of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
		gpiopin = ints[0];
		headsetdebounce = ints[1];
		mt_gpio_set_debounce(gpiopin,headsetdebounce);
		accdet_irq = irq_of_parse_and_map(node,0);
		ret = request_irq(accdet_irq,accdet_eint_func,IRQF_TRIGGER_NONE,"ACCDET-eint",NULL);
		if(ret>0){
            ACCDET_DEBUG("[Accdet]EINT IRQ LINE�NNOT AVAILABLE\n");
		}
	}
	else {
        ACCDET_DEBUG("[Accdet]%s can't find compatible node\n", __func__);
	}
#else
	mt_eint_set_hw_debounce(CUST_EINT_ACCDET_NUM, CUST_EINT_ACCDET_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ACCDET_NUM, CUST_EINT_ACCDET_TYPE, accdet_eint_func, 0);
	ACCDET_DEBUG("[Accdet]accdet set EINT finished, accdet_eint_num=%d, accdet_eint_debounce_en=%d, accdet_eint_polarity=%d\n", CUST_EINT_ACCDET_NUM, CUST_EINT_ACCDET_DEBOUNCE_EN, CUST_EINT_ACCDET_TYPE);
	mt_eint_unmask(CUST_EINT_ACCDET_NUM);  
#endif

	return 0;
}
#endif
#endif

#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);

#define KEY_SAMPLE_PERIOD        (60)            
#define MULTIKEY_ADC_CHANNEL	 (8)

#define NO_KEY			 (0x0)
#define UP_KEY			 (0x01)
#define MD_KEY		  	 (0x02)
#define DW_KEY			 (0x04)


static DEFINE_MUTEX(accdet_multikey_mutex);




#define DW_KEY_HIGH_THR			(378) 
#define DW_KEY_THR			(178) 
#define UP_KEY_THR			(77) 
#define MD_KEY_THR			(0)

static int key_check(int b)
{
	
	
	
	ACCDET_DEBUG("[accdet] come in key_check!!\n");
	if((b<DW_KEY_HIGH_THR)&&(b >= DW_KEY_THR)) 
	{
		ACCDET_DEBUG("[accdet]adc_data: %d mv\n",b);
		return DW_KEY;
	} 
	else if ((b < DW_KEY_THR)&& (b >= UP_KEY_THR))
	{
		ACCDET_DEBUG("[accdet]adc_data: %d mv\n",b);
		return UP_KEY;
	}
	else if ((b < UP_KEY_THR) && (b >= MD_KEY_THR))
	{
		ACCDET_DEBUG("[accdet]adc_data: %d mv\n",b);
		return MD_KEY;
	}
	ACCDET_DEBUG("[accdet] leave key_check!!\n");
	return NO_KEY;
}
static void send_key_event(int keycode,int flag)
{
#if 0
    if(call_status == 0)
    {
                switch (keycode)
                {
                case DW_KEY:
					input_report_key(kpd_accdet_dev, KEY_NEXTSONG, flag);
					input_sync(kpd_accdet_dev);
					ACCDET_DEBUG("[accdet]KEY_NEXTSONG %d\n",flag);
					break;
				case UP_KEY:
		   	        input_report_key(kpd_accdet_dev, KEY_PREVIOUSSONG, flag);
                    input_sync(kpd_accdet_dev);
					ACCDET_DEBUG("[accdet]KEY_PREVIOUSSONG %d\n",flag);
		   	        break;			
				case MD_KEY:
		   	        input_report_key(kpd_accdet_dev, KEY_PLAYPAUSE, flag);
                    input_sync(kpd_accdet_dev);
					ACCDET_DEBUG("[accdet]KEY_PLAYPAUSE %d\n",flag);
		   	        break;
                }
     }
	else
	{
#endif
	if (!hi) {
		ACCDET_DEBUG("[accdet] %s: hi is NULL\n", __func__);
	} else {
		ACCDET_DEBUG("[accdet] %s: hs_mfg_mode %d\n", __func__, hi->hs_mfg_mode);
	}

	switch (keycode)
	{
		case DW_KEY:
			if (hi->hs_mfg_mode) {
				input_report_key(kpd_accdet_dev, KEY_NEXTSONG, flag);
				input_sync(kpd_accdet_dev);
				ACCDET_DEBUG("[accdet]KEY_NEXTSONG %d\n",flag);
			} else {
				input_report_key(kpd_accdet_dev, KEY_VOLUMEDOWN, flag);
				input_sync(kpd_accdet_dev);
				ACCDET_DEBUG("[accdet]KEY_VOLUMEDOWN %d\n",flag);
			}
			break;
		case UP_KEY:
			if (hi->hs_mfg_mode) {
				input_report_key(kpd_accdet_dev, KEY_PREVIOUSSONG, flag);
				input_sync(kpd_accdet_dev);
				ACCDET_DEBUG("[accdet]KEY_PREVIOUSSONG %d\n",flag);
			} else {
				input_report_key(kpd_accdet_dev, KEY_VOLUMEUP, flag);
				input_sync(kpd_accdet_dev);
				ACCDET_DEBUG("[accdet]KEY_VOLUMEUP %d\n",flag);
			}
			break;
		case MD_KEY:
			input_report_key(kpd_accdet_dev, KEY_PLAYPAUSE, flag);
			input_sync(kpd_accdet_dev);
			ACCDET_DEBUG("[accdet]KEY_PLAYPAUSE %d\n",flag);
		break;
	}
	
	cur_key_flag = flag;
	cur_key_report = keycode;

}

static void button_work_func(struct work_struct *work)
{
	int key = 0;
	int flag = 0;
	int force_report = 0;
	struct button_work *works;

	works = container_of(work, struct button_work, key_work.work);
	key = works->key_code;
	flag = works->key_flag;
	force_report = works->key_force_report;
	ACCDET_DEBUG("[Accdet] %s: key [%s] flag %d, force_report %d, accdet_status [%s], cur_key_report [%s] cur_key_flag %d\n",
		__func__, button_string[key], flag, force_report, accdet_status_string[accdet_status], button_string[cur_key_report], cur_key_flag);

	if (force_report) {
		if (cur_key_flag) {
			ACCDET_DEBUG("[Accdet] %s: only receive press flag\n", __func__);
			send_key_event(cur_key_report, 0);
		}
	} else {
		if (accdet_status == PLUG_OUT) {
			if (cur_key_flag && !flag) {
				ACCDET_DEBUG("[Accdet] %s: headset is already plug-out, but report release flag\n", __func__);
				send_key_event(key, flag);
			} else {
				ACCDET_DEBUG("[Accdet] %s: headset is already plug-out, don't send key event\n", __func__);
			}
		} else {
			send_key_event(key, flag);
		}
	}

	kfree(works);
}

static void multi_key_detection(int current_status)
{
	
	int m_key = 0;
	int cali_voltage=0;
	struct button_work *work;

	if(0 == current_status){
		cali_voltage = Accdet_PMIC_IMM_GetOneChannelValue(1);;
		ACCDET_DEBUG("[Accdet]adc cali_voltage1 = %d mv\n", cali_voltage);
		m_key = cur_key = key_check(cali_voltage);
	}

#if 1
	work = kzalloc(sizeof(struct button_work), GFP_KERNEL);
	if (!work) {
		ACCDET_DEBUG("[Accdet] %s: Failed to allocate button memory\n", __func__);
		return;
	}

	work->key_code = cur_key;
	work->key_flag = !current_status;
	work->key_force_report = 0;
	ACCDET_DEBUG("[Accdet] %s: delay wq with key [%s] flag %d, force_report %d\n",
		__func__, button_string[work->key_code], work->key_flag, work->key_force_report);
	wake_lock_timeout(&accdet_button_wake_lock, BUTTON_WAKE_LOCK_TIMEOUT);
	INIT_DELAYED_WORK(&work->key_work, button_work_func);
	queue_delayed_work(button_wq, &work->key_work, HS_JIFFIES_BUTTON);
#else
	send_key_event(cur_key, !current_status);
#endif
}

#endif
static void accdet_workqueue_func(void)
{
	int ret;
	ret = queue_work(accdet_workqueue, &accdet_work);
	if(!ret)
	{
		ACCDET_DEBUG("[Accdet]accdet_work return:%d!\n", ret);
	}
	
}
int accdet_irq_handler(void)
{
	U64 cur_time = 0;
	cur_time = accdet_get_current_time();
#ifdef ACCDET_EINT_IRQ
	if((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT)&&((pmic_pwrap_read(ACCDET_IRQ_STS) & EINT_IRQ_STATUS_BIT)!=EINT_IRQ_STATUS_BIT)) {
		clear_accdet_interrupt();	
		if (accdet_status == MIC_BIAS){
			accdet_auxadc_switch(1);
			pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
			pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_width));
		}
		accdet_workqueue_func();  
		while(((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT) && (accdet_timeout_ns(cur_time, ACCDET_TIME_OUT)))) {
		}
	}else if((pmic_wrap_read(ACCDET_IRQ_STS) & EINT_IRQ_STATUS_BIT)==EINT_IRQ_STATUS_BIT) {
		if(cur_eint_state ==  EINT_PIN_PLUG_IN ) {
			
			if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
				pmic_pwrap_write(ACCDET_IRQ_STS, pmic_pwrap_read(ACCDET_IRQ_STS)|EINT_IRQ_POL_HIGH);
			
			}else{
				pmic_pwrap_write(ACCDET_IRQ_STS, pmic_pwrap_read(ACCDET_IRQ_STS)&~EINT_IRQ_POL_LOW);
			}
			
		}else {
			
			if (CUST_EINT_ACCDET_TYPE == CUST_EINTF_TRIGGER_HIGH){
				pmic_pwrap_write(ACCDET_IRQ_STS, pmic_pwrap_read(ACCDET_IRQ_STS)&~EINT_IRQ_POL_LOW);
			}else{
			
				pmic_pwrap_write(ACCDET_IRQ_STS, pmic_pwrap_read(ACCDET_IRQ_STS)|EINT_IRQ_POL_HIGH);
			}
			
		}
		clear_accdet_eint_interrupt();
		while(((pmic_pwrap_read(ACCDET_IRQ_STS) & EINT_IRQ_STATUS_BIT) && (accdet_timeout_ns(cur_time, ACCDET_TIME_OUT)))) {
		}
		accdet_eint_func();  
	}else {
		 ACCDET_DEBUG("ACCDET IRQ and EINT IRQ don't be triggerred!!\n");
	}	
#else
	if((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT)) {
		clear_accdet_interrupt();
	}
    if (accdet_status == MIC_BIAS){
		accdet_auxadc_switch(1);
    	pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
	 	pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_width));
    }
    accdet_workqueue_func();  
	while(((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT) && 
		   (accdet_timeout_ns(cur_time, ACCDET_TIME_OUT)))) {
	}
#endif
#ifdef ACCDET_NEGV_IRQ
	cur_time = accdet_get_current_time();
	if((pmic_pwrap_read(ACCDET_IRQ_STS) & NEGV_IRQ_STATUS_BIT)==NEGV_IRQ_STATUS_BIT) {
		ACCDET_DEBUG("[ACCDET NEGV detect]plug in a error Headset\n\r");
		pmic_pwrap_write(ACCDET_IRQ_STS, (IRQ_NEGV_CLR_BIT));
		while(((pmic_pwrap_read(ACCDET_IRQ_STS) & NEGV_IRQ_STATUS_BIT) && 
			   (accdet_timeout_ns(cur_time, ACCDET_TIME_OUT)))) {
		} 
		
		pmic_pwrap_write(ACCDET_IRQ_STS, (pmic_pwrap_read(ACCDET_IRQ_STS)&(~IRQ_NEGV_CLR_BIT))); 
	}
#endif

    return 1;
}

static inline void clear_accdet_interrupt(void)
{
	
	pmic_pwrap_write(ACCDET_IRQ_STS, (IRQ_CLR_BIT));
	ACCDET_DEBUG("[Accdet]clear_accdet_interrupt: ACCDET_IRQ_STS = 0x%x\n", pmic_pwrap_read(ACCDET_IRQ_STS));
}
static inline void clear_accdet_eint_interrupt(void)
{
	pmic_pwrap_write(ACCDET_IRQ_STS, (IRQ_EINT_CLR_BIT));
	ACCDET_DEBUG("[Accdet]clear_accdet_eint_interrupt: ACCDET_IRQ_STS = 0x%x\n", pmic_pwrap_read(ACCDET_IRQ_STS));
}



static inline void check_cable_type(void)
{
    int current_status = 0;
	int irq_temp = 0; 
	int wait_clear_irq_times = 0;
#ifdef ACCDET_PIN_RECOGNIZATION
    int pin_adc_value = 0;
#define PIN_ADC_CHANNEL 5
#endif
    
    current_status = ((pmic_pwrap_read(ACCDET_STATE_RG) & 0xc0)>>6); 
    ACCDET_DEBUG("[Accdet]accdet interrupt happen:[%s]current AB = %d\n", 
		accdet_status_string[accdet_status], current_status);
	    	
    button_status = 0;
    pre_status = accdet_status;

    ACCDET_DEBUG("[Accdet]check_cable_type: ACCDET_IRQ_STS = 0x%x\n", pmic_pwrap_read(ACCDET_IRQ_STS));
    IRQ_CLR_FLAG = FALSE;
	switch(accdet_status)
    {
        case PLUG_OUT:
			  #ifdef ACCDET_PIN_RECOGNIZATION
			    pmic_pwrap_write(ACCDET_DEBOUNCE1, cust_headset_settings->debounce1);
			  #endif
            if(current_status == 0)
            {
				#ifdef ACCDET_PIN_RECOGNIZATION
				
				pmic_pwrap_write(ACCDET_PWM_WIDTH, cust_headset_settings->pwm_width);
    	  		pmic_pwrap_write(ACCDET_PWM_THRESH, cust_headset_settings->pwm_width);
		  		ACCDET_DEBUG("[Accdet]PIN recognition micbias always on!\n");
				 ACCDET_DEBUG("[Accdet]before adc read, pin_adc_value = %d mv!\n", pin_adc_value);
				 msleep(1000);
				 current_status = ((pmic_pwrap_read(ACCDET_STATE_RG) & 0xc0)>>6); 
				 if (current_status == 0 && show_icon_delay != 0)
				 {
					accdet_auxadc_switch(1);
					pin_adc_value = Accdet_PMIC_IMM_GetOneChannelValue(1);
					ACCDET_DEBUG("[Accdet]pin_adc_value = %d mv!\n", pin_adc_value);
					accdet_auxadc_switch(0);			
					if (200 > pin_adc_value && pin_adc_value> 100) 
					{
						mutex_lock(&accdet_eint_irq_sync_mutex);
						if(1 == eint_accdet_sync_flag) {
						cable_type = HEADSET_NO_MIC;
						accdet_status = HOOK_SWITCH;
						cable_pin_recognition = 1;
						ACCDET_DEBUG("[Accdet] cable_pin_recognition = %d\n", cable_pin_recognition);
						}else {
							ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
						}		
						mutex_unlock(&accdet_eint_irq_sync_mutex);
					}
					else
					{
						mutex_lock(&accdet_eint_irq_sync_mutex);
						if(1 == eint_accdet_sync_flag) {
							cable_type = HEADSET_NO_MIC;
							accdet_status = HOOK_SWITCH;
						}else {
							ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
						}	
						mutex_unlock(&accdet_eint_irq_sync_mutex);
					}
				 }
				 #else
				  mutex_lock(&accdet_eint_irq_sync_mutex);
				  if(1 == eint_accdet_sync_flag) {
					cable_type = HEADSET_NO_MIC;
					accdet_status = HOOK_SWITCH;
				  }else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
				  }
				  mutex_unlock(&accdet_eint_irq_sync_mutex);
           		 #endif
            }
			else if(current_status == 1)
            {
				mutex_lock(&accdet_eint_irq_sync_mutex);
				if(1 == eint_accdet_sync_flag) {
					accdet_status = MIC_BIAS;		
	         		cable_type = HEADSET_MIC;
				}else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
				}
				mutex_unlock(&accdet_eint_irq_sync_mutex);
				
                
                pmic_pwrap_write(ACCDET_DEBOUNCE0, button_press_debounce);
			   
			   #ifdef ACCDET_PIN_RECOGNIZATION
				pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
                pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_thresh));
			   #endif
				
				
				
            }
            else
            {
                ACCDET_DEBUG("[Accdet]PLUG_OUT can't change to this state!\n"); 
            }
            break;

	    case MIC_BIAS:
	    
            
            pmic_pwrap_write(ACCDET_DEBOUNCE0, cust_headset_settings->debounce0);
			
            if(current_status == 0)
            {
            
			mutex_lock(&accdet_eint_irq_sync_mutex);
			if(1 == eint_accdet_sync_flag) {
				while((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT) && (wait_clear_irq_times<3))
	        	{
		          ACCDET_DEBUG("[Accdet]check_cable_type: MIC BIAS clear IRQ on-going1....\n");	
				  wait_clear_irq_times++;
				  msleep(5);
	        	}
				irq_temp = pmic_pwrap_read(ACCDET_IRQ_STS);
				irq_temp = irq_temp & (~IRQ_CLR_BIT);
				pmic_pwrap_write(ACCDET_IRQ_STS, irq_temp);
            	IRQ_CLR_FLAG = TRUE;
		    	accdet_status = HOOK_SWITCH;
			}else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
			}
			mutex_unlock(&accdet_eint_irq_sync_mutex);
		    button_status = 1;
			if(button_status)
		    {	
			    mutex_lock(&accdet_eint_irq_sync_mutex);
				if(1 == eint_accdet_sync_flag) {   
					multi_key_detection(current_status);
				}else {
					ACCDET_DEBUG("[Accdet] multi_key_detection: Headset has plugged out\n");
				}
				mutex_unlock(&accdet_eint_irq_sync_mutex);
				accdet_auxadc_switch(0);
			
                pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
                pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_thresh));
	     	}
	   	  }
          else if(current_status == 1)
          {
          	 mutex_lock(&accdet_eint_irq_sync_mutex);
			 if(1 == eint_accdet_sync_flag) {
                accdet_status = MIC_BIAS;		
	            cable_type = HEADSET_MIC;
                ACCDET_DEBUG("[Accdet]MIC_BIAS state not change!\n");
			 }else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
			 }
			 mutex_unlock(&accdet_eint_irq_sync_mutex);
          }
          else
           {
               ACCDET_DEBUG("[Accdet]MIC_BIAS can't change to this state!\n"); 
           }
          break;

	case HOOK_SWITCH:
            if(current_status == 0)
            {
				mutex_lock(&accdet_eint_irq_sync_mutex);
		        if(1 == eint_accdet_sync_flag) {
					
					
		        	
                	ACCDET_DEBUG("[Accdet]HOOK_SWITCH state not change!\n");
				}else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
			 	}
			 	mutex_unlock(&accdet_eint_irq_sync_mutex);
	    	}
            else if(current_status == 1)
            {
				mutex_lock(&accdet_eint_irq_sync_mutex);
		        if(1 == eint_accdet_sync_flag) {			
					multi_key_detection(current_status);
					accdet_status = MIC_BIAS;		
	        		cable_type = HEADSET_MIC;
				}else {
					ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
			 	}
			 	mutex_unlock(&accdet_eint_irq_sync_mutex);
				accdet_auxadc_switch(0);
				#ifdef ACCDET_PIN_RECOGNIZATION
				cable_pin_recognition = 0;
				ACCDET_DEBUG("[Accdet] cable_pin_recognition = %d\n", cable_pin_recognition);
				pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
                pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_thresh));
				#endif
		
         
                pmic_pwrap_write(ACCDET_DEBOUNCE0, button_press_debounce);
				
				
				
            }
            else
            {
                ACCDET_DEBUG("[Accdet]HOOK_SWITCH can't change to this state!\n"); 
            }
            break;			
	case STAND_BY:
			if(current_status == 3)
			{
                 #if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
						ACCDET_DEBUG("[Accdet]accdet do not send plug out event in stand by!\n");
		    	 #else
				 		mutex_lock(&accdet_eint_irq_sync_mutex);
		       			 if(1 == eint_accdet_sync_flag) {
							accdet_status = PLUG_OUT;		
							cable_type = NO_DEVICE;
						 }else {
							ACCDET_DEBUG("[Accdet] Headset has plugged out\n");
			 			 }
			 			mutex_unlock(&accdet_eint_irq_sync_mutex);
			    #endif
			 }
			 else
			{
					ACCDET_DEBUG("[Accdet]STAND_BY can't change to this state!\n"); 
			}
			break;
			
			default:
				ACCDET_DEBUG("[Accdet]check_cable_type: accdet current status error!\n");
			break;
						
}
			
		if(!IRQ_CLR_FLAG)
		{
			mutex_lock(&accdet_eint_irq_sync_mutex);
			if(1 == eint_accdet_sync_flag) {
				while((pmic_pwrap_read(ACCDET_IRQ_STS) & IRQ_STATUS_BIT) && (wait_clear_irq_times<3))
				{
				  ACCDET_DEBUG("[Accdet]check_cable_type: Clear interrupt on-going2....\n");
				  wait_clear_irq_times++;
				  msleep(5);
				}
			}
			irq_temp = pmic_pwrap_read(ACCDET_IRQ_STS);
			irq_temp = irq_temp & (~IRQ_CLR_BIT);
			pmic_pwrap_write(ACCDET_IRQ_STS, irq_temp);
			mutex_unlock(&accdet_eint_irq_sync_mutex);
			IRQ_CLR_FLAG = TRUE;
			ACCDET_DEBUG("[Accdet]check_cable_type:Clear interrupt:Done[0x%x]!\n", pmic_pwrap_read(ACCDET_IRQ_STS));	
			
		}
		else
		{
			IRQ_CLR_FLAG = FALSE;
		}

		ACCDET_DEBUG("[Accdet]cable type:[%s], status switch:[%s]->[%s]\n",
        accdet_report_string[cable_type], accdet_status_string[pre_status], 
        accdet_status_string[accdet_status]);
} 
static void accdet_work_callback(struct work_struct *work)
{

    wake_lock(&accdet_irq_lock);
    check_cable_type();
	
	mutex_lock(&accdet_eint_irq_sync_mutex);
    if(1 == eint_accdet_sync_flag) {
		switch_set_state((struct switch_dev *)&accdet_data, cable_type);
		
		if(hi) hi->hs_35mm_type = cable_type;
		
    }else {
		ACCDET_DEBUG("[Accdet] Headset has plugged out don't set accdet state\n");
		
		if(hi) hi->hs_35mm_type = HTC_HEADSET_UNPLUG;
		
	}
	mutex_unlock(&accdet_eint_irq_sync_mutex);
	ACCDET_DEBUG( " [accdet] set state in cable_type  status\n");

    wake_unlock(&accdet_irq_lock);
}

static inline void accdet_init(void)
{ 
	ACCDET_DEBUG("[Accdet]accdet hardware init\n");
    
    pmic_pwrap_write(TOP_CKPDN_CLR, RG_ACCDET_CLK_CLR);  
	ACCDET_DEBUG("[Accdet]accdet TOP_CKPDN=0x%x!\n", pmic_pwrap_read(TOP_CKPDN));	
    
	ACCDET_DEBUG("ACCDET reset : reset start!! \n\r");
	pmic_pwrap_write(TOP_RST_ACCDET_SET, ACCDET_RESET_SET);
	ACCDET_DEBUG("ACCDET reset function test: reset finished!! \n\r");
	pmic_pwrap_write(TOP_RST_ACCDET_CLR, ACCDET_RESET_CLR);
	
    pmic_pwrap_write(ACCDET_PWM_WIDTH, REGISTER_VALUE(cust_headset_settings->pwm_width));
    pmic_pwrap_write(ACCDET_PWM_THRESH, REGISTER_VALUE(cust_headset_settings->pwm_thresh));
	pmic_pwrap_write(ACCDET_STATE_SWCTRL, 0x07);
	
   	
	pmic_pwrap_write(ACCDET_EN_DELAY_NUM,
		(cust_headset_settings->fall_delay << 15 | cust_headset_settings->rise_delay));
    
   #ifdef ACCDET_PIN_RECOGNIZATION
    pmic_pwrap_write(ACCDET_DEBOUNCE0, cust_headset_settings->debounce0);
    pmic_pwrap_write(ACCDET_DEBOUNCE1, 0xFFFF);
    pmic_pwrap_write(ACCDET_DEBOUNCE3, cust_headset_settings->debounce3);	
	pmic_pwrap_write(ACCDET_DEBOUNCE4, ACCDET_DE4);	
   #else
    pmic_pwrap_write(ACCDET_DEBOUNCE0, cust_headset_settings->debounce0);
    pmic_pwrap_write(ACCDET_DEBOUNCE1, cust_headset_settings->debounce1);
    pmic_pwrap_write(ACCDET_DEBOUNCE3, cust_headset_settings->debounce3);	
	pmic_pwrap_write(ACCDET_DEBOUNCE4, ACCDET_DE4);
   #endif
   
    pmic_pwrap_write(ACCDET_IRQ_STS, pmic_pwrap_read(ACCDET_IRQ_STS)&(~IRQ_CLR_BIT));
	pmic_pwrap_write(INT_CON_ACCDET_SET, RG_ACCDET_IRQ_SET);
   #ifdef ACCDET_EINT_IRQ
	pmic_pwrap_write(INT_CON_ACCDET_SET, RG_ACCDET_EINT_IRQ_SET);
   #endif
   #ifdef ACCDET_NEGV_IRQ
	pmic_pwrap_write(INT_CON_ACCDET_SET, RG_ACCDET_NEGV_IRQ_SET);
   #endif
   
   #ifndef ACCDET_WQHD
   	
    mt6331_upmu_set_rg_audmicbias1vref(ACCDET_MIC_VOL);
   #else
   	pmic_pwrap_write(ACCDET_ADC_REG, 0x068F);
   #endif
	pmic_pwrap_write(ACCDET_EINT_NV, ACCDET_BF_MOD);
	pmic_pwrap_write(ACCDET_RSV, 0x0290);    
   #ifdef ACCDET_EINT_IRQ
	pmic_pwrap_write(ACCDET_RSV, pmic_pwrap_read(ACCDET_RSV)|ACCDET_INPUT_MICP);
    pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_EINT_CON_EN);
   #endif
   #ifdef ACCDET_NEGV_IRQ
	pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_NEGV_DT_EN);
   #endif
   ACCDET_DEBUG(" ACCDET_ADC_REG =%x\n",pmic_pwrap_read(ACCDET_ADC_REG));
   ACCDET_DEBUG(" ACCDET_EINT_NV =%x\n",pmic_pwrap_read(ACCDET_EINT_NV));
   ACCDET_DEBUG(" ACCDET_RSV =%x\n",pmic_pwrap_read(ACCDET_RSV));
    
   #if defined ACCDET_EINT
    
	pre_state_swctrl = pmic_pwrap_read(ACCDET_STATE_SWCTRL);
    pmic_pwrap_write(ACCDET_CTRL, ACCDET_DISABLE);
    pmic_pwrap_write(ACCDET_STATE_SWCTRL, 0x0);
	pmic_pwrap_write(TOP_CKPDN_SET, RG_ACCDET_CLK_SET);
   #elif defined ACCDET_EINT_IRQ
    pmic_pwrap_write(ACCDET_EINT_CTL, pmic_pwrap_read(ACCDET_EINT_CTL)|EINT_IRQ_DE_IN);
    
	pre_state_swctrl = pmic_pwrap_read(ACCDET_STATE_SWCTRL);
    pmic_pwrap_write(ACCDET_CTRL, ACCDET_DISABLE);
	pmic_pwrap_write(ACCDET_CTRL, ACCDET_EINT_EN);
	pmic_pwrap_write(ACCDET_STATE_SWCTRL, pmic_pwrap_read(ACCDET_STATE_SWCTRL)&(~ACCDET_SWCTRL_EN));
    pmic_pwrap_write(ACCDET_STATE_SWCTRL, pmic_pwrap_read(ACCDET_STATE_SWCTRL)|ACCDET_EINT_PWM_EN);
   #else
    
   
    pmic_pwrap_write(ACCDET_CTRL, ACCDET_ENABLE); 
   #endif
   #ifdef ACCDET_NEGV_IRQ
   	pmic_pwrap_write(ACCDET_EINT_PWM_DELAY, pmic_pwrap_read(ACCDET_EINT_PWM_DELAY)&(~0x1F)); 
	pmic_pwrap_write(ACCDET_EINT_PWM_DELAY, pmic_pwrap_read(ACCDET_EINT_PWM_DELAY)|0x0F); 
   	pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)|ACCDET_NEGV_EN);
   #endif
   
	pmic_pwrap_write(ACCDET_AUXADC_AUTO_SPL, (pmic_pwrap_read(ACCDET_AUXADC_AUTO_SPL)|ACCDET_AUXADC_AUTO_SET));
   
#ifdef GPIO_FSA8049_PIN
    
#endif
#ifdef FSA8049_V_POWER
    hwPowerOn(FSA8049_V_POWER, VOL_2800, "ACCDET");
#endif

}
#if DEBUG_THREAD
static int dump_register(void)
{
   int i=0;
   for (i=0x077A; i<= 0x07AA; i+=2)
   {
     ACCDET_DEBUG(" ACCDET_BASE + %x=%x\n",i,pmic_pwrap_read(ACCDET_BASE + i));
   }

   ACCDET_DEBUG(" TOP_RST_ACCDET =%x\n",pmic_pwrap_read(TOP_RST_ACCDET));
   ACCDET_DEBUG(" INT_CON_ACCDET =%x\n",pmic_pwrap_read(INT_CON_ACCDET));
   ACCDET_DEBUG(" TOP_CKPDN =%x\n",pmic_pwrap_read(TOP_CKPDN));
   ACCDET_DEBUG(" ACCDET_ADC_REG =%x\n",pmic_pwrap_read(ACCDET_ADC_REG));
  #ifdef ACCDET_PIN_SWAP
   
  #endif
  return 0;
}

static ssize_t accdet_store_call_state(struct device_driver *ddri, const char *buf, size_t count)
{
	if (sscanf(buf, "%u", &call_status) != 1) {
			ACCDET_DEBUG("accdet: Invalid values\n");
			return -EINVAL;
	}

	switch(call_status)
    {
        case CALL_IDLE :
			ACCDET_DEBUG("[Accdet]accdet call: Idle state!\n");
     		break;
            
		case CALL_RINGING :
			
			ACCDET_DEBUG("[Accdet]accdet call: ringing state!\n");
			break;

		case CALL_ACTIVE :
			ACCDET_DEBUG("[Accdet]accdet call: active or hold state!\n");	
			ACCDET_DEBUG("[Accdet]accdet_ioctl : Button_Status=%d (state:%d)\n", button_status, accdet_data.state);	
			
			break;
            
		default:
   		    ACCDET_DEBUG("[Accdet]accdet call : Invalid values\n");
            break;
     }
	return count;
}


static ssize_t show_pin_recognition_state(struct device_driver *ddri, char *buf)
{
   #ifdef ACCDET_PIN_RECOGNIZATION
	ACCDET_DEBUG("ACCDET show_pin_recognition_state = %d\n", cable_pin_recognition);
	return sprintf(buf, "%u\n", cable_pin_recognition);
   #else
    return sprintf(buf, "%u\n", 0);
   #endif
}

static DRIVER_ATTR(accdet_pin_recognition,      0664, show_pin_recognition_state,  NULL);
static DRIVER_ATTR(accdet_call_state,      0664, NULL,         accdet_store_call_state);

static int g_start_debug_thread =0;
static struct task_struct *thread = NULL;
static int g_dump_register=0;
static int dbug_thread(void *unused) 
{
   while(g_start_debug_thread)
   	{
      if(g_dump_register)
	  {
	    dump_register();
		
      }

	  msleep(500);

   	}
   return 0;
}
static ssize_t store_accdet_start_debug_thread(struct device_driver *ddri, const char *buf, size_t count)
{
	
	unsigned int start_flag;
	int error;

	if (sscanf(buf, "%u", &start_flag) != 1) {
		ACCDET_DEBUG("accdet: Invalid values\n");
		return -EINVAL;
	}

	ACCDET_DEBUG("[Accdet] start flag =%d \n",start_flag);

	g_start_debug_thread = start_flag;

    if(1 == start_flag)
    {
	   thread = kthread_run(dbug_thread, 0, "ACCDET");
       if (IS_ERR(thread)) 
	   { 
          error = PTR_ERR(thread);
          ACCDET_DEBUG( " failed to create kernel thread: %d\n", error);
       }
    }

	return count;
}
static ssize_t store_accdet_set_headset_mode(struct device_driver *ddri, const char *buf, size_t count)
{

    unsigned int value;
	

	if (sscanf(buf, "%u", &value) != 1) {
		ACCDET_DEBUG("accdet: Invalid values\n");
		return -EINVAL;
	}

	ACCDET_DEBUG("[Accdet]store_accdet_set_headset_mode value =%d \n",value);

	return count;
}

static ssize_t store_accdet_dump_register(struct device_driver *ddri, const char *buf, size_t count)
{
    unsigned int value;

	if (sscanf(buf, "%u", &value) != 1) 
	{
		ACCDET_DEBUG("accdet: Invalid values\n");
		return -EINVAL;
	}

	g_dump_register = value;

	ACCDET_DEBUG("[Accdet]store_accdet_dump_register value =%d \n",value);

	return count;
}

static DRIVER_ATTR(dump_register,      S_IWUSR | S_IRUGO, NULL,         store_accdet_dump_register);

static DRIVER_ATTR(set_headset_mode,      S_IWUSR | S_IRUGO, NULL,         store_accdet_set_headset_mode);

static DRIVER_ATTR(start_debug,      S_IWUSR | S_IRUGO, NULL,         store_accdet_start_debug_thread);

static struct driver_attribute *accdet_attr_list[] = {
	&driver_attr_start_debug,        
	&driver_attr_set_headset_mode,
	&driver_attr_dump_register,
	&driver_attr_accdet_call_state,
	
	&driver_attr_accdet_pin_recognition,
	
};

static int accdet_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(accdet_attr_list)/sizeof(accdet_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, accdet_attr_list[idx])))
		{            
			ACCDET_DEBUG("driver_create_file (%s) = %d\n", accdet_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

#endif

int switch_send_event(unsigned int bit, int on)
{
	unsigned long state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&hi->sdev_h2w);
	state &= ~(bit);

	if (on)
		state |= bit;

	switch_set_state(&hi->sdev_h2w, state);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static void set_35mm_hw_state(int state)
{
	ACCDET_DEBUG();
}

static ssize_t headset_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int length = 0;
	char *state = NULL;

	ACCDET_DEBUG("%s \n", __func__);
	if (!hi) {
		ACCDET_DEBUG("%s: hi is NULL\n", __func__);
		state = "error";
		length = snprintf(buf, PAGE_SIZE, "%s\n", state);
		return length;
	}

	switch (hi->hs_35mm_type) {
	case HTC_HEADSET_UNPLUG:
		state = "headset_unplug";
		break;
	case HTC_HEADSET_NO_MIC:
		state = "headset_no_mic";
		break;
	case HTC_HEADSET_MIC:
		state = "headset_mic";
		break;
	case HTC_HEADSET_METRICO:
		state = "headset_metrico";
		break;
	case HTC_HEADSET_UNKNOWN_MIC:
		state = "headset_unknown_mic";
		break;
	case HTC_HEADSET_TV_OUT:
		state = "headset_tv_out";
		break;
	case HTC_HEADSET_UNSTABLE:
		state = "headset_unstable";
		break;
	case HTC_HEADSET_INDICATOR:
		state = "headset_indicator";
		break;
	case HTC_HEADSET_UART:
		state = "headset_uart";
		break;
	default:
		state = "error_state";
	}

	length = snprintf(buf, PAGE_SIZE, "%s\n", state);

	return length;
}

static ssize_t headset_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ACCDET_DEBUG("%s \n", __func__);
	return 0;
}

static DEVICE_HEADSET_ATTR(state, 0644, headset_state_show,
			   headset_state_store);

static ssize_t headset_simulate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ACCDET_DEBUG("%s \n", __func__);
	return snprintf(buf, PAGE_SIZE, "Command is not supported\n");
}

static ssize_t headset_simulate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int type = NO_DEVICE;

	ACCDET_DEBUG("%s \n", __func__);
	if (!hi) {
		ACCDET_DEBUG("%s: hi is NULL\n", __func__);
		return -1;
	}

	switch_set_state((struct switch_dev *)&accdet_data, type);
	if (strncmp(buf, "headset_unplug", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_unplug");
		hi->hs_35mm_type = HTC_HEADSET_UNPLUG;
		set_35mm_hw_state(0);
		return count;
	}
	set_35mm_hw_state(1);

	if (strncmp(buf, "headset_no_mic", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_no_mic");
		hi->hs_35mm_type = HTC_HEADSET_NO_MIC;
		type= HEADSET_NO_MIC;
	} else if (strncmp(buf, "headset_mic", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_mic");
		hi->hs_35mm_type = HTC_HEADSET_MIC;
		type= HEADSET_MIC;
	} else if (strncmp(buf, "headset_metrico", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_metrico");
		
		hi->hs_35mm_type = HTC_HEADSET_METRICO;
		type= HEADSET_NO_MIC;
	} else if (strncmp(buf, "headset_unknown_mic", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_unknown_mic");
		hi->hs_35mm_type = HTC_HEADSET_UNKNOWN_MIC;
		type= HEADSET_NO_MIC;
	} else if (strncmp(buf, "headset_tv_out", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_tv_out");
		
		hi->hs_35mm_type = HTC_HEADSET_TV_OUT;
		type = HEADSET_NO_MIC;
#if defined(CONFIG_FB_MSM_TVOUT) && defined(CONFIG_ARCH_MSM8X60)
		
#endif
	} else if (strncmp(buf, "headset_indicator", count - 1) == 0) {
		ACCDET_DEBUG("Headset simulation: headset_indicator");
		
		hi->hs_35mm_type = HTC_HEADSET_INDICATOR;
	} else {
		ACCDET_DEBUG("Invalid parameter");
		hi->hs_35mm_type = HTC_HEADSET_UNPLUG;
		return count;
	}
	switch_set_state((struct switch_dev *)&accdet_data, type);
	return count;
}

static DEVICE_HEADSET_ATTR(simulate, 0644, headset_simulate_show,
			   headset_simulate_store);

static ssize_t headset_mfg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int length = 0;
	char *state = NULL;

	ACCDET_DEBUG("[accdet] %s\n", __func__);
	if (!hi) {
		ACCDET_DEBUG("[accdet] %s: hi is NULL\n", __func__);
		state = "hi is NULL";
		length = snprintf(buf, PAGE_SIZE, "%s\n", state);
		return length;
	}

	if (hi->hs_mfg_mode) {
		state = "it is MFG rom";
	} else {
		state = "it is not MFG rom";
	}

	length = snprintf(buf, PAGE_SIZE, "%s\n", state);

	return length;
}

static ssize_t headset_mfg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int type = NO_DEVICE;

	ACCDET_DEBUG("[accdet] %s: buf %s\n", __func__, buf);
	if (!hi) {
		ACCDET_DEBUG("[accdet] %s: hi is NULL\n", __func__);
		return -1;
	}

	if (strncmp(buf, "headset_mfg_mode", count - 1) == 0) {
		ACCDET_DEBUG("[accdet] %s: headset_mfg_mode = 1", __func__);
		hi->hs_mfg_mode = true;
	} else {
		hi->hs_mfg_mode = false;
		ACCDET_DEBUG("Invalid parameter");
	}
	return count;
}

static DEVICE_HEADSET_ATTR(mfg, 0644, headset_mfg_show,
			   headset_mfg_store);

static int register_attributes(void)
{
	int ret = 0;

	ACCDET_DEBUG("[accdet] %s ++\n", __func__);
	if (!hi) {
		ACCDET_DEBUG("[accdet] %s: hi is NULL\n", __func__);
		return -1;
	}

	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		printk(KERN_ERR "[accdet] %s: err_create_class\n", __func__);
		goto err_create_class;
	}

	
	hi->headset_dev = device_create(hi->htc_accessory_class,
					NULL, 0, "%s", "headset");
	if (unlikely(IS_ERR(hi->headset_dev))) {
		ret = PTR_ERR(hi->headset_dev);
		hi->headset_dev = NULL;
		printk(KERN_ERR "[accdet] %s: err_create_headset_device\n", __func__);
		goto err_create_headset_device;
	}

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_state);
	if (ret) {
		printk(KERN_ERR "[accdet] %s: err_create_headset_state_device_file\n", __func__);
		goto err_create_headset_state_device_file;
	}

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_simulate);
	if (ret) {
		printk(KERN_ERR "[accdet] %s: err_create_headset_simulate_device_file\n", __func__);
		goto err_create_headset_simulate_device_file;
	}

	ret = device_create_file(hi->headset_dev, &dev_attr_headset_mfg);
	if (ret) {
		printk(KERN_ERR "[accdet] %s: err_create_headset_simulate_device_mfg\n", __func__);
		goto err_create_headset_mfg_device_file;
	}

#if 0
	
	hi->tty_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		printk(KERN_ERR "%s: err_create_tty_device\n", __func__);
		goto err_create_tty_device;
	}

	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret) {
		printk(KERN_ERR "%s: err_create_tty_device_file\n", __func__);
		goto err_create_tty_device_file;
	}

	
	hi->fm_dev = device_create(hi->htc_accessory_class,
				   NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		printk(KERN_ERR "%s: err_create_fm_device\n", __func__);
		goto err_create_fm_device;
	}

	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret) {
		printk(KERN_ERR "%s: err_create_fm_device_file\n", __func__);
		goto err_create_fm_device_file;
	}

	
	hi->debug_dev = device_create(hi->htc_accessory_class,
				      NULL, 0, "%s", "debug");
	if (unlikely(IS_ERR(hi->debug_dev))) {
		ret = PTR_ERR(hi->debug_dev);
		hi->debug_dev = NULL;
		printk(KERN_ERR "%s: err_create_debug_device\n", __func__);
		goto err_create_debug_device;
	}

	
	ret = device_create_file(hi->debug_dev, &dev_attr_debug);
	if (ret) {
		printk(KERN_ERR "%s: err_create_debug_device_file\n", __func__);
		goto err_create_debug_device_file;
	}

#endif
	ACCDET_DEBUG("[accdet] %s --\n", __func__);
	return 0;
#if 0
err_create_debug_device_file:
	device_unregister(hi->debug_dev);

err_create_debug_device:
	device_remove_file(hi->fm_dev, &dev_attr_fm);

err_create_fm_device_file:
	device_unregister(hi->fm_dev);

err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);

err_create_tty_device_file:
	device_unregister(hi->tty_dev);
#endif
err_create_headset_mfg_device_file:
	device_remove_file(hi->headset_dev, &dev_attr_headset_simulate);

err_create_headset_simulate_device_file:
	device_remove_file(hi->headset_dev, &dev_attr_headset_state);

err_create_headset_state_device_file:
	device_unregister(hi->headset_dev);

err_create_headset_device:
	class_destroy(hi->htc_accessory_class);

err_create_class:

	printk(KERN_ERR "[accdet] %s: create error %d \n", __func__, ret);
	return ret;
}

static void unregister_attributes(void)
{
	ACCDET_DEBUG("[accdet] %s ++\n", __func__);
	if (hi) {
#if 0
		device_remove_file(hi->debug_dev, &dev_attr_debug);
		device_unregister(hi->debug_dev);
		device_remove_file(hi->fm_dev, &dev_attr_fm);
		device_unregister(hi->fm_dev);
		device_remove_file(hi->tty_dev, &dev_attr_tty);
		device_unregister(hi->tty_dev);
#endif
		device_remove_file(hi->headset_dev, &dev_attr_headset_mfg);
		device_remove_file(hi->headset_dev, &dev_attr_headset_simulate);
		device_remove_file(hi->headset_dev, &dev_attr_headset_state);
		device_unregister(hi->headset_dev);
		class_destroy(hi->htc_accessory_class);
	} else {
		ACCDET_DEBUG("[accdet] %s: hi is NULL\n", __func__);
	}
	ACCDET_DEBUG("[accdet] %s --\n", __func__);
}

int mt_accdet_probe(void)	
{
	int ret = 0;
#ifdef SW_WORK_AROUND_ACCDET_REMOTE_BUTTON_ISSUE
     struct task_struct *keyEvent_thread = NULL;
	 int error=0;
#endif
#if DEBUG_THREAD
		 struct platform_driver accdet_driver_hal = accdet_driver_func();
#endif

	struct headset_key_custom* press_key_time = get_headset_key_custom_setting();

	ACCDET_DEBUG("[Accdet]accdet_probe begin!\n");

	
	
	
	
	accdet_data.name = "h2w";
	accdet_data.index = 0;
	accdet_data.state = NO_DEVICE;

	cust_headset_settings = get_cust_headset_settings();
	
	ret = switch_dev_register(&accdet_data);
	if(ret)
	{
		ACCDET_DEBUG("[Accdet]switch_dev_register returned:%d!\n", ret);
		return 1;
	}
		
	
	
	
	ret = alloc_chrdev_region(&accdet_devno, 0, 1, ACCDET_DEVNAME);
	if (ret)
	{
		ACCDET_DEBUG("[Accdet]alloc_chrdev_region: Get Major number error!\n");			
	} 
		
	accdet_cdev = cdev_alloc();
    accdet_cdev->owner = THIS_MODULE;
    accdet_cdev->ops = accdet_get_fops();
    ret = cdev_add(accdet_cdev, accdet_devno, 1);
	if(ret)
	{
		ACCDET_DEBUG("[Accdet]accdet error: cdev_add\n");
	}
	
	accdet_class = class_create(THIS_MODULE, ACCDET_DEVNAME);

    
	accdet_nor_device = device_create(accdet_class, NULL, accdet_devno, NULL, ACCDET_DEVNAME);  
	
	
	
	
	kpd_accdet_dev = input_allocate_device();
	if (!kpd_accdet_dev) 
	{
		ACCDET_DEBUG("[Accdet]kpd_accdet_dev : fail!\n");
		return -ENOMEM;
	}

	
	__set_bit(EV_KEY, kpd_accdet_dev->evbit);
	__set_bit(KEY_CALL, kpd_accdet_dev->keybit);
	__set_bit(KEY_ENDCALL, kpd_accdet_dev->keybit);
    __set_bit(KEY_NEXTSONG, kpd_accdet_dev->keybit);
    __set_bit(KEY_PREVIOUSSONG, kpd_accdet_dev->keybit);
    __set_bit(KEY_PLAYPAUSE, kpd_accdet_dev->keybit);
    __set_bit(KEY_STOPCD, kpd_accdet_dev->keybit);
	__set_bit(KEY_VOLUMEDOWN, kpd_accdet_dev->keybit);
    __set_bit(KEY_VOLUMEUP, kpd_accdet_dev->keybit);
	
	kpd_accdet_dev->id.bustype = BUS_HOST;
	kpd_accdet_dev->name = "ACCDET";
	if(input_register_device(kpd_accdet_dev))
	{
		ACCDET_DEBUG("[Accdet]kpd_accdet_dev register : fail!\n");
	}else
	{
		ACCDET_DEBUG("[Accdet]kpd_accdet_dev register : success!!\n");
	} 
	
	
	
	accdet_workqueue = create_singlethread_workqueue("accdet");
	INIT_WORK(&accdet_work, accdet_work_callback);

	button_wq = create_singlethread_workqueue("button");
    
	
	
	wake_lock_init(&accdet_suspend_lock, WAKE_LOCK_SUSPEND, "accdet wakelock");
	wake_lock_init(&accdet_irq_lock, WAKE_LOCK_SUSPEND, "accdet irq wakelock");
	wake_lock_init(&accdet_key_lock, WAKE_LOCK_SUSPEND, "accdet key wakelock");
	wake_lock_init(&accdet_timer_lock, WAKE_LOCK_SUSPEND, "accdet timer wakelock");
	wake_lock_init(&accdet_button_wake_lock, WAKE_LOCK_SUSPEND, "accdet button wakelock");
#if DEBUG_THREAD
 	 if((ret = accdet_create_attr(&accdet_driver_hal.driver))!=0)
	 {
		ACCDET_DEBUG("create attribute err = %d\n", ret);
	
	 }
#endif
 
	 long_press_time = press_key_time->headset_long_press_time;

	ACCDET_DEBUG("[Accdet]accdet_probe : ACCDET_INIT\n");  
	if (g_accdet_first == 1) 
	{	
		long_press_time_ns = (s64)long_press_time * NSEC_PER_MSEC;
		
		eint_accdet_sync_flag = 1;
		#ifdef ACCDET_EINT_IRQ
          accdet_eint_workqueue = create_singlethread_workqueue("accdet_eint");
	      INIT_WORK(&accdet_eint_work, accdet_eint_work_callback);
		  accdet_disable_workqueue = create_singlethread_workqueue("accdet_disable");
	      INIT_WORK(&accdet_disable_work, disable_micbias_callback);
        #endif
	    
		accdet_init();   
		queue_work(accdet_workqueue, &accdet_work); 
		#ifdef ACCDET_EINT
		  accdet_disable_workqueue = create_singlethread_workqueue("accdet_disable");
	      INIT_WORK(&accdet_disable_work, disable_micbias_callback);
          accdet_eint_workqueue = create_singlethread_workqueue("accdet_eint");
	      INIT_WORK(&accdet_eint_work, accdet_eint_work_callback);
	      accdet_setup_eint();
        #endif
		g_accdet_first = 0;
	}

	if (!hi) {
		hi = kzalloc(sizeof(struct htc_headset_info), GFP_KERNEL);
		if (!hi) {
			printk(KERN_ERR "[Accdet] %s: alloc hi failed\n", __func__);
		} else {
			
			hi->hs_35mm_type = HTC_HEADSET_UNPLUG;
			hi->hs_mfg_mode = false;
			mutex_init(&hi->mutex_lock);
		}
	}
	ret = register_attributes();
	if (ret)
		printk(KERN_ERR "[Accdet] %s: register_attributes error\n", __func__);
        ACCDET_DEBUG("[Accdet]accdet_probe done!\n");
	
	
		
	    return 0;
}

void mt_accdet_remove(void)	
{
	ACCDET_DEBUG("[Accdet]accdet_remove begin!\n");
	
	
	#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
	destroy_workqueue(accdet_eint_workqueue);
	#endif
	destroy_workqueue(accdet_workqueue);
	destroy_workqueue(button_wq);
	switch_dev_unregister(&accdet_data);
	device_del(accdet_nor_device);
	class_destroy(accdet_class);
	cdev_del(accdet_cdev);
	unregister_chrdev_region(accdet_devno,1);	
	input_unregister_device(kpd_accdet_dev);
	unregister_attributes();
	if (hi) {
		kfree(hi);
		hi = NULL;
	}
	ACCDET_DEBUG("[Accdet]accdet_remove Done!\n");

}

void mt_accdet_suspend(void)  
{
	

#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
	ACCDET_DEBUG("[Accdet] in suspend1: ACCDET_IRQ_STS = 0x%x\n", pmic_pwrap_read(ACCDET_IRQ_STS));
#else
#if 0
    
    if(call_status == 0)
    {
       pre_state_swctrl = accdet_get_swctrl();
       accdet_disable_hal();
       
       accdet_disable_clk(); 
    }
#endif 
	printk(KERN_DEBUG "[Accdet]accdet_suspend: ACCDET_CTRL=[0x%x], STATE=[0x%x]->[0x%x]\n", pmic_pwrap_read(ACCDET_CTRL), pre_state_swctrl, pmic_pwrap_read(ACCDET_STATE_SWCTRL));
#endif
}

void mt_accdet_resume(void) 
{
#if defined ACCDET_EINT || defined ACCDET_EINT_IRQ
	ACCDET_DEBUG("[Accdet] in resume1: ACCDET_IRQ_STS = 0x%x\n", pmic_pwrap_read(ACCDET_IRQ_STS));
#else
#if 0
	if(call_status == 0)
	{
       accdet_enable_hal(pre_state_swctrl);
	}
#endif
	printk(KERN_DEBUG "[Accdet]accdet_resume: ACCDET_CTRL=[0x%x], STATE_SWCTRL=[0x%x]\n", pmic_pwrap_read(ACCDET_CTRL), pmic_pwrap_read(ACCDET_STATE_SWCTRL));

#endif

}
#ifdef ACCDET_PIN_RECOGNIZATION	
struct timer_list accdet_disable_ipoh_timer;
static void mt_accdet_pm_disable(unsigned long a)
{
	if (cable_type == NO_DEVICE && eint_accdet_sync_flag ==0) {
		
		pre_state_swctrl = pmic_pwrap_read(ACCDET_STATE_SWCTRL);	
		pmic_pwrap_write(ACCDET_STATE_SWCTRL, 0);
    	#ifdef ACCDET_EINT
   		pmic_pwrap_write(ACCDET_CTRL, ACCDET_DISABLE);
		
    	pmic_pwrap_write(TOP_CKPDN_SET, RG_ACCDET_CLK_SET); 
   		#endif
   		#ifdef ACCDET_EINT_IRQ
   		pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)&(~(ACCDET_ENABLE)));
   		#endif
		printk("[Accdet]daccdet_pm_disable: disable!\n");
	}
	else
	{
		printk("[Accdet]daccdet_pm_disable: enable!\n");
	}
}
#endif
void mt_accdet_pm_restore_noirq(void)
{
	int current_status_restore = 0;
    printk("[Accdet]accdet_pm_restore_noirq start!\n");
	
    ACCDET_DEBUG("accdet: enable_accdet\n");
    
    pmic_pwrap_write(TOP_CKPDN_CLR, RG_ACCDET_CLK_CLR); 
  #ifdef ACCDET_EINT_IRQ
	pmic_pwrap_write(TOP_CKPDN_CLR, RG_ACCDET_EINT_IRQ_CLR); 
    pmic_pwrap_write(ACCDET_RSV, pmic_pwrap_read(ACCDET_RSV)|ACCDET_INPUT_MICP);
    pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_EINT_CON_EN);
	pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_EINT_CON_EN);
    pmic_pwrap_write(ACCDET_CTRL, ACCDET_EINT_EN);
  #endif
  #ifdef ACCDET_NEGV_IRQ
	pmic_pwrap_write(TOP_CKPDN_CLR, RG_ACCDET_NEGV_IRQ_CLR);
  	pmic_pwrap_write(ACCDET_EINT_NV, pmic_pwrap_read(ACCDET_EINT_NV)|ACCDET_NEGV_DT_EN);
   	pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)|ACCDET_NEGV_EN);
  #endif
    enable_accdet(ACCDET_SWCTRL_EN);
  	pmic_pwrap_write(ACCDET_STATE_SWCTRL, (pmic_pwrap_read(ACCDET_STATE_SWCTRL)|ACCDET_SWCTRL_IDLE_EN));
	
	eint_accdet_sync_flag = 1;
	current_status_restore = ((pmic_pwrap_read(ACCDET_STATE_RG) & 0xc0)>>6); 
		
	switch (current_status_restore) {
		case 0:     
			cable_type = HEADSET_NO_MIC;
			accdet_status = HOOK_SWITCH;
			break;
		case 1:     
			cable_type = HEADSET_MIC;
			accdet_status = MIC_BIAS;
			break;
		case 3:     
			cable_type = NO_DEVICE;
			accdet_status = PLUG_OUT;
			break;
		default:
			printk("[Accdet]accdet_pm_restore_noirq: accdet current status error!\n");
			break;
	}
	switch_set_state((struct switch_dev *)&accdet_data, cable_type);
	if (cable_type == NO_DEVICE) {
	#ifdef ACCDET_PIN_RECOGNIZATION	
		init_timer(&accdet_disable_ipoh_timer);
		accdet_disable_ipoh_timer.expires = jiffies + 3*HZ;
		accdet_disable_ipoh_timer.function = &mt_accdet_pm_disable;
		accdet_disable_ipoh_timer.data = ((unsigned long) 0 );
		add_timer(&accdet_disable_ipoh_timer);
		printk("[Accdet]enable! pm timer\n");	

    #else
		
		pre_state_swctrl = pmic_pwrap_read(ACCDET_STATE_SWCTRL);	
	   	pmic_pwrap_write(ACCDET_STATE_SWCTRL, 0);
    	#ifdef ACCDET_EINT
   		pmic_pwrap_write(ACCDET_CTRL, ACCDET_DISABLE);
		
    	pmic_pwrap_write(TOP_CKPDN_SET, RG_ACCDET_CLK_SET);
   		#endif
   		#ifdef ACCDET_EINT_IRQ
   		pmic_pwrap_write(ACCDET_CTRL, pmic_pwrap_read(ACCDET_CTRL)&(~(ACCDET_ENABLE)));
   		#endif
	#endif
	}

}
long mt_accdet_unlocked_ioctl(unsigned int cmd, unsigned long arg)
{
		
    switch(cmd)
    {
        case ACCDET_INIT :
     		break;
            
		case SET_CALL_STATE :
			call_status = (int)arg;
			ACCDET_DEBUG("[Accdet]accdet_ioctl : CALL_STATE=%d \n", call_status);
			break;

		case GET_BUTTON_STATUS :
			ACCDET_DEBUG("[Accdet]accdet_ioctl : Button_Status=%d (state:%d)\n", button_status, accdet_data.state);	
			return button_status;
            
		default:
   		    ACCDET_DEBUG("[Accdet]accdet_ioctl : default\n");
            break;
  }
  return 0;
}

