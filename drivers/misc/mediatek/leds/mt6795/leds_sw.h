#include <cust_leds.h>
#include <cust_leds_def.h>

#define NLED_OFF 0
#define NLED_ON 1
#define NLED_BLINK 2
#define MIN_FRE_OLD_PWM 32	
#define PWM_DIV_NUM 8
#define ERROR_BL_LEVEL 0xFFFFFFFF

struct nled_setting {
	u8 nled_mode;		
	u32 blink_on_time;
	u32 blink_off_time;
};

typedef enum {
	PMIC_PWM_0 = 0,
	PMIC_PWM_1 = 1,
	PMIC_PWM_2 = 2
} MT65XX_PMIC_PWM_NUMBER;

typedef enum {
	ISINK_0 = 0,		
	ISINK_1 = 1,		
	ISINK_2 = 2,		
	ISINK_3 = 3,		
	ISINK_4 = 4,		
	ISINK_5 = 5		
} MT65XX_PMIC_ISINK_STEP;

typedef enum {
	
	ISINK_1KHZ = 0,
	ISINK_200HZ = 4,
	ISINK_5HZ = 199,
	ISINK_2HZ = 499,
	ISINK_1HZ = 999,
	ISINK_05HZ = 1999,
	ISINK_02HZ = 4999,
	ISINK_01HZ = 9999,
	
	ISINK_2M_20KHZ = 2,
	ISINK_2M_1KHZ = 61,
	ISINK_2M_200HZ = 311,
	ISINK_2M_5HZ = 12499,
	ISINK_2M_2HZ = 31249,
	ISINK_2M_1HZ = 62499
} MT65XX_PMIC_ISINK_FSEL;

typedef enum {
	ISINK_PWM_MODE = 0,
	ISINK_BREATH_MODE = 1,
	ISINK_REGISTER_MODE = 2
} MT65XX_PMIC_ISINK_MODE;


struct mt65xx_led_data {
	struct led_classdev cdev;
	struct cust_mt65xx_led cust;
	struct work_struct work;
	struct work_struct blink_work;
	struct workqueue_struct *led_wq;
	int level;
	int delay_on;
	int delay_off;
};
