/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (c) 2015 Illes Pal Zoltan <illespal@gmail.com> fingerprint mod
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/input.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_input.h>
#else
#include "fpc1020_common.h"
#include "fpc1020_input.h"
#endif

#ifdef CONFIG_FINGERPRINT_MOD
// tbalden
#include <linux/input/max1187x.h>
#include <linux/input/fpc1020_inc.h>
#include <linux/config/fingerprint_mod.h>
#include <linux/CwMcuSensor.h>
#endif

#define FPC1020_KEY_FINGER_PRESENT	KEY_WAKEUP	

#define FPC1020_INPUT_POLL_TIME_MS	1000u

int fpc1020_input_task_report_key(fpc1020_data_t *fpc1020, int down, int up, int key)
{
    if (down)
    {
				input_report_key(fpc1020->input_dev,
						key, 0);
				input_report_key(fpc1020->input_dev,
						key, 1);

				input_sync(fpc1020->input_dev);
    }
    if (up)
    {
		input_report_key(fpc1020->input_dev,
				key, 0);
		input_sync(fpc1020->input_dev);
    }
}

#ifdef CONFIG_FINGERPRINT_MOD
//int fingerprint_screenoff = 1; // fingerprint tweaks on/off
//int longpress_mode = 1; // long press check for fingerprint reader to trigger events on/off
//int doublepress_mode = 1; // double press check for fingerprint reader to trigger events on/off
//int doublepress_menu = 1; // doublepress_menu == 1 -> trigger MENU on doublepress, == 0 -> trigger POWER key on doublepress

fpc1020_data_t * fpc1020_dev = 0;

static struct input_dev * power_onoff_pwrdev_input;
static DEFINE_MUTEX(pwrlock_input);

static int count_finger_down = 0;
static int count_finger_up = 0;
static unsigned long count_finger_down_start_time = 0;
static unsigned long count_finger_up_start_time = 0;
static unsigned long count_finger_down_last_time = 0;
static unsigned long count_finger_up_last_time = 0;

extern void power_onoff_setdev_input(struct input_dev * input_device) {
        power_onoff_pwrdev_input = input_device;
        return;
}
EXPORT_SYMBOL(power_onoff_setdev_input);

static void power_onoff_presspwr(struct work_struct * power_onoff_presspwr_work) {

        if (!mutex_trylock(&pwrlock_input))
            return;

        printk("sending event KEY_POWER 1\n");
//TODO  if (sleep_wake_vibration_time)
//      {
//              vibrate(sleep_wake_vibration_time * 5);
//      }
        input_event(power_onoff_pwrdev_input, EV_KEY, KEY_POWER, 1);
        input_sync(power_onoff_pwrdev_input);
        msleep(100);
        input_event(power_onoff_pwrdev_input, EV_KEY, KEY_POWER, 0);
        input_sync(power_onoff_pwrdev_input);
        msleep(100);

        mutex_unlock(&pwrlock_input);
        return;
}
static DECLARE_WORK(power_onoff_presspwr_input_work, power_onoff_presspwr);

void press_pwrtrigger_input(void) {
        schedule_work(&power_onoff_presspwr_input_work);
        return;
}

// finger up timer part

int interrupt_finger_up_timer = 0;
int fingersleep_waiting_for_second_up = 0;

// signal if doubletap is going on and doubletap with long second press is being watched in timer timeout branch...
int check_doubletap_long = 0;

static unsigned long DOUBLEPRESS_TIMEOUT(void) { 
    return 23 + (fingerprint_mod_opt_timeout * 2);
}

static void fpc1020_finger_up_timer(struct work_struct * fpc1020_finger_up_timer_work) {
	while (1)
	{
		unsigned long current_time = jiffies;
		if (interrupt_finger_up_timer == 1) 
		{
		    printk(KERN_ERR "tb [fingersleep] finger_up_timer interrupted \n");
		    break;
		}
		if (current_time - count_finger_up_start_time > DOUBLEPRESS_TIMEOUT())
		{
			fingersleep_waiting_for_second_up = 0;
			if (check_doubletap_long == 0) { // only press FINGERPRINT, if doubletap_long check is not in progress, because then this will trigger an unneeded button press, 
							// as that will cause a timeout here, waiting for doubletap long
				if (is_screen_on()) { // the finger up can timeout when interruption on Down timeout happens in msleep... ? check screen is on and only trigger FINGERPRINT if it's on
					printk(KERN_ERR "tb [fingersleep] finger_up_timer timed off - sync FINGERPRINT key! \n");
					fpc1020_input_task_report_key(fpc1020_dev,1,0, FPC1020_KEY_FINGER_PRESENT);
					fpc1020_input_task_report_key(fpc1020_dev,0,1, FPC1020_KEY_FINGER_PRESENT);
				}
			}
			break;
		}
		msleep(10);
	}
}
static DECLARE_WORK(fpc1020_finger_up_timer_work, fpc1020_finger_up_timer);

void finger_up_input(void) {
	interrupt_finger_up_timer = 0;
        schedule_work(&fpc1020_finger_up_timer_work);
        return;
}


// for skipping next up in normal part, in case of longtap key event
int skip_next_finger_up_event = 0;


// extralong press part
int interrupt_finger_down_extralong_timer = 0;
int extralong_timer_started = 1;

static void fpc1020_finger_down_extralong_timer(struct work_struct * fpc1020_finger_down_extralong_timer_work) {
	while (1)
	{
		unsigned long current_time = jiffies;
		if (interrupt_finger_down_extralong_timer == 1) 
		{
		    printk(KERN_ERR "tb [fingersleep] finger_down_extralong_timer interrupted, ending work! \n");
		    break;
		}
		if ( current_time - count_finger_down_start_time > (DOUBLEPRESS_TIMEOUT()*2) )
		{
			extralong_timer_started = 0; // 0 this out, so no key press will be triggered on Finger up
			skip_next_finger_up_event = 1;
			printk(KERN_ERR "tb [fingersleep] finger_down_extralong_timer timed off - long press - screen off\n");
			press_pwrtrigger_input();
			break;
		}
		msleep(10);
	}
}
static DECLARE_WORK(fpc1020_finger_down_extralong_timer_work, fpc1020_finger_down_extralong_timer);

void finger_down_extralong_input(void) {
	extralong_timer_started = 1;
	interrupt_finger_down_extralong_timer = 0;
        schedule_work(&fpc1020_finger_down_extralong_timer_work);
        return;
}


// long press part
int interrupt_finger_down_timer = 0;

static void fpc1020_finger_down_timer(struct work_struct * fpc1020_finger_down_timer_work) {
	while (1)
	{
		unsigned long current_time = jiffies;
		if (interrupt_finger_down_timer == 1) 
		{
		    printk(KERN_ERR "tb [fingersleep] finger_down_timer interrupted, ending work! \n");
		    check_doubletap_long = 0; // don't wait for this anymore, or it will block finger up timer timeout part press FINGERPRINT part
		    break;
		}
		if (current_time - count_finger_down_start_time > DOUBLEPRESS_TIMEOUT())
		{
			if (fingerprint_mod_opt_longtap_mode == FINGERPRINT_MOD_OPT_LONGTAP_MODE_KEY) {
				if (fingerprint_mod_opt_extralong_mode == 1) {
					printk("%s [fingersleep] long press timed out, starting Extra long timer... !\n", __func__);
					count_finger_down_start_time = jiffies;
					finger_down_extralong_input();
					// vibration feedback press
					fpc1020_input_task_report_key(fpc1020_dev,1,0,KEY_SCALE); // just for haptic
					fpc1020_input_task_report_key(fpc1020_dev,0,1,KEY_SCALE); // just for haptic
				} else {
					printk("%s [fingersleep] long press timed out, press HOME... !\n", __func__);
					interrupt_finger_up_timer = 1; // don't wait for finger up work timeout, as that would mean a FINGERPRINT press after this MENU
					skip_next_finger_up_event = 1;
					fpc1020_input_task_report_key(fpc1020_dev,1,0,KEY_HOME);
					fpc1020_input_task_report_key(fpc1020_dev,0,1,KEY_HOME);
				}
			} else {
				if (check_doubletap_long == 1) {
					printk(KERN_ERR "tb [fingersleep] finger_down_timer timed off - doubletap long press - press HOME\n");
					interrupt_finger_up_timer = 1; // don't wait for finger up work timeout, as that would mean a FINGERPRINT press after this MENU
					skip_next_finger_up_event = 1;
					fpc1020_input_task_report_key(fpc1020_dev,1,0,KEY_HOME);
					fpc1020_input_task_report_key(fpc1020_dev,0,1,KEY_HOME);
				} else {
					printk(KERN_ERR "tb [fingersleep] finger_down_timer timed off - long press - screen off\n");
					interrupt_finger_up_timer = 1; // don't wait for finger up work timeout, as that would mean a FINGERPRINT press after this MENU
					press_pwrtrigger_input();
				}
			}
			break;
		}
		msleep(10);
	}
}
static DECLARE_WORK(fpc1020_finger_down_timer_work, fpc1020_finger_down_timer);

void finger_down_input(void) {
	interrupt_finger_down_timer = 0;
	unsigned long current_time = jiffies;
	unsigned long delta = current_time - count_finger_up_start_time;
	// if time passed after last tap and down start is < timeout, and doubletap_long mode is on, then let's switch to Check_doubletap_long 1, so upon timeout, we can do button press...
	if (fingerprint_mod_opt_doubletaplong_mode == 1 && delta < DOUBLEPRESS_TIMEOUT()) {
		check_doubletap_long = 1;
	} else {
		check_doubletap_long = 0;
	}
        schedule_work(&fpc1020_finger_down_timer_work);
        return;
}


#endif


static int fpc1020_write_lpm_setup(fpc1020_data_t *fpc1020);

static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020);





int fpc1020_input_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = FPC1020_DEV_NAME;

		set_bit(EV_KEY,		fpc1020->input_dev->evbit);

		set_bit(FPC1020_KEY_FINGER_PRESENT, fpc1020->input_dev->keybit);
		set_bit(KEY_MENU, fpc1020->input_dev->keybit);
		set_bit(KEY_HOME, fpc1020->input_dev->keybit);
		set_bit(KEY_SCALE, fpc1020->input_dev->keybit);
		set_bit(KEY_F11, fpc1020->input_dev->keybit);

		error = input_register_device(fpc1020->input_dev);
	}

	if (error) {
		dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
		input_free_device(fpc1020->input_dev);
		fpc1020->input_dev = NULL;
	}

	return error;
}

void fpc1020_input_destroy(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	wake_lock_destroy(&fpc1020->report_wake_lock);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}


int fpc1020_input_enable(fpc1020_data_t *fpc1020, bool enabled)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input.enabled = enabled;

	return 0;
}


int fpc1020_input_task(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);
	printk("%s [fingersleep] input_task start... state %d \n", __func__, fpc1020->finger_state);

	int count_report = 0;
	while (!fpc1020->worker.stop_request && !error) {

		error = fpc1020_wait_finger_present_lpm(fpc1020);

		if (error == 0 && !fpc1020->file_opened) {

			wake_lock_timeout(&fpc1020->report_wake_lock,
				  msecs_to_jiffies(500));
			if(fpc1020->finger_state == FPC1020_FINGER_STATE_UP) {
				printk("%s [fingersleep] FINGER STATE IS UP !\n", __func__);
#ifdef CONFIG_FINGERPRINT_MOD
				if (!is_screen_on() && get_proximity_adc() > 0) {
					// screen is off, do nothin, proximity sensor covered
				} else
				if (fingerprint_mod_enabled == 0 || (fingerprint_mod_opt_doubletap_mode == 0 && fingerprint_mod_opt_longtap_mode == 0) || !is_screen_on()) {
				    if (fingerprint_mod_enabled == 1) {
					    fpc1020_input_task_report_key(fpc1020,1,0,KEY_F11); // F11 should mapped in keylayout to FINGERPRINT always, to let HTC setting for Fingerprint wake on/off work, 
												// as we remap KEY_WAKE to BACK and such and that wakes device
				    } else {
#endif
				    fpc1020_input_task_report_key(fpc1020,1,0,FPC1020_KEY_FINGER_PRESENT);
#ifdef CONFIG_FINGERPRINT_MOD
				    }
				} else 
				{
					fpc1020_input_task_report_key(fpc1020,1,0,KEY_SCALE); // just for haptic
					fpc1020_input_task_report_key(fpc1020,0,1,KEY_SCALE); // just for haptic
					count_finger_down_start_time = jiffies;
					if (fingerprint_mod_opt_longtap_mode >= 1) {
						fpc1020_dev = fpc1020;
						finger_down_input();
					}
					count_finger_down++;
					printk("%s [fingersleep] hiding Down event sync !\n", __func__);
				}
				printk("%s [fingersleep] setting finger_state DOWN (down part)!\n", __func__);
			if (is_screen_on() || !is_screen_on() && get_proximity_adc() == 0)
#endif
				fpc1020->finger_state = FPC1020_FINGER_STATE_DOWN;

#ifdef DEBUG
				printk("%s Report Finger Down input key !\n", __func__);
#endif
			}
		}
		if(fpc1020->finger_state == FPC1020_FINGER_STATE_DOWN)
			msleep(FPC1020_INPUT_WAIT_FINGER_DELAY_MS);

	}
	if(fpc1020->finger_state == FPC1020_FINGER_STATE_DOWN) {
		printk("%s [fingersleep] FINGER STATE IS DOWN !\n", __func__);
		wake_lock_timeout(&fpc1020->report_wake_lock,
                                  msecs_to_jiffies(500));
#ifdef CONFIG_FINGERPRINT_MOD
		if (fingerprint_mod_enabled == 0 || (fingerprint_mod_opt_doubletap_mode == 0 && fingerprint_mod_opt_longtap_mode == 0) || !is_screen_on()) {
		    if (fingerprint_mod_enabled == 1) {
			    fpc1020_input_task_report_key(fpc1020,0,1,KEY_F11); // F11 mapped in mod keylayout to FINGERPRINT always, to let HTC setting for Fingerprint wake on/off work, 
										// as we remap KEY_WAKE to BACK and such and that wakes device
		    } else {
#endif
		    fpc1020_input_task_report_key(fpc1020,0,1,FPC1020_KEY_FINGER_PRESENT);
#ifdef CONFIG_FINGERPRINT_MOD
		    }
		} else {
			if (fingerprint_mod_opt_longtap_mode >= 1) {
				// long press sleep mode branch
				interrupt_finger_down_timer = 1; // break worker, and there it will press input/sync it, if it didnt already timed out and turned screen off.
				interrupt_finger_down_extralong_timer = 1; // break extralong worker too
				if (extralong_timer_started == 0 && fingerprint_mod_opt_doubletap_mode == 0)
				{ // if no doublepress mode check needed, trigger FINGERPRINT key event
					printk("%s [fingersleep] long press timeout: Fingerprint up !\n", __func__);
					fpc1020_input_task_report_key(fpc1020,1,0,FPC1020_KEY_FINGER_PRESENT);
					fpc1020_input_task_report_key(fpc1020,0,1,FPC1020_KEY_FINGER_PRESENT);
				}
				if (extralong_timer_started == 1) {
					if (fingerprint_mod_opt_longtap_mode == FINGERPRINT_MOD_OPT_LONGTAP_MODE_KEY) {
						printk("%s [fingersleep] extra long press interrupted: Longtap in key mode, triggering press HOME !\n", __func__);
						skip_next_finger_up_event = 1;
						fpc1020_input_task_report_key(fpc1020,1,0,KEY_HOME);
						fpc1020_input_task_report_key(fpc1020,0,1,KEY_HOME);
					}
				}
			}
			if (extralong_timer_started == 0 && fingerprint_mod_opt_doubletap_mode >= 1)
			{
				// double press sleep mode branch
				printk("%s [fingersleep] hiding Up event sync !\n", __func__);
				count_finger_up++;
				count_finger_up_last_time = count_finger_up_start_time;
				count_finger_up_start_time = jiffies;
				unsigned long delta = count_finger_up_start_time - count_finger_up_last_time;
				int start_timer = 1;
				printk("%s [fingersleep] between two Ups delta = %lu !\n", __func__, delta);
				if (fingersleep_waiting_for_second_up == 1 && delta < DOUBLEPRESS_TIMEOUT())
				{
					printk("%s [fingersleep] Two ups in row in time, trigger set event!\n", __func__);
					interrupt_finger_up_timer = 1;
					fingersleep_waiting_for_second_up = 0;
					start_timer = 0; // don't start timer, double press happened just now
					if (fingerprint_mod_opt_doubletap_mode == FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_KEY) {
						// double press represents KEY_MENU
						printk("%s [fingersleep] double press: menu !\n", __func__);
						fpc1020_input_task_report_key(fpc1020,1,0,KEY_MENU);
						fpc1020_input_task_report_key(fpc1020,0,1,KEY_MENU);
					} else {
						// double press represents POWER key
						printk("%s [fingersleep] double press: power !\n", __func__);
						press_pwrtrigger_input();
					}
				}
				if (start_timer == 1 && skip_next_finger_up_event == 0) {
					// no doubletap happened and successful long press case (check down timer timeout code) 
					// did not indicate to pass next finger up state
					printk("%s [fingersleep] start to wait for second up, starting work !\n", __func__);
					fingersleep_waiting_for_second_up = 1;
					fpc1020_dev = fpc1020;
					finger_up_input(); // start counting work
				}
			}
		}
		// reset signal of skip_next_finger_up_event
		skip_next_finger_up_event = 0;
		// reset extralong timer started state, finger is up
		extralong_timer_started = 0;
		printk("%s [fingersleep] setting finger_state UP !\n", __func__);
#endif
		fpc1020->finger_state = FPC1020_FINGER_STATE_UP;
		error = 0;
#ifdef DEBUG
		printk("%s Report Finger Up input key !\n", __func__);
#endif
	}

	error = fpc1020_wake_up(fpc1020);

	if (!error)
		error = fpc1020_calc_finger_detect_threshold_min(fpc1020);

	if (error >= 0)
		error = fpc1020_set_finger_detect_threshold(fpc1020, error);

	if (error >= 0) 
		error = fpc1020_write_lpm_setup(fpc1020);

	error = fpc1020_sleep(fpc1020,false);

	if (error == -EAGAIN) {
		error = fpc1020_read_irq(fpc1020, true);
		error = fpc1020_sleep(fpc1020, false);
	}
	if(error != 0) {
		printk("%s failed",__func__);
	}

	return error;
}


static int fpc1020_write_lpm_setup(fpc1020_data_t *fpc1020)
{
	const int mux = FPC1020_MAX_ADC_SETTINGS - 1;
	int error = 0;
	u16 temp_u16;
	fpc1020_reg_access_t reg;

	dev_dbg(&fpc1020->spi->dev, "%s %d\n", __func__, mux);

	error = fpc1020_write_sensor_setup(fpc1020);
	if(error)
		goto out;

	temp_u16 = fpc1020->setup.adc_shift[mux];
	temp_u16 <<= 8;
	temp_u16 |= fpc1020->setup.adc_gain[mux];

	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_ADC_SHIFT_GAIN, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

	temp_u16 = fpc1020->setup.pxl_ctrl[mux];
	FPC1020_MK_REG_WRITE(reg, FPC102X_REG_PXL_CTRL, &temp_u16);
	error = fpc1020_reg_access(fpc1020, &reg);
	if (error)
		goto out;

out:
	return error;
}


static int fpc1020_wait_finger_present_lpm(fpc1020_data_t *fpc1020)
{
	const int lpm_poll_delay_ms = FPC1020_INPUT_POLL_TIME_MS;
	const int zmask_5 = 1 << 5;
	const int zmask_6 = 1 << 6;
	const int zmask_ext = FPC1020_FINGER_DETECT_ZONE_MASK;

	int error = 0;
	int zone_raw = 0;

	bool wakeup_center = false;
	bool wakeup_ext    = false;
	bool wakeup        = false;

#ifdef HTC_WAKEUP 

	if(fpc1020->finger_state == FPC1020_FINGER_STATE_UP) {

		error = fpc1020_wake_up(fpc1020);

		if (!error)
			error = fpc1020_calc_finger_detect_threshold_min(fpc1020);

		if (error >= 0)
			error = fpc1020_set_finger_detect_threshold(fpc1020, error);

		if (error >= 0)
			error = fpc1020_write_lpm_setup(fpc1020);

		if (!error) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN) {
				error = fpc1020_sleep(fpc1020, false);
				if(error == -EAGAIN)
					error = 0;
			}

		}

	}
#else

	error = fpc1020_wake_up(fpc1020);

	if (!error)
		error = fpc1020_calc_finger_detect_threshold_min(fpc1020);

	if (error >= 0)
		error = fpc1020_set_finger_detect_threshold(fpc1020, error);

	if (error >= 0)
		error = fpc1020_write_lpm_setup(fpc1020);

	if (!error) {
		error = fpc1020_sleep(fpc1020, false);

		if (error == -EAGAIN) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN)
				error = 0;
		}
	}
#endif
	while (!fpc1020->worker.stop_request && !error && !wakeup) {

#ifdef HTC_WAKEUP
		error = fpc1020_check_finger_present_sum(fpc1020);
		if (error == 0 ){
                        wakeup = true;
			printk("%s Finger present sum == %d !\n", __func__, error);
			return RET_FINGER_UP;
		}else if (error > fpc1020->setup.input_finger_down_threshold) {
			error = 0;
			wakeup = true;
			return error;
		}
#endif

		if (!error)
			error = fpc1020_wait_finger_present(fpc1020);
#ifdef HTC_WAKEUP 
		if (!error)
			error = fpc1020_check_finger_present_sum(fpc1020);

		if (error > fpc1020->setup.input_finger_down_threshold) {
			error = 0;
			wakeup = true;
			printk("%s Wake up !\n", __func__);
		} else if (error > 0){
			printk("%s Finger present sum %d <= Threshold %d !\n", __func__, error, fpc1020->setup.input_finger_down_threshold);
			error = 0;
		} else if (fpc1020->worker.req_mode == 2 && (error == 0 || error == -ETIMEDOUT)){
			wakeup = true;
			error = 0;
			printk("%s Finger present sum == %d !\n", __func__, error);
			return RET_FINGER_UP;
		}

#else
		if (!error)
			error = fpc1020_check_finger_present_raw(fpc1020);

		zone_raw = (error >= 0) ? error : 0;

		if (error >= 0) {
			error = 0;

			wakeup_center = (zone_raw & zmask_5) ||
					(zone_raw & zmask_6);

			
			wakeup_ext = ((zone_raw & zmask_ext) == zmask_ext);

		} else {
			wakeup_center =
			wakeup_ext    = false;
		}

		if (wakeup_center && wakeup_ext) {
			dev_dbg(&fpc1020->spi->dev,
				"%s Wake up !\n", __func__);
			wakeup = true;
		}

		if (!wakeup && !error) {
			error = fpc1020_sleep(fpc1020, false);

			if (error == -EAGAIN)
				error = 0;

			if (!error)
				msleep(lpm_poll_delay_ms);
		}
#endif
	}

	if (error < 0)
		dev_dbg(&fpc1020->spi->dev,
			"%s %s %d!\n", __func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);

	return error;
}



