/*
 * Copyright (C) 2006-2015 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Steward Fu
 * Maintain: Luca Hsu, Tigers Huang
 */
 
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/proc_fs.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,8)
	#include <linux/input/mt.h>
#endif
#include "ilitek.h"

#ifdef WATCHDOG_ENABLE
#include <linux/workqueue.h>
#endif 

//module information
MODULE_AUTHOR("Steward_Fu");
MODULE_DESCRIPTION("ILITEK I2C touchscreen driver for linux platform");
MODULE_LICENSE("GPL");

int _driver_information[] = {DERVER_VERSION_MAJOR, DERVER_VERSION_MINOR, RELEASE_VERSION};

int touch_key_hold_press = 0;
//int touch_key_code[] = {KEY_MENU, KEY_HOME/*KEY_HOMEPAGE*/, KEY_BACK, KEY_VOLUMEDOWN, KEY_VOLUMEUP};
int touch_key_code[] = {KEY_CONFIG, KEY_PHONE, KEY_MENU, KEY_BACK, KEY_F13};


int touch_key_press[] = {0, 0, 0, 0, 0};
unsigned long touch_time = 0;

static int ilitek_i2c_register_device(void);
static void ilitek_set_input_param(struct input_dev*, int, int, int);
static void ilitek_set_input2_param(struct input_dev*); 	//hsguy.son (add)
static int ilitek_i2c_read_tp_info(void);
static int ilitek_init(void);
static void ilitek_exit(void);

//i2c functions
static int ilitek_i2c_transfer(struct i2c_client*, struct i2c_msg*, int);
static int ilitek_i2c_write_read(struct i2c_client *client, uint8_t *cmd,int write_len ,int delay, uint8_t *data, int read_len);
static int ilitek_i2c_process_and_report(void);
static int ilitek_i2c_suspend(struct device *dev);
static int ilitek_i2c_resume(struct device *dev);
static void ilitek_i2c_shutdown(struct i2c_client*);
static int ilitek_i2c_probe(struct i2c_client*, const struct i2c_device_id*);
static int ilitek_i2c_remove(struct i2c_client*);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_early_suspend(struct early_suspend *h);
static void ilitek_i2c_late_resume(struct early_suspend *h);
#endif

static irqreturn_t ilitek_i2c_isr(int, void*);
static void ilitek_i2c_irq_work_queue_func(struct work_struct*);

//file operation functions
static int ilitek_file_open(struct inode*, struct file*);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static ssize_t ilitek_file_write(struct file*, const char*, size_t, loff_t*);
static ssize_t ilitek_file_read(struct file*, char*, size_t, loff_t*);
static int ilitek_file_close(struct inode*, struct file*);

static void ilitek_i2c_irq_enable(void);
static void ilitek_i2c_irq_disable(void);

static int ilitek_i2c_reset(void);
static int ilitek_i2c_palm(struct i2c_client *client, uint8_t flag);

//global variables
static struct i2c_data i2c;
static struct dev_data dev;
static char DBG_FLAG;
static char Report_Flag;
volatile static char int_Flag;
volatile static char update_Flag;
static int update_timeout;
int gesture_flag, gesture_count, getstatus, watchdog_flag;
static atomic_t ilitek_cmd_response = ATOMIC_INIT(0);

#ifdef USE_ILITEK_SYSFS
static struct class *ilitek_class = NULL; // jhee.jeong (add)
//static struct ilitek_dbg ilitekdbg; 
#endif 

//i2c id table
static const struct i2c_device_id ilitek_i2c_id[] = {
	{ILITEK_I2C_DRIVER_NAME, 0}, {}
};
MODULE_DEVICE_TABLE(i2c, ilitek_i2c_id);

#ifdef CONFIG_OF //jhee.jeong(add)
static struct of_device_id ilitek_dt_ids[] = {
	{ .compatible = "ilitek,i2c"},
	{ }
};
#endif

static SIMPLE_DEV_PM_OPS(ilitek_i2c_pm,
			 ilitek_i2c_suspend, ilitek_i2c_resume);

//declare i2c function table
static struct i2c_driver ilitek_i2c_driver = {
	.id_table = ilitek_i2c_id,
	.driver = {
		.name = ILITEK_I2C_DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ilitek_dt_ids),	
#endif
		.pm = &ilitek_i2c_pm,
	},
	.shutdown = ilitek_i2c_shutdown,
	.probe = ilitek_i2c_probe,
	.remove = ilitek_i2c_remove,
};

//declare file operations
struct file_operations ilitek_fops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	.unlocked_ioctl = ilitek_file_ioctl,
#else
	.ioctl = ilitek_file_ioctl,
#endif
	.read = ilitek_file_read,
	.write = ilitek_file_write,
	.open = ilitek_file_open,
	.release = ilitek_file_close,
};

/*
description
	open function for character device driver
prarmeters
	inode
	    inode
	filp
	    file pointer
return
	status
*/
static int ilitek_file_open(struct inode *inode, struct file *filp)
{
	DBG("%s\n", __func__);
	return 0; 
}

/*
description
	write function for character device driver
prarmeters
	filp
		file pointer
	buf
		buffer
	count
		buffer length
	f_pos
		offset
return
	status
*/
static ssize_t ilitek_file_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	int ret;

	unsigned char buffer[128] = {0};
        
	//before sending data to touch device, we need to check whether the device is working or not
	if(i2c.valid_i2c_register == 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c device driver doesn't be registered\n", __func__);
		return -1;
	}

	//check the buffer size whether it exceeds the local buffer size or not
	if(count > 128)
	{
		printk(ILITEK_ERROR_LEVEL "%s, buffer exceed 128 bytes\n", __func__);
		return -1;
	}

	//copy data from user space
	ret = copy_from_user(buffer, buf, count - 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed", __func__);
		return -1;
	}

	//parsing command
	if(strcmp(buffer, "dbg") == 0)
	{
		DBG_FLAG = !DBG_FLAG;
		printk(ILITEK_DEBUG_LEVEL "%s, %s DBG message(%X).\n", __func__, DBG_FLAG ? "Enabled" : "Disabled", DBG_FLAG);
	}
	else if(strcmp(buffer, "info") == 0)
	{
		ilitek_i2c_read_tp_info();
	}
	else if(strcmp(buffer, "report") == 0)
	{
		Report_Flag =! Report_Flag;
	}
	else if(strcmp(buffer, "stop_report") == 0)
	{
		i2c.report_status = 0;
		printk(ILITEK_DEBUG_LEVEL "The report point function is disable.\n");
	}
	else if(strcmp(buffer, "start_report") == 0)
	{
		i2c.report_status = 1;
		printk(ILITEK_DEBUG_LEVEL "The report point function is enable.\n");
	}
	else if(strcmp(buffer, "update_flag") == 0)
	{
		printk(ILITEK_DEBUG_LEVEL "update_Flag=%d\n", update_Flag);
	}
	else if(strcmp(buffer, "irq_status") == 0)
	{
		printk(ILITEK_DEBUG_LEVEL "i2c.irq_status = %d\n", i2c.irq_status);
	}
	else if(strcmp(buffer, "disable_irq") == 0)
	{
		ilitek_i2c_irq_disable();
		printk(ILITEK_DEBUG_LEVEL "i2c.irq_status = %d\n", i2c.irq_status);
	}
	else if(strcmp(buffer, "enable_irq") == 0)
	{
		ilitek_i2c_irq_enable();
		printk(ILITEK_DEBUG_LEVEL "i2c.irq_status=%d\n", i2c.irq_status);
	}
	else if(strcmp(buffer, "reset") == 0)
	{
		printk(ILITEK_DEBUG_LEVEL "start reset\n");
		ilitek_i2c_reset();
		printk(ILITEK_DEBUG_LEVEL "end reset\n");
	}
	return -1;
}

/*
description
	ioctl function for character device driver
prarmeters
	inode
		file node
	filp
		file pointer
	cmd
		command
	arg
		arguments
return
	status
*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	static unsigned char buffer[64] = {0};
	static int len = 0, i;
	int ret;
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = len, .buf = buffer,}
	};

	//parsing ioctl command
	switch(cmd)
	{
		case ILITEK_IOCTL_I2C_WRITE_DATA:
			ret = copy_from_user(buffer, (unsigned char*)arg, len);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
				return -1;
			}
		#ifdef	SET_RESET
			if(buffer[0] == 0x60)
			{
				ilitek_i2c_reset();
			}
		#endif
			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
				return -1;
			}
			break;
		case ILITEK_IOCTL_I2C_READ_DATA:
			msgs[0].flags = I2C_M_RD;
			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, i2c read, failed\n", __func__);
				return -1;
			}
			ret = copy_to_user((unsigned char*)arg, buffer, len);
			
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
			break;
		case ILITEK_IOCTL_I2C_WRITE_LENGTH:
		case ILITEK_IOCTL_I2C_READ_LENGTH:
			len = arg;
			break;
		case ILITEK_IOCTL_DRIVER_INFORMATION:
			for(i = 0; i < 3; i++)
			{
				buffer[i] = _driver_information[i];
			}
			ret = copy_to_user((unsigned char*)arg, buffer, 3);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
			break;
		case ILITEK_IOCTL_I2C_UPDATE:
			break;
		case ILITEK_IOCTL_I2C_INT_FLAG:
			if(update_timeout == 1)
			{
				buffer[0] = int_Flag;
				ret = copy_to_user((unsigned char*)arg, buffer, 1);
				if(ret < 0)
				{
					printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
					return -1;
				}
			}
			else
			{
				update_timeout = 1;
			}
			break;
		case ILITEK_IOCTL_START_READ_DATA:
			i2c.stop_polling = 0;
			if(i2c.client->irq != 0)
			{
				ilitek_i2c_irq_enable();
			}
			i2c.report_status = 1;
			printk(ILITEK_DEBUG_LEVEL "The report point function is enable.\n");
			break;
		case ILITEK_IOCTL_STOP_READ_DATA:
			i2c.stop_polling = 1;
			if(i2c.client->irq != 0)
			{
				ilitek_i2c_irq_disable();
			}
			i2c.report_status = 0;
			printk(ILITEK_DEBUG_LEVEL "The report point function is disable.\n");
			break;
		case ILITEK_IOCTL_I2C_SWITCH_IRQ:
			ret = copy_from_user(buffer, (unsigned char*)arg, 1);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
			
			if(buffer[0] == 0)
			{
				if(i2c.client->irq != 0)
				{
					ilitek_i2c_irq_disable();
				}
			}
			else
			{
				if(i2c.client->irq != 0)
				{
					ilitek_i2c_irq_enable();				
				}
			}
			break;	
		case ILITEK_IOCTL_UPDATE_FLAG:
			update_timeout = 1;
			update_Flag = arg;
			DBG("%s, update_Flag = %d\n", __func__, update_Flag);
			break;
		case ILITEK_IOCTL_I2C_UPDATE_FW:
			ret = copy_from_user(buffer, (unsigned char*)arg, 35);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
				return -1;
			}
			msgs[0].len = buffer[34];
			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
				return -1;
			}
			
			int_Flag = 0;
			update_timeout = 0;
			
			break;
		default:
			return -1;
	}
    return 0;
}

/*
description
	read function for character device driver
prarmeters
	filp
	    file pointer
	buf
	    buffer
	count
	    buffer length
	f_pos
	    offset
return
	status
*/
static ssize_t ilitek_file_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

/*
description
	close function
prarmeters
	inode
	    inode
	filp
	    file pointer
return
	status
*/
static int ilitek_file_close(struct inode *inode, struct file *filp)
{
	DBG("%s\n", __func__);
	return 0;
}

/*
description
	set input device's parameter
prarmeters
	input
		input device data
	max_tp
		single touch or multi touch
	max_x
		maximum	x value
	max_y
		maximum y value
return
	nothing
*/
static void ilitek_set_input_param(struct input_dev *input, int max_tp, int max_x, int max_y)
{
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#ifndef ROTATE_FLAG
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_x + 2, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_y + 2, 0, 0);
#else
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, max_y + 2, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, max_x + 2, 0, 0);
#endif
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
#ifdef TOUCH_PROTOCOL_B
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	input_mt_init_slots(input,max_tp,INPUT_MT_DIRECT);
	#else
	input_mt_init_slots(input,max_tp);
	#endif
#else
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, max_tp, 0, 0);
#endif
	set_bit(INPUT_PROP_DIRECT, input->propbit);

	input->name = "sec_touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &(i2c.client)->dev;
}

//hsguy.son (add)
/*
description
	set input device's parameter
prarmeters
	input
		input device data
return
	nothing
*/
static void ilitek_set_input2_param(struct input_dev *input)
{
	int i = 0;
	input->name = "sec_touchkey";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &(i2c.client)->dev;

	set_bit(EV_ABS, input->evbit);
	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	for(i=0; i<(sizeof(touch_key_code)/sizeof(int)); i++)
	{
		if(touch_key_code[i] <= 0)
		{
			continue;
		}
		//set_bit(touch_key_code[i] & KEY_MAX, input->keybit);
		set_bit(touch_key_code[i], input->keybit);
		printk(ILITEK_DEBUG_LEVEL "touch_key_code %d : %d\n", i,  touch_key_code[i]);
	}
}
//hsguy.son (add_end)

/*
description
	send message to i2c adaptor
parameter
	client
		i2c client
	msgs
		i2c message
	cnt
		i2c message count
return
	>= 0 if success
	others if error
*/
static int ilitek_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
	int ret, count = ILITEK_I2C_RETRY_COUNT;
	while(count > 0)
	{
		count -= 1;
		down_interruptible(&i2c.wr_sem);
		ret = i2c_transfer(client->adapter, msgs, cnt);
		up(&i2c.wr_sem);
		if(ret < 0)
		{
			msleep(100);
			continue;
		}
		break;
	}
	return ret;
}

#ifdef SYSFS_SAMPLE_RATE
static int mxt_get_sampling_rate(void)
{
	bool sampling_rate = 0;
	sampling_rate = i2c.ilitekdbg.tsp_sampling_rate;

	return sampling_rate;
}

static void measure_sample_rate(struct sample_rate_struct *sample_rate, bool push)
{
	if(push)
	{
		if(sample_rate->touch_enable == 0)
		{
			sample_rate->jiffies_touch = jiffies;
			sample_rate->interrupt_count = 0;
		}
		else
		{
			sample_rate->interrupt_count++;
		}
	}

	else
	{
		if(sample_rate->touch_enable)
		{
			int msecs = jiffies_to_msecs(jiffies - sample_rate->jiffies_touch);
			int hz = msecs ? (1000 * sample_rate->interrupt_count / msecs) : 0;

			printk(ILITEK_DEBUG_LEVEL "Sampling_rate = %d[SPS]\n", hz);
			//printk(" interrupt_count : %d , time : %d \n\n",sample_rate->interrupt_count, msecs);
		}
	}

	sample_rate->touch_enable = push;
}
#endif


/*
description
	process i2c data and then report to kernel
parameters
	none
return
	status
*/
static int ilitek_i2c_process_and_report(void)
{
#ifdef ROTATE_FLAG
	int org_x = 0, org_y = 0;
#endif
	int i, ret = 0, x = 0, y = 0, packet = 0, tp_status = 0, j, release_flag[10] = {0};
#ifdef VIRTUAL_KEY_PAD
	unsigned char key_id = 0, key_flag = 1;
#endif
	struct input_dev *input = i2c.input_dev;
	struct input_dev *input2 = i2c.input_dev2;
    unsigned char buf[64] = {0};
	unsigned char max_point = 6;
	unsigned char release_counter = 0 ;
	struct timeval now;
	DBG("%s,enter\n",__func__);
	do_gettimeofday(&now); 
	if((long)(now.tv_sec * 1000 + now.tv_usec / 1000) - i2c.pre_sec_time > KEY_HOLD_TIME && i2c.touch_flag == 0)
	{
		if(i2c.ilitekdbg.tsp_print_coordinate)	
		printk(ILITEK_DEBUG_LEVEL "time:%lu\n",(long)(now.tv_sec * 1000 + now.tv_usec / 1000) - i2c.pre_sec_time);
		i2c.keyflag = 0;
	}
	if(i2c.report_status == 0)
	{
		return 1;
	} 

	//mutli-touch for protocol 3.1
	if((i2c.protocol_ver & 0x300) == 0x300)
	{
		buf[0] = ILITEK_TP_CMD_READ_DATA;
		ret = ilitek_i2c_write_read(i2c.client, buf, 1, 0, buf, 31);
		if(ret < 0)
		{
			return ret;
		}
		packet = buf[0];
		ret = 1;
		if(packet == 2)
		{
			ret = ilitek_i2c_write_read(i2c.client, buf, 0, 0, buf + 31, 20);
			if(ret < 0)
			{
				return ret;
			}
			max_point = 10;
		}
		DBG("max_point=%d\n", max_point);
		//read touch point
		for(i = 0; i < max_point; i++)
		{
			tp_status = buf[i * 5 + 1] >> 7;
			i2c.touchinfo[i].status = tp_status;
		#ifndef ROTATE_FLAG
			x = (((buf[i * 5 + 1] & 0x3F) << 8) + buf[i * 5 + 2]);
			y = (buf[i * 5 + 3] << 8) + buf[i * 5 + 4];
		#else
			org_x = (((buf[i * 5 + 1] & 0x3F) << 8) + buf[i * 5 + 2]);
			org_y = (buf[i * 5 + 3] << 8) + buf[i * 5 + 4];
			x = i2c.max_y - org_y + 1;
			y = org_x + 1;					
		#endif
			if(tp_status)
			{

//modify
#if 0
#ifdef TOUCH_PROTOCOL_B
				input_mt_slot(i2c.input_dev, i);
				input_mt_report_slot_state(i2c.input_dev, MT_TOOL_FINGER,true);
				i2c.touchinfo[i].flag = 1;
				#endif
				input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_X, x);
				input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_Y, y);
				input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);

#ifndef TOUCH_PROTOCOL_B
				input_event(i2c.input_dev, EV_ABS, ABS_MT_TRACKING_ID, i);
				input_report_key(i2c.input_dev, BTN_TOUCH,  1);
				input_mt_sync(i2c.input_dev);
#endif
				release_flag[i] = 1;
				//i2c.keyflag = 1;
				i2c.keyflag = 0;
				DBG("Point, ID=%02X, X=%04d, Y=%04d,release_flag[%d]=%d,tp_status=%d,keyflag=%d\n",i, x,y,i,release_flag[i],tp_status,i2c.keyflag);

#endif
//modify end
				if(i2c.keyflag == 0)
				{
					for(j = 0; j <= i2c.keycount; j++)
					{
						if((x >= i2c.keyinfo[j].x && x <= i2c.keyinfo[j].x + i2c.key_xlen) && (y >= i2c.keyinfo[j].y && y <= i2c.keyinfo[j].y + i2c.key_ylen))
						{
							input_report_key(input2,  touch_key_code[j], 1);
							i2c.keyinfo[j].status = 1;
							touch_key_hold_press = 1;
							release_flag[0] = 1;
							DBG("Key, Keydown ID=%d, X=%d, Y=%d, key_status=%d,keyflag=%d\n", touch_key_code[j] ,x ,y , i2c.keyinfo[j].status,i2c.keyflag);
							break;
						}
					}
				}
				if((touch_key_hold_press == 0) && (x < i2c.max_x) && (y < i2c.max_y))
				{	
					#ifdef TOUCH_PROTOCOL_B
						input_mt_slot(i2c.input_dev, i);
						input_mt_report_slot_state(i2c.input_dev, MT_TOOL_FINGER,true);
						i2c.touchinfo[i].flag = 1;
					#endif	
					input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_X, x);
					input_event(i2c.input_dev, EV_ABS, ABS_MT_POSITION_Y, y);
					input_event(i2c.input_dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);

					#ifndef TOUCH_PROTOCOL_B
						input_event(i2c.input_dev, EV_ABS, ABS_MT_TRACKING_ID, i);
						input_report_key(i2c.input_dev, BTN_TOUCH,  1);
						input_mt_sync(i2c.input_dev);
					#endif
					release_flag[i] = 1;
					i2c.keyflag = 1;
					i2c.touch_flag = 1;

					if(i2c.ilitekdbg.tsp_print_coordinate)
						printk(ILITEK_DEBUG_LEVEL "Point, ID=%02X, X=%04d, Y=%04d,release_flag[%d]=%d,tp_status=%d,keyflag=%d\n",i, x,y,i,release_flag[i],tp_status,i2c.keyflag);	
				}
				if(touch_key_hold_press == 1)
				{
					for(j = 0; j <= i2c.keycount; j++)
					{
						if((i2c.keyinfo[j].status == 1) && (x < i2c.keyinfo[j].x || x > i2c.keyinfo[j].x + i2c.key_xlen || y < i2c.keyinfo[j].y || y > i2c.keyinfo[j].y + i2c.key_ylen))
						{
							//input_report_key(input2,  i2c.keyinfo[j].id, 0);
							input_report_key(input2,  touch_key_code[j], 0);
							i2c.keyinfo[j].status = 0;
							touch_key_hold_press = 0;
							//DBG("Key, Keyout ID=%d, X=%d, Y=%d, key_status=%d\n", i2c.keyinfo[j].id ,x ,y , i2c.keyinfo[j].status);
							DBG("Key, Keyout ID=%d, X=%d, Y=%d, key_status=%d\n", touch_key_code[j] ,x ,y , i2c.keyinfo[j].status);
							break;
						}
					}
				}
					
				ret = 0;
			}
			else
			{
				release_flag[i] = 0;
				if(i2c.ilitekdbg.tsp_print_coordinate)	
					printk(ILITEK_DEBUG_LEVEL "Point, ID=%02X, X=%04d, Y=%04d,release_flag[%d]=%d,tp_status=%d\n",i, x,y,i,release_flag[i],tp_status);	
				#ifdef TOUCH_PROTOCOL_B
					if(i2c.touchinfo[i].flag == 1)
					{
						input_mt_slot(i2c.input_dev, i);
						input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
						i2c.touchinfo[i].flag = 0;
					}
				#else
				input_mt_sync(i2c.input_dev);
				#endif
			} 
				
		}
		if(packet == 0 )
		{
			DBG("packet=%d\n", packet);
			i2c.keyflag = 0;
			#ifdef TOUCH_PROTOCOL_B
				for(i = 0; i < i2c.max_tp; i++){
					if(i2c.touchinfo[i].flag == 1){
						input_mt_slot(i2c.input_dev, i);
						input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
					}
					i2c.touchinfo[i].flag = 0;
				}
			#else
				input_report_key(i2c.input_dev, BTN_TOUCH, 0);
				input_mt_sync(i2c.input_dev);
			#endif
		}
		else
		{
			for(i = 0; i < max_point; i++)
			{			
				if(release_flag[i] == 0)
				{
					release_counter++;
				}
			}
			if(release_counter == max_point)
			{
			#ifdef TOUCH_PROTOCOL_B
				for(i = 0; i < i2c.max_tp; i++){
					if(i2c.touchinfo[i].flag == 1){
						input_mt_slot(i2c.input_dev, i);
						input_mt_report_slot_state(input, MT_TOOL_FINGER,false);
					}
					i2c.touchinfo[i].flag = 0;
				}
			#else
				input_report_key(i2c.input_dev, BTN_TOUCH, 0);
				input_mt_sync(i2c.input_dev);
			#endif
				i2c.touch_flag = 0;
				i2c.pre_sec_time = (long)(now.tv_sec * 1000 + now.tv_usec / 1000);
				//i2c.keyflag = 0;
				if(touch_key_hold_press == 1)
				{
					for(i = 0; i < i2c.keycount; i++)
					{
						if(i2c.keyinfo[i].status)
						{
							//input_report_key(input2, i2c.keyinfo[i].id, 0);
							input_report_key(input2, touch_key_code[i], 0);
							i2c.keyinfo[i].status = 0;
							touch_key_hold_press = 0;
							//DBG("Key, Keyup ID=%d, X=%d, Y=%d, key_status=%d, touch_key_hold_press=%d\n", i2c.keyinfo[i].id ,x ,y , i2c.keyinfo[i].status, touch_key_hold_press);
							DBG("Key, Keyup ID=%d, X=%d, Y=%d, key_status=%d, touch_key_hold_press=%d\n", touch_key_code[i] ,x ,y , i2c.keyinfo[i].status, touch_key_hold_press);
						}
					}
				}
			}
			//All touch release 
			
			DBG("release_counter=%d,packet=%d\n", release_counter, packet);
		}

		if(i2c.ilitekdbg.tsp_print_coordinate)
			if(release_counter == max_point)
				printk(ILITEK_DEBUG_LEVEL "tsp_coordinate : - All touch release - \n");

#ifdef SYSFS_SAMPLE_RATE
		if(mxt_get_sampling_rate())
		{
			if(release_counter == max_point)
				measure_sample_rate(&i2c.sample_rate, 0);
			else{ 
				measure_sample_rate(&i2c.sample_rate, 1);
				//printk ("sampling rate ++\n");
			}
		}
#endif 
	}
	#ifdef TOUCH_PROTOCOL_B
	input_mt_report_pointer_emulation(i2c.input_dev, true);
	#endif
	input_sync(i2c.input_dev);
	input_sync(i2c.input_dev2);
	ilitek_i2c_irq_enable();
    return ret;
}

/*
description
	work queue function for irq use
parameter
	work
		work queue
return
	none
*/
static void ilitek_i2c_irq_work_queue_func(struct work_struct *work)
{
	int ret;
	ret = ilitek_i2c_process_and_report();
	DBG("%s,enter\n",__func__);
#ifdef CLOCK_INTERRUPT
	ilitek_i2c_irq_enable();
#else

#endif
}

/*
description
	i2c interrupt service routine
parameters
	irq
		interrupt number
	dev_id
		device parameter
return
	status
*/
static irqreturn_t ilitek_i2c_isr(int irq, void *dev_id)
{
	DBG("%s,ilitek_i2c_isr enter\n", __func__);
	ilitek_i2c_irq_disable();
	atomic_set(&ilitek_cmd_response, 1);
	if(update_Flag == 1)
	{	
		int_Flag = 1;
	}
	else
	{
		queue_work(i2c.irq_work_queue, &i2c.irq_work);
	}
	return IRQ_HANDLED;
}

/*
description
	i2c early suspend function
parameters
	h
		early suspend pointer
return
	none
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_early_suspend(struct early_suspend *h)
{
	ilitek_i2c_suspend(i2c.client, PMSG_SUSPEND);
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
}
#endif

/*
description
	i2c later resume function
parameters
	h
		early suspend pointer
return
	none
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ilitek_i2c_late_resume(struct early_suspend *h)
{
	ilitek_i2c_resume(i2c.client);
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
}
#endif

/*
description
	i2c irq enable function
*/
static void ilitek_i2c_irq_enable(void)
{
	if (i2c.irq_status == 0)
	{
		i2c.irq_status = 1;
		enable_irq(i2c.client->irq);
		DBG("enable\n");	
	}
	else
	{
		DBG("no enable\n");
	}
}

/*
description
    i2c irq disable function
*/
static void ilitek_i2c_irq_disable(void)
{
	if (i2c.irq_status == 1)
	{
		i2c.irq_status = 0;
		disable_irq_nosync(i2c.client->irq);
		DBG("disable\n");
	}
	else
	{
		DBG("no disable\n");
	}
}

/*
description
	i2c suspend function
parameters
	client
		i2c client data
	mesg
		suspend data
return
	status
*/
static int ilitek_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	char buf[64] = {0};
	int ret; 

	buf[0] = ILITEK_TP_CMD_SLEEP;

	ret = ilitek_i2c_write_read(client, buf, 1, 0, buf, 0);
	if(ret < 0)
	{
		return ret;
	}
	
	if(i2c.valid_irq_request != 0)
	{
		ilitek_i2c_irq_disable();
	}
	else
	{
		i2c.stop_polling = 1;
		printk(ILITEK_DEBUG_LEVEL "%s, stop i2c thread polling\n", __func__);
  	}
	return 0;
}

/*
description
	i2c resume function
parameters
	client
		i2c client data
return
	status
*/
static int ilitek_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

    	if(i2c.valid_irq_request != 0)
	{
		ilitek_i2c_irq_enable();
	}
	else
	{
		i2c.stop_polling = 0;
		printk(ILITEK_DEBUG_LEVEL "%s, start i2c thread polling\n", __func__);
	}
	return 0;
}

/*
description
	reset touch ic 
prarmeters
	reset_pin
		reset pin
return
	status
*/
static int ilitek_i2c_reset(void)
{
	int ret = 0;
#ifndef SET_RESET
	static unsigned char buffer[64]={0};
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = 1, .buf = buffer,}
    };
	buffer[0] = 0x60;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
#else
	/*
	
	____         ___________
		|_______|
		   1ms      100ms
	*/
#endif
	msleep(100);
	return ret; 
}

/*
description
	i2c shutdown function
parameters
	client
		i2c client data
return
	nothing
*/
static void ilitek_i2c_shutdown(struct i2c_client *client)
{
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
	i2c.stop_polling = 1;
}
#ifdef CONFIG_OF
static int ilitek_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int irq_gpio;
	int reset;
	int ret;

#if 0
	irq_gpio = of_get_named_gpio(np, "ilitek,irq", 0);
	
	if(!gpio_is_valid(irq_gpio)){
		printk(ILITEK_ERROR_LEVEL "%s, Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}
		
	//client->irq = __gpio_to_irq(irq_gpio);
	//client->irq = 160 + 32 + 18;
#endif 

	client->irq = irq_of_parse_and_map(np, 0);
	
	reset = of_get_named_gpio(np, "ilitek,reset", 0);
	if(gpio_is_valid(reset)){
		ret = gpio_request_one(reset, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "ilitek,reset");
		if(ret){
			printk(ILITEK_ERROR_LEVEL "%s, Failed to get reset gpio \n", __func__);
			return -EINVAL;
		}
	}
	else{
		printk(ILITEK_ERROR_LEVEL "%s, Failed to get reset gpio \n", __func__);
		return -EINVAL;

	}

	i2c.reset_gpio = reset;

	gpio_set_value(reset, 1);
	
	return 0;
}
#endif
/* add palm on/off function */

static int ilitek_i2c_palm(struct i2c_client *client, uint8_t flag)
{
	int ret;
	unsigned char buf[10] = {0};
	struct i2c_msg msgs_send[] = {
		{.addr = client->addr, .flags = 0, .len = 2, .buf = buf,},
	};
	i2c.report_status = 0;
	buf[0] = 0x31;
	if(flag == 1)
		buf[1] = 0x50;
	else
		buf[1] = 0x54;
	
	ret = ilitek_i2c_transfer(client, msgs_send, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c write error, ret = %d\n", __func__, ret);
	}
	i2c.report_status = 1;
	return ret;
}


#ifdef USE_ILITEK_SYSFS
//tsp_sampling_rate
static ssize_t ilitek_tsp_sampling_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", i2c.ilitekdbg.tsp_sampling_rate);
	
	return ret;
}

static ssize_t ilitek_tsp_sampling_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	if(sscanf(buf, "%u", &i) == 1 && i<2)
	{
		i2c.ilitekdbg.tsp_sampling_rate = i;
		if(i)
			printk(ILITEK_DEBUG_LEVEL "%s, Ilitek tsp_sampling_rate ON\n" ,__func__);
		else
			printk(ILITEK_DEBUG_LEVEL "%s, Ilitek tsp_sampling_rate OFF\n",__func__);
	}
	else
	{
		 printk(ILITEK_DEBUG_LEVEL "%s, Error\n", __func__);
	}

	return count;

}

//tsp_fw_ver
static ssize_t ilitek_tsp_fw_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, TSP_BUF_SIZE, "%X.%X.%X.%X\n", i2c.firmware_ver[0], i2c.firmware_ver[1], i2c.firmware_ver[2], i2c.firmware_ver[3]);
}

//tsp_coordinate

static ssize_t ilitek_tsp_coordinate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", i2c.ilitekdbg.tsp_print_coordinate);
	
	return ret;
}

static ssize_t ilitek_tsp_coordinate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i<2)
	{
		i2c.ilitekdbg.tsp_print_coordinate = i;

		if(i)
			printk(ILITEK_DEBUG_LEVEL "%s, Ilitek tsp_print_coordinate ON\n" ,__func__);
		else
			printk(ILITEK_DEBUG_LEVEL "%s, Ilitek tsp_print_coordinate OFF\n",__func__);
	}
	else
	{
		printk(ILITEK_DEBUG_LEVEL "%s, Error\n", __func__);
	}


	return count;
}

/* add plam control sysfs */

static ssize_t ilitek_tsp_palm_disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", i2c.ilitekdbg.tsp_palm_disable);
	
	return ret;
}

static ssize_t ilitek_tsp_palm_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret;

	if (sscanf(buf, "%u", &i) == 1 && i<2)
	{
			ret = ilitek_i2c_palm(i2c.client,i);
			if(ret < 0)
				printk(ILITEK_DEBUG_LEVEL "%s, palm on/off set failed...\n",__func__);
			else{
				printk(ILITEK_DEBUG_LEVEL "%s, palm on/off set success!\n",__func__);
				i2c.ilitekdbg.tsp_palm_disable = i;	
			}
	}
	else
	{
		printk(ILITEK_DEBUG_LEVEL "%s, Error(palm disable on:1, off:0)\n", __func__);
	}

	return count;
}

/* add, touch power control */

static ssize_t ilitek_tsp_touch_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", gpio_get_value(i2c.reset_gpio));

	return ret;
}

static ssize_t ilitek_tsp_touch_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret;

	if (sscanf(buf, "%u", &i) == 1 && i<2)
	{
		gpio_set_value(i2c.reset_gpio , i);
	}
	else
	{
		printk(ILITEK_ERROR_LEVEL "%s, Error(touch on:1, off:0)\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(tsp_palm_disable, S_IRUGO | S_IWUSR | S_IWGRP,
	ilitek_tsp_palm_disable_show, ilitek_tsp_palm_disable_store);

static DEVICE_ATTR(tsp_sampling_rate, S_IRUGO | S_IWUSR | S_IWGRP,
	ilitek_tsp_sampling_rate_show, ilitek_tsp_sampling_rate_store);

static DEVICE_ATTR(tsp_fw_ver, S_IRUGO | S_IWUSR | S_IWGRP,
	ilitek_tsp_fw_ver_show, NULL );

static DEVICE_ATTR(tsp_print_coordinate, S_IRUGO | S_IWUSR | S_IWGRP,
	ilitek_tsp_coordinate_show, ilitek_tsp_coordinate_store);

static DEVICE_ATTR(tsp_touch_en, S_IRUGO | S_IWUSR | S_IWGRP,
	ilitek_tsp_touch_en_show, ilitek_tsp_touch_en_store);

static struct attribute *ilitek_attributes[] = {
	&dev_attr_tsp_sampling_rate.attr,
	&dev_attr_tsp_fw_ver.attr,
	&dev_attr_tsp_print_coordinate.attr,
	&dev_attr_tsp_palm_disable.attr,
	&dev_attr_tsp_touch_en.attr,
	//&dev_attr_tsp_cfg_crc.attr,
	NULL,
};

static struct attribute_group ilitek_attr_group = {
	.attrs = ilitek_attributes,
};

#endif 

#ifdef WATCHDOG_ENABLE

static void ilitek_wd_work_handler(struct work_struct *w)
{

	int ret;
	unsigned char _buf[4] = {0};

	//printk("[%s][%d] touch work handler called! \n\n",__func__,__LINE__);
	if(watchdog_flag == 0)
	{
		DBG("Touch IC watch dog, disable\n");
		return;
	}
	schedule_delayed_work(&i2c.wd_work, msecs_to_jiffies(1000));
	
	if(atomic_read(&ilitek_cmd_response) == 0)
	{
		DBG("Touch IC watch dog ... \n");
		_buf[0] = ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
		if(ilitek_i2c_write_read(i2c.client, _buf, 1, 10, _buf, 4) < 0)
		{
			printk(ILITEK_DEBUG_LEVEL "Touch IC Reboot ... \n");
			gpio_set_value(i2c.reset_gpio , 0);
			mdelay(300);
			gpio_set_value(i2c.reset_gpio , 1);
			mdelay(300);
		}
	}
	atomic_set(&ilitek_cmd_response, 0);
	return;
}
#endif 

/*
description
	when adapter detects the i2c device, this function will be invoked.
parameters
	client
		i2c client data
	id
		i2c data
return
	status
*/
static int ilitek_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//register i2c device
	int ret = 0; 

	//allocate character device driver buffer
	ret = alloc_chrdev_region(&dev.devno, 0, 1, ILITEK_FILE_DRIVER_NAME);
	if(ret)
	{
		printk(ILITEK_ERROR_LEVEL "%s, can't allocate chrdev\n", __func__);
		goto err_chrdev_region;
	}
	printk(ILITEK_DEBUG_LEVEL "%s, register chrdev(%d, %d)\n", __func__, MAJOR(dev.devno), MINOR(dev.devno));
	
	//initialize character device driver
	cdev_init(&dev.cdev, &ilitek_fops);
	dev.cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev.cdev, dev.devno, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, add character device error, ret %d\n", __func__, ret);
		goto err_cdev_add;
	}
	dev.class = class_create(THIS_MODULE, ILITEK_FILE_DRIVER_NAME);
	if(IS_ERR(dev.class))
	{
		printk(ILITEK_ERROR_LEVEL "%s, create class, error\n", __func__);
		goto err_class_create;
	}
	device_create(dev.class, NULL, dev.devno, NULL, "ilitek_ctrl");
	if(IS_ERR(dev.class))
	{
		printk(ILITEK_ERROR_LEVEL "%s, create device, error\n", __func__);
		goto err_device_create;
	}
	
	proc_create("ilitek_ctrl", 0666, NULL, &ilitek_fops);
	
	Report_Flag = 0;
	
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(ILITEK_ERROR_LEVEL "%s, I2C_FUNC_I2C not support\n", __func__);
		ret = -EINVAL;
		goto err_device_create;
	}
	i2c.client = client;
	printk(ILITEK_DEBUG_LEVEL "%s, i2c new style format\n", __func__);
	#ifdef CONFIG_OF
	ret = ilitek_parse_dt(i2c.client);

	if(ret) {
		printk(ILITEK_DEBUG_LEVEL "%s, parse dt error \n", __func__);
		goto err_device_create;
	}
	#endif
	printk(ILITEK_DEBUG_LEVEL "%s,IRQ: 0x%X\n", __func__, client->irq);

	//ilitek_i2c_register_device();
	ret = ilitek_i2c_register_device();
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, register i2c device, error\n", __func__);
		goto err_device_create;
	}

#ifdef WATCHDOG_ENABLE
	INIT_DELAYED_WORK(&i2c.wd_work, ilitek_wd_work_handler);
	schedule_delayed_work(&i2c.wd_work, msecs_to_jiffies(1000));
#endif 

#ifdef USE_ILITEK_SYSFS
	ilitek_class = class_create(THIS_MODULE, "ilitek_touchscreen");
	if(IS_ERR(ilitek_class)){
		printk(ILITEK_ERROR_LEVEL "%s, class_create failed\n", __func__);
		ret = -EINVAL;
		goto err_ilitek_class_create;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ilitek_attr_group);
	if(ret){
		printk(ILITEK_ERROR_LEVEL "%s, Failed to create ilitek sysfs group\n", __func__);
		goto err_sysfs_create_group;
	}

	//ilitek_class->dev_attrs = ilitek_dev_attrs;
	i2c.sample_rate.touch_enable = 1;
	i2c.ilitekdbg.tsp_palm_disable = 1;
#endif 

	printk(ILITEK_DEBUG_LEVEL "[%s][%d]ILITEK I2C TOUCH PROBE DONE! \n",__func__,__LINE__);
	return ret;
	
#ifdef USE_ILITEK_SYSFS
err_sysfs_create_group:
	sysfs_remove_group(&client->dev.kobj, &ilitek_attr_group);
err_ilitek_class_create:
	class_destroy(ilitek_class);
#endif 
err_device_create:
	device_destroy(dev.class, dev.devno);
	gpio_free(i2c.reset_gpio); 
err_class_create:
	class_destroy(dev.class);
err_cdev_add:
	cdev_del(&dev.cdev);
err_chrdev_region:
	unregister_chrdev_region(dev.devno, 1);
	return ret;
}

/*
description
	when the i2c device want to detach from adapter, this function will be invoked.
parameters
	client
		i2c client data
return
	status
*/
static int ilitek_i2c_remove(struct i2c_client *client)
{
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
	i2c.stop_polling = 1;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&i2c.early_suspend);
#endif

#ifdef WATCHDOG_ENABLE
	cancel_delayed_work_sync(&i2c.wd_work);
#endif

	//delete i2c driver
	if(i2c.client->irq != 0)
	{
		if(i2c.valid_irq_request != 0)
		{
			free_irq(i2c.client->irq, &i2c);
			printk(ILITEK_DEBUG_LEVEL "%s, free irq\n", __func__);
			if(i2c.irq_work_queue)
			{
				destroy_workqueue(i2c.irq_work_queue);
				printk(ILITEK_DEBUG_LEVEL "%s, destory work queue\n", __func__);
			}
		}
	}
	else
	{
		if(i2c.thread != NULL)
		{
			kthread_stop(i2c.thread);
			printk(ILITEK_DEBUG_LEVEL "%s, stop i2c thread\n", __func__);
		}
	}
	if(i2c.valid_input_register != 0)
	{
		input_unregister_device(i2c.input_dev);
		input_unregister_device(i2c.input_dev2);
		printk(ILITEK_DEBUG_LEVEL "%s, unregister i2c input device\n", __func__);
	}
        
	//delete character device driver
	cdev_del(&dev.cdev);
	unregister_chrdev_region(dev.devno, 1);
	device_destroy(dev.class, dev.devno);
	class_destroy(dev.class);
	class_destroy(ilitek_class);
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
	return 0;
}

/*
description
	write data to i2c device and read data from i2c device
parameter
	client
		i2c client data
	cmd
		data for write
	delay
		delay time(ms) after write
	data
		data for read
	length
		data length
return
	status
*/
static int ilitek_i2c_write_read(struct i2c_client *client, uint8_t *cmd,int write_len, int delay, uint8_t *data, int read_len)
{
	int ret = 0;
	struct i2c_msg msgs_send[] = {
		{.addr = client->addr, .flags = 0, .len = write_len, .buf = cmd,},
	};
	struct i2c_msg msgs_receive[] = {
		{.addr = client->addr, .flags = I2C_M_RD, .len = read_len, .buf = data,}
	};
	if(write_len > 0)
	{
	ret = ilitek_i2c_transfer(client, msgs_send, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c write error, ret = %d\n", __func__, ret);
	}
	}
	if(delay > 0)
	msleep(delay);
	if(read_len > 0)
	{
		ret = ilitek_i2c_transfer(client, msgs_receive, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret = %d\n", __func__, ret);
		}
	}
	return ret;
}

/*
description
	read touch information
parameters
	none
return
	status
*/
static int ilitek_i2c_read_tp_info(void)
{
	int i;
	unsigned char buf[64] = {0};
	
	//read driver version
	printk(ILITEK_DEBUG_LEVEL "%s, driver version:%d.%d.%d\n", __func__, _driver_information[0], _driver_information[1], _driver_information[2]);
	buf[0] = ILITEK_TP_CMD_GET_FIRMWARE_VERSION;
	//read firmware version
	if(ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 4) < 0)
	{
		return -1;
	}
#ifdef UPGRADE_FIRMWARE_ON_BOOT
	for(i = 0; i < 4; i++)
	{
		i2c.firmware_ver[i] = buf[i];
	}
#endif
	printk(ILITEK_DEBUG_LEVEL "%s, firmware version:%X.%X.%X.%X\n", __func__, buf[0], buf[1], buf[2], buf[3]);
	buf[0] = ILITEK_TP_CMD_GET_PROTOCOL_VERSION;
	//read protocol version
	if(ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 2) < 0)
	{
		return -1;
	}	
	i2c.protocol_ver = (((int)buf[0]) << 8) + buf[1];
	printk(ILITEK_DEBUG_LEVEL "%s, protocol version:%d.%d\n", __func__, buf[0], buf[1]);

    //read touch resolution
	i2c.max_tp = 2;
	buf[0] = ILITEK_TP_CMD_GET_RESOLUTION;
	if(ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 10) < 0)
	{
		return -1;
	}
	
	//calculate the resolution for x and y direction
	i2c.max_x = buf[0];
	i2c.max_x+= ((int)buf[1]) * 256;
	i2c.max_y = buf[2];
	i2c.max_y+= ((int)buf[3]) * 256;
	i2c.x_ch = buf[4];
	i2c.y_ch = buf[5];
	//maximum touch point
	i2c.max_tp = buf[6];
	//maximum button number
	i2c.max_btn = buf[7];
	//key count
	i2c.keycount = buf[8];
	
	printk(ILITEK_DEBUG_LEVEL "%s, max_x: %d, max_y: %d, ch_x: %d, ch_y: %d, max_tp: %d, max_btn: %d, key_count: %d\n", __func__, i2c.max_x, i2c.max_y, i2c.x_ch, i2c.y_ch, i2c.max_tp, i2c.max_btn, i2c.keycount);
	
	if((i2c.protocol_ver & 0x300) == 0x300)
	{
		buf[0] = ILITEK_TP_CMD_GET_KEY_INFORMATION;
		//get key infotmation	

		if(i2c.keycount)
		{
			if(ilitek_i2c_write_read(i2c.client, buf, 1, 0, buf, 29) < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, write read firmware version failed\n", __func__);
				return -1;
			}
			if(i2c.keycount > 5)
			{
				if(ilitek_i2c_write_read(i2c.client, buf, 0, 0, buf+29, 25) < 0)
				{
					printk(ILITEK_ERROR_LEVEL "%s, read firmware version failed\n", __func__);
					return -1;
				}
			}
		}
		i2c.key_xlen = (buf[0] << 8) + buf[1];
		i2c.key_ylen = (buf[2] << 8) + buf[3];
		printk(ILITEK_DEBUG_LEVEL "%s, key_xlen: %d, key_ylen: %d\n", __func__, i2c.key_xlen, i2c.key_ylen);
			
		//print key information
		for(i = 0; i < i2c.keycount; i++)
		{
			i2c.keyinfo[i].id = buf[i*5+4];	
			i2c.keyinfo[i].x = (buf[i*5+5] << 8) + buf[i*5+6];
			i2c.keyinfo[i].y = (buf[i*5+7] << 8) + buf[i*5+8];
			i2c.keyinfo[i].status = 0;
			printk(ILITEK_DEBUG_LEVEL "%s, key_id: %d, key_x: %d, key_y: %d, key_status: %d\n", __func__, i2c.keyinfo[i].id, i2c.keyinfo[i].x, i2c.keyinfo[i].y, i2c.keyinfo[i].status);
		}
	}
	
	return 0;
}
#ifdef UPGRADE_FIRMWARE_ON_BOOT
static int ilitek_upgrade_firmware(void)
{
	int ret=0,upgrade_status=0,i,j,k = 0,ap_len = 0,df_len = 0;
	unsigned char buf[128]={0};
	unsigned int ap_startaddr,df_startaddr,ap_endaddr,df_endaddr,ap_checksum = 0,df_checksum = 0;
	unsigned char firmware_ver[4];
	ap_startaddr = ( CTPM_FW[0] << 16 ) + ( CTPM_FW[1] << 8 ) + CTPM_FW[2];
	ap_endaddr = ( CTPM_FW[3] << 16 ) + ( CTPM_FW[4] << 8 ) + CTPM_FW[5];
	ap_checksum = ( CTPM_FW[6] << 16 ) + ( CTPM_FW[7] << 8 ) + CTPM_FW[8];
	df_startaddr = ( CTPM_FW[9] << 16 ) + ( CTPM_FW[10] << 8 ) + CTPM_FW[11];
	df_endaddr = ( CTPM_FW[12] << 16 ) + ( CTPM_FW[13] << 8 ) + CTPM_FW[14];
	df_checksum = ( CTPM_FW[15] << 16 ) + ( CTPM_FW[16] << 8 ) + CTPM_FW[17];
	firmware_ver[0] = CTPM_FW[18];
	firmware_ver[1] = CTPM_FW[19];
	firmware_ver[2] = CTPM_FW[20];
	firmware_ver[3] = CTPM_FW[21];
	df_len = ( CTPM_FW[26] << 16 ) + ( CTPM_FW[27] << 8 ) + CTPM_FW[28];
	ap_len = ( CTPM_FW[29] << 16 ) + ( CTPM_FW[30] << 8 ) + CTPM_FW[31];
	ap_endaddr = ap_endaddr + 32;
	ap_checksum = ap_checksum + 32*0xFF;
	if(ap_endaddr%32 != 0)
	{
		ap_checksum = ap_checksum + (32 - ap_endaddr%32 - 1) * 0xff;
		ap_endaddr = ap_endaddr + 32 - ap_endaddr%32 - 1;
	}
	printk(ILITEK_DEBUG_LEVEL "ap_startaddr=0x%x,ap_endaddr=0x%x,ap_checksum=0x%x,ap_len=0x%x\n",ap_startaddr,ap_endaddr,ap_checksum,ap_len);
	printk(ILITEK_DEBUG_LEVEL "df_startaddr=0x%x,df_endaddr=0x%d,df_checksum=0x%x\n",df_startaddr,df_endaddr,df_checksum);	
	buf[0] = 0xc0;
	printk(ILITEK_DEBUG_LEVEL "%x,%x,%x,%x,%x,%x\n",CTPM_FW[3], CTPM_FW[4], CTPM_FW[5], CTPM_FW[6], CTPM_FW[7], CTPM_FW[8]);
	if(ap_endaddr )
	ret = ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 1);
	if(ret < 0){
		return 3;
	}
	if(buf[0] != 0x55)
	{
		for(i = 0; i < 4; i++)
		{
			if(buf[0] != 0x55)
			{
				buf[0]=0xc4;
				buf[1]=0x5A;
				buf[2]=0xA5;
				ret = ilitek_i2c_write_read(i2c.client, buf, 3, 30, buf, 0);
				if(ret < 0){
					return 3;
				}
				buf[0]=0xc2;
				ret = ilitek_i2c_write_read(i2c.client, buf, 1, 100, buf, 0);
				if(ret < 0){
					return 3;
				}		
				msleep(100);
				buf[0] = 0xc0;
				ret = ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 1);
				if(ret < 0){
					return 3;
				}		

			}
			else
				break;
		}
	}
	if(buf[0] != 0x55)
	{
		printk(ILITEK_ERROR_LEVEL "Change to BL mode Fail.\n");
		return 4;
	}
	printk(ILITEK_DEBUG_LEVEL "ic. mode =%d\n",buf[0]);
	buf[0]=0xc4;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0x81;
	buf[4] = 0;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;
	buf[8] = 0;
	buf[9] = 0;
	ret = ilitek_i2c_write_read(i2c.client, buf, 10, 30, buf, 0);
	if(ret < 0){
		return 3;
	}
	msleep(30);
	printk(ILITEK_DEBUG_LEVEL "ILITEK:%s, upgrade firmware...\n", __func__);
	j = 0;
	for(i = df_startaddr; i < df_endaddr; i += 32)
	{
		j+= 1;
		if((j % 16) == 1){
			msleep(60);
		}
		for(k = 0; k < 32; k++){
			buf[1 + k] = CTPM_FW[i + 32 + k];
		}

		buf[0]=0xc3;
		ret = ilitek_i2c_write_read(i2c.client, buf, 33, 10, buf, 0);
		if(ret < 0){
			return 3;
		}
		upgrade_status = (i * 100) / df_len;
		printk(ILITEK_DEBUG_LEVEL "%cILITEK: Firmware Upgrade(Data flash), %02d%c. ",0x0D,upgrade_status,'%');
	}

	buf[0]=0xc4;
	buf[1] = 0x5A;
	buf[2] = 0xA5;
	buf[3] = 0;
	buf[4] = ap_endaddr >> 16;
	buf[5] = ap_endaddr >> 8;
	buf[6] = ap_endaddr;
	buf[7] = ap_checksum >> 16;
	buf[8] = ap_checksum >> 8;
	buf[9] = ap_checksum ;
	ret = ilitek_i2c_write_read(i2c.client, buf, 10, 30, buf, 0);
	if(ret < 0){
		return 3;	}
	j=0;
	for(i = ap_startaddr; i < ap_endaddr; i += 32)
	{
		j+= 1;
		if((j % 16) == 1){
			msleep(30);
		}
		for(k = 0; k < 32; k++){
			if(i + k < ap_len)
				buf[1 + k] = CTPM_FW[i + 32 + k];
			else
				buf[1 + k] = 0xFF;
		}

		buf[0]=0xc3;
		ret = ilitek_i2c_write_read(i2c.client, buf, 33, 10, buf, 0);
		if(ret < 0){
			return 3;
		}
		upgrade_status = (i * 100) / ap_len;
		printk(ILITEK_DEBUG_LEVEL "%cILITEK: Firmware Upgrade(AP), %02d%c. ",0x0D,upgrade_status,'%');
	}

	printk(ILITEK_DEBUG_LEVEL "ILITEK:%s, upgrade firmware completed\n", __func__);
	for(i = 0; i < 4; i++)
	{
		if(buf[0] != 0x5A)
		{
			buf[0]=0xc4;
			buf[1]=0x5A;
			buf[2]=0xA5;
			ret = ilitek_i2c_write_read(i2c.client, buf, 3, 30, buf, 0);
			if(ret < 0){
				return 3;
			}
			msleep(30);
			buf[0]=0xc1;
			ret = ilitek_i2c_write_read(i2c.client, buf, 1, 100, buf, 0);
			if(ret < 0){
				return 3;
			}
			buf[0]=0xc0;
			ret = ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 1);
			if(ret < 0){
				return 3;
			}
		}
		else
			break;
	}
	if(buf[0] != 0x5A)
	msleep(30);
	printk(ILITEK_DEBUG_LEVEL "ic. mode =%d , it's  %s \n",buf[0],((buf[0] == 0x5A)?"AP MODE":"BL MODE"));
	if(buf[0] != 0x5A)
		return 5;
	buf[0]=0x60;

	ret = ilitek_i2c_write_read(i2c.client, buf, 1, 10, buf, 0);
	if(ret < 0)
	{
		return 3;
	}

	msleep(200);
	return 2;
}
#endif

/*
description
	register i2c device and its input device
parameters
	none
return
	status
*/
static int ilitek_i2c_register_device(void)
{
	int ret = 0, i = 0;
	//int read_info_ret = 0;
	printk(ILITEK_DEBUG_LEVEL "%s, client.addr: 0x%X\n", __func__, (unsigned int)i2c.client->addr);
	printk(ILITEK_DEBUG_LEVEL "%s, client.adapter: 0x%X\n", __func__, (unsigned int)i2c.client->adapter);
	//printk(ILITEK_DEBUG_LEVEL "%s, client.driver: 0x%X\n", __func__, (unsigned int)i2c.client->driver);
	if((i2c.client->addr == 0) || (i2c.client->adapter == 0))//|| (i2c.client->driver == 0))
	{
		printk(ILITEK_ERROR_LEVEL "%s, invalid register\n", __func__);
		return ret;
	}
	//read touch parameter
	//ilitek_i2c_read_tp_info();
	ret = ilitek_i2c_read_tp_info();
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, ilitek_i2c_read_tp_info failed ... \n", __func__);
		return ret;
		//read_info_ret = -1;
	}
	if(i2c.client->irq != 0)
	{ 
		i2c.irq_work_queue = create_singlethread_workqueue("ilitek_i2c_irq_queue");
		if(i2c.irq_work_queue)
		{
			INIT_WORK(&i2c.irq_work, ilitek_i2c_irq_work_queue_func);
		#ifdef CLOCK_INTERRUPT
			if(request_irq(i2c.client->irq, ilitek_i2c_isr, /*IRQF_DISABLED | */IRQF_SHARED | IRQF_ONESHOT | IRQF_TRIGGER_RISING , "ilitek_i2c_irq", &i2c))
			{
				printk(ILITEK_ERROR_LEVEL "%s, 1 request irq, error\n", __func__);
			}
			else
			{
				i2c.valid_irq_request = 1;
				i2c.irq_status = 1;
				printk(ILITEK_DEBUG_LEVEL "%s, request irq(Trigger Falling), success\n", __func__);
			}				
		#endif
		}
	}	
#ifdef UPGRADE_FIRMWARE_ON_BOOT
	//check firmware version
	i2c.report_status = 0;
	for(i = 0; i < 4; i++)
	{
		printk(ILITEK_DEBUG_LEVEL "i2c.firmware_ver[%d] = %X, firmware_ver[%d] = %X\n", i, i2c.firmware_ver[i], i, CTPM_FW[i + 18]);
		if((i2c.firmware_ver[i] > CTPM_FW[i + 18]) || ((i == 3) && (i2c.firmware_ver[3] == CTPM_FW[3 + 18])))
		{
			ret = 1;
			break;
		}
		else if(i2c.firmware_ver[i] < CTPM_FW[i + 18])
		{
			break;
		}
	}
	
	if(ret == 1)
	{
		printk(ILITEK_DEBUG_LEVEL "Do not need update\n");
	}
	else
	{
		//upgrade fw
		update_Flag = 1;
		ilitek_i2c_irq_disable();
		msleep(10);
		ret = ilitek_upgrade_firmware();
		if(ret == 2)
		{
			printk(ILITEK_DEBUG_LEVEL "update end\n");
		}
		else if(ret == 3)
		{
			printk(ILITEK_ERROR_LEVEL "i2c communication error\n");
		}
		update_Flag = 0;
		//reset
		ilitek_i2c_reset();
		//read touch parameter
		ilitek_i2c_read_tp_info();
		ilitek_i2c_irq_enable();
	}
	i2c.report_status = 1;
#endif
	
	//register input device
	i2c.input_dev = input_allocate_device();
	if(i2c.input_dev == NULL)
	{
		printk(ILITEK_ERROR_LEVEL "%s, allocate input device, error\n", __func__);
		return -1;
	}
	//hsguy.son (add)
	i2c.input_dev2 = input_allocate_device();
	if(i2c.input_dev2 == NULL)
	{
		printk(ILITEK_ERROR_LEVEL "%s, allocate input device, error\n", __func__);
		return -1;
	}
	//hsguy.son (add_end)
	ilitek_set_input_param(i2c.input_dev, i2c.max_tp, i2c.max_x, i2c.max_y);
	ilitek_set_input2_param(i2c.input_dev2); 	//hsguy.son (add)
	ret = input_register_device(i2c.input_dev);
	if(ret)
	{
		printk(ILITEK_ERROR_LEVEL "%s, register input device, error\n", __func__);
		return ret;
	}
	//hsguy.son (add)
	ret = input_register_device(i2c.input_dev2);
	if(ret)
	{
		printk(ILITEK_ERROR_LEVEL "%s, register input device, error\n", __func__);
		return ret;
	}
	//hsguy.son (add_end)
	printk(ILITEK_DEBUG_LEVEL "%s, register input device, success\n", __func__);
	i2c.valid_input_register = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
	i2c.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	i2c.early_suspend.suspend = ilitek_i2c_early_suspend;
	i2c.early_suspend.resume = ilitek_i2c_late_resume;
	register_early_suspend(&i2c.early_suspend);
#endif

	return 0;
}

/*
description
	initiali function for driver to invoke.
parameters
	none
return
	status
*/
static int __init ilitek_init(void)
{
	int ret = 0;
	//initialize global variable
	memset(&dev, 0, sizeof(struct dev_data));
	memset(&i2c, 0, sizeof(struct i2c_data));

	//initialize mutex object
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
	init_MUTEX(&i2c.wr_sem);
#else
	sema_init(&i2c.wr_sem, 1);
#endif
	i2c.wr_sem.count = 1;
	watchdog_flag = 1;
	i2c.report_status = 1;
	ret = i2c_add_driver(&ilitek_i2c_driver);
	if(ret == 0)
	{
		i2c.valid_i2c_register = 1;
		printk(ILITEK_DEBUG_LEVEL "%s, add i2c device, success\n", __func__);
		if(i2c.client == NULL)
		{
			printk(ILITEK_ERROR_LEVEL "%s, no i2c board information\n", __func__);
		}
	}
	else
	{
		printk(ILITEK_ERROR_LEVEL "%s, add i2c device, error\n", __func__);
	}

	return ret;
}

/*
description
	driver exit function
parameters
	none
return
	nothing
*/
static void __exit ilitek_exit(void)
{
	printk(ILITEK_DEBUG_LEVEL "%s, enter\n", __func__);
	sysfs_remove_group(&i2c.client->dev.kobj, &ilitek_attr_group);
	if(i2c.valid_i2c_register != 0)
	{
		printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
		i2c_del_driver(&ilitek_i2c_driver);
		printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
    }
	else
	{
		printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver Fail\n", __func__);
	}
	
	remove_proc_entry("ilitek_ctrl", NULL);
}

//set init and exit function for this module
module_init(ilitek_init);
module_exit(ilitek_exit);
