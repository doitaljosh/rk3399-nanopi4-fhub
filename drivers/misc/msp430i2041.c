#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm-generic/ioctl.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>

#define MSP430_DEV_NAME 	"msp430"
#define MSP430_MAGIC 'J'
#define MSP430_POWER_READ	_IOW(MSP430_MAGIC, 0x01,int)
#define MSP430_VERSION_READ	_IOW(MSP430_MAGIC, 0x02,char*)
#define MSP_DEBUG
#define MSP430_RESET_TIME	1
#ifdef MSP_DEBUG 
#define msp_log(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define msp_log(fmt, ...)
#endif 

#define msp_err(fmt, ...) printk(fmt, ##__VA_ARGS__)

static int msp430_dev_major;
static struct i2c_client * save_client;
static dev_t msp430_dev_num;
static struct cdev msp430_cdev;
static struct device *msp430_device = NULL;
static struct class * msp430_class = NULL;
static struct mutex msp_mutex;
static int reset_gpio;

typedef struct msp430_offset {
	u8 offset;
	char name[10];
}_OFFSET;

_OFFSET msp430_data[4] = {

	{ 0x00, "power"},
	{ 0x04, "current"},
	{ 0x08, "voltage"},
	{ 0x0C, "version"}
};

static const struct i2c_device_id msp430_id[] = {
	{ "msp430", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, msp430_id);


static void msp430_power_reset(void){

	//msp430 power reset 
	msp_log(KERN_INFO "[%s][%d] msp430 reset \n",__func__,__LINE__);

	gpio_set_value(reset_gpio, 0);
	msleep(MSP430_RESET_TIME);
	gpio_set_value(reset_gpio, 1);
}

static int msp430_read_power(void){
	
	u8 val[4];
	int ret = 0;
	unsigned int result =0;
	int App_value = 0;
	
	mutex_lock(&msp_mutex);

	//offset set 0
	ret = i2c_smbus_write_byte_data(save_client, 0, msp430_data[0].offset);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c write failed ... \n",__func__,__LINE__);
		mutex_unlock(&msp_mutex);
		goto error;
	}
	
	ret = i2c_smbus_read_i2c_block_data(save_client, 0 , 4, val);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c read failed ... \n",__func__,__LINE__);
		mutex_unlock(&msp_mutex);
		goto error;
	}

	result = ((val[0] << 0) | (val[1] << 8 ) | (val[2] << 16) | (val[3] << 24));

	//msp_log(KERN_INFO "POWER : val[0] =%02X, val[1] =%02X, val[2] =%02X, val[3] =%02X , result = %02X\n", val[0], val[1], val[2], val[3], result );	
	//msp_log(KERN_INFO "POWER : %d \n", result);

	if((result%10) >= 5)
		App_value = result/10 + 1;
	else
		App_value = result/10;

	//msp_log(KERN_INFO "App_value : %d \n", App_value);

	mutex_unlock(&msp_mutex);
	return App_value;

error :
	msp430_power_reset();
	return -1;

}

static int msp430_read_version(char* ver){
	
	u8 val[4];
	int ret = 0;
	unsigned int result =0;
	
	mutex_lock(&msp_mutex);

	//offset set C
	ret = i2c_smbus_write_byte_data(save_client, 0, msp430_data[3].offset);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c write failed ... \n",__func__,__LINE__);
		mutex_unlock(&msp_mutex);
		goto error;
	}
	
	ret = i2c_smbus_read_i2c_block_data(save_client, 0 , 4, val);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c read failed ... \n",__func__,__LINE__);
		mutex_unlock(&msp_mutex);
		goto error;
	}
	msp_log(KERN_INFO "msp: val[0] =%d, val[1] =%d, val[2] =%d, val[3] =%d \n", val[0], val[1], val[2], val[3]);	

	ret = sprintf(ver, "%d%d%d%d", val[3],val[2],val[1],val[0]);

	msp_log(KERN_INFO "Msp430 Version : %s", ver);
	
	mutex_unlock(&msp_mutex);
	
	return 0;

error :
	return -1;

}

int msp430_file_open(struct inode *inode, struct file *fp){
	
	return generic_file_open(inode, fp);	
}

static int msp430_file_close(struct inode *inode, struct file *file)
{
	return 0;
}

long msp430_ioctl(struct file *fp, unsigned int cmd, unsigned long args){

	long ret = 0;
	int power = 0;
	char ver[100];
	if(_IOC_TYPE(cmd) != MSP430_MAGIC)
		return -ENOTTY;

	switch (cmd) {
		case MSP430_POWER_READ:
			{
				power = msp430_read_power();
				if(power < 0)
					msp_log(KERN_INFO "[%s][%d] msp430 i2c read failed ... \n", __func__, __LINE__);
				//else
					//msp_log(KERN_INFO "[%s][%d] Power : %d \n",__func__,__LINE__,power);
				
				if(copy_to_user((int*)args, &power, sizeof(int))){
					msp_err(KERN_CRIT "[%s][%d] copy_to_user failed ... \n",__func__,__LINE__);
				}
				break;
			}	
		
		case MSP430_VERSION_READ:
			{
				ret = msp430_read_version(ver);
				if(ret < 0){
					msp_log(KERN_INFO "[%s][%d] msp430 i2c read version failed ... \n", __func__, __LINE__);
					return ret;
				}else
					msp_log(KERN_INFO "[%s][%d] version : %s \n",__func__,__LINE__,ver);

				if(copy_to_user((char*)args, ver, sizeof(ver))){
					msp_err(KERN_CRIT "[%s][%d] copy_to_user failed ... \n",__func__,__LINE__);
				}
				break;
			}	

		default:
			break;
	}
	return ret;

}

const struct file_operations msp430_fops = 
{
	.owner = THIS_MODULE,
//	.llseek = msp430_file_llseek,
//	.read = msp430_file_read,
	.open = msp430_file_open,
	.release = msp430_file_close,
	.unlocked_ioctl = msp430_ioctl,

};

static int msp430_parse_dt(struct i2c_client *client)
{

	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret; 

	reset_gpio = of_get_named_gpio(np, "msp430,reset", 0);

	if(gpio_is_valid(reset_gpio)){
		ret = gpio_request_one(reset_gpio, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "msp430,reset");
		
		if(ret){
			msp_err(KERN_CRIT "[%s][%d] Failed to get reset gpio \n", __func__, __LINE__);
			return -EINVAL;
		}
	}
	else {
		msp_err(KERN_CRIT "[%s][%d] Failed to get reset gpio \n", __func__, __LINE__);
		return -EINVAL;
	}

	gpio_set_value(reset_gpio, 1);

	return 0;
}

static ssize_t power_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int power = 0;
	power = msp430_read_power();
	return sprintf(buf, "%d", power);
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char ver[100];
	int ret;
	ret = msp430_read_version(ver);
	if (ret < 0)
		return sprintf(buf, "failed to get fw version");

	return sprintf(buf, "%s", ver);

}

static DEVICE_ATTR(power_value, S_IRUGO, power_value_show, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO, fw_ver_show, NULL);

static struct attribute *msp_attributes[] = {
	&dev_attr_power_value.attr,
	&dev_attr_fw_ver.attr,
	NULL,
};

static struct attribute_group msp_attr_group = {
	.attrs = msp_attributes,
};

static int msp430_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	u8 val[4];
	//char buf[100];
	
	save_client = client;
	
	ret = msp430_parse_dt(client);
	
	if(ret){
		msp_err(KERN_CRIT "[%s][%d] parse dt error\n", __func__, __LINE__);
		goto err;	
	}

	//msp_log(KERN_INFO "client addr : %d\n save_client : %d\n ", client->addr, save_client->addr);
	ret = i2c_smbus_write_byte_data(save_client, msp430_data[0].offset, 0);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c write failed ... \n",__func__,__LINE__);
		goto err;
	}

	ret = i2c_smbus_read_i2c_block_data(save_client, 0 , 4, val);
	if(ret < 0){
		msp_err(KERN_CRIT "[%s][%d] i2c read failed ... \n",__func__,__LINE__);
		goto err;
	}

	//ret = msp430_read_version(buf);
	//msp_log(KERN_INFO "msp430 version : %s \n",buf);

	//add sysfs for pba check
	ret = sysfs_create_group(&client->dev.kobj, &msp_attr_group);
	if(ret){
		msp_err(KERN_CRIT "%s, Failed to create msp sysfs group\n", __func__);
		goto err_sysfs_create_group;

	}

	msp_log(KERN_INFO "msp430 probe done! \n");
	return 0;

err_sysfs_create_group:
	sysfs_remove_group(&client->dev.kobj, &msp_attr_group);

err:
	//i2c_del_driver(&msp430_driver);
	class_destroy(msp430_class);
	cdev_del(&msp430_cdev);
	return ret;
}

static int msp430_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id msp430_dt_ids[] = {
	{ .compatible = "ti,msp430", },
	{ }
};

MODULE_DEVICE_TABLE(of, msp430_dt_ids);

static struct i2c_driver msp430_driver = {
	.driver = {
		.name	= "msp430",
		.of_match_table = msp430_dt_ids,
	},
	.probe		= msp430_probe,
	.remove		= msp430_remove,
	.id_table	= msp430_id,
};

static int msp430_init(void)
{

	int ret = 0;

	msp430_dev_major = 0;
	msp430_dev_num = 0;

	__mutex_init(&msp_mutex,0,0);
	
	ret = alloc_chrdev_region(&msp430_dev_num, 0, 1, MSP430_DEV_NAME);
	if(ret < 0) {
		msp_err(KERN_CRIT "[%s][%d] \"alloc_chrdev_region\" Fail..\n", __func__, __LINE__);
	}

	cdev_init(&msp430_cdev, &msp430_fops);
	msp430_cdev.owner = THIS_MODULE;

	ret = cdev_add(&msp430_cdev, msp430_dev_num, 1);
	if(ret) {
		msp_err(KERN_CRIT "[%s][%d] \"cdev_add\" Fail..\n", __func__, __LINE__);
	}

	msp430_class = class_create(THIS_MODULE, MSP430_DEV_NAME);
	if (IS_ERR(msp430_class)) 
	{
		msp_err(KERN_CRIT "[%s][%d] \"class_create\" Fail..\n", __func__, __LINE__);
		goto fail_class_create;
	}

	msp430_device = device_create(msp430_class, NULL, msp430_dev_num, NULL, MSP430_DEV_NAME);
	if(IS_ERR(msp430_device)) {
		goto fail_device_create;
	}
	return i2c_add_driver(&msp430_driver);

fail_device_create:
	class_destroy(msp430_class);
fail_class_create:
	cdev_del(&msp430_cdev);
	
	return 0;
}
module_init(msp430_init);

static void __exit msp430_exit(void)
{
	mutex_destroy(&msp_mutex);
	
	sysfs_remove_group(&save_client->dev.kobj, &msp_attr_group);

	i2c_del_driver(&msp430_driver);
	
	if (msp430_device)
	{
		device_destroy(msp430_class, msp430_dev_num);
	}
	if (msp430_class)
	{
		class_destroy(msp430_class);
	}
	cdev_del(&msp430_cdev);

}
module_exit(msp430_exit);

MODULE_AUTHOR("jhee jeong <jhee.jeong@samsung.com>");
MODULE_DESCRIPTION("msp430 driver");
MODULE_LICENSE("GPL");

