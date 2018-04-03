#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/input.h>
//#include "../../include/mt-plat/mt_gpio.h"
#include <asm-generic/gpio.h>
#include "mt_boot_common.h"
#include "ln4913.h"

struct input_dev *hall_input_dev;

#define HALL_EINT_PIN_PLUG_OPEN        (1)
#define HALL_EINT_PIN_PLUG_CLOSE       (0)
static unsigned int g_cur_eint_state = HALL_EINT_PIN_PLUG_CLOSE;

static struct switch_dev ln4913_data;
static unsigned int sys_boot_mode = 0;

int hall_irq;
unsigned int hall_eint_type;

#define HALL_SENSOR_NAME	"ln4913"

static struct work_struct hall_eint_work;
static struct workqueue_struct *hall_eint_workqueue;

enum ln4913_report_state
{
    STATE_NEAR =0,
    STATE_FAR = 1,
};


static int HALL_DEBUG_enable = 1;
#define HALL_DEBUG(format, args...) do { \
	if (HALL_DEBUG_enable) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)
static unsigned long GPIO_HALL_INT_NUM;
static unsigned int GPIO_HALL_INT_VALUE;
unsigned int g_is_suspend; 
EXPORT_SYMBOL(g_is_suspend);

void hall_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt8173-hall");

	GPIO_HALL_INT_NUM 	= of_get_named_gpio(node, "hall_int", 0);
	GPIO_HALL_INT_VALUE = __gpio_get_value(GPIO_HALL_INT_NUM);

	//GPIO_HALL_INT_VALUE = 1;//远离 GPIO_HALL_INT_VALUE = 0;//靠近
}
	
static void hall_eint_work_callback(struct work_struct *work)
{
	if(g_cur_eint_state ==  HALL_EINT_PIN_PLUG_OPEN ) 
	{
		switch_set_state((struct switch_dev *)&ln4913_data, STATE_NEAR);
		HALL_DEBUG("[ln4913] hall close\n");
	}	
	else
	{
		switch_set_state((struct switch_dev *)&ln4913_data, STATE_FAR);
		HALL_DEBUG("[ln4913] hall open\n");
	}	
	printk("wisky_hall: enter \n");
	if (sys_boot_mode != FACTORY_BOOT)
	{
		//熄屏且远离  亮屏且靠近
		//GPIO_HALL_INT_VALUE = 1;//远离
		//GPIO_HALL_INT_VALUE = 0;//靠近
		GPIO_HALL_INT_VALUE = __gpio_get_value(GPIO_HALL_INT_NUM);
		printk("wisky_hall: g_is_suspend = %d, GPIO_HALL_INT_VALUE = %d\n", g_is_suspend, GPIO_HALL_INT_VALUE);
		if ((g_is_suspend == 0 && GPIO_HALL_INT_VALUE == 0) || (g_is_suspend == 1 && GPIO_HALL_INT_VALUE == 1))
		{
			input_report_key(hall_input_dev, KEY_POWER, 1);//按下
			input_sync(hall_input_dev);
			input_report_key(hall_input_dev, KEY_POWER, 0);//弹起
			input_sync(hall_input_dev);
		}	
	}
	enable_irq(hall_irq);
}

static irqreturn_t ln4913_eint_func(int irq,void *data)
{

	HALL_DEBUG("[ln4913]ln4913_eint_func enter,g_cur_eint_state=%d,hall_eint_type=%d\n",g_cur_eint_state,hall_eint_type);

	disable_irq_nosync(hall_irq);

	g_cur_eint_state = !g_cur_eint_state;
	if (hall_eint_type == IRQ_TYPE_EDGE_RISING)
	{
			hall_eint_type = IRQ_TYPE_EDGE_FALLING;
	}
	else
	{
		hall_eint_type = IRQ_TYPE_EDGE_RISING;
	}

	irq_set_irq_type(hall_irq, hall_eint_type);
	
	queue_work(hall_eint_workqueue, &hall_eint_work);
				
	return IRQ_HANDLED;	
	
}

static inline int ln4913_setup_eint(void) 
{
	int ret;
	struct device_node *node;
	unsigned int ln4913_int_pin = 0;
	hall_eint_type = IRQ_TYPE_EDGE_FALLING;
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt8173-hall");

	ln4913_int_pin = of_get_named_gpio(node, "hall_int", 0);
	ret = gpio_request_one(ln4913_int_pin, GPIOF_OUT_INIT_HIGH,
					 "hall_int");
	if (ret < 0) {
			HALL_DEBUG("Unable to request gpio hall_int\n");
	}
	gpio_direction_input(ln4913_int_pin);	
	
	if (node) 
	{
		hall_irq = irq_of_parse_and_map(node, 0);
		HALL_DEBUG("[hall]hall_irq=%d", hall_irq);
		ret = request_irq(hall_irq, ln4913_eint_func,IRQ_TYPE_EDGE_FALLING, "hall_int_default", NULL);//wisky-lxh@20160505	
		if (ret > 0)
			HALL_DEBUG("[hall]EINT IRQ LINE NOT AVAILABLE\n");
		else {
			HALL_DEBUG("[hall]hall set EINT finished, hall_irq=%d\n",hall_irq);
		}		
		//enable_irq_wake(hall_irq);	
	}

	return 0;
}


static int ln4913_probe(struct platform_device *dev)
{
	int r=0;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_int;

	
	HALL_DEBUG("[ln4913]ln4913_probe !\n");
	
	hall_input_dev = input_allocate_device();
	if (!hall_input_dev)
		return -ENOMEM;
	
	hall_input_dev->name = HALL_SENSOR_NAME;
	__set_bit(EV_KEY, hall_input_dev->evbit);
	input_set_capability(hall_input_dev, EV_KEY, KEY_POWER);
	r = input_register_device(hall_input_dev);
	if (r) {
		printk("hall sensor register input device failed (%d)\n", r);
		input_free_device(hall_input_dev);
		return r;
	}

	#if 1
	
	pinctrl = devm_pinctrl_get(&dev->dev);
	if (IS_ERR(pinctrl)) {
		r = PTR_ERR(pinctrl);
		dev_err(&dev->dev, "hall INT pin, failure of setting\n");
	} else {
		pins_int = pinctrl_lookup_state(pinctrl, "hall_int_default");
		if (IS_ERR(pins_int)) {
			r = PTR_ERR(pins_int);
			dev_err(&dev->dev, "cannot find hall INT pin\n");
		} else {
			r = pinctrl_select_state(pinctrl, pins_int);
		}
	}
	#endif
	hall_get_gpio_infor();
	ln4913_setup_eint();
	ln4913_data.name = "hall";
	ln4913_data.index = 0;
	ln4913_data.state = STATE_FAR;	
	r = switch_dev_register(&ln4913_data);
	if(r)
	{
		HALL_DEBUG("[ln4913]switch_dev_register returned:%d!\n", r);
		return r;
	}
	switch_set_state((struct switch_dev *)&ln4913_data, STATE_FAR);
	hall_eint_workqueue = create_singlethread_workqueue("hall_eint");
	INIT_WORK(&hall_eint_work, hall_eint_work_callback);
	device_init_wakeup(&dev->dev, 1);	
  sys_boot_mode = get_boot_mode();
    
	return 0;
}

static int ln4913_remove(struct platform_device *dev)
{
	destroy_workqueue(hall_eint_workqueue);
	switch_dev_unregister(&ln4913_data);
	return 0;
}

static int hall_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_debug("Enter %s\n",__FUNCTION__);

        if (device_may_wakeup(&pdev->dev))
                enable_irq_wake(hall_irq);

        /*
         * IRQ must be disabled during suspend because if it happens
         * while suspended it will be handled before resuming I2C.
         *
         * When device is woken up from suspend (e.g. by RTC wake alarm),
         * an interrupt occurs before resuming I2C bus controller.
         * Interrupt handler tries to read registers but this read
         * will fail because I2C is still suspended.
         */
        disable_irq(hall_irq);

	return 0;
}

static int hall_resume(struct platform_device *pdev)
{
	pr_debug("Enter %s\n",__FUNCTION__);

        if (device_may_wakeup(&pdev->dev))
                disable_irq_wake(hall_irq);

        enable_irq(hall_irq);

	return 0;
}

#if 0//defined(CONFIG_OF)
struct platform_device ln4913_device = {
	.name	  ="ln4913_Driver",
	.id		  = -1,
	/* .dev    ={ */
	/* .release = ln4913_dumy_release, */
	/* } */
};

#endif
static const struct of_device_id ln4913_id[] = {
	{.compatible = "mediatek,mt8173-hall"},
	{},
};
MODULE_DEVICE_TABLE(of, ln4913_id);

static struct platform_driver ln4913_driver = {
	.probe = ln4913_probe,
	.suspend      = hall_suspend, 
	.resume               = hall_resume,
	.remove = ln4913_remove,
	.driver = {
		   .name = "ln4913_Driver",
		   .of_match_table = of_match_ptr(ln4913_id),	
		   },
};

static int ln4913_mod_init(void)
{
	int ret = 0;

	HALL_DEBUG("[ln4913]ln4913_mod_init begin!\n");

#if 0//defined(CONFIG_OF)
    ret = platform_device_register(&ln4913_device);
    printk("[%s]: ln4913_device, retval=%d \n!", __func__, ret);

	if (ret != 0)
	{
		printk("platform device ln4913 register error:(%d)\n", ret);
		return ret;
	}
	else
	{
		printk("platform device ln4913 registerr done!\n");
	}
#endif	


	ret = platform_driver_register(&ln4913_driver);
	if (ret) {
		HALL_DEBUG("[ln4913]platform driver register error:(%d)\n", ret);
		return ret;
	} else {
		HALL_DEBUG("[ln4913]platform driver register done!\n");
	}

	HALL_DEBUG("[ln4913]ln4913_mod_init done!\n");
	return 0;

}

static void ln4913_mod_exit(void)
{
	HALL_DEBUG("[ln4913]ln4913_mod_exit\n");
	platform_driver_unregister(&ln4913_driver);

	HALL_DEBUG("[ln4913]ln4913_mod_exit Done!\n");
}

module_init(ln4913_mod_init);
module_exit(ln4913_mod_exit);

MODULE_DESCRIPTION("WISKY Hall sensor driver");
MODULE_AUTHOR("liuxianghui <liuxianghui@wisky.com.cn>");
MODULE_LICENSE("GPL");
