#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <mach/upmu_sw.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#include "lq101r1sx01a_wqxga_dsi_vdo.h"


static int lcm_request_gpio_control(struct device *dev)
{
	 struct regulator *reg;
	int ret;
	GPIO_LCD_PWR_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr_en", 0);
	ret = gpio_request(GPIO_LCD_PWR_EN, "GPIO_LCD_PWR_EN");
	if (ret) {
		printk("[KE/LCM] gpio request GPIO_LCD_PWR_EN = 0x%x fail with %d\n", GPIO_LCD_PWR_EN, ret);
	}
	
	GPIO_LED = of_get_named_gpio(dev->of_node, "gpio_led", 0);
	ret = gpio_request(GPIO_LED, "GPIO_LED");
	if (ret) {
		printk("[KE/LCM] gpio request gpio_led = 0x%x fail with %d\n", GPIO_LED, ret);
	}
	
	GPIO_LCD_RESET_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);
	ret = gpio_request(GPIO_LCD_RESET_EN, "GPIO_LCD_RESET_EN");
	if (ret) {
		printk("[KE/LCM] gpio request GPIO_LCD_RESET_EN = 0x%x fail with %d\n", GPIO_LCD_RESET_EN, ret);
	}
	
      reg= regulator_get(dev, "LCM1V8");
       if (IS_ERR(reg))
                printk("regulator_get() vgp4 failed!\n");
        else
        {
                ret = regulator_set_voltage(reg, 1800000,1800000);     
                ret = regulator_enable(reg);   // enable regulator 
        if (ret)
               printk("wisky enable reg failed!\n");
        }

/*
	GPIO_LCD_PWR2_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_pwr2_en", 0);
	ret = gpio_request(GPIO_LCD_PWR2_EN, "GPIO_LCD_PWR2_EN");
	if (ret) {
		pr_err("[KE/LCM] gpio request GPIO_LCD_PWR_EN = 0x%x fail with %d\n", GPIO_LCD_PWR_EN, ret);
	}
	*/
	/*
	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_bl_en", 0);
	ret = gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	if (ret) {
		pr_err("[KE/LCM] gpio request GPIO_LCD_BL_EN = 0x%x fail with %d\n", GPIO_LCD_BL_EN, ret);
	}
	*/
	return ret;
}

static int lcm_probe(struct device *dev)
{
	int ret;

	printk("%s\n",__func__);
	ret = lcm_request_gpio_control(dev);
	if (ret)
	{
			pr_err("lq101r1sx01a_wqxga_dsi_vdo lcm_request_gpio_control fail\n");
			return ret;
	}
	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,mt8173-lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "lq101r1sx01a_wqxga_dsi_vdo",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_err("LCM: Register panel driver for lq101r1sx01a_wqxga_dsi_vdo\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_err("LCM: Unregister this driver done\n");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
