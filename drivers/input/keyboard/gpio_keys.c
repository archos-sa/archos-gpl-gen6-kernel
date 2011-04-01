/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/signal.h>

#include <asm/gpio.h>

#ifdef CONFIG_KEYBOARD_COREDUMPS_AVOS
static int is_avos(struct task_struct * p)
{
	if (p->pid) {
		char *c, *d = "avos";
		for (c = p->comm; *c != 0 ; ++c){
			if (*c == *d){
				++d;
			}
			if (*d == 0 ) {
				if (*(c+1) == 0) {
					// avos$, so either arm_avos or avos
					return 1;
				} else {
					// avos.+, so probably avos_helper.sh
					return 0;
				}
			}
		}
	}
	return 0;
}

static void avos_kill(struct work_struct *work)
{
	struct task_struct *g, *p;
	struct mm_struct *mm = NULL;

	read_lock(&tasklist_lock);

	do_each_thread(g, p)
		if (is_avos(p)) {
			mm = get_task_mm(p);
			printk(KERN_ERR "Killing process %d (%s).\n", p->pid, p->comm);
			// we kill the process group, so no need to send signal to all threads
			group_send_sig_info(SIGABRT, (void*)1L, p);
			goto done;
		}
	while_each_thread(g, p);
done:
	read_unlock(&tasklist_lock);
	if (mm)
		mmput(mm);

	return;
}

static DECLARE_WORK(kill_work,(work_func_t)avos_kill);

static void check_magic_keypress(struct gpio_keys_platform_data *pdata)
{
	int i;
	int keys_down = 0;
	static int avos_killed = 0;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int gpio = button->gpio;
		int code = button->code;
		
		if (code == KEY_VOLUMEUP || code == KEY_VOLUMEDOWN) {
			int state = (gpio_get_value(gpio) ? 1 : 0) ^ button->active_low;
			if (state)
				keys_down++;
		}
	}

	/* if both keys pressed ...*/
	if (keys_down == 2) {
		if (!avos_killed)
			schedule_work(&kill_work);
		avos_killed = 1;
	} else
		avos_killed = 0;
}
#endif

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	int i;
	struct platform_device *pdev = dev_id;
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int gpio = button->gpio;

		if (irq == gpio_to_irq(gpio)) {
			unsigned int type = button->type ?: EV_KEY;
			int state = (gpio_get_value(gpio) ? 1 : 0) ^ button->active_low;

			input_event(input, type, button->code, !!state);
			input_sync(input);
		}
	}

#ifdef CONFIG_KEYBOARD_COREDUMPS_AVOS
	check_magic_keypress(pdata);
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;
	
	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, pdev);
	}
	return 0;
}

static int gpio_keys_resume(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int error;
	int i;
	
	// register the gpio interrupts for the different keys
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int irq = gpio_to_irq(button->gpio);

		error = request_irq(irq, gpio_keys_isr, IRQF_SAMPLE_RANDOM|
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				     button->desc ? button->desc : "gpio_keys",
				     pdev);
		if (error) {
			printk(KERN_ERR "gpio-keys: unable to claim irq %d; error %d\n",
				irq, error);
		}
	}
	return 0;
}
#endif

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	int i, error;

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	platform_set_drvdata(pdev, input);

	input->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	// register the gpio interrupts for the different keys
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int irq = gpio_to_irq(button->gpio);
		unsigned int type = button->type ?: EV_KEY;

		error = request_irq(irq, gpio_keys_isr, IRQF_SAMPLE_RANDOM|
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				     button->desc ? button->desc : "gpio_keys",
				     pdev);
		if (error) {
			printk(KERN_ERR "gpio-keys: unable to claim irq %d; error %d\n",
				irq, error);
			goto fail;
		}

		input_set_capability(input, type, button->code);
	}

	// register the keyboard driver at the linux input layer
	error = input_register_device(input);
	if (error) {
		printk(KERN_ERR "Unable to register gpio-keys input device\n");
		goto fail;
	}

	// start condition of the keyboard state (for the recovery)
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int gpio = button->gpio;

		unsigned int type = button->type ?: EV_KEY;
		int state = (gpio_get_value(gpio) ? 1 : 0) ^ button->active_low;

		input_event(input, type, button->code, !!state);
		input_sync(input);
	}

	// exit succeed
	return 0;

	// exit fail
 fail:
	for (i = i - 1; i >= 0; i--)
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), pdev);

	input_free_device(input);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, pdev);
	}

	input_unregister_device(input);

	return 0;
}

struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
#ifdef CONFIG_PM
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
#endif
	.driver		= {
		.name	= "gpio-keys",
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
