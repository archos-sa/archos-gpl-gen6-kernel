/*
 * linux/arch/arm/mach-omap2/pwm-gp.c
 *
 * OMAP2 GP timer and pwm support.
 *
 * inspired by timer-gp.c 
 *
 */
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach/time.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/mux.h>
#include <asm/arch/pwm-gp.h>

extern void gp_irblaster_pwm_specific_init(void);
extern void gp_irblaster_ctrl_specific_init(void);
extern void omap_dm_timer_post_stop(struct omap_dm_timer *timer);
extern void omap_dm_timer_post_start(struct omap_dm_timer *timer);

static void gp_lcd_bkl_pwm_specific_init(void) 
{
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	pwm_gpt_start( LCD_BKL );
	pwm_gpt_set_speed( LCD_BKL, 10000, 200 );
#else
	/* set modulation mod */
	/* trig on overflow and match */
	omap_dm_timer_set_pwm(gpt_pwm_list[LCD_BKL].timer,0,1,2);
	omap_dm_timer_disable(gpt_pwm_list[LCD_BKL].timer);
#endif
	if ( gpt_pwm_list[LCD_BKL].mux_config )
		omap_cfg_reg(gpt_pwm_list[LCD_BKL].mux_config);
}

struct omap_pwm_timer gpt_pwm_list[] = {
#ifdef CONFIG_IRBLASTER_G6
	[ IRBLASTER_PWM ] = {
			.no = 8,
			.name = "irblaster pwm",
			.source = OMAP_TIMER_SRC_SYS_CLK,
			.init = NULL,
			.mux_config = AD25_3430_GPT08,
	}, 
	[ IRBLASTER_TIMER_CTRL ] = {
			.no = 3,
			.name = "irblaster control",
			.source = OMAP_TIMER_SRC_SYS_CLK,
			.init = NULL,
	},
#endif
	[ LCD_BKL ] = {
			.no = 10,
			.name = "lcd backlight pwm",
			.source = OMAP_TIMER_SRC_SYS_CLK,
			.init = gp_lcd_bkl_pwm_specific_init,
			.mux_config = AB25_3430_GPT10,
	},
};
EXPORT_SYMBOL(gpt_pwm_list);

#define PWM_NUMBER ARRAY_SIZE(gpt_pwm_list)

static inline int _check_id(int id) {
	if (id >= PWM_NUMBER) {
		printk(KERN_DEBUG "invalid pwm id %i\n", id);
		return 0;
	}
	return 1;
}

static inline int _check_timer(int id) {

	if ( !gpt_pwm_list[id].timer) {
		printk(KERN_DEBUG "no gpt associated with: %s\n", gpt_pwm_list[id].name);
		return 0;
	}

	return 1;
}

void pwm_gpt_dump(int id) {
	
	int i;

	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	for (i=0;i<(17*4);i+=4)
		printk("0x%02x: 0x%08x\n", i, omap_dm_timer_read_reg(gpt_pwm_list[id].timer,i));
}

void pwm_gpt_set_speed(int id, int frequency, int duty_cycle) {

	u32 val;
	u32 period;
	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */


	period = (gpt_pwm_list[id].rate/frequency);
	gpt_pwm_list[id].period = period;
	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt_pwm_list[id].timer, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt_pwm_list[id].timer, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt_pwm_list[id].timer,0xFFFFFFFE);
}


void pwm_gpt_start(int id) {

	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	/* enable the timer, start the clock */
	omap_dm_timer_enable(gpt_pwm_list[id].timer);

	if (id == LCD_BKL)
		omap_dm_timer_set_pwm(gpt_pwm_list[id].timer,0,1,2);

/*	do not wait for ack, just set up the reg and leave */
/*	this is better if you setup 2 correlated timers */
	omap_dm_timer_post_start(gpt_pwm_list[id].timer);
}


void pwm_gpt_stop(int id) {

	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	omap_dm_timer_stop(gpt_pwm_list[id].timer);
	omap_dm_timer_disable(gpt_pwm_list[id].timer);
}

static int __init omap2_gp_pwm_init(void)
{
	int i;

	omap_dm_timer_init();
	/* Set MUX to PWM */
	/* pwm request */
	for (i=0; i < PWM_NUMBER; i++) {
		struct omap_dm_timer *gpt = NULL;

		if ( gpt_pwm_list[i].no > 0 )
			gpt = omap_dm_timer_request_specific(gpt_pwm_list[i].no);
		else if ( gpt_pwm_list[i].no < 0 )
			gpt = omap_dm_timer_request();

		if (gpt != NULL) {
			gpt_pwm_list[i].timer = gpt;
			omap_dm_timer_set_source(gpt, gpt_pwm_list[i].source);
			gpt_pwm_list[i].rate = clk_get_rate(omap_dm_timer_get_fclk(gpt));
			printk(KERN_DEBUG "%s timer rate is: %d\n",gpt_pwm_list[i].name,gpt_pwm_list[i].rate);
		} else if (gpt_pwm_list[i].no != 0)
			printk(KERN_ERR "%s: failed to request dm-timer\n" , gpt_pwm_list[i].name);

	}
	
	/* pwm default init */
	for (i=0;i<PWM_NUMBER;i++) {
		switch(i) {
		case IRBLASTER_PWM:
		case IRBLASTER_TIMER_CTRL:
		case LCD_BKL:
			if ( gpt_pwm_list[i].init )
				(*gpt_pwm_list[i].init)();
			else
				/* if no init function exists, make sure timer
				 * is disabled.
				 */
				omap_dm_timer_disable(gpt_pwm_list[i].timer);
			break;
		default:
			break;
		}
	}
 
	return 0;

}

subsys_initcall(omap2_gp_pwm_init);

EXPORT_SYMBOL(pwm_gpt_dump);
EXPORT_SYMBOL(pwm_gpt_set_speed);
EXPORT_SYMBOL(pwm_gpt_start);
EXPORT_SYMBOL(pwm_gpt_stop);
