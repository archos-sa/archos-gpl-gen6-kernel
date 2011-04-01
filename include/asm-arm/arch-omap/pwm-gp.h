#ifndef __ASM_ARCH_PWM_H
#define __ASM_ARCH_PWM_H

enum {
	IRBLASTER_PWM = 0,
	IRBLASTER_TIMER_CTRL,
	LCD_BKL,
};

struct omap_pwm_timer {
	char *name;
	int no;
	int source;
	struct omap_dm_timer *timer;
	int rate;
	unsigned int period;
	void (*init)(void);
	int mux_config;
};

extern struct omap_pwm_timer gpt_pwm_list[];

extern void pwm_gpt_dump(int id);
extern void pwm_gpt_start(int id);
extern void pwm_gpt_stop(int id);
extern void pwm_gpt_set_speed(int id, int frequency, int cycle);
#endif
