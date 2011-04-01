#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/msp430.h>

#include <asm/arch/resource.h>

#include "prcm-regs.h"

#define PRCM_INTERRUPT_MASK	(1 << 11)
#define UART3_INTERRUPT_MASK	(1 << 10)

static struct constraint_handle *vdd1_opp_co;
static struct constraint_id reset_constr = {
	.type = RES_FREQ_CO,
	.data = (void*)"vdd1_opp",
};

void complete_board_wakeup(void)
{
}

void  init_wakeupconfig(void)
{
}

void setup_board_wakeup_source(u32 wakeup_source)
{
	/* Unmasking the PRCM interrupts */
	INTC_MIR_0 &= ~PRCM_INTERRUPT_MASK;
}

void archosg6_power_off(void)
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_SHUTDOWN, 0,0,0 };

	/* make sure VDD1 is in OPP3 before reset */
	vdd1_opp_co = constraint_get("reset", &reset_constr);
	constraint_set(vdd1_opp_co, 3);

	atmega_write_table( &table );
}

void archosg6_reset_board(void)
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_DAVINCI, 0,0,0 };

	/* make sure VDD1 is in OPP3 before reset */
	vdd1_opp_co = constraint_get("reset", &reset_constr);
	constraint_set(vdd1_opp_co, 3);
	
	atmega_write_table( &table );
}
