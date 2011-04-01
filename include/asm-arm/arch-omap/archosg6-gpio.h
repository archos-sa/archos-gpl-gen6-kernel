#ifndef _ARCH_ARCHOSG6_GPIO_H_
#define _ARCH_ARCHOSG6_GPIO_H_


#define _EXIST(x) ( x.nb > 0 )
#define _MUX_CFG(x) ( x.mux_cfg )
#define _PIN_NB(x) ( x.nb )

#define _INIT_OUTPUT(x) {\
		if ( _EXIST(x) ) {\
			omap_cfg_reg( _MUX_CFG(x) ) ;\
			if ( omap_request_gpio( _PIN_NB(x) ) < 0 )\
				printk(KERN_ERR "init: cannot acquire GPIO%d \n", _PIN_NB(x) );\
			omap_set_gpio_direction( _PIN_NB(x), GPIO_DIR_OUTPUT );\
		}\
}


#define _INIT_INPUT(x) {\
		if ( _EXIST(x) ) {\
			omap_cfg_reg( _MUX_CFG(x) ) ;\
			if ( omap_request_gpio( _PIN_NB(x) ) < 0 )\
				printk(KERN_ERR "init: cannot acquire GPIO%d \n", _PIN_NB(x) );\
			omap_set_gpio_direction( _PIN_NB(x), GPIO_DIR_INPUT );\
		}\
}

struct g6_gpio {
	int nb;
	int mux_cfg;
};

#endif
