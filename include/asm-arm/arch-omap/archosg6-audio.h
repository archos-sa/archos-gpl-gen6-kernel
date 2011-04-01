#ifndef _ARCH_ARCHOSG6_AUDIO_H_
#define _ARCH_ARCHOSG6_AUDIO_H_

#define PLAT_ON 1
#define PLAT_OFF 0

struct audio_device_gpiop_config {
	int nb;
	int mux_cfg;
};

struct audio_device_config {
	void (*set_spdif)(int onoff);
	int (*get_headphone_plugged)(void);
	void (*set_codec_master_clk_state)(int state);
	void (*set_speaker_state)(int state);
};

struct audio_device_config * archosg6_audio_get_io(void);

#endif
