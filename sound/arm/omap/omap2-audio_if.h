/*
 * sound/arm/omap/omap2-audio_if.h
 *
 * OMAP alsa codec interface
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Based on: sound/oss/omap2-audio.h
 * Copyright (C) 2004-2006 Texas Instruments, Inc.
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
 *
 */

#ifndef __OMAP_AUDIO_IF_H
#define __OMAP_AUDIO_IF_H

// #include <sound/typedefs.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>

/* Codec specific information and function pointers.
 * Codec driver is responsible for defining the function pointers.
 */
struct omap_alsa_codec {
	char 	*name;

	struct snd_pcm_hw_constraint_list *pcm_hw_constraint_list;
	struct snd_pcm_hardware *pcm_hardware_playback;
	struct snd_pcm_hardware *pcm_hardware_capture;

	int	(*codec_probe)(void);
	int	(*codec_init)(void);
	void	(*codec_shutdown)(void);
	int     (*codec_sidle)(u32 level);
	int	(*codec_set_samplerate)(long);
#ifdef CONFIG_ARCH_OMAP3
	int	(*codec_set_stereomode)(int mode, int dsp);
#endif
	int	(*codec_default_samplerate)(void);
	int	(*codec_transfer_init)(int mode);
	int	(*codec_transfer_start)(int mode, void *buffer_phy,u32 size, void *arg);
	int	(*codec_transfer_stop)(int mode);
	int	(*codec_transfer_posn)(int mode);

	int	(*mixer_init)(struct snd_card *card);
	int	(*mixer_shutdown)(struct snd_card *card);
};

extern int audio_register_codec(struct omap_alsa_codec *codec);
extern int audio_unregister_codec(struct omap_alsa_codec *codec);
#if defined(CONFIG_SND_OMAP_3430SDP) || defined (CONFIG_SND_OMAP3_EVM)
extern void audio_period_handler(void *arg);
#endif

#endif	/* End of #ifndef __OMAP_AUDIO_IF_H */
