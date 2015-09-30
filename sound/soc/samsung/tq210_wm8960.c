/*
 *  smdk_wm8960.c
 *
 *  Copyright (c) 2009 Samsung Electronics Co. Ltd
 *  Author: Jaswinder Singh <jassi.brar@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/mach-types.h>

#include "../codecs/wm8960.h"
#include "i2s.h"


static int set_epll_rate(unsigned long rate)
{
	struct clk *fout_epll;

	fout_epll = clk_get(NULL, "fout_epll");
	if (IS_ERR(fout_epll)) {
		printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return -ENOENT;
	}

	if (rate == clk_get_rate(fout_epll))
		goto out;

	clk_set_rate(fout_epll, rate);
	
out:
	clk_put(fout_epll);

	return 0;
}

static int smdk_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int rclk, psr = 1;
	int bfs, rfs, ret;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
		bfs = 48;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		break;
	default:
		return -EINVAL;
	}
	
	printk( "-%s(): format=%d bfs=%d\n", __FUNCTION__, params_format(params), bfs);

	/* The Fvco for WM8960 PLLs must fall within [90,100]MHz.
	* This criterion can't be met if we request PLL output
	* as {8000x256, 64000x256, 11025x256}Hz.
	* As a wayout, we rather change rfs to a minimum value that
	* results in (params_rate(params) * rfs), and itself, acceptable
	* to both - the CODEC and the CPU.
	*/
	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
			break;
	default:
		return -EINVAL;
	}

	printk( "-%s(): rate=%d rfs=%d\n", __FUNCTION__, params_rate(params), rfs);

	rclk = params_rate(params) * rfs;

	switch (rclk) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;
	case 67737600:
	case 73728000:
		psr = 1;
		break;
	default:
		printk(KERN_ERR "Not yet supported!\n");
		return -EINVAL;
	}

	printk( "-%s(): rclk=%d psr=%d\n", __FUNCTION__, rclk, psr);

	set_epll_rate(rclk * psr);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if( ret < 0 ){
		printk( "-%s(): Codec DAI configuration error, %d\n", __FUNCTION__, ret );
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if( ret < 0 ){
		printk( "-%s(): AP DAI configuration error, %d\n", __FUNCTION__, ret );
		return ret;
	}
	
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_RCLKSRC_1, 0, SND_SOC_CLOCK_IN);
	if( ret < 0 ){
		printk( "-%s(): AP RCLKSRC_1 setting error, %d\n", __FUNCTION__, ret );
		return ret;
	}
	
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK, rfs , SND_SOC_CLOCK_OUT);
	if( ret < 0 ){
		printk( "-%s(): AP CDCLK setting error, %d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if( ret < 0 ){
		printk( "-%s(): AP BCLK setting error, %d %d\n", __FUNCTION__, ret, bfs);
		return ret;
	}

	return 0;
}

/*
 * SMDK WM8960 DAI operations.
 */
static struct snd_soc_ops smdk_ops = {
	.hw_params = smdk_hw_params,
};

/* SMDK Playback widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_pbk[] = {
        SND_SOC_DAPM_SPK("Speaker", NULL),
        SND_SOC_DAPM_HP("HP", NULL),
};

/* SMDK Capture widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_cpt[] = {
        SND_SOC_DAPM_MIC("MicIn", NULL),
};

/* SMDK-PAIFTX connections */
static const struct snd_soc_dapm_route audio_map_tx[] = {
        /* MicIn feeds LINPUT1/2 */
        {"LINPUT1", NULL, "MicIn"},
};

/* SMDK-PAIFRX connections */
static const struct snd_soc_dapm_route audio_map_rx[] = {
        {"Speaker", NULL, "SPK_LP"},
        {"Speaker", NULL, "SPK_LN"},
        {"Speaker", NULL, "SPK_RP"},
        {"Speaker", NULL, "SPK_RN"},
        {"HP", NULL, "HP_L"},
        {"HP", NULL, "HP_R"},
};

static int smdk_wm8960_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add smdk specific Capture widgets */
	snd_soc_dapm_new_controls(dapm, wm8960_dapm_widgets_cpt,
				  ARRAY_SIZE(wm8960_dapm_widgets_cpt));

	/* Set up PAIFTX audio path */
	snd_soc_dapm_add_routes(dapm, audio_map_tx, ARRAY_SIZE(audio_map_tx));

	/* Enabling the microphone requires the fitting of a 0R
	 * resistor to connect the line from the microphone jack.
	 */
	snd_soc_dapm_disable_pin(dapm, "MicIn");

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);

	return 0;
}

static int smdk_wm8960_init_paifrx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add smdk specific Playback widgets */
	snd_soc_dapm_new_controls(dapm, wm8960_dapm_widgets_pbk,
				  ARRAY_SIZE(wm8960_dapm_widgets_pbk));

	/* Set up PAIFRX audio path */
	snd_soc_dapm_add_routes(dapm, audio_map_rx, ARRAY_SIZE(audio_map_rx));

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);

	return 0;
}

enum {
	PRI_PLAYBACK = 0,
	PRI_CAPTURE,
	SEC_PLAYBACK,
};

static struct snd_soc_dai_link smdk_dai[] = {
	[PRI_PLAYBACK] = { /* Primary Playback i/f */
		.name = "WM8960 PAIF RX",
		.stream_name = "Playback",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-audio",
		.codec_name = "wm8960.0-001a",
		.init = smdk_wm8960_init_paifrx,
		.ops = &smdk_ops,
	},
	[PRI_CAPTURE] = { /* Primary Capture i/f */
		.name = "WM8960 PAIF TX",
		.stream_name = "Capture",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-audio",
		.codec_name = "wm8960.0-001a",
		.init = smdk_wm8960_init_paiftx,
		.ops = &smdk_ops,
	},
};

static struct snd_soc_card smdk = {
	.name = "SMDK-I2S",
	.dai_link = smdk_dai,
	.num_links = 2,
};

static int __devinit smdk_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &smdk;

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed:%d\n", ret);

	return ret;
}

static int __devexit smdk_audio_remove(struct platform_device *pdev)
{
 	struct snd_soc_card *card = platform_get_drvdata(pdev);
 
 	snd_soc_unregister_card(card);
 
 	return 0;
}

static struct platform_driver smdk_audio_driver = {
	.driver		= {
		.name	= "smdk-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= smdk_audio_probe,
	.remove		= __devexit_p(smdk_audio_remove),
};

module_platform_driver(smdk_audio_driver);

MODULE_AUTHOR("Jaswinder Singh, jassi.brar@samsung.com");
MODULE_DESCRIPTION("ALSA SoC SMDK WM8960");
MODULE_LICENSE("GPL");
