/*
 * lm-imx-wm8960.c
 *
 *  Created on: Sep 1, 2015
 *      Author: hcl
 */


/*
 * Copyright (C) 2015 DATA RESPONS AS
 *
 * Based on imx-wm8962.c
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Based on imx-sgtl5000.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/pinctrl/consumer.h>

#include "../codecs/wm8960.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_wm8960_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
	struct clk *codec_clk;
	bool stream_active[2];
};

struct imx_priv {
	int hp_gpio;
	int hp_active_level;
	int mic_gpio;
	int mic_active_level;
	int iphone_jack;
	int spk_amp_gpio;
	int spk_amp_active_level;
	bool auto_switch_speaker;
	bool broken_mic_detect;
	int hsjack_last_status;
	struct snd_soc_codec *codec;
	struct platform_device *pdev;
	struct snd_kcontrol *headphone_kctl;
	struct snd_card *snd_card;
};
static struct imx_priv card_priv;

static struct snd_soc_jack imx_hp_jack;
static struct snd_soc_jack_pin imx_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};


static struct snd_soc_jack_gpio imx_hp_jack_gpio= {
		.name = "headphone detect",
		.report = SND_JACK_HEADPHONE,
		.debounce_time = 500,
};

static struct snd_soc_jack imx_mic_jack;
static struct snd_soc_jack_pin imx_mic_jack_pins[] = {
	{
		.pin = "AMIC",
		.mask = SND_JACK_MICROPHONE,
	},
};
static struct snd_soc_jack_gpio imx_mic_jack_gpio = {
	.name = "microphone detect",
	.report = SND_JACK_MICROPHONE,
	.debounce_time = 50,
};

static int hpjack_status_check(void)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	char *envp[3], *buf;
	int hp_status, ret;
	if (!gpio_is_valid(priv->hp_gpio))
		return 0;
	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;

	dev_dbg(&pdev->dev, "%s: hpdet = %d (%d)\n", __func__, hp_status, priv->hp_active_level);
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdev->dev, "%s kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	envp[0] = "NAME=headphone";
	ret = SND_JACK_HEADPHONE;

	if (hp_status == priv->hp_active_level) {
		snprintf(buf, 32, "STATE=%d", 2);
		if (priv->auto_switch_speaker) {
			snd_soc_dapm_enable_pin(&priv->codec->dapm, "Headphone Jack");
			snd_soc_dapm_disable_pin(&priv->codec->dapm, "Ext Spk");
		}

		if (!gpio_is_valid(priv->mic_gpio)) {
			ret = SND_JACK_HEADSET;
			envp[0] = "NAME=headset";

		}

		if (priv->iphone_jack || !gpio_is_valid(priv->mic_gpio)) {
			snd_soc_dapm_force_enable_pin(&priv->codec->dapm, "MICB");
		}

		snd_kctl_jack_report(priv->snd_card, priv->headphone_kctl, 1);
	} else {
		snprintf(buf, 32, "STATE=%d", 0);
		if (priv->auto_switch_speaker) {
			snd_soc_dapm_enable_pin(&priv->codec->dapm, "Ext Spk");
			snd_soc_dapm_disable_pin(&priv->codec->dapm, "Headphone Jack");
		}
		if (priv->iphone_jack || !gpio_is_valid(priv->mic_gpio))
			snd_soc_dapm_disable_pin(&priv->codec->dapm, "MICB");

		ret = 0;
		snd_kctl_jack_report(priv->snd_card, priv->headphone_kctl, 0);
	}

	snd_soc_dapm_sync(&priv->codec->dapm);

	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	dev_dbg(&pdev->dev, "%s: Send event %s %s, status %d\n", __func__, envp[0], envp[1], ret);
	kfree(buf);

	return ret;
}

static int micjack_status_check(void)
{
	struct imx_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	char *envp[3], *buf;
	int mic_status, ret=0, hp_status;
	if (!gpio_is_valid(priv->mic_gpio))
		return 0;

	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;
	mic_status = gpio_get_value(priv->mic_gpio) ? 1 : 0;

	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdev->dev, "%s kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	if (priv->iphone_jack) {
		if (hp_status != priv->hp_active_level) {
			dev_dbg(&pdev->dev, "%s: no iphone jack present\n", __func__);
			snprintf(buf, 32, "STATE=%d", 0);
			ret = 0;
		}
		else {
			if (mic_status == priv->mic_active_level) {
				snprintf(buf, 32, "STATE=%d", 2);
				ret = imx_mic_jack_gpio.report;
				snd_soc_dapm_force_enable_pin(&priv->codec->dapm, "AMIC");
			} else {
				snprintf(buf, 32, "STATE=%d", 0);
				ret = 0;
				snd_soc_dapm_disable_pin(&priv->codec->dapm, "AMIC");
			}
		}
	}
	else {
		if (mic_status == priv->mic_active_level) {
			snprintf(buf, 32, "STATE=%d", 2);
			dev_dbg(&pdev->dev, "%s: Enable AMIC\n", __func__);
			snd_soc_dapm_force_enable_pin(&priv->codec->dapm, "MICB");
			snd_soc_dapm_force_enable_pin(&priv->codec->dapm, "AMIC");
			ret = imx_mic_jack_gpio.report;
		} else {
			snprintf(buf, 32, "STATE=%d", 0);
			dev_dbg(&pdev->dev, "%s: Disable AMIC\n", __func__);
			snd_soc_dapm_disable_pin(&priv->codec->dapm, "AMIC");
			snd_soc_dapm_disable_pin(&priv->codec->dapm, "MICB");
			ret = 0;
		}
	}
	snd_soc_dapm_sync(&priv->codec->dapm);

	envp[0] = "NAME=microphone";
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	dev_dbg(&pdev->dev, "%s: Send event %s %s, status %d\n", __func__, envp[0], envp[1], ret);
	kfree(buf);

	return ret;
}

static int imx_wm8960_spk_amp_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *k, int event)
{
	struct imx_priv *priv = &card_priv;
	if (gpio_is_valid(priv->spk_amp_gpio)) {
		dev_dbg(&priv->pdev->dev, "%s: Spk Amp %d\n", __func__, event);
		if (SND_SOC_DAPM_EVENT_ON(event))
			gpio_set_value(priv->spk_amp_gpio, priv->spk_amp_active_level);
		else
			gpio_set_value(priv->spk_amp_gpio, priv->spk_amp_active_level ? 0 : 1);
	}
	return 0;
}

static const struct snd_soc_dapm_widget imx_wm8960_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Headphone Jack"),
	SND_SOC_DAPM_SPK("Ext Spk", imx_wm8960_spk_amp_event),
	SND_SOC_DAPM_MIC("AMIC", NULL),
};

struct clock_scheme {
	int rate;	/* Sample rate */
	u32 pll_freq;
	int sys_clk_div;
	int dac_div;
	int bclk_div[3];
	int dclk_div;
} ;

static struct clock_scheme wm8960_clocksetup[] = {
	{ 48000, 24576000, 2, 0, {7,4,4}, 7 },
	{ 44100, 22579200, 2, 0, {7,4,4}, 7 },
	{ 32000, 24576000, 2, 1, {6,3,3}, 7 },
	{ 24000, 24576000, 2, 2, {10,7,7}, 7 },
	{ 22050, 22579200, 2, 2, {10,7,7}, 7 },
	{ 16000, 24576000, 2, 3, {12,7,7}, 7 },
	{ 11025, 22579200, 2, 4, {13,10,10}, 7 },
	{ 12000, 24576000, 2, 4, {13,10,10}, 7 },
};

static int imx_hifi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct imx_priv *priv = &card_priv;
	struct device *dev = &priv->pdev->dev;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	unsigned int sample_rate = params_rate(params);
	int width = snd_pcm_format_width(params_format(params));
	int ret = 0;
	int n;
	int sel_bclk = width <= 16 ? 0 : 2;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	data->clk_frequency = clk_get_rate(data->codec_clk);

	data->stream_active[tx] = true;
	dev_dbg(dev, "%s: [%s]\n", __func__, tx ? "tx" : "rx");
	if (data->stream_active[!tx]) {
		dev_dbg(dev, "A stream [%s] is already present, can not configure clocks\n", tx ? "rx" : "tx");
		return 0;
	}

	for (n=0; n < ARRAY_SIZE(wm8960_clocksetup); n++) {
		if (sample_rate == wm8960_clocksetup[n].rate) {
			break;
		}
	}

	if (n == ARRAY_SIZE(wm8960_clocksetup))
	{
		dev_err(dev, "%s: unsupported sample rate %d\n", __func__, sample_rate);
		return -EINVAL;
	}

	dev_dbg(dev, "%s: codec_clk=%d, ch=%d, rate=%d, pll=%d, sys_div=%d, dac_div=%d, bclk_div=%d\n",
			__func__,
			data->clk_frequency,
			params_channels(params),
			wm8960_clocksetup[n].rate,
			wm8960_clocksetup[n].pll_freq,
			wm8960_clocksetup[n].sys_clk_div,
			wm8960_clocksetup[n].dac_div,
			wm8960_clocksetup[n].bclk_div[sel_bclk]);

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);

	snd_soc_dai_set_clkdiv(codec_dai, WM8960_SYSCLKDIV, wm8960_clocksetup[n].sys_clk_div);
	ret = snd_soc_dai_set_pll(codec_dai, WM8960_PLL1, WM8960_PLL1,
			data->clk_frequency, wm8960_clocksetup[n].pll_freq);
	if (ret) {
		dev_err(dev, "failed to set PLL: %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8960_SYSCLK_PLL,
			wm8960_clocksetup[n].pll_freq/wm8960_clocksetup[n].sys_clk_div,
			0);

	return 0;
}

static int imx_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int ret = clk_prepare_enable(data->codec_clk);
	if (ret)
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
	dev_dbg(card->dev, "%s: [%s]\n", __func__, tx ? "tx" : "rx");
	return ret;
}

static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->codec->card;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	dev_dbg(card->dev, "%s: [%s]\n", __func__, tx ? "tx" : "rx");
	data->stream_active[tx] = false;
	clk_disable_unprepare(data->codec_clk);
}

static struct snd_soc_ops imx_hifi_ops = {
	.hw_params = imx_hifi_hw_params,
	.startup = imx_hifi_startup,
	.shutdown = imx_hifi_shutdown,
};

static int imx_wm8960_gpio_init(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct imx_priv *priv = &card_priv;
	int ret;

	priv->codec = codec;

	if (gpio_is_valid(priv->hp_gpio)) {
		imx_hp_jack_gpio.gpio = priv->hp_gpio;
		imx_hp_jack_gpio.jack_status_check = hpjack_status_check;
		imx_hp_jack_gpio.invert = priv->hp_active_level ? 0 : 1;

		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE, &imx_hp_jack);
		snd_soc_jack_add_pins(&imx_hp_jack, ARRAY_SIZE(imx_hp_jack_pins), imx_hp_jack_pins);
		snd_soc_jack_add_gpios(&imx_hp_jack, 1, &imx_hp_jack_gpio);
	}

	if (gpio_is_valid(priv->mic_gpio)) {
		imx_mic_jack_gpio.gpio = priv->mic_gpio;
		imx_mic_jack_gpio.jack_status_check = micjack_status_check;
		imx_mic_jack_gpio.invert = priv->mic_active_level ? 0 : 1;

		snd_soc_jack_new(codec, "AMIC", SND_JACK_MICROPHONE, &imx_mic_jack);
		snd_soc_jack_add_pins(&imx_mic_jack, ARRAY_SIZE(imx_mic_jack_pins), imx_mic_jack_pins);
		snd_soc_jack_add_gpios(&imx_mic_jack, 1, &imx_mic_jack_gpio);
	}
	return 0;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int hp_status;

	if (!gpio_is_valid(priv->hp_gpio)) {
		strcpy(buf, "no detect gpio connected\n");
		return strlen(buf);
	}

	/* Check if headphone is plugged in */
	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;

	if (hp_status == priv->hp_active_level)
		strcpy(buf, "present\n");
	else
		strcpy(buf, "none\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

static ssize_t show_mic(struct device_driver *dev, char *buf)
{
	struct imx_priv *priv = &card_priv;
	int mic_status, hp_status;

	hp_status = gpio_get_value(priv->hp_gpio) ? 1 : 0;
	if (!gpio_is_valid(priv->mic_gpio)) {
		if (hp_status == priv->hp_active_level)
			strcpy(buf, "present\n");
		else
			strcpy(buf, "none\n");
		return strlen(buf);
	}

	/* Check if analog microphone is plugged in */
	mic_status = gpio_get_value(priv->mic_gpio) ? 1 : 0;
	strcpy(buf, "none\n");
	if (priv->iphone_jack) {
		if (mic_status == priv->mic_active_level && hp_status == priv->hp_active_level)
			strcpy(buf, "present\n");
	}
	else {
		if (mic_status == priv->mic_active_level)
			strcpy(buf, "present\n");
	}
	return strlen(buf);
}

static DRIVER_ATTR(microphone, S_IRUGO | S_IWUSR, show_mic, NULL);

static int imx_wm8960_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_dai *codec_dai = card->rtd[0].codec_dai;
	struct imx_wm8960_data *data = snd_soc_card_get_drvdata(card);
	int ret = clk_prepare_enable(data->codec_clk);
	if (ret) {
		dev_err(card->dev, "Failed to enable MCLK: %d\n", ret);
		return ret;
	}

	/* Use 0.9 factor on MIC BIAS voltage */
	snd_soc_update_bits(codec_dai->codec, WM8960_ADDCTL4, 0x1, 0x0);
	snd_soc_dai_set_clkdiv(codec_dai, WM8960_DCLKDIV, 7);
	/* Use MONO mixer */
	snd_soc_update_bits(codec_dai->codec, WM8960_ADDCTL1, 0x10, 0x10);
	clk_disable_unprepare(data->codec_clk);
	return 0;
}

static int imx_wm8960_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_np, *codec_np=0;
	struct platform_device *cpu_pdev;
	struct imx_priv *priv = &card_priv;
	struct i2c_client *codec_dev;
	struct imx_wm8960_data *data;
	int int_port, ext_port;
	int ret;
	int nc_pins = 0;
	char nc_name[80];
	const char *nc_name_p = nc_name;
	int n;
	enum of_gpio_flags gpio_flags;

	priv->pdev = pdev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (!strstr(cpu_np->name, "ssi"))
		goto audmux_bypass;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

audmux_bypass:
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev || !codec_dev->dev.driver) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = devm_clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		dev_err(&codec_dev->dev, "failed to get codec clk: %d\n", ret);
		goto fail;
	}

	data->clk_frequency = clk_get_rate(data->codec_clk);
	dev_dbg(&pdev->dev, "%s: codec clock is %d\n", __func__, data->clk_frequency);

	priv->hp_gpio = of_get_named_gpio_flags(np, "hp-det-gpios", 0, &gpio_flags);
	priv->hp_active_level = (gpio_flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

	priv->iphone_jack = of_property_read_bool(np, "iphone-jack");

	priv->mic_gpio = of_get_named_gpio_flags(np, "mic-det-gpios", 0, &gpio_flags);
	priv->mic_active_level = gpio_flags & OF_GPIO_ACTIVE_LOW ? 0 : 1;

	priv->spk_amp_gpio = of_get_named_gpio_flags(np, "speaker-amp-gpios", 0, &gpio_flags);
	if (gpio_is_valid(priv->spk_amp_gpio)) {
		priv->spk_amp_active_level = gpio_flags & OF_GPIO_ACTIVE_LOW ? 0 : 1;
		if (gpio_request_one(priv->spk_amp_gpio, 0, "wm8960-speaker-amp")) {
			dev_err(&pdev->dev, "%s: Unable to request gpio %d for amp\n", __func__, priv->spk_amp_gpio);
			priv->spk_amp_gpio = -1;
		}

	}

	priv->auto_switch_speaker = of_property_read_bool(np, "speaker-auto-switch");

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "wm8960-hifi";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
	data->dai.ops = &imx_hifi_ops;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;
	data->dai.symmetric_channels = 1;
	data->dai.symmetric_rates = 1;
	data->dai.symmetric_samplebits = 1;

	data->card.dev = &pdev->dev;
	data->card.late_probe = imx_wm8960_late_probe;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_wm8960_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8960_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	priv->snd_card = data->card.snd_card;
	priv->headphone_kctl = snd_kctl_jack_new("Headphone", 0, NULL);
	ret = snd_ctl_add(data->card.snd_card, priv->headphone_kctl);
	if (ret)
		goto fail;

	imx_wm8960_gpio_init(&data->card);

	if (gpio_is_valid(priv->hp_gpio)) {
		ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
		if (ret) {
			dev_err(&pdev->dev, "create hp attr failed (%d)\n", ret);
			goto fail_hp;
		}
	}

	if (gpio_is_valid(priv->mic_gpio) || (priv->iphone_jack && gpio_is_valid(priv->hp_gpio))) {
		ret = driver_create_file(pdev->dev.driver, &driver_attr_microphone);
		if (ret) {
			dev_err(&pdev->dev, "create mic attr failed (%d)\n", ret);
			goto fail_mic;
		}
	}

	/* Check for pins to NC */
	nc_pins = of_property_count_strings(np, "ncpins");
	dev_dbg(&pdev->dev, "Found %d NC pins\n", nc_pins);
	if (nc_pins > 0) {
		for (n=0; n < nc_pins; n++) {
			ret = of_property_read_string_index(np, "ncpins", n, &nc_name_p);
			if (ret) {
				dev_err(&pdev->dev, "Unable to read NC pin\n");
				continue;
			}
			snd_soc_dapm_nc_pin(&data->card.dapm, nc_name_p);
			dev_dbg(&pdev->dev, "NC on pin %s\n", nc_name_p);
		}
	}
	goto fail;

fail_mic:
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
fail_hp:
	snd_soc_unregister_card(&data->card);
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_wm8960_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	driver_remove_file(pdev->dev.driver, &driver_attr_microphone);
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_wm8960_dt_ids[] = {
	{ .compatible = "fsl,lm-imx-audio-wm8960", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8960_dt_ids);

static struct platform_driver lm_imx_wm8960_driver = {
	.driver = {
		.name = "lm-imx-wm8960",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8960_dt_ids,
	},
	.probe = imx_wm8960_probe,
	.remove = imx_wm8960_remove,
};
module_platform_driver(lm_imx_wm8960_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_AUTHOR("DATA RESPONS AS");
MODULE_DESCRIPTION("Laerdal i.MX WM8960 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:lm-imx-wm8960");
