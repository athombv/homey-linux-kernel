/*
 * Machine driver for EVAL-ADAU1x61 on IMX6 boards.
 *
 * Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <linux/platform_data/adau17x1.h>

#include "imx-audmux.h"

#include "../codecs/adau17x1.h"

#define SND_JACK_HEADPHONE    0x0001

struct imx_adau1x61_data {
	struct clk *codec_clk;
	unsigned int clk_frequency;
};


static int imx_adau1x61_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct imx_adau1x61_data *data = snd_soc_card_get_drvdata(rtd->card);
	u32 dai_format;
	int pll_rate;
	int ret;

	switch (params_rate(params)) {
	case 48000:
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 96000:
		pll_rate = 48000 * 1024;
		break;
	case 44100:
	case 7350:
	case 11025:
	case 14700:
	case 22050:
	case 29400:
	case 88200:
		pll_rate = 44100 * 1024;
		break;
	default:
		return -EINVAL;
	}

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		         SND_SOC_DAIFMT_CBM_CFM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret) return ret;

	ret = snd_soc_dai_set_pll(codec_dai, ADAU17X1_PLL, ADAU17X1_PLL_SRC_MCLK, data->clk_frequency, pll_rate);
	if (ret) return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, ADAU17X1_CLK_SRC_PLL, pll_rate, SND_SOC_CLOCK_IN);

	return ret;
}

static struct snd_soc_jack hp_jack;

static struct snd_soc_jack_gpio hp_jack_gpio = {
    .gpio                   = -1,
    .name                   = "hp-gpio",
    .report                 = SND_JACK_HEADPHONE,
    .invert                 = false,
    .debounce_time          = 200,
};

static int imx_adau1x61_jackdetect_gpio_init(struct device *dev, struct device_node *np, struct snd_soc_card *card) {
    int ret = 0;
    enum of_gpio_flags of_flags;

    hp_jack_gpio.gpio = of_get_named_gpio_flags(np, "jack-detect-gpio", 0, &of_flags);

    if (gpio_is_valid(hp_jack_gpio.gpio)) {
        if(of_flags & OF_GPIO_ACTIVE_LOW) {
            hp_jack_gpio.invert = true;
        }
        snd_soc_card_jack_new(card, "Jack Detect", SND_JACK_HEADPHONE,
                &hp_jack, NULL, 0);

        snd_soc_jack_add_gpios(&hp_jack, 1,
                &hp_jack_gpio);

    }

	return ret;
}

static int imx_adau1x61_audmux_init(struct device *dev, struct device_node *np)
{
	int int_port, ext_port;
	int ret;

    ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(dev, "mux-ext-port missing or invalid\n");
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
		dev_err(dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int imx_adau1x61_runtime_init(struct snd_soc_pcm_runtime *runtime) {
    int ret;
    struct device *dev = runtime->card->dev;

    ret = imx_adau1x61_jackdetect_gpio_init(dev, dev->of_node, runtime->card);
	if (ret) {
		dev_err(dev, "failed to init jack detect GPIO\n");
		return ret;
	}
    return ret;
}

static const struct snd_soc_ops imx_adau1x61_ops = {
	.hw_params = imx_adau1x61_hw_params,
};

static struct snd_soc_dai_link imx_adau1x61_dai = {
	.name = "adau1x61",
	.stream_name = "adau1761-hifi",
	.codec_dai_name = "adau-hifi",
	.cpu_dai_name = "2028000.ssi",
	.ops = &imx_adau1x61_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM,
    .init = &imx_adau1x61_runtime_init,
};

static struct snd_soc_card imx_adau1x61 = {
	.driver_name = "imx-adau1x61",
	.dai_link = &imx_adau1x61_dai,
	.num_links = 1,

	.fully_routed = true,
};

static struct adau1761_platform_data adau1761_data = {
    .input_differential = true,
	.lineout_mode = ADAU1761_OUTPUT_MODE_LINE,
	.headphone_mode = ADAU1761_OUTPUT_MODE_LINE,

	.digmic_jackdetect_pin_mode = ADAU1761_DIGMIC_JACKDET_PIN_MODE_JACKDETECT,

	.jackdetect_debounce_time = ADAU1761_JACKDETECT_DEBOUNCE_40MS,
	.jackdetect_active_low = true,

	.micbias_voltage = ADAU17X1_MICBIAS_0_90_AVDD,
};



static int imx_adau1x61_probe(struct platform_device *pdev)
{
    int ret;
    struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct i2c_client *codec_dev;
	struct imx_adau1x61_data *data = NULL;


    ssi_np = of_parse_phandle(pdev->dev.of_node, "ssi-controller", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev->dev.platform_data = &adau1761_data;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		goto fail;
	}

    clk_prepare_enable(data->codec_clk);

	data->clk_frequency = clk_get_rate(data->codec_clk);
	dev_err(&pdev->dev, "read clk freq: %d\n", data->clk_frequency);

	imx_adau1x61.dev = &pdev->dev;

    ret = snd_soc_of_parse_audio_routing(&imx_adau1x61, "audio-routing");
	if (ret) {
    	dev_err(&pdev->dev, "snd_soc_of_parse_audio_routing failed (%d)\n", ret);
		goto fail;
    }

    ret = snd_soc_of_parse_audio_simple_widgets(&imx_adau1x61, "audio-widgets");
	if (ret) {
    	dev_err(&pdev->dev, "snd_soc_of_parse_audio_simple_widgets failed (%d)\n", ret);
		goto fail;
    }

    ret = snd_soc_of_parse_card_name(&imx_adau1x61, "model");
    if (ret) {
    	dev_err(&pdev->dev, "snd_soc_of_parse_card_name failed (%d)\n", ret);
		goto fail;
    }

    imx_adau1x61_dai.codec_of_node = codec_np;
	imx_adau1x61_dai.cpu_of_node = ssi_np;
	imx_adau1x61_dai.platform_of_node = ssi_np;

    ret = imx_adau1x61_audmux_init(&pdev->dev, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "failed to init audmux\n");
		goto fail;
	}

	platform_set_drvdata(pdev, &imx_adau1x61);
	snd_soc_card_set_drvdata(&imx_adau1x61, data);

    ret = devm_snd_soc_register_card(&pdev->dev, &imx_adau1x61);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	}

    goto done;

fail:
	if (data && !IS_ERR(data->codec_clk))
		clk_put(data->codec_clk);
done:
	of_node_put(ssi_np);
	of_node_put(codec_np);

	return ret;
}

static int imx_adau1x61_remove(struct platform_device *pdev)
{
    struct snd_soc_card *card = platform_get_drvdata(pdev);
    struct imx_adau1x61_data *data = snd_soc_card_get_drvdata(card);

    if(data && !IS_ERR(data->codec_clk)) {
        clk_disable_unprepare(data->codec_clk);
	    clk_put(data->codec_clk);
    }

    snd_soc_jack_free_gpios(&hp_jack, 1,
            &hp_jack_gpio);

	return 0;
}

static const struct of_device_id imx_adau1x61_dt_ids[] = {
    { .compatible = "fsl,imx-audio-adau1761", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_adau1x61_dt_ids);

static struct platform_driver imx_adau1x61_driver = {
	.driver = {
		.name = "imx-adau1x61",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_adau1x61_dt_ids,
	},
	.probe = imx_adau1x61_probe,
	.remove = imx_adau1x61_remove,
};
module_platform_driver(imx_adau1x61_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_AUTHOR("Jeroen Vollenbrock <jeroen@athom.nl>");
MODULE_DESCRIPTION("ALSA SoC Freescale i.MX ADAU1x61 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-adau1x61");

