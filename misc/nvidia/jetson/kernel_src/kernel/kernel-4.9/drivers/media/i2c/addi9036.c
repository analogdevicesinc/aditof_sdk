// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the ADDI9036 ToF camera sensor.
 *
 * Copyright (C) 2020 Analog Devices Inc.
 *
 *
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include "addi9036_mode_tbls.h"

#define ADDI9036_DEFAULT_MODE		ADDI9036_MODE_640X480_CROP_30FPS
#define ADDI9036_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12

#define ADDI9036_DEFAULT_WIDTH	640
#define ADDI9036_DEFAULT_HEIGHT	480
#define ADDI9036_DEFAULT_CLK_FREQ	37125000

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY  (V4L2_CID_USER_ADDI9036_BASE + 0)
#define V4L2_CID_AD_DEV_REG_READ_QUERY  (V4L2_CID_USER_ADDI9036_BASE + 1)

struct addi9036 {
	struct camera_common_power_rail	power;
	int				numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;

	struct regmap			*regmap;
	/* CTRLS */
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static bool addi9306_readable_register(struct device *dev, unsigned int reg)
{
	if (((reg >= 0x4000) && (reg <= 0x6FFF)) ||
	    ((reg >= 0x7C00) && (reg <= 0x7C9F)) ||
	    ((reg >= 0x7CE0) && (reg <= 0x7FFF)) ||
	    ((reg >= 0xC000) && (reg <= 0xC0FF)) ||
	    ((reg >= 0xC110) && (reg <= 0xC200)) ||
	    ((reg >= 0xC300) && (reg <= 0xC6BF)))
		return true;
	else
		return false;
}

static bool addi9306_writeable_register(struct device *dev, unsigned int reg)
{
	if (((reg >= 0x4000) && (reg <= 0x6FFF)) ||
	    ((reg >= 0x7C00) && (reg <= 0x7FFF)) ||
	    ((reg >= 0xC000) && (reg <= 0xC200)) ||
	    ((reg >= 0xC300) && (reg <= 0xC7FF)))
		return true;
	else
		return false;
}

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	
	.max_register = 0xC7FF,
	.writeable_reg = addi9306_writeable_register,
	.readable_reg = addi9306_readable_register,
	.cache_type = REGCACHE_NONE,
};

static int addi9036_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct addi9036 *priv =
		container_of(ctrl->handler, struct addi9036, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {

	default:
			dev_err(dev, "%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int addi9036_set_chip_config(struct v4l2_ctrl *ctrl)
{
	struct addi9036 *priv =
		container_of(ctrl->handler, struct addi9036, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	unsigned short *reg, *val;
	int ret, index;

	reg = (unsigned short *)(ctrl->p_new.p_u16);
	val = (unsigned short *)(ctrl->p_new.p_u16 + 1);

	for (index = 0; index < ctrl->elems; index += 2) {
		ret = regmap_write(priv->regmap, *reg, *val);
		if (ret)
			dev_dbg(dev,
				 "could not write to register %x\n", *reg);

		reg += 2;
		val += 2;
	}

	return 0;
}

static int addi9036_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct addi9036 *priv =
		container_of(ctrl->handler, struct addi9036, ctrl_handler);
	struct camera_common_data	*s_data = priv->s_data;
	struct device *dev = &priv->i2c_client->dev;
	unsigned int val;
	int ret = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY:
		ret = addi9036_set_chip_config(ctrl);
		break;
	case V4L2_CID_AD_DEV_REG_READ_QUERY:
		ret = regmap_read(priv->regmap, *(u16 *)(ctrl->p_new.p_u16),
				  &val);
		if (ret)
			dev_warn(dev, "could not read from register\n");
		else
			*(u16 *)(ctrl->p_new.p_u16) = val;
		break;
	case V4L2_CID_PIXEL_RATE:
		break;
	case V4L2_CID_LINK_FREQ:
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		s_data->sensor_mode_id = (int) (*ctrl->p_new.p_s64);
		break;
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops addi9036_ctrl_ops = {
	.g_volatile_ctrl = addi9036_g_volatile_ctrl,
	.s_ctrl = addi9036_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops 		= &addi9036_ctrl_ops,
		.id 		= TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name 		= "Sensor Mode",
		.type 		= V4L2_CTRL_TYPE_INTEGER64,
		.flags 		= V4L2_CTRL_FLAG_SLIDER,
		.min 		= 0,
		.max 		= 0xFF,
		.def 		= 0xFE,
		.step 		= 1,
	},
	{
		.ops		= &addi9036_ctrl_ops,
		.id		= V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY,
		.name		= "chip_config",
		.type		= V4L2_CTRL_TYPE_U16,
		.def		= 0xFF,
		.min		= 0x00,
		.max		= 0xFFFF,
		.step		= 1,
		.dims		= { 2048 },
		.elem_size	= 2
	},
	{
		.ops		= &addi9036_ctrl_ops,
		.id		= V4L2_CID_AD_DEV_REG_READ_QUERY,
		.name		= "reg_read",
		.type		= V4L2_CTRL_TYPE_U16,
		.def		= 0,
		.min		= 0x00,
		.max		= 0xFFFF,
		.step		= 1
	}
};

static inline int addi9036_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;
	u32 reg_val = 0;
	int ret = 0;

	ret = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return ret;
}

static int addi9036_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;
	struct device *dev = &priv->i2c_client->dev;
	int ret;

	ret = regmap_write(priv->regmap, addr, val);
	if (ret)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return ret;
}

static int addi9036_power_on(struct camera_common_data *s_data)
{
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;
	int ret;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (priv->pdata && priv->pdata->power_on) {
		ret = priv->pdata->power_on(pw);
		if (ret)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return ret;
	}

	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
	}

	regmap_register_patch(priv->regmap, addi9036_power_up, 
			      ARRAY_SIZE(addi9036_power_up));

	pw->state = SWITCH_ON;
	return 0;

}

static int addi9036_power_off(struct camera_common_data *s_data)
{
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;
	int ret;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_off) {
		ret = priv->pdata->power_off(pw);
		if (!ret)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return ret;
	}
	
	regmap_register_patch(priv->regmap, addi9036_power_down, 
			      ARRAY_SIZE(addi9036_power_down));

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int addi9036_power_get(struct addi9036 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;

	pw->reset_gpio = pdata->reset_gpio;

	pw->state = SWITCH_OFF;
	return 0;
}

static int addi9036_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;

	dev_dbg(dev, "stream %d\n", enable);

	return 0;
}

static int addi9036_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops addi9036_subdev_video_ops = {
	.s_stream = addi9036_s_stream,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	.g_mbus_config	= camera_common_g_mbus_config,
#endif
	.g_input_status = addi9036_g_input_status,
};

static struct v4l2_subdev_core_ops addi9036_subdev_core_ops = {
	.s_power = camera_common_s_power,
};

static int addi9036_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int addi9036_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops addi9036_subdev_pad_ops = {
	.set_fmt = addi9036_set_fmt,
	.get_fmt = addi9036_get_fmt,
	.enum_mbus_code	= camera_common_enum_mbus_code,
	.enum_frame_size = camera_common_enum_framesizes,
};

static struct v4l2_subdev_ops addi9036_subdev_ops = {
	.core = &addi9036_subdev_core_ops,
	.video = &addi9036_subdev_video_ops,
	.pad = &addi9036_subdev_pad_ops,
};

static struct camera_common_sensor_ops addi9036_common_ops = {
	.power_on = addi9036_power_on,
	.power_off = addi9036_power_off,
	.write_reg = addi9036_write_reg,
	.read_reg = addi9036_read_reg,
};

static int addi9036_ctrls_init(struct addi9036 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static struct camera_common_pdata *addi9036_parse_dt(struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	int gpio;

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "reset-gpios not found %d\n", gpio);
		gpio = 0;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	return board_priv_pdata;
}

static int addi9036_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops addi9036_subdev_internal_ops = {
	.open = addi9036_open,
};

static const struct media_entity_operations addi9036_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int addi9036_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct addi9036 *priv;
	int err;

	dev_info(&client->dev, "probing addi9036 v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct addi9036) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	if (client->dev.of_node)
		priv->pdata = addi9036_parse_dt(client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &addi9036_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &addi9036_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  ADDI9036_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(addi9036_frmfmt);
	common_data->def_mode = ADDI9036_DEFAULT_MODE;
	common_data->def_width = ADDI9036_DEFAULT_WIDTH;
	common_data->def_height = ADDI9036_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = ADDI9036_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;

	err = addi9036_power_get(priv);
	if (err)
		return err;

	err = camera_common_initialize(common_data, "addi9036");
	if (err) {
		dev_err(&client->dev, "Failed to initialize addi9036.\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &addi9036_subdev_ops);

	err = addi9036_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &addi9036_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &addi9036_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1,
				&priv->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(&client->dev, "Detected ADDI9036 sensor\n");

	return 0;
}

static int addi9036_remove(struct i2c_client *client) {
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct addi9036 *priv = (struct addi9036 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_cleanup(s_data);
	return 0;
}

static const struct i2c_device_id addi9036_id[] = {
	{ "addi9036", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, addi9036_id);

const static struct of_device_id addi9036_of_match[] = {
	{ .compatible = "adi,addi9036",},
	{ },
};
MODULE_DEVICE_TABLE(of, addi9036_of_match);

static struct i2c_driver addi9036_i2c_driver = {
	.driver = {
		.name = "addi9036",
		.owner = THIS_MODULE,
		.of_match_table = addi9036_of_match,
	},
	.probe = addi9036_probe,
	.remove = addi9036_remove,
	.id_table = addi9036_id,
};

module_i2c_driver(addi9036_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADDI9036 Camera Driver");
MODULE_LICENSE("GPL v2");
