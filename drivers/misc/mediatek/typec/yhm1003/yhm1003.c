// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include "../../../../drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"


#define YHM1003_I2C_NAME	"yhm1003-driver"

#define YHM1003_DEVICE_REG_VALUE 0x88

#define YHM1003_DEVICE_ID     									(0x00)
#define YHM1003_OVP_INTERRUPT_MASK        			(0x01)
#define YHM1003_OVP_INTERRUPT_FLAG        			(0x02)
#define YHM1003_OVP_STATUS        							(0x03)
#define YHM1003_SWITCH_USB_AUDIO  							(0x04)
#define YHM1003_SWITCH_SBU   										(0x05)
#define YHM1003_SWITCH_STATUS0           				(0x06)
#define YHM1003_SWITCH_STATUS1           				(0x07)
#define YHM1003_AUDIO_L_SLOW_ON_CONTROL         (0x08)
#define YHM1003_AUDIO_R_SLOW_ON_CONTROL         (0x09)
#define YHM1003_MIC_SLOW_ON_CONTROL  						(0x0A)
#define YHM1003_GSNS_SLOW_ON_CONTROL       			(0x0B)
#define YHM1003_AUDIO_GROUND_SLOW_ON_CONTROL		(0x0C)
#define YHM1003_TIMING_DELAY_BETWEEN_R_AND_L		(0x0D)
#define YHM1003_TIMING_DELAY_BETWEEN_MIC_AND_L 	(0x0E)
#define YHM1003_TIMING_DELAY_BETWEEN_GSNS_AND_L (0x0F)

#define YHM1003_TIMING_DELAY_BETWEEN_AGND_AND_L (0x10)
#define YHM1003_AUDIO_ACCESSERY_STATUS          (0x11)
#define YHM1003_FUNCTION_ENABLE                 (0x12)
#define YHM1003_MOISTURE_PIN_SETTING            (0x13)
#define YHM1003_MOISTURE_VALUE                  (0x14)
#define YHM1003_MOISTURE_INTERRUPT_THRESHOLD    (0x15)
#define YHM1003_MOISTURE_DETECTION_INTERVAL     (0x16)
#define YHM1003_AUDIO_JACK_STATUS               (0x17)
#define YHM1003_DETCTION_INTERRUPT              (0x18)
#define YHM1003_DETCTION_INTERRUPT_MASK         (0x19)
#define YHM1003_AUDIO_DETECTION_REG1            (0x1A)
#define YHM1003_AUDIO_DETECTION_REG2            (0x1B)
#define YHM1003_MIC_THRESHOLD_DATA0             (0x1C)
#define YHM1003_MIC_THRESHOLD_DATA1             (0x1D)
#define YHM1003_EOB_I2C_RESET                   (0x1E)
#define YHM1003_CURRENT_SOURCE_SETTING          (0x1F)

#define YHM1003_OVP_THRESHOLD                   (0x20)
#define YHM1003_OVP_HYSTERESIS                  (0x21)
#define YHM1003_WATCH_DOG_TIMER                 (0x22)
#define YHM1003_WATCH_DOG_CONTROL               (0x23)
#undef dev_dbg
#define dev_dbg dev_info
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver begin
bool yhm1003_load = false;
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver end
enum switch_vendor {
    YHM1003 = 0,
};

struct yhm1003_priv {
	struct regmap *regmap;
	struct device *dev;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head yhm1003_notifier;
	struct mutex notification_lock;
	unsigned int hs_det_pin;
	enum switch_vendor vendor;
	bool plug_state;
};

struct yhm1003_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config yhm1003_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = YHM1003_CURRENT_SOURCE_SETTING,
};
/*
static const struct yhm1003_reg_val fsa_reg_i2c_defaults[] = {
	{YHM1003_SLOW_L, 0x00},
	{YHM1003_SLOW_R, 0x00},
	{YHM1003_SLOW_MIC, 0x00},
	{YHM1003_SLOW_SENSE, 0x00},
	{YHM1003_SLOW_GND, 0x00},
	{YHM1003_DELAY_L_R, 0x00},
	{YHM1003_DELAY_L_MIC, 0x00},
	{YHM1003_DELAY_L_SENSE, 0x00},
	{YHM1003_DELAY_L_AGND, 0x09},
	{YHM1003_SWITCH_SETTINGS, 0x98},
};*/


static int yhm1003_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct yhm1003_priv *fsa_priv =
			container_of(nb, struct yhm1003_priv, pd_nb);
	struct device *dev;
	struct tcp_notify *noti = ptr;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (fsa_priv->vendor == YHM1003) {
		dev_err(dev, "%s: switch chip is YHM1003\n", __func__);
	}

	dev_info(dev, "%s: typeC event: %d\n", __func__, evt);

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:
		dev_info(dev, "%s: old_state: %d, new_state: %d\n",
			__func__, noti->typec_state.old_state, noti->typec_state.new_state);
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* AUDIO plug in */
			dev_err(dev, "%s: audio plug in\n", __func__);
			fsa_priv->plug_state = true;
			pm_stay_awake(fsa_priv->dev);
			schedule_work(&fsa_priv->usbc_analog_work);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* AUDIO plug out */
			dev_err(dev, "%s: audio plug out\n", __func__);
			fsa_priv->plug_state = false;
			pm_stay_awake(fsa_priv->dev);
			schedule_work(&fsa_priv->usbc_analog_work);
		}
		break;
//	case TCP_NOTIFY_PLUG_OUT:
//		dev_err(dev, "%s: typec plug out\n", __func__);
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

static int yhm1003_usbc_analog_setup_switches(struct yhm1003_priv *fsa_priv)
{
	int rc = 0;
	struct device *dev;
	unsigned int switch_status = 0;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);

	dev_info(dev, "%s: plug_state %d\n", __func__, fsa_priv->plug_state);
	if (fsa_priv->plug_state) {
    	rc = regmap_write(fsa_priv->regmap, YHM1003_SWITCH_USB_AUDIO, 0x7f);        
        rc = regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x20); 
//Antai <AI_BSP_Audio> <hehl> <2022-08-01> #43753 begin 		
        rc = regmap_write(fsa_priv->regmap, YHM1003_MIC_THRESHOLD_DATA0, 0x0A);        
        rc = regmap_write(fsa_priv->regmap, YHM1003_OVP_THRESHOLD, 0xFA); 
//Antai <AI_BSP_Audio> <hehl> <2022-08-01> #43753 end         
            
        regmap_read(fsa_priv->regmap, YHM1003_CURRENT_SOURCE_SETTING, &switch_status);
        dev_info(dev, "%s: hhl SOURCE_SETTING=0x%x\n", __func__,switch_status);
//Antai <AI_BSP_Audio> <hehl> <2022-07-28> #43700 begin         
        switch_status = (switch_status & 0xF0) +0x02;
//Antai <AI_BSP_Audio> <hehl> <2022-07-28> #43700 end         
        dev_info(dev, "%s: hhl SOURCE_SETTING=0x%x\n", __func__,switch_status);
        regmap_write(fsa_priv->regmap, YHM1003_CURRENT_SOURCE_SETTING, switch_status); 
        
         msleep(30);
        
        regmap_read(fsa_priv->regmap, YHM1003_FUNCTION_ENABLE, &switch_status);
        dev_info(dev, "%s: hhl SOURCE_SETTING=0x%x\n", __func__,switch_status);
        switch_status = (switch_status & 0xFE) +0x01;
        dev_info(dev, "%s: hhl SOURCE_SETTING=0x%x\n", __func__,switch_status);
        regmap_write(fsa_priv->regmap, YHM1003_FUNCTION_ENABLE, switch_status); 
         
//Antai <AI_BSP_Audio> <hehl> <2022-07-28> #43706 begin        
        msleep(150);   
//Antai <AI_BSP_Audio> <hehl> <2022-07-28> #43706 end         
      	/*do{  
        	regmap_read(fsa_priv->regmap, YHM1003_DETCTION_INTERRUPT, &switch_status);
        	dev_info(dev, "%s: reg[0x%x]=0x%x.wait for jack detect finish\n", __func__, YHM1003_DETCTION_INTERRUPT, switch_status);
        }while((switch_status&0x04)!=0x04);     //wait for jack detect finish*/
		
       
      	regmap_read(fsa_priv->regmap, YHM1003_AUDIO_JACK_STATUS, &switch_status);
        dev_info(dev, "%s:  reg[0x%x]=0x%x.jack_status\n", __func__, YHM1003_AUDIO_JACK_STATUS, switch_status);												                      
      	if((switch_status &0x01)==0x1)  //No Audio Device
      	{
        	dev_info(dev, "%s: No Audio jack insert\n", __func__);
      	}
      	else
      	{
      	if((switch_status &0x0e)==0x08)  //SBU1 to AGND, SBU2 to MIC
      	{ 	
			regmap_write(fsa_priv->regmap, YHM1003_SWITCH_USB_AUDIO, 0x4f); 
			regmap_write(fsa_priv->regmap, YHM1003_CURRENT_SOURCE_SETTING, 0x00); 
			regmap_write(fsa_priv->regmap, YHM1003_FUNCTION_ENABLE, 0x8); 
        	regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x94); 
        	dev_info(dev, "%s: SBU1 to AGND, SBU2 to MIC\n", __func__);
		}
      	if((switch_status &0x0e)==0x04)  //SBU1 to MIC, SBU2 to AGND
      	{ 
			regmap_write(fsa_priv->regmap, YHM1003_SWITCH_USB_AUDIO, 0x4f); 
			regmap_write(fsa_priv->regmap, YHM1003_CURRENT_SOURCE_SETTING, 0x00); 
			regmap_write(fsa_priv->regmap, YHM1003_FUNCTION_ENABLE, 0x8); 
        	regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x61);    
        	dev_info(dev, "%s: SBU1 to MIC, SBU2 to AGND\n", __func__);
      	}
      	if((switch_status &0x0e)==0x02)//SBU1 to AGND, GSBU1 to GSNS
      	{
      		// rc = yhm1003_write_reg(data, YHM1003_SWITCH_SBU, 0x84); 
                regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x94); //0x84
        	dev_info(dev, "%s: SBU1 to AGND, GSBU1 to GSNS\n", __func__);
      	}
      		          	      		          	      		          			
    }
	/*
	regmap_write(fsa_priv->regmap, YHM1003_SWITCH_USB_AUDIO, 0x4F);
	regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x94); 	
	*/

	} else {
		/* deactivate switches */
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> usb is no detected problem begin		
		//yhm1003_usbc_update_settings(fsa_priv, 0x18, 0x98);
        regmap_write(fsa_priv->regmap, YHM1003_SWITCH_USB_AUDIO, 0x45);
        regmap_write(fsa_priv->regmap, YHM1003_SWITCH_SBU, 0x00);
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> usb is no detected problem end		
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}


static int yhm1003_parse_dt(struct yhm1003_priv *fsa_priv,
	struct device *dev)
{
    struct device_node *dNode = dev->of_node;
    int ret = 0;

    struct pinctrl_state *hs_det_gpio;
    struct pinctrl *pinctrl1;

    if (dNode == NULL)
    {
        return -ENODEV;
    }
    
	if (fsa_priv == NULL) {
		pr_err("%s: fsa_priv is NULL\n", __func__);
		return -ENOMEM;
	}

//    if ((20630 == get_project()) || (20631 == get_project()) || (20632 == get_project())) {
        pinctrl1 = devm_pinctrl_get(dev);
        if (IS_ERR(pinctrl1)) {
            ret = PTR_ERR(pinctrl1);
            pr_err("can not find yhm1003 pinctrl 0\n");
            return ret;
        }
        pr_info("do set gpio high 1\n");
        hs_det_gpio = pinctrl_lookup_state(pinctrl1, "hs_det_gpio_high");
        if (IS_ERR(hs_det_gpio)) {
            ret = PTR_ERR(hs_det_gpio);
            pr_err("error to set gpio state 1\n");
            return ret;
        } else {
            ret = pinctrl_select_state(pinctrl1, hs_det_gpio);
            if (ret < 0) {
                pr_err("error to set gpio state 2\n");
                return ret;
            }
        }
//    }

	fsa_priv->hs_det_pin = of_get_named_gpio(dNode,
	        "yhm1003,hs-det-gpio", 0);
	if (!gpio_is_valid(fsa_priv->hs_det_pin)) {
	    pr_warning("%s: hs-det-gpio in dt node is missing\n", __func__);
	    return -ENODEV;
	}
	ret = gpio_request(fsa_priv->hs_det_pin, "yhm1003_hs_det");
	if (ret) {
		pr_warning("%s: hs-det-gpio request fail\n", __func__);
		return ret;
	}

	gpio_direction_output(fsa_priv->hs_det_pin, 1);

	return ret;
}

static void yhm1003_usbc_analog_work_fn(struct work_struct *work)
{
	struct yhm1003_priv *fsa_priv =
		container_of(work, struct yhm1003_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	yhm1003_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}
/*
static void yhm1003_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}*/

//Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 begin 
static struct yhm1003_priv *fsa_priv_global;
void yhm1003_reset(void)
{
    regmap_write(fsa_priv_global->regmap, YHM1003_EOB_I2C_RESET, 0x01);
    printk("%s\n", __func__);
}
//Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 end

static int yhm1003_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct yhm1003_priv *fsa_priv;
	int rc = 0;
	unsigned int reg_value = 0;

  //  return 0;
    
	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> add for ASW2200 YHM1003 driver begin     
    i2c->addr = 0x42;
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> add for ASW2200 YHM1003 driver end     
//Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 begin 
    fsa_priv_global = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv_global),
				GFP_KERNEL);
	if (!fsa_priv_global)
		return -ENOMEM;
 //Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 end    
	fsa_priv->dev = &i2c->dev;

	yhm1003_parse_dt(fsa_priv, &i2c->dev);

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &yhm1003_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	//yhm1003_update_reg_defaults(fsa_priv->regmap);

	regmap_read(fsa_priv->regmap, YHM1003_DEVICE_ID, &reg_value);
	dev_err(fsa_priv->dev, "%s: device id reg value: 0x%x\n", __func__, reg_value);
	if (YHM1003_DEVICE_REG_VALUE == reg_value) {
		dev_err(fsa_priv->dev, "%s: switch chip is YHM1003\n", __func__);
		fsa_priv->vendor = YHM1003;
	} else {
		dev_err(fsa_priv->dev, "%s: read yhm1003 chip id fail\n", __func__);
        goto err_data;
	}

	fsa_priv->plug_state = false;
	fsa_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!fsa_priv->tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		goto err_data;
	}

	fsa_priv->pd_nb.notifier_call = yhm1003_usbc_event_changed;
	fsa_priv->pd_nb.priority = 0;
	rc = register_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (rc < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		goto err_data;
	}

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  yhm1003_usbc_analog_work_fn);

	fsa_priv->yhm1003_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((fsa_priv->yhm1003_notifier).rwsem);
	fsa_priv->yhm1003_notifier.head = NULL;
//Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 begin 
	fsa_priv_global = fsa_priv;
//Antai <AI_BSP_USB> <hehl> <2022-05-09> #41934 end 
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver begin
    yhm1003_load = true;
//Antai <AI_BSP_Audio> <hehl> <2022-06-13> coompatiable ic YHM1003 driver end	
	return 0;

err_data:
	if (gpio_is_valid(fsa_priv->hs_det_pin)) {
		gpio_free(fsa_priv->hs_det_pin);
	}
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int yhm1003_remove(struct i2c_client *i2c)
{
	struct yhm1003_priv *fsa_priv =
			(struct yhm1003_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
	dev_set_drvdata(&i2c->dev, NULL);

	return 0;
}

static const struct of_device_id yhm1003_i2c_dt_match[] = {
	{
		.compatible = "mediatek,yhm1003-i2c",
	},
	{}
};

static struct i2c_driver yhm1003_i2c_driver = {
	.driver = {
		.name = YHM1003_I2C_NAME,
		.of_match_table = yhm1003_i2c_dt_match,
	},
	.probe = yhm1003_probe,
	.remove = yhm1003_remove,
};

static int __init yhm1003_init(void)
{
	int rc;
	pr_info("yhm1003_init enter \n");
	rc = i2c_add_driver(&yhm1003_i2c_driver);
	if (rc)
		pr_err("yhm1003: Failed to register I2C driver: %d\n", rc);

	return rc;
}
late_initcall_sync(yhm1003_init);
//module_init(yhm1003_init);

/*static void __exit yhm1003_exit(void)
{
	i2c_del_driver(&yhm1003_i2c_driver);
}
module_exit(yhm1003_exit);
*/
MODULE_DESCRIPTION("YHM1003 I2C driver");
MODULE_LICENSE("GPL v2");
