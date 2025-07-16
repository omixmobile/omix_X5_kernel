/* Copyright Statement:
 * //Antaiui <AI_BSP_MISC> <yaoyc> <2022-04-26> add begin
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#ifndef CONFIG_RTC_DRV_MT6397
#include <mt-plat/mtk_rtc.h>
#endif
//#include <linux/kobject.h>
//#include <asm/atomic.h>
//#include <asm/uaccess.h>
static struct notifier_block ai_nb;

#ifdef CONFIG_RTC_DRV_MT6397
  #ifdef CONFIG_AI_BSP_CMD_TO_FACTORY_BOOTMENU	
  extern void rtc_mark_bootmenu(void);	
  extern void rtc_mark_factory(void);	
  #endif
#endif
static int ai_reboot_notifier(struct notifier_block *nb,
 				    unsigned long code, void *data)
{
	char *cmd = (char *) data;
	pr_info("%s  code:%ld, cmd:%s\n", __func__, code, cmd);
	if (cmd && !strcmp(cmd, "bootmenu")){
		//pr_info("%s bootmenu \n", __func__);
		rtc_mark_bootmenu();
	}else if(cmd && !strcmp(cmd, "factory")) {
		//pr_info("%s factory \n", __func__);
		rtc_mark_factory();
	}
 	return 0;
}
static int __init ai_reboot_mod_init(void)
{
	int ret = 0;
	pr_info("%s\n", __func__);
	ai_nb.notifier_call = ai_reboot_notifier;
	ret = register_reboot_notifier(&ai_nb);
	if(ret) {
		pr_err("%s register reboot error ret :%d\n", ret);
	}
	return ret;
}

/* should never be called */
static void __exit ai_reboot_mod_exit(void)
{
	pr_info("%s\n", __func__);
}

//module_init core_initcall
subsys_initcall_sync(ai_reboot_mod_init);
module_exit(ai_reboot_mod_exit);

MODULE_AUTHOR("yoayc <yaoyc@antaiui.com>");
MODULE_DESCRIPTION("ai_reboot Driver v0.1");
MODULE_LICENSE("GPL");
//Antaiui <AI_BSP_MISC> <yaoyc> <2022-04-26> add end
