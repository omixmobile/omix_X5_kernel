//Antai <ANT_BSP_SYS> <luobowen> <20200725> ufs begin

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "../../../vendor/mediatek/proprietary/bootable/bootloader/preloader/Antai_Memory_type.h"


#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
#include <linux/ai_device_check.h>
extern int ai_set_device_info(struct ai_device_info ai_dev_info);
#endif

#define MAX_PRODUCT_ID_LEN 16

static int antaimeminfo_proc_show(struct seq_file *m, void *v)
{
    char *ptr;
    char antaimeminfo_value[MAX_PRODUCT_ID_LEN+1] = {0};
    int len = 0;
    ptr = strstr(saved_command_line, "antaimeminfo=");
    if (ptr != NULL){
        len = strlen("antaimeminfo=");
        if (strlen(ptr) > 0){
            ptr += len;
            strncpy(antaimeminfo_value, ptr, MAX_PRODUCT_ID_LEN+1);
            seq_printf(m, "%s\n", antaimeminfo_value);
        }else{
            seq_printf(m, "%s\n", "0");
        }
    }else{
        seq_printf(m, "%s\n", "0");
    }
	return 0;
}


static int antaimeminfo_add_to_aidev(void)
{
    char *ptr;
    int len = 0;
	int i = 0;
	int ret =0;
	char  antaimeminfo[10] = {0};
	struct ai_device_info ai_mydev_info;
	printk("antaimeminfo_add_to_aidev\n");
    ptr = strstr(saved_command_line, "antaimeminfo=");
    if (ptr != NULL){
        len = strlen("antaimeminfo=");
		printk("antaimeminfo_value = %s \n",ptr);
        if (strlen(ptr) > 0){
            ptr += len;
			printk("antaimeminfo_value = %s \n",ptr);
			strncpy(antaimeminfo, ptr, 1);
			ret = kstrtouint(antaimeminfo, 0, &i);
	        if (ret < 0 || (i >= EMMC_COMPATIBLE_NUM)){
				printk("antaimeminfo_value kstrtouint err \n");
		    return ret;
			}
	
			printk("antaimeminfo_value = %s \n",Antai_emmc_support[i]);
		
			#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK	
			ai_mydev_info.ai_dev_type = AI_DEVICE_TYPE_DDR;
			strlcpy(ai_mydev_info.name, Antai_emmc_support[i], MAX_PRODUCT_ID_LEN + 1);
			ai_set_device_info(ai_mydev_info);
			#endif
			printk("antaimeminfo_value = %s \n",ai_mydev_info.name);
        }else{
		   printk(" antaimeminfo len is 0 \n");
        }
    }else{
		printk(" antaimeminfo is null \n");
    }
	return 0;
}

static int antaimeminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, antaimeminfo_proc_show, NULL);
}

static const struct file_operations antaimeminfo_proc_fops = {
	.open		= antaimeminfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_antaimeminfo_init(void)
{

	antaimeminfo_add_to_aidev();
	proc_create("antaimeminfo", 0444, NULL, &antaimeminfo_proc_fops);
	
	return 0;
}
fs_initcall(proc_antaimeminfo_init);

//Antai <ANT_BSP_SYS> <luobowen> <20200725> for ufs id end
