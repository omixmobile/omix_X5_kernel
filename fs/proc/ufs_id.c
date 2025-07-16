//Antai <ANT_BSP_SYS> <luobowen> <20200725> ufs begin

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>


#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
#include <linux/ai_device_check.h>
extern int ai_set_device_info(struct ai_device_info ai_dev_info);
#endif

#define MAX_PRODUCT_ID_LEN 16

static int ufs_id_proc_show(struct seq_file *m, void *v)
{
    char *ptr;
    char ufs_id_value[MAX_PRODUCT_ID_LEN+1] = {0};
    int len = 0;
    ptr = strstr(saved_command_line, "ufs_id=");
    if (ptr != NULL){
        len = strlen("ufs_id=");
        if (strlen(ptr) > 0){
            ptr += len;
            strncpy(ufs_id_value, ptr, MAX_PRODUCT_ID_LEN+1);
            seq_printf(m, "%s\n", ufs_id_value);
        }else{
            seq_printf(m, "%s\n", "0");
        }
    }else{
        seq_printf(m, "%s\n", "0");
    }
	return 0;
}


static int ufs_id_add_to_aidev(void)
{
    char *ptr;
    int len = 0;
	struct ai_device_info ai_mydev_info;
    ptr = strstr(saved_command_line, "ufs_id=");
    if (ptr != NULL){
        len = strlen("ufs_id=");
        if (strlen(ptr) > 0){
            ptr += len;
			#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK	
			ai_mydev_info.ai_dev_type = AI_DEVICE_TYPE_MEMORY;
			strlcpy(ai_mydev_info.name, ptr, MAX_PRODUCT_ID_LEN + 1);
			ai_set_device_info(ai_mydev_info);
			#endif
			printk("ufs_id_value = %s \n",ai_mydev_info.name);
        }else{
		   printk(" ufs_id len is 0 \n");
        }
    }else{
		printk(" ufs_id is null \n");
    }
	return 0;
}

static int ufs_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_id_proc_show, NULL);
}

static const struct file_operations ufs_id_proc_fops = {
	.open		= ufs_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_ufs_id_init(void)
{

	ufs_id_add_to_aidev();
	proc_create("ufs_id", 0444, NULL, &ufs_id_proc_fops);
	

	
	return 0;
}
fs_initcall(proc_ufs_id_init);

//Antai <ANT_BSP_SYS> <luobowen> <20200725> for ufs id end
