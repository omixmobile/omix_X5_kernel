//Antai <ANT_BSP_SYS> <chuqf/chendonghai/lihl> <20161028> for secure_boot, begin

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int sbcflag_proc_show(struct seq_file *m, void *v)
{
    char *ptr;
    char sbc_value[5] = {0};
    int len = 0;
    ptr = strstr(saved_command_line, "sbc_flag=");
    if (ptr != NULL){
        len = strlen("sbc_flag=");
        if (strlen(ptr) > 0){
            ptr += len;
            strncpy(sbc_value, ptr, 1);
            seq_printf(m, "%s\n", sbc_value);
        }else{
            seq_printf(m, "%s\n", "0");
        }
    }else{
        seq_printf(m, "%s\n", "0");
    }
	return 0;
}

static int sbcflag_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sbcflag_proc_show, NULL);
}

static const struct file_operations sbcflag_proc_fops = {
	.open		= sbcflag_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_sbcflag_init(void)
{
	proc_create("sbcflag", 0444, NULL, &sbcflag_proc_fops);
	return 0;
}
fs_initcall(proc_sbcflag_init);

//Antai <ANT_BSP_SYS> <chuqf/chendonghai/lihl> <20161028> for secure_boot, end
