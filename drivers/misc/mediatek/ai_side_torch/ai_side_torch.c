#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/i2c.h>

#include "ai_side_torch.h"

#define SIDE_TORCH_DTS    "mediatek,side_torch_ctl"
#define SIDE_TORCH_NAME   "ai_side_torch"

static ssize_t store_side_torch_level(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int level;

    pr_debug("Entry.\n");

    level = simple_strtoul(buf, NULL, 0);
    pr_debug("level=%d\n", level);
    set_side_torch(level);

    pr_debug("Exit.\n");
    return len;
}

static ssize_t show_side_torch_level(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "torch current = %d (mA)\n", get_torch_level());
}

DEVICE_ATTR(side_torch_ctl, 0660, show_side_torch_level, store_side_torch_level);

static int side_torch_create_sysfs(struct device *dev)
{
    pr_info("E.\n");
    return device_create_file(dev, &dev_attr_side_torch_ctl);
}

static struct platform_device side_torch_pdev = {
    .name = SIDE_TORCH_NAME,
    .id = -1,
};

int side_torch_init(struct device *dev)
{
    int ret = 0;
    pr_debug("Entry!\n");

    ret = platform_device_register(&side_torch_pdev);
    if (ret != 0) {
        pr_err("register device failed (%d)\n", ret);
        return ret;
    }

    ret = side_torch_create_sysfs(&(side_torch_pdev.dev));
    if (ret) {
        platform_device_unregister(&side_torch_pdev);
        pr_err("create side torch sysfs fail! (%d)\n", ret);
        return ret;
    }

    pr_debug("Exit!\n");
    return ret;
}

int side_torch_exit(struct device *dev)
{
    pr_debug("E!\n");
    platform_device_unregister(&side_torch_pdev);
    return 0;
}
