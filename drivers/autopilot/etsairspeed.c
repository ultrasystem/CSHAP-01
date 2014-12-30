/*
 * Copyright (C) 2011 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/etsairspeed.h>

struct etsairspeed_data {
	struct i2c_client *client;
	struct mutex lock;
};

static int etsairspeed_transfer(struct i2c_client *client, const u8 *send, unsigned send_len,
                                u8 *recv, unsigned recv_len)
{
    struct i2c_msg msgv[2];
    unsigned msgs;
    int ret;
    unsigned retry_count = 0;

    do {
        msgs = 0;
        if (send_len > 0) {
            msgv[msgs].addr = client->addr;
            msgv[msgs].flags = 0;
            msgv[msgs].buf = (uint8_t *)send;
            msgv[msgs].len = send_len;
            msgs++;
        }

        if (recv_len > 0) {
            msgv[msgs].addr = client->addr;
            msgv[msgs].flags = I2C_M_RD;
            msgv[msgs].buf = recv;
            msgv[msgs].len = recv_len;
            msgs++;
        }

        if(msgs == 0) {
            return -EINVAL;
        }

        ret = i2c_transfer(client->adapter, msgv, ARRAY_SIZE(msgv));
        if(ret < 0) {
            continue;
        }

    } while (++retry_count < I2C_TRIES);

    return ret != ARRAY_SIZE(msgv) ? -EIO : 0;
}

static int etsairspeed_measure(struct etsairspeed_data *etsdata)
{
    int ret;
    u8 cmd = READ_CMD;
    ret = etsairspeed_transfer(etsdata->client, &cmd, 1, NULL, 0);
    if(ret < 0) {
        pr_err("%s: can't send CMD\n", __func__);
    }

    return ret;
}

static int etsairspeed_collect(struct etsairspeed_data *etsdata, uint16_t *raw_data)
{
    int      ret = -EIO;
    uint16_t diff_pres_pa_raw;
    /* read from the sensor */
    uint8_t  val[2] = {0, 0};

    ret = etsairspeed_transfer(etsdata->client, NULL, 0, val, 2);
    if (ret < 0) {
        pr_err("%s: can't read CMD\n", __func__);
        return ret;
    }

    diff_pres_pa_raw = val[1] << 8 | val[0];
    if (diff_pres_pa_raw == 0) {
        pr_debug("%s: zero value from sensor", __func__);
        return -1;
    }

    *raw_data = diff_pres_pa_raw;
    return 0;
}

static int common_open(struct inode *inode, struct file *file)
{
    nonseekable_open(inode, file);

    return 0;
}

static int common_release(struct inode *ignored, struct file *file)
{
    return 0;
}


static long common_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct etsairspeed_data *etsdata = file->private_data;
    long ret = -EINVAL;

    switch (cmd) {
    case ETS_AIRSPEED_IOCTL_PREREAD:
        ret = etsairspeed_measure(etsdata);
        break;
    default:
        break;
    }

    return ret;
}


static ssize_t common_read(struct file *file, char __user *buf,
               size_t len, loff_t *pos)
{
    int ret = -EIO;
    struct etsairspeed_data *etsdata = file->private_data;
    uint16_t raw_data = 0;

    if(len != sizeof(raw_data)) {
        return ret;
    }

    if(etsairspeed_collect(etsdata, &raw_data) == 0) {
        ret = copy_to_user(buf, &raw_data, sizeof(raw_data));
        if(ret == 0) {
            return sizeof(raw_data);
        }
    }

    return ret;
}

static ssize_t common_write(struct file *file, const char __user *buf,
               size_t len, loff_t *pos)
{
    return -EIO;
}

static unsigned common_poll(struct file *file, struct poll_table_struct *wait)
{
    return 0;
}


static struct file_operations common_dev_fops = {
    .owner = THIS_MODULE,
    .open = common_open,
    .release = common_release,
    .write = common_write,
    .read = common_read,
    .poll = common_poll,
    .llseek = no_llseek,
    .unlocked_ioctl = common_ioctl,
    .compat_ioctl = common_ioctl,
};

static struct miscdevice etsairspeed_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = ETSAIRSPEED_DRV_NAME,
    .fops = &common_dev_fops,
};

static int __devinit etsairspeed_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    struct etsairspeed_data *etsdata;

	pr_debug("%s: enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: client not i2c capable\n", __func__);
		return -EIO;
	}

    etsdata = kzalloc(sizeof(*etsdata), GFP_KERNEL);
    if (!etsdata) {
		pr_err("%s: failed to allocate memory for module\n", __func__);
		return -ENOMEM;
	}

    mutex_init(&etsdata->lock);
    etsdata->client = client;

    i2c_set_clientdata(client, etsdata);
    return 0;
}

static int __devexit etsairspeed_remove(struct i2c_client *client)
{
	/* TO DO: revisit ordering here once _probe order is finalized */
    struct etsairspeed_data *etsdata = i2c_get_clientdata(client);

    pr_debug("%s: etsairspeed_remove +\n", __func__);

    mutex_destroy(&etsdata->lock);
    kfree(etsdata);

    pr_debug("%s: etsairspeed_remove -\n", __func__);
	return 0;
}

static int etsairspeed_resume(struct device *dev)
{
	return 0;
}

static int etsairspeed_suspend(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id etsairspeed_id[] = {
    {ETSAIRSPEED_DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, etsairspeed_id);
static const struct dev_pm_ops etsairspeed_pm_ops = {
    .suspend	= etsairspeed_suspend,
    .resume		= etsairspeed_resume,
};

static struct i2c_driver etsairspeed_driver = {
	.driver = {
        .name	= ETSAIRSPEED_DRV_NAME,
		.owner	= THIS_MODULE,
        .pm	= &etsairspeed_pm_ops,
	},
    .probe		= etsairspeed_probe,
    .remove		= __devexit_p(etsairspeed_remove),
    .id_table	= etsairspeed_id,
};

static int __init etsairspeed_init(void)
{
    int ret = 0;
	pr_debug("%s: _init\n", __func__);

    ret = misc_register(&etsairspeed_misc);
    if (unlikely(ret)) {
        printk(KERN_ERR "%s: failed to register misc device!\n", __func__);
        return ret;
    }

    return i2c_add_driver(&etsairspeed_driver);
}

static void __exit etsairspeed_exit(void)
{
	pr_debug("%s: _exit +\n", __func__);
    misc_deregister(&etsairspeed_misc);
    i2c_del_driver(&etsairspeed_driver);
	pr_debug("%s: _exit -\n", __func__);
	return;
}

MODULE_AUTHOR("Liio Chen <liiochen@tencent.com>");
MODULE_DESCRIPTION("ETS Airspeed sensor driver");
MODULE_LICENSE("GPL v2");

module_init(etsairspeed_init);
module_exit(etsairspeed_exit);
