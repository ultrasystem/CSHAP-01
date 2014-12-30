#include <linux/export.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/ipc.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/security.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/personality.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/shmem_fs.h>
#include <linux/objmem.h>
#include <asm/pgtable.h>
#include <linux/poll.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/autopilot.h>

#define MAX_USER_BUFSIZE        128
#define MAX_DATA_BUFSIZE        2048

asmlinkage int sys_i2c_transfer(int busnum,
                                uint16_t addr,
                                const uint8_t __user *send,
                                unsigned send_len,
                                uint8_t __user *recv,
                                unsigned recv_len)
{
    struct i2c_adapter *adap = i2c_get_adapter(busnum);
    struct i2c_msg msgv[2];
    uint8_t ksend[MAX_USER_BUFSIZE], krecv[MAX_USER_BUFSIZE];
    unsigned msgs;
    int ret;
    unsigned retry_count = 0;

    if(adap == NULL) {
        printk(KERN_ERR "%s: Cannot find the adapter from bus%d\n", __func__, busnum);
        return -ENODEV;
    }

    printk(KERN_INFO "%s: Transfer message to adapter %x bus%d\n", __func__, adap, busnum);

    do {
        msgs = 0;
        if (send_len > 0) {

            if (copy_from_user(ksend, send, sizeof(send_len))) {
                printk(KERN_ERR "%s: Cannot copy data from userbuffer\n", __func__);
                return -EFAULT;
            }

            msgv[msgs].addr = addr;
            msgv[msgs].flags = 0;
            msgv[msgs].buf = ksend;
            msgv[msgs].len = send_len;
            msgs++;
        }

        if (recv_len > 0) {
            msgv[msgs].addr = addr;
            msgv[msgs].flags = I2C_M_RD;
            msgv[msgs].buf = krecv;
            msgv[msgs].len = recv_len;
            msgs++;
        }

        if(msgs == 0) {
            return -EINVAL;
        }

        ret = i2c_transfer(adap, msgv, ARRAY_SIZE(msgv));
        if(ret < 0) {
            printk(KERN_ERR "%s: Cannot transfer data to adapter bus%d, err = %d\n", __func__, busnum, ret);
            continue;
        }

        if ((recv_len > 0) && copy_to_user(recv, krecv, recv_len)) {

            printk(KERN_ERR "%s: Cannot copy data to userbuffer\n", __func__);
            return -ENOMEM;
        }

        break;

    } while (++retry_count < adap->retries);

    return ret != ARRAY_SIZE(msgv) ? -EIO : 0;
}

asmlinkage int sys_i2c_pack_transfer(int busnum,
                                uint16_t addr,
                                struct i2c_msg __user *msgv,
                                unsigned msgs
                                )
{
    int             ret = -ENOMEM;
    unsigned        msize = msgs * sizeof(struct i2c_msg);
    unsigned        dsize = MAX_DATA_BUFSIZE;
    uint8_t         *dbuf = NULL, *p;
    unsigned        i, retry_count = 0;
    struct i2c_msg  *kmsg = NULL;
    struct i2c_adapter *adap = i2c_get_adapter(busnum);

    if(adap == NULL) {
        printk(KERN_ERR "%s: Cannot find the adapter from bus%d\n", __func__, busnum);
        return -ENODEV;
    }

    if(msize == 0) {
        printk(KERN_ERR "%s: Invalid pack message paramter, size = %d\n", __func__, msize);
        return -EINVAL;
    }

    printk(KERN_INFO "%s: Transfer message(packs) to adapter %x bus%d\n", __func__, adap, busnum);

    kmsg = kmalloc(msize + dsize, GFP_KERNEL);
    if(kmsg == NULL) {
        printk(KERN_ERR "%s: memory alloc failed, size: %d\n", __func__, msize + dsize);
        return ret;
    }

    memcpy(kmsg, msgv, msize);
    p = dbuf = ((uint8_t *) kmsg + msize);

    /* force the device address into the message vector */
    for (i = 0; i < msgs; i++) {
        kmsg[i].addr = addr;
        if(kmsg[i].len != 0) {
            if( (kmsg[i].flags == 0) && copy_from_user(p, (void __user*)kmsg[i].buf, kmsg[i].len) ) {
                printk(KERN_ERR "%s: Cannot copy data from userbuffer\n", __func__);
                goto err;
            }
            kmsg[i].buf = p;
            p += kmsg[i].len;
        }
    }

    do {
        ret = i2c_transfer(adap, msgv, msgs);

        if(ret < 0) {
            continue;
        }

        for (i = 0; i < msgs; i++) {
            if(kmsg[i].len != 0) {
                if( (kmsg[i].flags != 0) && copy_to_user((void __user*)msgv[i].buf, p, kmsg[i].len) ) {
                    printk(KERN_ERR "%s: Cannot copy data to userbuffer\n", __func__);
                    goto err;
                }
            }
        }

        break;
    } while (retry_count++ < adap->retries);

err:
    kfree(kmsg);
    return ret != msgs ? -EIO : 0;
}
