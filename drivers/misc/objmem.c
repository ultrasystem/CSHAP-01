#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
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

#define OBJMEM_NAME_PREFIX "dev/ashmem/"
#define OBJMEM_NAME_PREFIX_LEN (sizeof(OBJMEM_NAME_PREFIX) - 1)
#define OBJMEM_FULL_NAME_LEN (OBJMEM_NAME_LEN + OBJMEM_NAME_PREFIX_LEN)

struct objmem_context {
    char            name[OBJMEM_FULL_NAME_LEN];    /* Memory Name */
    pid_t	    pid;
    size_t          size;                        /* size of the mapping, in bytes */
    struct file     *data;                  /* Memory Data */
    spinlock_t      lock;
    __u8            *buffer;
    atomic64_t      update_count;
    atomic_t        count;
    struct list_head entry;             /* Entry for Memory list */
    wait_queue_head_t inq;
    struct file     *owner;
};

struct objmem_data {
    struct objmem_context *context;
    struct file *owner;
    __u64 interval_time;
    __u8 updated;
    atomic_t expired;
    struct hrtimer delay_timer;
    atomic64_t read_count;
};


static DEFINE_MUTEX(objmem_mutex);
static LIST_HEAD(objmem_list);

static struct kmem_cache *objmem_area_cachep __read_mostly;
static struct kmem_cache *objmem_ctx_cachep __read_mostly;

static int objmem_context_queryopen(struct objmem_data *omdata, void __user *arg)
{
    struct objmem_context *next, *context = omdata->context;
    struct objmem_query_open data;

    printk(KERN_INFO "objmem: query & open context.\n");

    if (unlikely(copy_from_user(&data, arg, sizeof(data)))) {

        printk(KERN_ERR "objmem: can not copy user data.\n");
        return -EFAULT;
    }

    printk(KERN_INFO "objmem: name: %s, pid: %d\n", data.name, data.pid);
    if(context != NULL) {
        printk(KERN_ERR "objmem: context is already.\n");
        return -EINVAL;
    }

    mutex_lock(&objmem_mutex);

    if ((omdata->owner->f_flags & 0xff) == O_RDONLY) {

        printk(KERN_INFO "objmem: find context from exist entry.\n");

        list_for_each_entry_safe(context, next, &objmem_list, entry) {

            if( (context->pid == data.pid) &&
                (strncmp(context->name + OBJMEM_NAME_PREFIX_LEN, data.name, strlen(data.name)) == 0 ))
            {

                printk(KERN_INFO "objmem: Open exist context for %s, Count: %d\n", context->name, atomic_read(&context->count));
                atomic_inc(&context->count);
                omdata->context = context;
                break;
            }
        }

        if(omdata->context != NULL) {

            mutex_unlock(&objmem_mutex);
            return 0;
        }
    }

    mutex_unlock(&objmem_mutex);
    return -EIO;
}

static int objmem_context_pid(struct objmem_data *omdata, pid_t pid)
{
    struct objmem_context *next, *context = omdata->context;

    printk(KERN_INFO "objmem: open context by pid: %d.\n", pid);
    if(context != NULL) {
        printk(KERN_ERR "objmem: context is already.\n");
        return -EINVAL;
    }

    mutex_lock(&objmem_mutex);

    if ((omdata->owner->f_flags & 0xff) == O_RDONLY) {

        printk(KERN_INFO "objmem: find context from exist entry.\n");

        list_for_each_entry_safe(context, next, &objmem_list, entry) {

            if(context->pid == pid) {

                printk(KERN_INFO "objmem: Open exist context for %s, Count: %d\n", context->name, atomic_read(&context->count));
                atomic_inc(&context->count);
                omdata->context = context;
                break;
            }
        }

        if(omdata->context != NULL) {

            mutex_unlock(&objmem_mutex);
            return 0;
        }
    }

    mutex_unlock(&objmem_mutex);
    return -EIO;
}

static int objmem_context_create(struct objmem_data *omdata, void __user *arg)
{
    struct objmem_context *next, *context = omdata->context;
    struct objmem_create_data data;

    printk(KERN_INFO "objmem: create new context.\n");

    if (unlikely(copy_from_user(&data, arg, sizeof(data)))) {

        printk(KERN_ERR "objmem: can not copy user data.\n");
        return -EFAULT;
    }

    printk(KERN_INFO "objmem: name: %s, size: %d\n", data.name, data.size);

    if(context != NULL) {
        printk(KERN_ERR "objmem: context is already.\n");
        return -EINVAL;
    }

    mutex_lock(&objmem_mutex);

    if ((omdata->owner->f_flags & 0xff) == O_RDONLY) {

        printk(KERN_INFO "objmem: find context from exist entry.\n");

        list_for_each_entry_safe(context, next, &objmem_list, entry) {

            if(strncmp(context->name + OBJMEM_NAME_PREFIX_LEN, data.name, OBJMEM_NAME_LEN) == 0 ) {

                printk(KERN_INFO "objmem: Open exist context for %s, Count: %d\n", context->name, atomic_read(&context->count));
                atomic_inc(&context->count);
                omdata->context = context;
                break;
            }
        }

        if(omdata->context != NULL) {

            mutex_unlock(&objmem_mutex);
            return 0;
        }
    }

    if((omdata->owner->f_flags & 0xff) != O_WRONLY) {
        mutex_unlock(&objmem_mutex);
        printk(KERN_ERR "objmem: Invalid open mode.\n");
        return -EINVAL;
    }

    if( (data.size == 0) || (data.size > (32 * 4096)) ) {
        mutex_unlock(&objmem_mutex);
        printk(KERN_ERR "objmem: invalid data size.\n");
        return -EINVAL;
    }

    printk(KERN_INFO "objmem: allocate new context.\n");

    context = kmem_cache_zalloc(objmem_ctx_cachep, GFP_KERNEL);
    if (unlikely(!context)) {

        mutex_unlock(&objmem_mutex);
        return -ENOMEM;
    }

    memset(context, 0, sizeof(struct objmem_context));

    spin_lock_init(&context->lock);
    init_waitqueue_head(&context->inq);

    printk(KERN_INFO "objmem: save context name.\n");

    memcpy(context->name, OBJMEM_NAME_PREFIX, OBJMEM_NAME_PREFIX_LEN);
    memcpy(context->name + OBJMEM_NAME_PREFIX_LEN, data.name, OBJMEM_NAME_LEN);
    context->name[OBJMEM_NAME_LEN-1] = '\0';
    context->pid = current->pid;
    context->size = data.size;
    context->owner = omdata->owner;

    atomic_inc(&context->count);
    atomic64_set(&context->update_count, -1);

    printk(KERN_INFO "objmem: Save context.\n");

    list_add_tail(&context->entry, &objmem_list);
    omdata->context = context;

    printk(KERN_INFO "objmem: New context for %s, Count: %d\n", context->name, atomic_read(&context->count));
    mutex_unlock(&objmem_mutex);

    return 0;
}

static void onjmem_context_close(struct objmem_context *context)
{
    mutex_lock(&objmem_mutex);

    printk(KERN_INFO "objmem: Close context for %s, Count: %d\n",context->name, atomic_read(&context->count));

    if( atomic_dec_and_test(&context->count) ) {

        printk(KERN_INFO "objmem: Context for %s is deleted.\n", context->name);

        if(context->data) {
            fput(context->data);
            context->data = NULL;
        }

        if(context->buffer) {
            kfree(context->buffer);
            context->buffer = NULL;
        }

        list_del(&context->entry);
        kmem_cache_free(objmem_ctx_cachep, context);
    }

    mutex_unlock(&objmem_mutex);
}

static int objmem_open(struct inode *inode, struct file *file)
{
    struct objmem_data *omdata;

    printk(KERN_INFO "objmem: Open inode: %p, file: %p, flags: %x\n", inode, file, file->f_flags);

    nonseekable_open(inode, file);

    omdata = kmem_cache_zalloc(objmem_area_cachep, GFP_KERNEL);
    if (unlikely(!omdata))
        return -ENOMEM;

    if( ((file->f_flags & 0xff) == O_WRONLY) || ((file->f_flags & 0xff) == O_RDONLY) ){

        atomic64_set(&omdata->read_count, -1);
        atomic_set(&omdata->expired, 0);
        omdata->context = NULL;
        omdata->owner = file;
        omdata->updated = 0;
        omdata->interval_time = 0;
        file->private_data = omdata;
        return 0;
    }

    return -EINVAL;
}

static int objmem_release(struct inode *ignored, struct file *file)
{
    struct objmem_data *omdata = file->private_data;
    struct objmem_context *context = omdata->context;

    printk(KERN_INFO "objmem: Close inode: %p, file: %p, ctx: %p\n", ignored, file, context);

    if( context != NULL ) {

        onjmem_context_close(context);
    }

    if( omdata->interval_time != 0 ) {

        hrtimer_cancel(&omdata->delay_timer);
        omdata->interval_time = 0;
    }

    omdata->context = NULL;
    kmem_cache_free(objmem_area_cachep, omdata);
	return 0;
}

static enum hrtimer_restart objmem_hrtime_fun(struct hrtimer *hr_timer)
{
   struct objmem_data *omdata = container_of(hr_timer, struct objmem_data, delay_timer) ;

   atomic_set(&omdata->expired, 1);
   return HRTIMER_NORESTART;
}

static int objmem_hrtimer_create(struct objmem_data *omdata, uint64_t interval)
{
    if(interval == 0) {
        return -EINVAL;
    }

    if(omdata->context == NULL) {
        return -EINVAL;
    }

    hrtimer_init(&omdata->delay_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    omdata->delay_timer.function = objmem_hrtime_fun;
    omdata->interval_time = interval;
    hrtimer_start(&omdata->delay_timer, ns_to_ktime(omdata->interval_time), HRTIMER_MODE_REL);

    return 0;
}

static long objmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct objmem_data *omdata = file->private_data;
    long ret = -EINVAL;

    switch (cmd) {
    case OBJMEM_IOCTL_CREATE:
        ret = objmem_context_create(omdata, (void __user *) arg);
        break;
    case OBJMEM_IOCTL_INTERVAL:
        ret = objmem_hrtimer_create(omdata, (unsigned long) arg);
        break;
    case OBJMEM_IOCTL_GETPID:
        if(omdata->context == NULL) {
            return -EINVAL;
        }

        if (copy_to_user((void __user *)arg, &omdata->context->pid, sizeof(omdata->context->pid)))
            return -EFAULT;
        ret = 0;
        break;
    case OBJMEM_IOCTL_OPENBYPID:
        ret = objmem_context_pid(omdata, (pid_t) arg);
        break;
    case OBJMEM_IOCTL_QUERYOPEN:
        ret = objmem_context_queryopen(omdata, (void __user *) arg);
        break;
    }

    return ret;
}

static ssize_t objmem_read(struct file *file, char __user *buf,
			   size_t len, loff_t *pos)
{
    unsigned long flags = 0;
    struct objmem_data *omdata = file->private_data;
    struct objmem_context *context = omdata->context;
    int ret = 0;

    //printk(KERN_INFO "objmem: read file: %p, context: %p, buf: %p, size: %d.\n", file, context, buf, len);

    if(context == NULL) {

        printk(KERN_ERR "objmem: no read context.\n");
        return -EINVAL;
    }

    if(context->size == 0) {

        printk(KERN_ERR "objmem: read zero.\n");
        return -ENOMEM;
    }

    if( (buf == NULL) || (context->buffer == NULL) ) {

        printk(KERN_ERR "objmem: no buffers to read.\n");
        return -EBADF;
    }

    if(len != context->size) {

        printk(KERN_ERR "objmem: invalid read bytes.\n");
        return -ENOMEM;
    }

    if( atomic64_read(&omdata->read_count) == atomic64_read(&context->update_count) ) {

        return -EAGAIN;
    }

    spin_lock_irqsave(&context->lock, flags);
    ret = copy_to_user(buf, context->buffer, len);
    omdata->updated = 0;
    spin_unlock_irqrestore(&context->lock, flags);

    if(ret == 0) {

        if( atomic64_read(&omdata->read_count) == atomic64_read(&context->update_count) ) {

            return -EAGAIN;
        }

        omdata->read_count = context->update_count;
        return len;
    }

    return -EIO;
}

static ssize_t objmem_write(struct file *file, const char __user *buf,
               size_t len, loff_t *pos)
{
    unsigned long flags = 0;
    struct objmem_data *omdata = file->private_data;
    struct objmem_context *context = omdata->context;
    int ret = 0;

    //printk(KERN_INFO "objmem: write file: %p, context: %p, buf: %p, size: %d.\n", file, context, buf, len);

    if(context == NULL) {
        printk(KERN_ERR "objmem: no context.\n");
        return -EINVAL;
    }

    if(context->size == 0) {
        printk(KERN_ERR "objmem: no allocate memory.\n");
        return -ENOMEM;
    }

    if( (buf == NULL) ) {

        printk(KERN_ERR "objmem: no buffers to write.\n");
        return -EINVAL;
    }

    if(len != context->size) {

        printk(KERN_ERR "objmem: request bytes invalid.\n");
        return -ENOMEM;
    }

    if(context->buffer == NULL) {

        mutex_lock(&objmem_mutex);
        context->buffer = kmalloc(context->size, GFP_ATOMIC);
        mutex_unlock(&objmem_mutex);

        if(context->buffer == NULL) {

            printk(KERN_ERR "objmem: no memory.\n");
            return -ENOMEM;
        }
    }

    spin_lock_irqsave(&context->lock, flags);
    ret = copy_from_user(context->buffer, buf, len);
    spin_unlock_irqrestore(&context->lock, flags);

    if(ret == 0) {
        wake_up_all(&context->inq);
        atomic64_inc(&context->update_count);
        return len;
    }

    return -EIO;
}

static unsigned objmem_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int mask = 0;
    struct objmem_data *omdata = file->private_data;
    struct objmem_context *context = omdata->context;

    if(context == NULL) {
        return -EINVAL;
    }

    poll_wait(file, &context->inq, wait);

    while( atomic64_read(&context->update_count) != atomic64_read(&omdata->read_count) )
    {
        if( omdata->interval_time == 0 ) {
            mask = POLLIN;
            break;
        }

        if( omdata->updated != 0 ) {
            mask = POLLIN;
            break;
        }

        if( !atomic_cmpxchg(&omdata->expired, 1, 0) ) {
            break;
        }

        hrtimer_start(&omdata->delay_timer, ns_to_ktime(omdata->interval_time), HRTIMER_MODE_REL);

        omdata->updated = 1;
        mask = POLLIN;
        break;
    }

    return mask;
}

static struct file_operations objmem_fops = {
	.owner = THIS_MODULE,
    .open = objmem_open,
    .release = objmem_release,
    .write = objmem_write,
    .read = objmem_read,
    .poll = objmem_poll,
    .llseek = no_llseek,
    .unlocked_ioctl = objmem_ioctl,
    .compat_ioctl = objmem_ioctl,
};

static struct miscdevice objmem_misc = {
	.minor = MISC_DYNAMIC_MINOR,
    .name = "objmem",
    .fops = &objmem_fops,
};

static int __init objmem_init(void)
{
	int ret;

    printk(KERN_INFO "objmem: startup...\n");

    INIT_LIST_HEAD(&objmem_list);

    objmem_area_cachep = kmem_cache_create("objmem_area_cache",
                      sizeof(struct objmem_data),0, 0, NULL);
    if (unlikely(!objmem_area_cachep)) {
        printk(KERN_ERR "objmem: failed to create slab cache\n");
		return -ENOMEM;
	}

    objmem_ctx_cachep = kmem_cache_create("objmem_ctx_cache",
                      sizeof(struct objmem_context),
                      0, 0, NULL);
    if (unlikely(!objmem_ctx_cachep)) {
        printk(KERN_ERR "objmem: failed to create slab cache\n");
        return -ENOMEM;
    }

    ret = misc_register(&objmem_misc);
	if (unlikely(ret)) {
        printk(KERN_ERR "objmem: failed to register misc device!\n");
		return ret;
	}

    printk(KERN_INFO "objmem: initialized\n");

	return 0;
}

static void __exit objmem_exit(void)
{
	int ret;

    ret = misc_deregister(&objmem_misc);
	if (unlikely(ret))
        printk(KERN_ERR "objmem: failed to unregister misc device!\n");

    kmem_cache_destroy(objmem_ctx_cachep);
    kmem_cache_destroy(objmem_area_cachep);

    printk(KERN_INFO "objmem: unloaded\n");
}

module_init(objmem_init);
module_exit(objmem_exit);

MODULE_LICENSE("GPL");
