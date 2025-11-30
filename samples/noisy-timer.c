#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>

#include "noisy-timer.h"

#define DEVICE_NAME "noisy_timer"
#define BUFF_LEN 128
#define PERIOD 2

static dev_t nt_dev_number;

struct noisy_timer {
	bool reading;
	int buf_off;
	int buflen;
	atomic64_t counts;
	char buf[BUFF_LEN];
	struct cdev cd;
	struct task_struct *timer_task;
} *nt_devp;

static struct cdev *nt_cdev;
static struct class *nt_class;

static int timer_thread(void *arg)
{
	allow_signal(SIGKILL | SIGCHLD);
	while (!kthread_should_stop()) {
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(PERIOD*HZ);
		atomic64_inc(&nt_devp->counts);
	}
	return 0;
}

int nt_open(struct inode *inode, struct file *file)
{
	return 0;
}

int nt_release(struct inode *inode, struct file *file)
{
	nt_devp->reading = false;
	return 0;
}

ssize_t nt_read(struct file *file, char __user *buf, size_t count,
		loff_t *f_pos)
{
	ssize_t left = 0;
	if (!nt_devp->reading) {
		nt_devp->reading = true;
		nt_devp->buflen = snprintf(nt_devp->buf, sizeof(nt_devp->buf),
					   "%llu\n",
					   atomic64_read(&nt_devp->counts));
		nt_devp->buf_off = 0;
	}
	left = nt_devp->buflen;
	if (left > count)
		left = count;
	if (left <= 0)
		return left;
	int err = copy_to_user(buf, nt_devp->buf + nt_devp->buf_off, left);
	if (err) {
		printk(KERN_ERR "copy_to_user failed: %d\n", err);
		return err;
	}
	nt_devp->buflen -= left;
	nt_devp->buf_off += left;
	return left;
}

ssize_t nt_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static struct file_operations nt_fops = {
	.owner = THIS_MODULE,
	.open = nt_open,
	.release = nt_release,
	.read = nt_read,
	.write = nt_write,
};

static int __init prepare_cdev(struct cdev **cd, const dev_t devnum)
{
	int ret;
	*cd = cdev_alloc();
	kobject_set_name(&((*cd)->kobj), "%s", "noisy-timer");
	cdev_init(*cd, &nt_fops);
	ret = cdev_add(*cd, devnum, 1);
	if (ret) {
		printk(KERN_ERR "Failed to add cdev: %d\n", ret);
	}
	nt_cdev = *cd;
	nt_devp->cd = **cd;
	return ret;
}

static int spawn_timer_task(void) {
	struct task_struct *ttask =
		kthread_create(timer_thread, NULL, "nttimer");
	if (IS_ERR(ttask)) {
		return PTR_ERR(ttask);
	}
	nt_devp->timer_task = ttask;
	wake_up_process(ttask);
	return 0;
}

static int __init noisy_timer_init(void)
{
	int ret;
	dev_t devnum;
	struct cdev *cdev = NULL;
	ret = alloc_chrdev_region(&nt_dev_number, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		printk(KERN_ERR "device number allocation failed: %d\n", ret);
		return ret;
	}
	devnum = MKDEV(MAJOR(nt_dev_number), 0);
	register_chrdev_region(devnum, 1, "noisy-timer");

	nt_class = class_create(DEVICE_NAME);
	if (IS_ERR(nt_class)) {
		printk(KERN_ERR "device class creation failed: %d\n", ret);
		ret = -ENODEV;
		goto free_devnum;
	}
	nt_devp = kzalloc(sizeof(struct noisy_timer), GFP_KERNEL);
	if (!nt_devp) {
		printk(KERN_ERR "chardev data allocation failed\n");
		ret = -ENOMEM;
		goto destroy_class;
	}

	ret = prepare_cdev(&cdev, devnum);
	if (ret < 0) {
		goto free_devp;
	}
	struct device *dev =
		device_create(nt_class, NULL, devnum, NULL, "noisy_timer");
	if (IS_ERR(dev)) {
		printk(KERN_ERR "Failed to created device.\n");
		ret = -ENODEV;
		goto free_devp;
	}
	if (spawn_timer_task()) {
		goto free_devp;
	}
	return 0;
free_devp:
	kfree(nt_devp);
destroy_class:
	class_destroy(nt_class);
free_devnum:
	unregister_chrdev_region(nt_dev_number, 1);
	return ret;
}

static void __exit noisy_timer_exit(void)
{
	device_destroy(nt_class, MKDEV(MAJOR(nt_dev_number), 0));
	cdev_del(nt_cdev);
	kthread_stop(nt_devp->timer_task);
	kfree(nt_devp);
	class_destroy(nt_class);
	unregister_chrdev_region(nt_dev_number, 1);
}

module_init(noisy_timer_init);
module_exit(noisy_timer_exit);
MODULE_DESCRIPTION("cdev and timers");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("howdy doody");
