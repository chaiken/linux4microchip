
#include <linux/cdev.h>
#include <linux/fs.h>

int nt_open(struct inode *inode, struct file *file);
int nt_release(struct inode *inode, struct file *file);
ssize_t nt_read(struct file *file, char *buf, size_t count, loff_t *ppos);
ssize_t nt_write(struct file *file, const char *buf, size_t count,
		 loff_t *ppos);
