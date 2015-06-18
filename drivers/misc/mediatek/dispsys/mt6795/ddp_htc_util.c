#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/uaccess.h>
#include <primary_display.h>

#include "ddp_htc_util.h"

#define DEBUG_BUF 1024
#define MIN_COUNT 9

static ssize_t dsi_cmd_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[DEBUG_BUF];
	struct LCM_setting_table debug_cmd;
	u32 type, value;
	int cnt, i;
	char *tmp;

	if (count >= sizeof(debug_buf) || count < MIN_COUNT)
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	
	debug_buf[count] = 0;

	
	cnt = (count) / 3 - 2;

	
	sscanf(debug_buf, "%x", &type);

	sscanf(debug_buf+3, "%x", &debug_cmd.cmd);
	debug_cmd.count = cnt;

	
	for (i = 0; i < cnt; i++) {
		if (i >= DCS_MAX_CNT) {
			printk("%s: DCS command count over DCS_MAX_CNT, Skip these commands.\n", __func__);
			break;
		}
		tmp = debug_buf + (3 * (i + 2));
		sscanf(tmp, "%x", &value);
		debug_cmd.para_list[i] = value;
	}

	printk("TESTTT:%s:%d: debug cmd 0x%x, count %d\n", __FUNCTION__, __LINE__, debug_cmd.cmd, debug_cmd.count);
	for(i = 0; i < debug_cmd.count; i++)
		printk("TESTTT:%s:%d:  cmd %d = 0x%x\n", __FUNCTION__, __LINE__, i, debug_cmd.para_list[i]);

	primary_display_setlcm_cmd(&debug_cmd.cmd, &debug_cmd.count, debug_cmd.para_list);

	return count;
}

static const struct file_operations dsi_cmd_fops = {
	.write = dsi_cmd_write,
};

void htc_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("htc_debug", NULL);

	if (IS_ERR(dent)) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return;
	}

	if (debugfs_create_file("dsi_cmd", 0644, dent, 0, &dsi_cmd_fops) == NULL) {
		pr_err(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return;
	}

	return;
}
