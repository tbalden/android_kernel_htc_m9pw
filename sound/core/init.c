/*
 *  Initialization routines
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/ctype.h>
#include <linux/pm.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/info.h>

struct snd_monitor_file {
	struct file *file;
	const struct file_operations *disconnected_f_op;
	struct list_head shutdown_list;	
	struct list_head list;	
};

static DEFINE_SPINLOCK(shutdown_lock);
static LIST_HEAD(shutdown_files);

static const struct file_operations snd_shutdown_f_ops;

static unsigned int snd_cards_lock;	
struct snd_card *snd_cards[SNDRV_CARDS];
EXPORT_SYMBOL(snd_cards);

static DEFINE_MUTEX(snd_card_mutex);

static char *slots[SNDRV_CARDS];
module_param_array(slots, charp, NULL, 0444);
MODULE_PARM_DESC(slots, "Module names assigned to the slots.");

static int module_slot_match(struct module *module, int idx)
{
	int match = 1;
#ifdef MODULE
	const char *s1, *s2;

	if (!module || !module->name || !slots[idx])
		return 0;

	s1 = module->name;
	s2 = slots[idx];
	if (*s2 == '!') {
		match = 0; 
		s2++;
	}
	for (;;) {
		char c1 = *s1++;
		char c2 = *s2++;
		if (c1 == '-')
			c1 = '_';
		if (c2 == '-')
			c2 = '_';
		if (c1 != c2)
			return !match;
		if (!c1)
			break;
	}
#endif 
	return match;
}

#if defined(CONFIG_SND_MIXER_OSS) || defined(CONFIG_SND_MIXER_OSS_MODULE)
int (*snd_mixer_oss_notify_callback)(struct snd_card *card, int free_flag);
EXPORT_SYMBOL(snd_mixer_oss_notify_callback);
#endif

#ifdef CONFIG_PROC_FS
static void snd_card_id_read(struct snd_info_entry *entry,
			     struct snd_info_buffer *buffer)
{
	snd_iprintf(buffer, "%s\n", entry->card->id);
}

static inline int init_info_for_card(struct snd_card *card)
{
	int err;
	struct snd_info_entry *entry;

	if ((err = snd_info_card_register(card)) < 0) {
		snd_printd("unable to create card info\n");
		return err;
	}
	if ((entry = snd_info_create_card_entry(card, "id", card->proc_root)) == NULL) {
		snd_printd("unable to create card entry\n");
		return err;
	}
	entry->c.text.read = snd_card_id_read;
	if (snd_info_register(entry) < 0) {
		snd_info_free_entry(entry);
		entry = NULL;
	}
	card->proc_id = entry;
	return 0;
}
#else 
#define init_info_for_card(card)
#endif

int snd_card_create(int idx, const char *xid,
		    struct module *module, int extra_size,
		    struct snd_card **card_ret)
{
	struct snd_card *card;
	int err, idx2;

	if (snd_BUG_ON(!card_ret))
		return -EINVAL;
	*card_ret = NULL;

	if (extra_size < 0)
		extra_size = 0;
	card = kzalloc(sizeof(*card) + extra_size, GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	if (xid)
		strlcpy(card->id, xid, sizeof(card->id));
	err = 0;
	mutex_lock(&snd_card_mutex);
	if (idx < 0) {
		for (idx2 = 0; idx2 < SNDRV_CARDS; idx2++)
			
			if (~snd_cards_lock & idx & 1<<idx2) {
				if (module_slot_match(module, idx2)) {
					idx = idx2;
					break;
				}
			}
	}
	if (idx < 0) {
		for (idx2 = 0; idx2 < SNDRV_CARDS; idx2++)
			
			if (~snd_cards_lock & idx & 1<<idx2) {
				if (!slots[idx2] || !*slots[idx2]) {
					idx = idx2;
					break;
				}
			}
	}
	if (idx < 0)
		err = -ENODEV;
	else if (idx < snd_ecards_limit) {
		if (snd_cards_lock & (1 << idx))
			err = -EBUSY;	
	} else if (idx >= SNDRV_CARDS)
		err = -ENODEV;
	if (err < 0) {
		mutex_unlock(&snd_card_mutex);
		snd_printk(KERN_ERR "cannot find the slot for index %d (range 0-%i), error: %d\n",
			 idx, snd_ecards_limit - 1, err);
		goto __error;
	}
	snd_cards_lock |= 1 << idx;		
	if (idx >= snd_ecards_limit)
		snd_ecards_limit = idx + 1; 
	mutex_unlock(&snd_card_mutex);
	card->number = idx;
	card->module = module;
	INIT_LIST_HEAD(&card->devices);
	init_rwsem(&card->controls_rwsem);
	rwlock_init(&card->ctl_files_rwlock);
	mutex_init(&card->user_ctl_lock);
	INIT_LIST_HEAD(&card->controls);
	INIT_LIST_HEAD(&card->ctl_files);
	spin_lock_init(&card->files_lock);
	INIT_LIST_HEAD(&card->files_list);
	init_waitqueue_head(&card->shutdown_sleep);
	atomic_set(&card->refcount, 0);
#ifdef CONFIG_PM
	mutex_init(&card->power_lock);
	init_waitqueue_head(&card->power_sleep);
#endif
	
	
	err = snd_ctl_create(card);
	if (err < 0) {
		snd_printk(KERN_ERR "unable to register control minors\n");
		goto __error;
	}
	err = snd_info_card_create(card);
	if (err < 0) {
		snd_printk(KERN_ERR "unable to create card info\n");
		goto __error_ctl;
	}
	if (extra_size > 0)
		card->private_data = (char *)card + sizeof(struct snd_card);
	*card_ret = card;
	return 0;

      __error_ctl:
	snd_device_free_all(card, SNDRV_DEV_CMD_PRE);
      __error:
	kfree(card);
  	return err;
}
EXPORT_SYMBOL(snd_card_create);

int snd_card_locked(int card)
{
	int locked;

	mutex_lock(&snd_card_mutex);
	locked = snd_cards_lock & (1 << card);
	mutex_unlock(&snd_card_mutex);
	return locked;
}

static loff_t snd_disconnect_llseek(struct file *file, loff_t offset, int orig)
{
	return -ENODEV;
}

static ssize_t snd_disconnect_read(struct file *file, char __user *buf,
				   size_t count, loff_t *offset)
{
	return -ENODEV;
}

static ssize_t snd_disconnect_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *offset)
{
	return -ENODEV;
}

static int snd_disconnect_release(struct inode *inode, struct file *file)
{
	struct snd_monitor_file *df = NULL, *_df;

	spin_lock(&shutdown_lock);
	list_for_each_entry(_df, &shutdown_files, shutdown_list) {
		if (_df->file == file) {
			df = _df;
			list_del_init(&df->shutdown_list);
			break;
		}
	}
	spin_unlock(&shutdown_lock);

	if (likely(df)) {
		if ((file->f_flags & FASYNC) && df->disconnected_f_op->fasync)
			df->disconnected_f_op->fasync(-1, file, 0);
		return df->disconnected_f_op->release(inode, file);
	}

	panic("%s(%p, %p) failed!", __func__, inode, file);
}

static unsigned int snd_disconnect_poll(struct file * file, poll_table * wait)
{
	return POLLERR | POLLNVAL;
}

static long snd_disconnect_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	return -ENODEV;
}

static int snd_disconnect_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -ENODEV;
}

static int snd_disconnect_fasync(int fd, struct file *file, int on)
{
	return -ENODEV;
}

static const struct file_operations snd_shutdown_f_ops =
{
	.owner = 	THIS_MODULE,
	.llseek =	snd_disconnect_llseek,
	.read = 	snd_disconnect_read,
	.write =	snd_disconnect_write,
	.release =	snd_disconnect_release,
	.poll =		snd_disconnect_poll,
	.unlocked_ioctl = snd_disconnect_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = snd_disconnect_ioctl,
#endif
	.mmap =		snd_disconnect_mmap,
	.fasync =	snd_disconnect_fasync
};

int snd_card_disconnect(struct snd_card *card)
{
	struct snd_monitor_file *mfile;
	int err;

	if (!card)
		return -EINVAL;

	spin_lock(&card->files_lock);
	if (card->shutdown) {
		spin_unlock(&card->files_lock);
		return 0;
	}
	card->shutdown = 1;
	spin_unlock(&card->files_lock);

	
	mutex_lock(&snd_card_mutex);
	snd_cards[card->number] = NULL;
	snd_cards_lock &= ~(1 << card->number);
	mutex_unlock(&snd_card_mutex);
	
	
	
	spin_lock(&card->files_lock);
	list_for_each_entry(mfile, &card->files_list, list) {
		
		
		mfile->disconnected_f_op = mfile->file->f_op;

		spin_lock(&shutdown_lock);
		list_add(&mfile->shutdown_list, &shutdown_files);
		spin_unlock(&shutdown_lock);

		mfile->file->f_op = &snd_shutdown_f_ops;
		fops_get(mfile->file->f_op);
	}
	spin_unlock(&card->files_lock);	

	
	

#if defined(CONFIG_SND_MIXER_OSS) || defined(CONFIG_SND_MIXER_OSS_MODULE)
	if (snd_mixer_oss_notify_callback)
		snd_mixer_oss_notify_callback(card, SND_MIXER_OSS_NOTIFY_DISCONNECT);
#endif

	
	err = snd_device_disconnect_all(card);
	if (err < 0)
		snd_printk(KERN_ERR "not all devices for card %i can be disconnected\n", card->number);

	snd_info_card_disconnect(card);
	if (card->card_dev) {
		device_unregister(card->card_dev);
		card->card_dev = NULL;
	}
#ifdef CONFIG_PM
	wake_up(&card->power_sleep);
#endif
	return 0;	
}

EXPORT_SYMBOL(snd_card_disconnect);

static int snd_card_do_free(struct snd_card *card)
{
#if defined(CONFIG_SND_MIXER_OSS) || defined(CONFIG_SND_MIXER_OSS_MODULE)
	if (snd_mixer_oss_notify_callback)
		snd_mixer_oss_notify_callback(card, SND_MIXER_OSS_NOTIFY_FREE);
#endif
	if (snd_device_free_all(card, SNDRV_DEV_CMD_PRE) < 0) {
		snd_printk(KERN_ERR "unable to free all devices (pre)\n");
		
	}
	if (snd_device_free_all(card, SNDRV_DEV_CMD_NORMAL) < 0) {
		snd_printk(KERN_ERR "unable to free all devices (normal)\n");
		
	}
	if (snd_device_free_all(card, SNDRV_DEV_CMD_POST) < 0) {
		snd_printk(KERN_ERR "unable to free all devices (post)\n");
		
	}
	if (card->private_free)
		card->private_free(card);
	snd_info_free_entry(card->proc_id);
	if (snd_info_card_free(card) < 0) {
		snd_printk(KERN_WARNING "unable to free card info\n");
		
	}
	kfree(card);
	return 0;
}

void snd_card_unref(struct snd_card *card)
{
	if (atomic_dec_and_test(&card->refcount)) {
		wake_up(&card->shutdown_sleep);
		if (card->free_on_last_close)
			snd_card_do_free(card);
	}
}
EXPORT_SYMBOL(snd_card_unref);

int snd_card_free_when_closed(struct snd_card *card)
{
	int ret;

	atomic_inc(&card->refcount);
	ret = snd_card_disconnect(card);
	if (ret) {
		atomic_dec(&card->refcount);
		return ret;
	}

	card->free_on_last_close = 1;
	if (atomic_dec_and_test(&card->refcount))
		snd_card_do_free(card);
	return 0;
}

EXPORT_SYMBOL(snd_card_free_when_closed);

int snd_card_free(struct snd_card *card)
{
	int ret = snd_card_disconnect(card);
	if (ret)
		return ret;

	
	wait_event(card->shutdown_sleep, !atomic_read(&card->refcount));
	snd_card_do_free(card);
	return 0;
}

EXPORT_SYMBOL(snd_card_free);

static const char *retrieve_id_from_card_name(const char *name)
{
	const char *spos = name;

	while (*name) {
		if (isspace(*name) && isalnum(name[1]))
			spos = name + 1;
		name++;
	}
	return spos;
}

static bool card_id_ok(struct snd_card *card, const char *id)
{
	int i;
	if (!snd_info_check_reserved_words(id))
		return false;
	for (i = 0; i < snd_ecards_limit; i++) {
		if (snd_cards[i] && snd_cards[i] != card &&
		    !strcmp(snd_cards[i]->id, id))
			return false;
	}
	return true;
}

static void copy_valid_id_string(struct snd_card *card, const char *src,
				 const char *nid)
{
	char *id = card->id;

	while (*nid && !isalnum(*nid))
		nid++;
	if (isdigit(*nid))
		*id++ = isalpha(*src) ? *src : 'D';
	while (*nid && (size_t)(id - card->id) < sizeof(card->id) - 1) {
		if (isalnum(*nid))
			*id++ = *nid;
		nid++;
	}
	*id = 0;
}

static void snd_card_set_id_no_lock(struct snd_card *card, const char *src,
				    const char *nid)
{
	int len, loops;
	bool with_suffix;
	bool is_default = false;
	char *id;
	
	copy_valid_id_string(card, src, nid);
	id = card->id;

 again:
	if (!*id || !strncmp(id, "card", 4)) {
		strcpy(id, "Default");
		is_default = true;
	}

	with_suffix = false;
	for (loops = 0; loops < SNDRV_CARDS; loops++) {
		if (card_id_ok(card, id))
			return; 

		len = strlen(id);
		if (!with_suffix) {
			
			char *spos = id + len;
			if (len >  sizeof(card->id) - 3)
				spos = id + sizeof(card->id) - 3;
			strcpy(spos, "_1");
			with_suffix = true;
		} else {
			
			if (id[len - 1] != '9')
				id[len - 1]++;
			else
				id[len - 1] = 'A';
		}
	}
	
	if (!is_default) {
		*id = 0;
		goto again;
	}
	
	snd_printk(KERN_ERR "unable to set card id (%s)\n", id);
	if (card->proc_root->name)
#if 1
		
		strncpy(card->id, card->proc_root->name, sizeof(card->id));
#else
		strcpy(card->id, card->proc_root->name);
#endif
}

void snd_card_set_id(struct snd_card *card, const char *nid)
{
	
	if (card->id[0] != '\0')
		return;
	mutex_lock(&snd_card_mutex);
	snd_card_set_id_no_lock(card, nid, nid);
	mutex_unlock(&snd_card_mutex);
}
EXPORT_SYMBOL(snd_card_set_id);

static ssize_t
card_id_show_attr(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	struct snd_card *card = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", card ? card->id : "(null)");
}

static ssize_t
card_id_store_attr(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t count)
{
	struct snd_card *card = dev_get_drvdata(dev);
	char buf1[sizeof(card->id)];
	size_t copy = count > sizeof(card->id) - 1 ?
					sizeof(card->id) - 1 : count;
	size_t idx;
	int c;

	for (idx = 0; idx < copy; idx++) {
		c = buf[idx];
		if (!isalnum(c) && c != '_' && c != '-')
			return -EINVAL;
	}
	memcpy(buf1, buf, copy);
	buf1[copy] = '\0';
	mutex_lock(&snd_card_mutex);
	if (!card_id_ok(NULL, buf1)) {
		mutex_unlock(&snd_card_mutex);
		return -EEXIST;
	}
	strcpy(card->id, buf1);
	snd_info_card_id_change(card);
	mutex_unlock(&snd_card_mutex);

	return count;
}

static struct device_attribute card_id_attrs =
	__ATTR(id, S_IRUGO | S_IWUSR, card_id_show_attr, card_id_store_attr);

static ssize_t
card_number_show_attr(struct device *dev,
		     struct device_attribute *attr, char *buf)
{
	struct snd_card *card = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%i\n", card ? card->number : -1);
}

static struct device_attribute card_number_attrs =
	__ATTR(number, S_IRUGO, card_number_show_attr, NULL);

int snd_card_register(struct snd_card *card)
{
	int err;

	if (snd_BUG_ON(!card))
		return -EINVAL;

	if (!card->card_dev) {
		card->card_dev = device_create(sound_class, card->dev,
					       MKDEV(0, 0), card,
					       "card%i", card->number);
		if (IS_ERR(card->card_dev))
			card->card_dev = NULL;
	}

	if ((err = snd_device_register_all(card)) < 0)
		return err;
	mutex_lock(&snd_card_mutex);
	if (snd_cards[card->number]) {
		
		mutex_unlock(&snd_card_mutex);
		return 0;
	}
	if (*card->id) {
		
		char tmpid[sizeof(card->id)];
		memcpy(tmpid, card->id, sizeof(card->id));
		snd_card_set_id_no_lock(card, tmpid, tmpid);
	} else {
		
		const char *src;
		src = *card->shortname ? card->shortname : card->longname;
		snd_card_set_id_no_lock(card, src,
					retrieve_id_from_card_name(src));
	}
	snd_cards[card->number] = card;
	mutex_unlock(&snd_card_mutex);
	init_info_for_card(card);
#if defined(CONFIG_SND_MIXER_OSS) || defined(CONFIG_SND_MIXER_OSS_MODULE)
	if (snd_mixer_oss_notify_callback)
		snd_mixer_oss_notify_callback(card, SND_MIXER_OSS_NOTIFY_REGISTER);
#endif
	if (card->card_dev) {
		err = device_create_file(card->card_dev, &card_id_attrs);
		if (err < 0)
			return err;
		err = device_create_file(card->card_dev, &card_number_attrs);
		if (err < 0)
			return err;
	}

	return 0;
}

EXPORT_SYMBOL(snd_card_register);

#ifdef CONFIG_PROC_FS
static struct snd_info_entry *snd_card_info_entry;

static void snd_card_info_read(struct snd_info_entry *entry,
			       struct snd_info_buffer *buffer)
{
	int idx, count;
	struct snd_card *card;

	for (idx = count = 0; idx < SNDRV_CARDS; idx++) {
		mutex_lock(&snd_card_mutex);
		if ((card = snd_cards[idx]) != NULL) {
			count++;
			snd_iprintf(buffer, "%2i [%-15s]: %s - %s\n",
					idx,
					card->id,
					card->driver,
					card->shortname);
			snd_iprintf(buffer, "                      %s\n",
					card->longname);
		}
		mutex_unlock(&snd_card_mutex);
	}
	if (!count)
		snd_iprintf(buffer, "--- no soundcards ---\n");
}

#ifdef CONFIG_SND_OSSEMUL

void snd_card_info_read_oss(struct snd_info_buffer *buffer)
{
	int idx, count;
	struct snd_card *card;

	for (idx = count = 0; idx < SNDRV_CARDS; idx++) {
		mutex_lock(&snd_card_mutex);
		if ((card = snd_cards[idx]) != NULL) {
			count++;
			snd_iprintf(buffer, "%s\n", card->longname);
		}
		mutex_unlock(&snd_card_mutex);
	}
	if (!count) {
		snd_iprintf(buffer, "--- no soundcards ---\n");
	}
}

#endif

#ifdef MODULE
static struct snd_info_entry *snd_card_module_info_entry;
static void snd_card_module_info_read(struct snd_info_entry *entry,
				      struct snd_info_buffer *buffer)
{
	int idx;
	struct snd_card *card;

	for (idx = 0; idx < SNDRV_CARDS; idx++) {
		mutex_lock(&snd_card_mutex);
		if ((card = snd_cards[idx]) != NULL)
			snd_iprintf(buffer, "%2i %s\n",
				    idx, card->module->name);
		mutex_unlock(&snd_card_mutex);
	}
}
#endif

int __init snd_card_info_init(void)
{
	struct snd_info_entry *entry;

	entry = snd_info_create_module_entry(THIS_MODULE, "cards", NULL);
	if (! entry)
		return -ENOMEM;
	entry->c.text.read = snd_card_info_read;
	if (snd_info_register(entry) < 0) {
		snd_info_free_entry(entry);
		return -ENOMEM;
	}
	snd_card_info_entry = entry;

#ifdef MODULE
	entry = snd_info_create_module_entry(THIS_MODULE, "modules", NULL);
	if (entry) {
		entry->c.text.read = snd_card_module_info_read;
		if (snd_info_register(entry) < 0)
			snd_info_free_entry(entry);
		else
			snd_card_module_info_entry = entry;
	}
#endif

	return 0;
}

int __exit snd_card_info_done(void)
{
	snd_info_free_entry(snd_card_info_entry);
#ifdef MODULE
	snd_info_free_entry(snd_card_module_info_entry);
#endif
	return 0;
}

#endif 

  
int snd_component_add(struct snd_card *card, const char *component)
{
	char *ptr;
	int len = strlen(component);

	ptr = strstr(card->components, component);
	if (ptr != NULL) {
		if (ptr[len] == '\0' || ptr[len] == ' ')	
			return 1;
	}
	if (strlen(card->components) + 1 + len + 1 > sizeof(card->components)) {
		snd_BUG();
		return -ENOMEM;
	}
	if (card->components[0] != '\0')
		strcat(card->components, " ");
	strcat(card->components, component);
	return 0;
}

EXPORT_SYMBOL(snd_component_add);

int snd_card_file_add(struct snd_card *card, struct file *file)
{
	struct snd_monitor_file *mfile;

	mfile = kmalloc(sizeof(*mfile), GFP_KERNEL);
	if (mfile == NULL)
		return -ENOMEM;
	mfile->file = file;
	mfile->disconnected_f_op = NULL;
	INIT_LIST_HEAD(&mfile->shutdown_list);
	spin_lock(&card->files_lock);
	if (card->shutdown) {
		spin_unlock(&card->files_lock);
		kfree(mfile);
		return -ENODEV;
	}
	list_add(&mfile->list, &card->files_list);
	atomic_inc(&card->refcount);
	spin_unlock(&card->files_lock);
	return 0;
}

EXPORT_SYMBOL(snd_card_file_add);

int snd_card_file_remove(struct snd_card *card, struct file *file)
{
	struct snd_monitor_file *mfile, *found = NULL;

	spin_lock(&card->files_lock);
	list_for_each_entry(mfile, &card->files_list, list) {
		if (mfile->file == file) {
			list_del(&mfile->list);
			spin_lock(&shutdown_lock);
			list_del(&mfile->shutdown_list);
			spin_unlock(&shutdown_lock);
			if (mfile->disconnected_f_op)
				fops_put(mfile->disconnected_f_op);
			found = mfile;
			break;
		}
	}
	spin_unlock(&card->files_lock);
	if (!found) {
		snd_printk(KERN_ERR "ALSA card file remove problem (%p)\n", file);
		return -ENOENT;
	}
	kfree(found);
	snd_card_unref(card);
	return 0;
}

EXPORT_SYMBOL(snd_card_file_remove);

#ifdef CONFIG_PM
int snd_power_wait(struct snd_card *card, unsigned int power_state)
{
	wait_queue_t wait;
	int result = 0;

	
	if (snd_power_get_state(card) == power_state)
		return 0;
	init_waitqueue_entry(&wait, current);
	add_wait_queue(&card->power_sleep, &wait);
	while (1) {
		if (card->shutdown) {
			result = -ENODEV;
			break;
		}
		if (snd_power_get_state(card) == power_state)
			break;
		set_current_state(TASK_UNINTERRUPTIBLE);
		snd_power_unlock(card);
		schedule_timeout(30 * HZ);
		snd_power_lock(card);
	}
	remove_wait_queue(&card->power_sleep, &wait);
	return result;
}

EXPORT_SYMBOL(snd_power_wait);
#endif 
