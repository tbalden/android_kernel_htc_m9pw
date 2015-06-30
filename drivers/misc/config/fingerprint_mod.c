/*
 * Author: Illes Pal Zoltan @tabledn <illespal@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Possible values for "fingerprint_mod_enabled" are :
 *
 *   0 - disabled (default)
 *   1 - enabled
*/

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/config/fingerprint_mod.h>

int fingerprint_mod_enabled = 0;
int fingerprint_mod_opt_doubletap_mode = 0;
int fingerprint_mod_opt_longtap_mode = 0;
int fingerprint_mod_opt_timeout = 0; // 0 - 10 -> 25 + x * 2 jiffies
int fingerprint_mod_opt_extralong_mode = 0;
int fingerprint_mod_opt_doubletaplong_mode = 0;

// --------------------------------------------------------------------------------------------------------------------------

/* sysfs interface for "fingerprint_mod_enabled" */
static ssize_t fingerprint_mod_enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_enabled);
}

static ssize_t fingerprint_mod_enabled_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_enabled;

	sscanf(buf, "%du", &new_fingerprint_mod_enabled);

	if (new_fingerprint_mod_enabled >= FINGERPRINT_MOD_DISABLED && new_fingerprint_mod_enabled <= FINGERPRINT_MOD_ENABLED) {
		/* update only if valid value provided */
		fingerprint_mod_enabled = new_fingerprint_mod_enabled;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_enabled_attribute =
__ATTR(fingerprint_mod_enabled, 0666, fingerprint_mod_enabled_show, fingerprint_mod_enabled_store);

// --------------------------------------------------------------------------------------------------------------------------

/* sysfs interface for "fingerprint_mod_opt_doubletap_mode" */
static ssize_t fingerprint_mod_opt_doubletap_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_opt_doubletap_mode);
}

static ssize_t fingerprint_mod_opt_doubletap_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_opt_doubletap_mode;

	sscanf(buf, "%du", &new_fingerprint_mod_opt_doubletap_mode);

	if (new_fingerprint_mod_opt_doubletap_mode >= FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_DISABLED && new_fingerprint_mod_opt_doubletap_mode <= FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_SLEEP) {
		/* update only if valid value provided */
		fingerprint_mod_opt_doubletap_mode = new_fingerprint_mod_opt_doubletap_mode;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_opt_doubletap_mode_attribute =
__ATTR(fingerprint_mod_opt_doubletap_mode, 0666, fingerprint_mod_opt_doubletap_mode_show, fingerprint_mod_opt_doubletap_mode_store);

// --------------------------------------------------------------------------------------------------------------------------

/* sysfs interface for "fingerprint_opt_longtap_mode" */
static ssize_t fingerprint_mod_opt_longtap_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_opt_longtap_mode);
}

static ssize_t fingerprint_mod_opt_longtap_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_opt_longtap_mode;

	sscanf(buf, "%du", &new_fingerprint_mod_opt_longtap_mode);

	if (new_fingerprint_mod_opt_longtap_mode >= FINGERPRINT_MOD_OPT_LONGTAP_MODE_DISABLED && new_fingerprint_mod_opt_longtap_mode <= FINGERPRINT_MOD_OPT_LONGTAP_MODE_SLEEP) {
		/* update only if valid value provided */
		fingerprint_mod_opt_longtap_mode = new_fingerprint_mod_opt_longtap_mode;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_opt_longtap_mode_attribute =
__ATTR(fingerprint_mod_opt_longtap_mode, 0666, fingerprint_mod_opt_longtap_mode_show, fingerprint_mod_opt_longtap_mode_store);

// --------------------------------------------------------------------------------------------------------------------------


/* sysfs interface for "fingerprint_opt_extralong_mode" */
static ssize_t fingerprint_mod_opt_extralong_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_opt_extralong_mode);
}

static ssize_t fingerprint_mod_opt_extralong_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_opt_extralong_mode;

	sscanf(buf, "%du", &new_fingerprint_mod_opt_extralong_mode);

	if (new_fingerprint_mod_opt_extralong_mode >= FINGERPRINT_MOD_OPT_LONGTAP_MODE_DISABLED && new_fingerprint_mod_opt_extralong_mode <= FINGERPRINT_MOD_OPT_LONGTAP_MODE_SLEEP) {
		/* update only if valid value provided */
		fingerprint_mod_opt_extralong_mode = new_fingerprint_mod_opt_extralong_mode;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_opt_extralong_mode_attribute =
__ATTR(fingerprint_mod_opt_extralong_mode, 0666, fingerprint_mod_opt_extralong_mode_show, fingerprint_mod_opt_extralong_mode_store);

// --------------------------------------------------------------------------------------------------------------------------


/* sysfs interface for "fingerprint_opt_doubletaplong_mode" */
static ssize_t fingerprint_mod_opt_doubletaplong_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_opt_doubletaplong_mode);
}

static ssize_t fingerprint_mod_opt_doubletaplong_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_opt_doubletaplong_mode;

	sscanf(buf, "%du", &new_fingerprint_mod_opt_doubletaplong_mode);

	if (new_fingerprint_mod_opt_doubletaplong_mode >= FINGERPRINT_MOD_OPT_LONGTAP_MODE_DISABLED && new_fingerprint_mod_opt_doubletaplong_mode <= FINGERPRINT_MOD_OPT_LONGTAP_MODE_SLEEP) {
		/* update only if valid value provided */
		fingerprint_mod_opt_doubletaplong_mode = new_fingerprint_mod_opt_doubletaplong_mode;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_opt_doubletaplong_mode_attribute =
__ATTR(fingerprint_mod_opt_doubletaplong_mode, 0666, fingerprint_mod_opt_doubletaplong_mode_show, fingerprint_mod_opt_doubletaplong_mode_store);

// --------------------------------------------------------------------------------------------------------------------------

/* sysfs interface for "fingerprint_opt_timeout" */
static ssize_t fingerprint_mod_opt_timeout_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fingerprint_mod_opt_timeout);
}

static ssize_t fingerprint_mod_opt_timeout_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int new_fingerprint_mod_opt_timeout;

	sscanf(buf, "%du", &new_fingerprint_mod_opt_timeout);

	if (new_fingerprint_mod_opt_timeout >= FINGERPRINT_MOD_OPT_TIMEOUT_MIN && new_fingerprint_mod_opt_timeout <= FINGERPRINT_MOD_OPT_TIMEOUT_MAX) {
		/* update only if valid value provided */
		fingerprint_mod_opt_timeout = new_fingerprint_mod_opt_timeout;
	}

	return count;
}

static struct kobj_attribute fingerprint_mod_opt_timeout_attribute =
__ATTR(fingerprint_mod_opt_timeout, 0666, fingerprint_mod_opt_timeout_show, fingerprint_mod_opt_timeout_store);

// --------------------------------------------------------------------------------------------------------------------------


// gathering attributes
static struct attribute *fingerprint_mod_attrs[] = {
	&fingerprint_mod_enabled_attribute.attr,
	&fingerprint_mod_opt_doubletap_mode_attribute.attr,
	&fingerprint_mod_opt_longtap_mode_attribute.attr,
	&fingerprint_mod_opt_extralong_mode_attribute.attr,
	&fingerprint_mod_opt_doubletaplong_mode_attribute.attr,
	&fingerprint_mod_opt_timeout_attribute.attr,
	NULL,
};

// attr group
static struct attribute_group fingerprint_mod_attr_group = {
	.attrs = fingerprint_mod_attrs,
};

/* Initialize fast charge sysfs folder */
static struct kobject *fingerprint_mod_kobj;

int fingerprint_mod_init(void)
{
	int fingerprint_mod_retval;

	fingerprint_mod_enabled = 1;//FINGERPRINT_MOD_DISABLED; /* mmod disabled by default */
	fingerprint_mod_opt_doubletap_mode = 1; //FINGERPRINT_MOD_DISABLED; /* mmod disabled by default */
	fingerprint_mod_opt_longtap_mode = 2; //FINGERPRINT_MOD_DISABLED; /* mmod disabled by default */
	fingerprint_mod_opt_extralong_mode = 1; //FINGERPRINT_MOD_DISABLED; /* mmod disabled by default */
	fingerprint_mod_opt_doubletaplong_mode = 0; //FINGERPRINT_MOD_DISABLED; /* mmod disabled by default */
	fingerprint_mod_opt_timeout = FINGERPRINT_MOD_OPT_TIMEOUT_MAX; /* mmod disabled by default */

	fingerprint_mod_kobj = kobject_create_and_add("fingerprint_mod", kernel_kobj);
	if (!fingerprint_mod_kobj) {
			return -ENOMEM;
	}

	fingerprint_mod_retval = sysfs_create_group(fingerprint_mod_kobj, &fingerprint_mod_attr_group);

	if (fingerprint_mod_retval)
		kobject_put(fingerprint_mod_kobj);

	if (fingerprint_mod_retval)
		kobject_put(fingerprint_mod_kobj);

	return (fingerprint_mod_retval);
}

void fingerprint_mod_exit(void)
{
	kobject_put(fingerprint_mod_kobj);
}

module_init(fingerprint_mod_init);
module_exit(fingerprint_mod_exit);

