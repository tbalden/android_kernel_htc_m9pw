/*
 * Author: Pal Zoltan Illes @tbalden <illespal@gmail.com>
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

#ifndef _LINUX_FINGERPRINT_MOD_H
#define _LINUX_FINGERPRINT_MOD_H

extern int fingerprint_mod_enabled;
extern int fingerprint_mod_opt_doubletap_mode;
extern int fingerprint_mod_opt_longtap_mode;
extern int fingerprint_mod_opt_extralong_mode;
extern int fingerprint_mod_opt_timeout;
extern int fingerprint_mod_opt_doubletaplong_mode;

#define FINGERPRINT_MOD_DISABLED 0	/* default */
#define FINGERPRINT_MOD_ENABLED 1


#define FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_DISABLED 0	/* default */
#define FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_KEY 1
#define FINGERPRINT_MOD_OPT_DOUBLETAP_MODE_SLEEP 2

#define FINGERPRINT_MOD_OPT_LONGTAP_MODE_DISABLED 0	/* default */
#define FINGERPRINT_MOD_OPT_LONGTAP_MODE_KEY 1
#define FINGERPRINT_MOD_OPT_LONGTAP_MODE_SLEEP 2

#define FINGERPRINT_MOD_OPT_TIMEOUT_MIN 0
#define FINGERPRINT_MOD_OPT_TIMEOUT_MAX 10	/* default */

#endif
