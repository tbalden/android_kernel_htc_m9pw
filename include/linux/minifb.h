#ifndef _MINI_FB_H_
#define _MINI_FB_H_

#include <uapi/linux/minifb.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef CONFIG_FB_MINIFB

#define MINIFB_NOREPEAT 0
#define MINIFB_REPEAT   1

int minifb_init(struct minifb_session *sess);
int minifb_terminate(struct minifb_session *sess);
int minifb_queuebuf(struct minifb_req *data);
int minifb_dequeuebuf(struct minifb_req *data);
int minifb_lockbuf(void ** vaddr, unsigned long *, int);
void minifb_unlockbuf(void);
int minifb_ioctl_handler(unsigned int cmd, void *argp);
#else
int minifb_init(void)
{
	return -ENODEV;
}

int minifb_terminate(void)
{
	return -ENODEV;
}

int minifb_queuebuf(struct minifb_req *data)
{
	return -ENODEV;
}

int minifb_dequeuebuf(struct minifb_req *data)
{
	return -ENODEV;
}

int minifb_lockbuf(void ** vaddr, unsigned long *)
{
	return -ENODEV;
}

void minifb_unlockbuf(void);
{

}

int minifb_ioctl_handler(unsigned int cmd, void *argp)
{
	return -ENODEV;
}

#endif
#endif 
