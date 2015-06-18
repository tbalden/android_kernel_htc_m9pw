/*
 *  Copyright (C) 2012-2013 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>

#include "exfat_version.h"
#include "exfat_config.h"
#include "exfat_data.h"
#include "exfat_oal.h"

#include "exfat_nls.h"
#include "exfat_api.h"
#include "exfat_super.h"
#include "exfat_core.h"



extern struct semaphore z_sem;





int FsInit(void)
{
	return ffsInit();
}

int FsShutdown(void)
{
	return ffsShutdown();
}


int FsMountVol(struct super_block *sb)
{
	int err;

	sm_P(&z_sem);

	err = buf_init(sb);
	if (!err)
		err = ffsMountVol(sb);
	else
		buf_shutdown(sb);

	sm_V(&z_sem);

	return err;
} 

int FsUmountVol(struct super_block *sb)
{
	int err;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	sm_P(&z_sem);

	
	sm_P(&p_fs->v_sem);

	err = ffsUmountVol(sb);
	buf_shutdown(sb);

	
	sm_V(&p_fs->v_sem);

	sm_V(&z_sem);

	return err;
} 

int FsGetVolInfo(struct super_block *sb, VOL_INFO_T *info)
{
	int err;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (info == NULL)
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsGetVolInfo(sb, info);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsSyncVol(struct super_block *sb, int do_sync)
{
	int err;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	err = ffsSyncVol(sb, do_sync);

	
	sm_V(&p_fs->v_sem);

	return err;
} 



int FsLookupFile(struct inode *inode, char *path, FILE_ID_T *fid)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if ((fid == NULL) || (path == NULL) || (*path == '\0'))
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsLookupFile(inode, path, fid);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsCreateFile(struct inode *inode, char *path, u8 mode, FILE_ID_T *fid)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if ((fid == NULL) || (path == NULL) || (*path == '\0'))
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsCreateFile(inode, path, mode, fid);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsReadFile(struct inode *inode, FILE_ID_T *fid, void *buffer, u64 count, u64 *rcount)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (fid == NULL)
		return FFS_INVALIDFID;

	
	if (buffer == NULL)
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsReadFile(inode, fid, buffer, count, rcount);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsWriteFile(struct inode *inode, FILE_ID_T *fid, void *buffer, u64 count, u64 *wcount)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (fid == NULL)
		return FFS_INVALIDFID;

	
	if (buffer == NULL)
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsWriteFile(inode, fid, buffer, count, wcount);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsTruncateFile(struct inode *inode, u64 old_size, u64 new_size)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	DPRINTK("FsTruncateFile entered (inode %p size %llu)\n", inode, new_size);

	err = ffsTruncateFile(inode, old_size, new_size);

	DPRINTK("FsTruncateFile exitted (%d)\n", err);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsMoveFile(struct inode *old_parent_inode, FILE_ID_T *fid, struct inode *new_parent_inode, struct dentry *new_dentry)
{
	int err;
	struct super_block *sb = old_parent_inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (fid == NULL)
		return FFS_INVALIDFID;

	
	sm_P(&p_fs->v_sem);

	err = ffsMoveFile(old_parent_inode, fid, new_parent_inode, new_dentry);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsRemoveFile(struct inode *inode, FILE_ID_T *fid)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (fid == NULL)
		return FFS_INVALIDFID;

	
	sm_P(&p_fs->v_sem);

	err = ffsRemoveFile(inode, fid);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsSetAttr(struct inode *inode, u32 attr)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	err = ffsSetAttr(inode, attr);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsReadStat(struct inode *inode, DIR_ENTRY_T *info)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	err = ffsGetStat(inode, info);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsWriteStat(struct inode *inode, DIR_ENTRY_T *info)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	DPRINTK("FsWriteStat entered (inode %p info %p\n", inode, info);

	err = ffsSetStat(inode, info);

	
	sm_V(&p_fs->v_sem);

	DPRINTK("FsWriteStat exited (%d)\n", err);

	return err;
} 

int FsMapCluster(struct inode *inode, s32 clu_offset, u32 *clu)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (clu == NULL)
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsMapCluster(inode, clu_offset, clu);

	
	sm_V(&p_fs->v_sem);

	return err;
} 


int FsCreateDir(struct inode *inode, char *path, FILE_ID_T *fid)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if ((fid == NULL) || (path == NULL) || (*path == '\0'))
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsCreateDir(inode, path, fid);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsReadDir(struct inode *inode, DIR_ENTRY_T *dir_entry)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (dir_entry == NULL)
		return FFS_ERROR;

	
	sm_P(&p_fs->v_sem);

	err = ffsReadDir(inode, dir_entry);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

int FsRemoveDir(struct inode *inode, FILE_ID_T *fid)
{
	int err;
	struct super_block *sb = inode->i_sb;
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	if (fid == NULL)
		return FFS_INVALIDFID;

	
	sm_P(&p_fs->v_sem);

	err = ffsRemoveDir(inode, fid);

	
	sm_V(&p_fs->v_sem);

	return err;
} 

EXPORT_SYMBOL(FsMountVol);
EXPORT_SYMBOL(FsUmountVol);
EXPORT_SYMBOL(FsGetVolInfo);
EXPORT_SYMBOL(FsSyncVol);
EXPORT_SYMBOL(FsLookupFile);
EXPORT_SYMBOL(FsCreateFile);
EXPORT_SYMBOL(FsReadFile);
EXPORT_SYMBOL(FsWriteFile);
EXPORT_SYMBOL(FsTruncateFile);
EXPORT_SYMBOL(FsMoveFile);
EXPORT_SYMBOL(FsRemoveFile);
EXPORT_SYMBOL(FsSetAttr);
EXPORT_SYMBOL(FsReadStat);
EXPORT_SYMBOL(FsWriteStat);
EXPORT_SYMBOL(FsMapCluster);
EXPORT_SYMBOL(FsCreateDir);
EXPORT_SYMBOL(FsReadDir);
EXPORT_SYMBOL(FsRemoveDir);

#ifdef CONFIG_EXFAT_KERNEL_DEBUG
int FsReleaseCache(struct super_block *sb)
{
	FS_INFO_T *p_fs = &(EXFAT_SB(sb)->fs_info);

	
	sm_P(&p_fs->v_sem);

	FAT_release_all(sb);
	buf_release_all(sb);

	
	sm_V(&p_fs->v_sem);

	return 0;
}

EXPORT_SYMBOL(FsReleaseCache);
#endif 

