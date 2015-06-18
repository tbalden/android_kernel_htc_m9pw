 /*!
@File
@Title          Functions for creating debugfs directories and entries.
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ 

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "pvr_debug.h"
#include "pvr_debugfs.h"

#define PVR_DEBUGFS_DIR_NAME "pvr"

static struct dentry *gpsPVRDebugFSEntryDir = NULL;

static struct mutex gDebugFSLock;

 

typedef struct _PVR_DEBUGFS_DRIVER_STAT_
{
	
	void				*pvData;
	PVRSRV_GET_NEXT_STAT_FUNC	*pfnGetNextStat;
	PVRSRV_INC_STAT_MEM_REFCOUNT_FUNC	*pfnIncStatMemRefCount;
	PVRSRV_DEC_STAT_MEM_REFCOUNT_FUNC	*pfnDecStatMemRefCount;
	IMG_UINT32			ui32RefCount;
	IMG_INT32			i32StatValue;
	IMG_CHAR			*pszStatFormat;
	void					*pvDebugFSEntry;
} PVR_DEBUGFS_DRIVER_STAT;
typedef struct _PVR_DEBUGFS_PRIV_DATA_
{
	struct seq_operations	*psReadOps;
	PVRSRV_ENTRY_WRITE_FUNC	*pfnWrite;
	void			*pvData;
	IMG_BOOL		bValid;
} PVR_DEBUGFS_PRIV_DATA;
typedef struct _PVR_DEBUGFS_DIR_DATA_ PVR_DEBUGFS_DIR_DATA;
typedef struct _PVR_DEBUGFS_DIR_DATA_
{
	struct dentry *psDir;
	PVR_DEBUGFS_DIR_DATA *psParentDir;
	IMG_UINT32	ui32RefCount;
} PVR_DEBUGFS_DIR_DATA;
typedef struct _PVR_DEBUGFS_ENTRY_DATA_
{
	struct dentry *psEntry;
	PVR_DEBUGFS_DIR_DATA *psParentDir;
	IMG_UINT32	ui32RefCount;
	PVR_DEBUGFS_DRIVER_STAT *psStatData;
} PVR_DEBUGFS_ENTRY_DATA;
static void _RefDirEntry(PVR_DEBUGFS_DIR_DATA *psDirEntry);
static void _UnrefAndMaybeDestroyDirEntry(PVR_DEBUGFS_DIR_DATA *psDirEntry);
#if 0
static void _RefDebugFSEntry(PVR_DEBUGFS_ENTRY_DATA *psDebugFSEntry);
#endif
static void _UnrefAndMaybeDestroyDebugFSEntry(PVR_DEBUGFS_ENTRY_DATA *psDebugFSEntry);
static IMG_BOOL _RefStatEntry(void *pvStatEntry);
static IMG_BOOL _UnrefAndMaybeDestroyStatEntry(void *pvStatEntry);

static void *_DebugFSStatisticSeqStart(struct seq_file *psSeqFile, loff_t *puiPosition)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)psSeqFile->private;
	IMG_BOOL bResult = IMG_FALSE;

	if (psStatData)
	{
		if (psStatData->pvData)
		{
			
			if (!_RefStatEntry((void*)psStatData))
			{
				
				return NULL;
			}
		}
		bResult = psStatData->pfnGetNextStat(psStatData->pvData,
							(IMG_UINT32)(*puiPosition),
							&psStatData->i32StatValue,
							&psStatData->pszStatFormat);
	}

	return bResult ? psStatData : NULL;
}

static void _DebugFSStatisticSeqStop(struct seq_file *psSeqFile, void *pvData)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)psSeqFile->private;

	if (psStatData)
	{
		
		if ((psStatData->ui32RefCount > 0) && (psStatData->pvData))
		{
			
			_UnrefAndMaybeDestroyStatEntry((void*)psStatData);
		}
	}
}

static void *_DebugFSStatisticSeqNext(struct seq_file *psSeqFile,
				      void *pvData,
				      loff_t *puiPosition)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)psSeqFile->private;
	IMG_BOOL bResult = IMG_FALSE;

	if (puiPosition)
	{
		(*puiPosition)++;

		if (psStatData)
		{
			if (psStatData->pvData)
			{
				bResult = psStatData->pfnGetNextStat(psStatData->pvData,
									(IMG_UINT32)(*puiPosition),
									&psStatData->i32StatValue,
									&psStatData->pszStatFormat);
			}
		}
	}
	return bResult ? psStatData : NULL;
}

static int _DebugFSStatisticSeqShow(struct seq_file *psSeqFile, void *pvData)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)pvData;

	if (psStatData != NULL)
	{
		if (psStatData->pszStatFormat == NULL)
		{
			return -EINVAL;
		}

		seq_printf(psSeqFile, psStatData->pszStatFormat, psStatData->i32StatValue);
	}

	return 0;
}

static struct seq_operations gsDebugFSStatisticReadOps =
{
	.start = _DebugFSStatisticSeqStart,
	.stop = _DebugFSStatisticSeqStop,
	.next = _DebugFSStatisticSeqNext,
	.show = _DebugFSStatisticSeqShow,
};


 

static int _DebugFSFileOpen(struct inode *psINode, struct file *psFile)
{
	PVR_DEBUGFS_PRIV_DATA *psPrivData = (PVR_DEBUGFS_PRIV_DATA *)psINode->i_private;
	int iResult;

	iResult = seq_open(psFile, psPrivData->psReadOps);
	if (iResult == 0)
	{
		struct seq_file *psSeqFile = psFile->private_data;

		

		psSeqFile->private = psPrivData->pvData;

		{
	
			if (!psPrivData->bValid)
#if 0
			{
				if (psStatData->ui32RefCount == 0)
				{
					PVR_DPF((PVR_DBG_ERROR, "%s: psStatData->ui32RefCount == 0", __FUNCTION__));
					seq_release(psINode, psFile);
					return -EIO;
				}
				psDebugFSEntry = (PVR_DEBUGFS_ENTRY_DATA *)psStatData->pvDebugFSEntry;
				if (psDebugFSEntry->ui32RefCount == 0)
				{
					PVR_DPF((PVR_DBG_ERROR, "%s: psDebugFSEntry->ui32RefCount == 0", __FUNCTION__));
					seq_release(psINode, psFile);
					return -EIO;
				}
			}
			else
#else
			{
				
				seq_release(psINode, psFile);
				return -EIO;
			}
#endif
		}

	}

	return iResult;
}

static ssize_t _DebugFSFileWrite(struct file *psFile,
				 const char __user *pszBuffer,
				 size_t uiCount,
				 loff_t *puiPosition)
{
	struct inode *psINode = psFile->f_path.dentry->d_inode;
	PVR_DEBUGFS_PRIV_DATA *psPrivData = (PVR_DEBUGFS_PRIV_DATA *)psINode->i_private;

	if (psPrivData->pfnWrite == NULL)
	{
		return -EIO;
	}

	return psPrivData->pfnWrite(pszBuffer, uiCount, *puiPosition, psPrivData->pvData);
}

static const struct file_operations gsPVRDebugFSFileOps =
{
	.owner = THIS_MODULE,
	.open = _DebugFSFileOpen,
	.read = seq_read,
	.write = _DebugFSFileWrite,
	.llseek = seq_lseek,
	.release = seq_release,
};


 

 
int PVRDebugFSInit(void)
{
	PVR_ASSERT(gpsPVRDebugFSEntryDir == NULL);

	mutex_init(&gDebugFSLock);

	gpsPVRDebugFSEntryDir = debugfs_create_dir(PVR_DEBUGFS_DIR_NAME, NULL);
	if (gpsPVRDebugFSEntryDir == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Cannot create '%s' debugfs root directory",
			 __FUNCTION__, PVR_DEBUGFS_DIR_NAME));

		return -ENOMEM;
	}

	return 0;
}

 
void PVRDebugFSDeInit(void)
{
	PVR_ASSERT(gpsPVRDebugFSEntryDir != NULL);

	debugfs_remove(gpsPVRDebugFSEntryDir);
	gpsPVRDebugFSEntryDir = NULL;
}

 
int PVRDebugFSCreateEntryDir(IMG_CHAR *pszName,
			     void *pvParentDir,
				 void **ppvNewDirHandle)
{
	PVR_DEBUGFS_DIR_DATA *psParentDir = (PVR_DEBUGFS_DIR_DATA*)pvParentDir;
	PVR_DEBUGFS_DIR_DATA *psNewDir;

	PVR_ASSERT(gpsPVRDebugFSEntryDir != NULL);

	if (pszName == NULL || ppvNewDirHandle == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s:   Invalid param  Exit", __FUNCTION__));
		return -EINVAL;
	}

	psNewDir = kmalloc(sizeof(*psNewDir), GFP_KERNEL);

	if (psNewDir == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Cannot allocate memory for '%s' pvr_debugfs structure",
			 __FUNCTION__, pszName));
		return -ENOMEM;
	}

	psNewDir->psParentDir = psParentDir;
	psNewDir->psDir = debugfs_create_dir(pszName, (psNewDir->psParentDir) ? psNewDir->psParentDir->psDir : gpsPVRDebugFSEntryDir);

	if (psNewDir->psDir == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Cannot create '%s' debugfs directory",
			 __FUNCTION__, pszName));

		kfree(psNewDir);
		return -ENOMEM;
	}

	*ppvNewDirHandle = (void*)psNewDir;
	psNewDir->ui32RefCount = 1;

	
	if (psNewDir->psParentDir)
	{
		_RefDirEntry(psNewDir->psParentDir);
	}
	return 0;
}

 
void PVRDebugFSRemoveEntryDir(void *pvDir)
{
	PVR_DEBUGFS_DIR_DATA *psDirEntry = (PVR_DEBUGFS_DIR_DATA*)pvDir;

	_UnrefAndMaybeDestroyDirEntry(psDirEntry);
}

 
int PVRDebugFSCreateEntry(const char *pszName,
			  void *pvDir,
			  struct seq_operations *psReadOps,
			  PVRSRV_ENTRY_WRITE_FUNC *pfnWrite,
			  void *pvData,
			  void **ppvNewEntry)
{
	PVR_DEBUGFS_PRIV_DATA *psPrivData;
	PVR_DEBUGFS_ENTRY_DATA *psDebugFSEntry;
	PVR_DEBUGFS_DIR_DATA *psDebugFSDir = (PVR_DEBUGFS_DIR_DATA*)pvDir;
	struct dentry *psEntry;
	umode_t uiMode;

	PVR_ASSERT(gpsPVRDebugFSEntryDir != NULL);

	psPrivData = kmalloc(sizeof(*psPrivData), GFP_KERNEL);
	if (psPrivData == NULL)
	{
		return -ENOMEM;
	}
	psDebugFSEntry = kmalloc(sizeof(*psDebugFSEntry), GFP_KERNEL);
	if (psDebugFSEntry == NULL)
	{
		kfree(psPrivData);
		return -ENOMEM;
	}

	psPrivData->psReadOps = psReadOps;
	psPrivData->pfnWrite = pfnWrite;
	psPrivData->pvData = (void*)pvData;
	psPrivData->bValid = IMG_TRUE;

	

	uiMode = S_IFREG;

	if (psReadOps != NULL)
	{
		uiMode |= S_IRUGO;
	}

	if (pfnWrite != NULL)
	{
		uiMode |= S_IWUSR;
	}

	psDebugFSEntry->psParentDir = psDebugFSDir;
	psDebugFSEntry->ui32RefCount = 1;
	psDebugFSEntry->psStatData = (PVR_DEBUGFS_DRIVER_STAT*)pvData;
	if (psDebugFSEntry->psStatData->pfnIncStatMemRefCount)
	{

	}
	if (psDebugFSEntry->psParentDir)
	{
		
		_RefDirEntry(psDebugFSEntry->psParentDir);
	}

	psEntry = debugfs_create_file(pszName,
				      uiMode,
				      (psDebugFSDir != NULL) ? psDebugFSDir->psDir : gpsPVRDebugFSEntryDir,
				      psPrivData,
				      &gsPVRDebugFSFileOps);
	if (IS_ERR(psEntry))
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Cannot create debugfs '%s' file",
			 __FUNCTION__, pszName));

		return PTR_ERR(psEntry);
	}

	psDebugFSEntry->psEntry = psEntry;
	
	*ppvNewEntry = (void*)psDebugFSEntry;

	return 0;
}

 
void PVRDebugFSRemoveEntry(void *pvDebugFSEntry)
{
	PVR_DEBUGFS_ENTRY_DATA *psDebugFSEntry = pvDebugFSEntry;

	_UnrefAndMaybeDestroyDebugFSEntry(psDebugFSEntry);
}

 
void *PVRDebugFSCreateStatisticEntry(const char *pszName,
				     void *pvDir,
				     PVRSRV_GET_NEXT_STAT_FUNC *pfnGetNextStat,
					 PVRSRV_INC_STAT_MEM_REFCOUNT_FUNC	*pfnIncStatMemRefCount,
					 PVRSRV_INC_STAT_MEM_REFCOUNT_FUNC	*pfnDecStatMemRefCount,
					 void *pvData)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData;
	PVR_DEBUGFS_ENTRY_DATA * psDebugFSEntry;
	IMG_UINT32 ui32R;
	int iResult;

	if (pszName == NULL || pfnGetNextStat == NULL)
	{
		return NULL;
	}
	if ((pfnIncStatMemRefCount != NULL || pfnDecStatMemRefCount != NULL) && pvData == NULL)
	{
		return NULL;
	}

	psStatData = kzalloc(sizeof(*psStatData), GFP_KERNEL);
	if (psStatData == NULL)
	{
		return NULL;
	}
	psStatData->pvData = (void*)pvData;
	psStatData->pfnGetNextStat = pfnGetNextStat;
	psStatData->pfnIncStatMemRefCount = pfnIncStatMemRefCount;
	psStatData->pfnDecStatMemRefCount = pfnDecStatMemRefCount;
	psStatData->ui32RefCount = 1;

	iResult = PVRDebugFSCreateEntry(pszName,
					pvDir,
					&gsDebugFSStatisticReadOps,
					NULL,
					psStatData,
					(void*)&psDebugFSEntry);
	if (iResult != 0)
	{
		kfree(psStatData);
		return NULL;
	}
	psStatData->pvDebugFSEntry = (void*)psDebugFSEntry;

	if (pfnIncStatMemRefCount)
	{
		
		ui32R = psStatData->pfnIncStatMemRefCount((void*)psStatData->pvData);
	}

	psDebugFSEntry->ui32RefCount = 1;

	return psStatData;
}

 
void PVRDebugFSRemoveStatisticEntry(void *pvStatEntry)
{
	
	_UnrefAndMaybeDestroyStatEntry(pvStatEntry);
}

static void _RefDirEntry(PVR_DEBUGFS_DIR_DATA *psDirEntry)
{
	mutex_lock(&gDebugFSLock);

	if (psDirEntry->ui32RefCount > 0)
	{
		
		psDirEntry->ui32RefCount++;
	}
	mutex_unlock(&gDebugFSLock);
}
static void _UnrefAndMaybeDestroyDirEntry(PVR_DEBUGFS_DIR_DATA *psDirEntry)
{
	mutex_lock(&gDebugFSLock);

	if (psDirEntry->ui32RefCount > 0)
	{
		
		if (--psDirEntry->ui32RefCount == 0)
		{
			
			debugfs_remove(psDirEntry->psDir);
			if (psDirEntry->psParentDir)
			{
				mutex_unlock(&gDebugFSLock);
				_UnrefAndMaybeDestroyDirEntry(psDirEntry->psParentDir);
				mutex_lock(&gDebugFSLock);
			}
			kfree(psDirEntry);
		}
	}
	mutex_unlock(&gDebugFSLock);
}

static void _UnrefAndMaybeDestroyDebugFSEntry(PVR_DEBUGFS_ENTRY_DATA *psDebugFSEntry)
{
	mutex_lock(&gDebugFSLock);
	
	PVR_ASSERT(psDebugFSEntry != IMG_NULL);

	if (psDebugFSEntry->ui32RefCount > 0)
	{
		if (--psDebugFSEntry->ui32RefCount == 0)
		{
			struct dentry *psEntry = psDebugFSEntry->psEntry;

			if (psEntry)
			{
				
				if (psEntry->d_inode->i_private != NULL)
				{
					PVR_DEBUGFS_PRIV_DATA *psPrivData = (PVR_DEBUGFS_PRIV_DATA*)psDebugFSEntry->psEntry->d_inode->i_private;

					
					psPrivData->bValid = IMG_FALSE;
					kfree(psEntry->d_inode->i_private);
				}
				debugfs_remove(psEntry);
			}
			
			if (psDebugFSEntry->psParentDir)
			{
				mutex_unlock(&gDebugFSLock);
				_UnrefAndMaybeDestroyDirEntry(psDebugFSEntry->psParentDir);
				mutex_lock(&gDebugFSLock);
			}

			
			kfree(psDebugFSEntry);
		}
	}
	mutex_unlock(&gDebugFSLock);
}

static IMG_BOOL _RefStatEntry(void *pvStatEntry)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)pvStatEntry;
	IMG_BOOL bResult = IMG_FALSE;

	mutex_lock(&gDebugFSLock);

	bResult = (psStatData->ui32RefCount > 0);
	if (bResult)
	{
		
		psStatData->ui32RefCount++;
	}
	mutex_unlock(&gDebugFSLock);

	return bResult;
}

static IMG_BOOL _UnrefAndMaybeDestroyStatEntry(void *pvStatEntry)
{
	PVR_DEBUGFS_DRIVER_STAT *psStatData = (PVR_DEBUGFS_DRIVER_STAT *)pvStatEntry;
	IMG_BOOL bResult;

	mutex_lock(&gDebugFSLock);

	bResult = (psStatData->ui32RefCount > 0);
	
	PVR_ASSERT(pvStatEntry != IMG_NULL);

	if (bResult)
	{
		if (--psStatData->ui32RefCount == 0)
		{
			if (psStatData->pvDebugFSEntry)
			{
				mutex_unlock(&gDebugFSLock);
				_UnrefAndMaybeDestroyDebugFSEntry((PVR_DEBUGFS_ENTRY_DATA*)psStatData->pvDebugFSEntry);
			}
			if (psStatData->pfnDecStatMemRefCount)
			{
				
				psStatData->pfnDecStatMemRefCount((void*)psStatData->pvData);
			}
		}
		else
		{
			mutex_unlock(&gDebugFSLock);
		}
	}
	else
	{
		mutex_unlock(&gDebugFSLock);
	}

	return bResult;
}
