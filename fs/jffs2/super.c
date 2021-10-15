/*
 * JFFS2 -- Journalling Flash File System, Version 2.
 *
 * Copyright © 2001-2007 Red Hat, Inc.
 *
 * Created by David Woodhouse <dwmw2@infradead.org>
 *
 * For licensing information, see the file 'LICENCE' in this directory.
 *
 * modify 2008/04/18 Epson
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/mount.h>
#include <linux/jffs2.h>
#include <linux/pagemap.h>
#include <linux/mtd/super.h>
#include <linux/ctype.h>
#include <linux/namei.h>
#include <linux/exportfs.h>
#include "compr.h"
#include "nodelist.h"

/* epson08 flash access control */
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/workqueue.h>

#define EPSON_JFFS2_TIMEOUT (600*HZ)
int epson_jffs2_gc_ctrl = 0;
int epson_jffs2_gc_status = 0;
int epson_jffs2_gc_flag = 0;
static struct semaphore epson_jffs2_sem;
static struct semaphore epson_jffs2_func_sem;
static char *msg = "EPSON_JFFS2_CTRL";
unsigned long epson_jffs2_timeout_jiffes = 0;
struct timer_list epson_jffs2_timer;
static int epson_jffs2_gc_ctrl_read(char *buf, char **start, off_t offset, int buf_len, int *eof, void *data);
static int epson_jffs2_gc_ctrl_write(struct file *pfile, const char *buf, unsigned long input_len, void *data);
static int epson_jffs2_gc_status_read(char *buf, char **start, off_t offset, int buf_len, int *eof, void *data);
static int epson_jffs2_gc_status_write(struct file *pfile, const char *buf, unsigned long input_len, void *data);
static void epson_jffs2_watchdog(void);
static DECLARE_WAIT_QUEUE_HEAD(epson_jffs2_wq);


static void jffs2_put_super(struct super_block *);

static struct kmem_cache *jffs2_inode_cachep;

static struct inode *jffs2_alloc_inode(struct super_block *sb)
{
	struct jffs2_inode_info *f;

	f = kmem_cache_alloc(jffs2_inode_cachep, GFP_KERNEL);
	if (!f)
		return NULL;
	return &f->vfs_inode;
}

static void jffs2_destroy_inode(struct inode *inode)
{
	kmem_cache_free(jffs2_inode_cachep, JFFS2_INODE_INFO(inode));
}

static void jffs2_i_init_once(void *foo)
{
	struct jffs2_inode_info *f = foo;

	mutex_init(&f->sem);
	inode_init_once(&f->vfs_inode);
}

static void jffs2_write_super(struct super_block *sb)
{
	struct jffs2_sb_info *c = JFFS2_SB_INFO(sb);

	lock_super(sb);
	sb->s_dirt = 0;

	if (!(sb->s_flags & MS_RDONLY)) {
		D1(printk(KERN_DEBUG "jffs2_write_super()\n"));
		jffs2_flush_wbuf_gc(c, 0);
	}

	unlock_super(sb);
}

static int jffs2_sync_fs(struct super_block *sb, int wait)
{
	struct jffs2_sb_info *c = JFFS2_SB_INFO(sb);

	jffs2_write_super(sb);

	mutex_lock(&c->alloc_sem);
	jffs2_flush_wbuf_pad(c);
	mutex_unlock(&c->alloc_sem);
	return 0;
}

static struct inode *jffs2_nfs_get_inode(struct super_block *sb, uint64_t ino,
					 uint32_t generation)
{
	/* We don't care about i_generation. We'll destroy the flash
	   before we start re-using inode numbers anyway. And even
	   if that wasn't true, we'd have other problems...*/
	return jffs2_iget(sb, ino);
}

static struct dentry *jffs2_fh_to_dentry(struct super_block *sb, struct fid *fid,
					 int fh_len, int fh_type)
{
        return generic_fh_to_dentry(sb, fid, fh_len, fh_type,
                                    jffs2_nfs_get_inode);
}

static struct dentry *jffs2_fh_to_parent(struct super_block *sb, struct fid *fid,
					 int fh_len, int fh_type)
{
        return generic_fh_to_parent(sb, fid, fh_len, fh_type,
                                    jffs2_nfs_get_inode);
}

static struct dentry *jffs2_get_parent(struct dentry *child)
{
	struct jffs2_inode_info *f;
	uint32_t pino;

	BUG_ON(!S_ISDIR(child->d_inode->i_mode));

	f = JFFS2_INODE_INFO(child->d_inode);

	pino = f->inocache->pino_nlink;

	JFFS2_DEBUG("Parent of directory ino #%u is #%u\n",
		    f->inocache->ino, pino);

	return d_obtain_alias(jffs2_iget(child->d_inode->i_sb, pino));
}

static const struct export_operations jffs2_export_ops = {
	.get_parent = jffs2_get_parent,
	.fh_to_dentry = jffs2_fh_to_dentry,
	.fh_to_parent = jffs2_fh_to_parent,
};

static const struct super_operations jffs2_super_operations =
{
	.alloc_inode =	jffs2_alloc_inode,
	.destroy_inode =jffs2_destroy_inode,
	.put_super =	jffs2_put_super,
	.write_super =	jffs2_write_super,
	.statfs =	jffs2_statfs,
	.remount_fs =	jffs2_remount_fs,
	.clear_inode =	jffs2_clear_inode,
	.dirty_inode =	jffs2_dirty_inode,
	.sync_fs =	jffs2_sync_fs,
};

/*
 * fill in the superblock
 */
static int jffs2_fill_super(struct super_block *sb, void *data, int silent)
{
	struct jffs2_sb_info *c;

	D1(printk(KERN_DEBUG "jffs2_get_sb_mtd():"
		  " New superblock for device %d (\"%s\")\n",
		  sb->s_mtd->index, sb->s_mtd->name));

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->mtd = sb->s_mtd;
	c->os_priv = sb;
	sb->s_fs_info = c;

	/* Initialize JFFS2 superblock locks, the further initialization will
	 * be done later */
	mutex_init(&c->alloc_sem);
	mutex_init(&c->erase_free_sem);
	init_waitqueue_head(&c->erase_wait);
	init_waitqueue_head(&c->inocache_wq);
	init_waitqueue_head(&epson_jffs2_wq);
	spin_lock_init(&c->erase_completion_lock);
	spin_lock_init(&c->inocache_lock);

	sb->s_op = &jffs2_super_operations;
	sb->s_export_op = &jffs2_export_ops;
	sb->s_flags = sb->s_flags | MS_NOATIME;
	sb->s_xattr = jffs2_xattr_handlers;
#ifdef CONFIG_JFFS2_FS_POSIX_ACL
	sb->s_flags |= MS_POSIXACL;
#endif
	return jffs2_do_fill_super(sb, data, silent);
}

static int jffs2_get_sb(struct file_system_type *fs_type,
			int flags, const char *dev_name,
			void *data, struct vfsmount *mnt)
{
	return get_sb_mtd(fs_type, flags, dev_name, data, jffs2_fill_super,
			  mnt);
}

static void jffs2_put_super (struct super_block *sb)
{
	struct jffs2_sb_info *c = JFFS2_SB_INFO(sb);

	D2(printk(KERN_DEBUG "jffs2: jffs2_put_super()\n"));

	lock_kernel();

	if (sb->s_dirt)
		jffs2_write_super(sb);

	mutex_lock(&c->alloc_sem);
	jffs2_flush_wbuf_pad(c);
	mutex_unlock(&c->alloc_sem);

	jffs2_sum_exit(c);

	jffs2_free_ino_caches(c);
	jffs2_free_raw_node_refs(c);
	if (jffs2_blocks_use_vmalloc(c))
		vfree(c->blocks);
	else
		kfree(c->blocks);
	jffs2_flash_cleanup(c);
	kfree(c->inocache_list);
	jffs2_clear_xattr_subsystem(c);
	if (c->mtd->sync)
		c->mtd->sync(c->mtd);

	unlock_kernel();

	D1(printk(KERN_DEBUG "jffs2_put_super returning\n"));
}

static void jffs2_kill_sb(struct super_block *sb)
{
	struct jffs2_sb_info *c = JFFS2_SB_INFO(sb);
	if (!(sb->s_flags & MS_RDONLY))
		jffs2_stop_garbage_collect_thread(c);
	kill_mtd_super(sb);
	kfree(c);
}

static struct file_system_type jffs2_fs_type = {
	.owner =	THIS_MODULE,
	.name =		"jffs2",
	.get_sb =	jffs2_get_sb,
	.kill_sb =	jffs2_kill_sb,
};

static int __init init_jffs2_fs(void)
{
	int ret;
	/* epson08 */
	struct proc_dir_entry *entry_ctrl;
	struct proc_dir_entry *entry_status;

	/* Paranoia checks for on-medium structures. If we ask GCC
	   to pack them with __attribute__((packed)) then it _also_
	   assumes that they're not aligned -- so it emits crappy
	   code on some architectures. Ideally we want an attribute
	   which means just 'no padding', without the alignment
	   thing. But GCC doesn't have that -- we have to just
	   hope the structs are the right sizes, instead. */
	BUILD_BUG_ON(sizeof(struct jffs2_unknown_node) != 12);
	BUILD_BUG_ON(sizeof(struct jffs2_raw_dirent) != 40);
	BUILD_BUG_ON(sizeof(struct jffs2_raw_inode) != 68);
	BUILD_BUG_ON(sizeof(struct jffs2_raw_summary) != 32);

	printk(KERN_INFO "JFFS2 version 2.2."
#ifdef CONFIG_JFFS2_FS_WRITEBUFFER
	       " (NAND)"
#endif
#ifdef CONFIG_JFFS2_SUMMARY
	       " (SUMMARY) "
#endif
	       " © 2001-2006 Red Hat, Inc.\n");

	/* for epson08 Flash access control */
	entry_ctrl = create_proc_entry("epson_jffs2_flash_ctrl",
					S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if(entry_ctrl != NULL){
		entry_ctrl->read_proc = epson_jffs2_gc_ctrl_read;
		entry_ctrl->write_proc = epson_jffs2_gc_ctrl_write;
	}
	else{
		printk(KERN_INFO "%s : create_proc_entry (ctrl)failed\n", msg);
		return -ENOMEM;
	}
	entry_status = create_proc_entry("epson_jffs2_flash_status",
					S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if(entry_status != NULL){
		entry_status->read_proc = epson_jffs2_gc_status_read;
		entry_status->write_proc = epson_jffs2_gc_status_write;
	}
	else{
		printk(KERN_INFO "%s : create_proc_entry (status)failed\n", msg);
		remove_proc_entry("epson_jffs2_flash_ctrl", NULL);
		return -ENOMEM;
	}



	sema_init(&epson_jffs2_sem, 1);
	sema_init(&epson_jffs2_func_sem, 1);
	/* epson08 falsh access flag watchdog */
	init_timer( &epson_jffs2_timer );
	epson_jffs2_timer.function = (void *) &epson_jffs2_watchdog;
        mod_timer( &epson_jffs2_timer, (jiffies + 2*HZ) );


	jffs2_inode_cachep = kmem_cache_create("jffs2_i",
					     sizeof(struct jffs2_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD),
					     jffs2_i_init_once);
	if (!jffs2_inode_cachep) {
		printk(KERN_ERR "JFFS2 error: Failed to initialise inode cache\n");
		return -ENOMEM;
	}
	ret = jffs2_compressors_init();
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to initialise compressors\n");
		goto out;
	}
	ret = jffs2_create_slab_caches();
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to initialise slab caches\n");
		goto out_compressors;
	}
	ret = register_filesystem(&jffs2_fs_type);
	if (ret) {
		printk(KERN_ERR "JFFS2 error: Failed to register filesystem\n");
		goto out_slab;
	}
	return 0;

 out_slab:
	jffs2_destroy_slab_caches();
 out_compressors:
	jffs2_compressors_exit();
 out:
	kmem_cache_destroy(jffs2_inode_cachep);
	return ret;
}

static void __exit exit_jffs2_fs(void)
{
	unregister_filesystem(&jffs2_fs_type);
	jffs2_destroy_slab_caches();
	jffs2_compressors_exit();
	kmem_cache_destroy(jffs2_inode_cachep);
	
	remove_proc_entry("epson_jffs2_flash_ctrl", NULL);
	remove_proc_entry("epson_jffs2_flash_status", NULL);
}


int epson_jffs2_gc_flag_check(int status){


	if(down_interruptible(&epson_jffs2_func_sem)){
		printk( KERN_INFO "%s: down_interruptible for epson_jffs2_gc_flag_check \n", msg);
		return -ERESTARTSYS;
	}
	if(down_interruptible(&epson_jffs2_sem)){
		printk( KERN_INFO "%s: down_interruptible for epson_jffs2_gc_flag_check \n", msg);
		up(&epson_jffs2_func_sem);
		return -ERESTARTSYS;
	}
	if(epson_jffs2_gc_flag == 0){ /* internal not working */
		/*  wait lock status 0 (unlock) */
		if(status == 1){
			for(;;){

				/* break loop, if unlock */
				if(epson_jffs2_gc_status == 0){
					epson_jffs2_gc_status = 1;
					epson_jffs2_gc_flag = 1; /* if epson_jffs2_gc_flag = 0, input status only 1 */
					up(&epson_jffs2_sem);
					break;
				}
				up(&epson_jffs2_sem);

				DEFINE_WAIT(jffs2_wait);
				prepare_to_wait(&epson_jffs2_wq, &jffs2_wait, TASK_INTERRUPTIBLE);
				schedule();
				finish_wait(&epson_jffs2_wq, &jffs2_wait);

				if(down_interruptible(&epson_jffs2_sem)){
					printk( KERN_INFO "%s: down_interruptible for epson_jffs2_gc_flag_check \n", msg);
					up(&epson_jffs2_func_sem);
					return -ERESTARTSYS;
				}
			}
		}
		else{
			printk(KERN_WARNING "epson_jffs2_gc_flag_check() bad arg arg=%d status=%d flag=%d\n",
						status, epson_jffs2_gc_status, epson_jffs2_gc_flag);
		}
	}
	else{ /* internal working */
		if(status == 0){
			epson_jffs2_gc_flag-- ;
			if( epson_jffs2_gc_flag == 0){
				epson_jffs2_gc_status = 0;
			}
			else{
				printk(KERN_WARNING "epson_jffs2_gc_flag_check() over call(count down ) arg=%d status=%d flag=%d\n",
						status, epson_jffs2_gc_status, epson_jffs2_gc_flag);
			}
		}
		else{
			epson_jffs2_gc_flag++ ;
			printk(KERN_WARNING "epson_jffs2_gc_flag_check() over call(count up) arg=%d status=%d flag=%d\n",
						status, epson_jffs2_gc_status, epson_jffs2_gc_flag);
		}
	}
	up(&epson_jffs2_sem);
	up(&epson_jffs2_func_sem);

	/* status reset timer overwrite */
	epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
	return 0;
};


static int epson_jffs2_gc_ctrl_read(char *buf, char **start, off_t offset, int buf_len, int *eof, void *data)
{
	if(down_interruptible(&epson_jffs2_sem)){
		printk( KERN_INFO "%s: down_interruptible for read epson_jffs2_gc_ctrl \n", msg);
		return -ERESTARTSYS;
	}

	if(epson_jffs2_gc_ctrl == 1){ /* reserve state */
		if(epson_jffs2_gc_status == 0){  /* abele to lock*/
			epson_jffs2_gc_ctrl = 2;
			epson_jffs2_gc_status = 1;
		}
	}

	buf_len = sprintf(buf, "%d", epson_jffs2_gc_ctrl);
	//printk("buf_len=%d\n", buf_len);

	up(&epson_jffs2_sem);
	return (buf_len);
}
static int epson_jffs2_gc_ctrl_write(struct file *pfile, const char *buf, unsigned long input_len, void *data)
{
	int tmp;

	if(down_interruptible(&epson_jffs2_sem)){
		printk( KERN_INFO "%s: down_interruptible for write epson_jffs2_gc_ctrl \n", msg);
		return -ERESTARTSYS;
	}

	/* status reset timer extend */
	epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;

	tmp = *buf - 0x30;
//	printk( KERN_INFO "DEBUG val = %d\n", tmp);
	if((tmp != 0) && (tmp != 1)){
		printk(KERN_INFO "not suport value \n");
		up(&epson_jffs2_sem);
		return (-1);
	}

	if(tmp == 1){ /* ctrl write 1(lock) */
		if(epson_jffs2_gc_ctrl == 2){
			epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
			printk(KERN_INFO "overwirte lock time out longer \n");
		}

		if(epson_jffs2_gc_ctrl == 0){

			/* break loop, if unlock */
			if(epson_jffs2_gc_status == 0){
				epson_jffs2_gc_ctrl = 2;
				epson_jffs2_gc_status = 1;
			}
/* comment out ,same to Rev254
			else if(epson_jffs2_gc_status == 1){
				epson_jffs2_gc_ctrl = 1;
			}
*/
		}
		if(epson_jffs2_gc_ctrl == 1){ /* already reserve state */
			up(&epson_jffs2_sem);
			printk(KERN_INFO "already lock reserved state \n");
			return(-1);

		}
	}
	else{ /* ctrl write 0(unlock) */
		if(epson_jffs2_gc_ctrl == 0){
			printk( KERN_INFO "overwrite unlock for ctrl_flag\n");
			epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
		}
		else if(epson_jffs2_gc_ctrl == 2){
			epson_jffs2_gc_ctrl = 0;
			epson_jffs2_gc_status = 0;
		}
		else{
			printk( KERN_INFO "reserved flag clear\n");
			epson_jffs2_gc_ctrl = 0; /* reserved status clear */
		}

	}

	up(&epson_jffs2_sem);
	if (waitqueue_active(&epson_jffs2_wq)) {
		wake_up(&epson_jffs2_wq);
	}

	return (input_len);
}

static int epson_jffs2_gc_status_read(char *buf, char **start, off_t offset, int buf_len, int *eof, void *data)
{
	if(down_interruptible(&epson_jffs2_sem)){
		printk( KERN_INFO "%s: down_interruptible for read epson_jffs2_gc_status \n", msg);
		return -ERESTARTSYS;
	}
	if(epson_jffs2_gc_ctrl == 1){ /* reserve state */
		if(epson_jffs2_gc_status == 0){  /* abele to lock*/
			epson_jffs2_gc_ctrl = 2;
			epson_jffs2_gc_status = 1;
		}
	}
	buf_len = sprintf(buf, "%d", epson_jffs2_gc_status);

	up(&epson_jffs2_sem);
	return (buf_len);
}
static int epson_jffs2_gc_status_write(struct file *pfile, const char *buf, unsigned long input_len, void *data)
{
	printk( KERN_INFO "NOT SUPORT write status \n");
	return (-1); /* allways err */
}



static void epson_jffs2_watchdog(void)
{
	if(down_interruptible(&epson_jffs2_sem)){
		printk( KERN_INFO "%s: down_interruptible for  epson_jffs2_watchdog \n", msg);
		return;
	}

	if(epson_jffs2_gc_ctrl == 0){
			epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
	}
	else{ /* ctrl val is reserved or locked */
	  if((epson_jffs2_gc_flag == 0)){ /*  outer locked */

		/* jiffies rollover (see include/linux/jiffies.h  ater boot 300HZ, roll over) */
		if(  ( (epson_jffs2_timeout_jiffes < EPSON_JFFS2_TIMEOUT) && (jiffies > (0xFFFFFFFF - EPSON_JFFS2_TIMEOUT)) )
		   ||( (jiffies < EPSON_JFFS2_TIMEOUT) && (epson_jffs2_timeout_jiffes > (0xFFFFFFFF - EPSON_JFFS2_TIMEOUT)) ) ){
			/* near roll over */
			if(  epson_jffs2_timeout_jiffes < jiffies){
				printk(KERN_DEBUG "jiffies near roll over\n");
				printk(KERN_DEBUG "jiffies=%lu, epson_jffs2_timeout_jiffes=%lu \n"
						, jiffies, epson_jffs2_timeout_jiffes);
			}
			else{
				printk(KERN_WARNING "flash access lock TIME OUT(watchdog)\n");
				epson_jffs2_gc_ctrl = 0;
				epson_jffs2_gc_status = 0;
				epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
			}
		}
		else{  /* normal case */
			if( epson_jffs2_timeout_jiffes < jiffies){
				printk(KERN_WARNING "flash access lock TIME OUT(watchdog)\n");
				epson_jffs2_gc_ctrl = 0;
				epson_jffs2_gc_status = 0;
				epson_jffs2_timeout_jiffes = jiffies + EPSON_JFFS2_TIMEOUT;
			}
			else{
				/* no timeout */
			}
		}

	  }
	}
	up(&epson_jffs2_sem);
        mod_timer( &epson_jffs2_timer, (jiffies + (4*HZ) ) );
	if (waitqueue_active(&epson_jffs2_wq)) {
                wake_up(&epson_jffs2_wq);
	}
}


int jffs2_flash_read(struct jffs2_sb_info *c, loff_t ofs, size_t len, size_t *retlen, u_char *buf)
{
	int     ret;
	epson_jffs2_gc_flag_check(1);
	ret =  c->mtd->read(c->mtd, ofs, len, retlen, buf);
	epson_jffs2_gc_flag_check(0);
	return ret;
}

module_init(init_jffs2_fs);
module_exit(exit_jffs2_fs);

MODULE_DESCRIPTION("The Journalling Flash File System, v2");
MODULE_AUTHOR("Red Hat, Inc.");
MODULE_LICENSE("GPL"); // Actually dual-licensed, but it doesn't matter for
		       // the sake of this tag. It's Free Software.
