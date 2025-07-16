# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux proc filesystem routines.
#

obj-y   += proc.o

CFLAGS_task_mmu.o	+= $(call cc-option,-Wno-override-init,)
proc-y			:= nommu.o task_nommu.o
proc-$(CONFIG_MMU)	:= task_mmu.o

CFLAGS_fd.o = -Wno-uninitialized
proc-y       += inode.o root.o base.o generic.o array.o \
		fd.o
proc-$(CONFIG_TTY)      += proc_tty.o
proc-y	+= cmdline.o
proc-y	+= consoles.o
proc-y	+= cpuinfo.o
proc-y	+= devices.o
proc-y	+= interrupts.o
proc-y	+= loadavg.o
proc-y	+= meminfo.o
proc-y	+= stat.o
proc-y	+= uptime.o
proc-y	+= util.o
proc-y	+= version.o
proc-y	+= softirqs.o
proc-y	+= namespaces.o
proc-y	+= self.o
proc-y	+= thread_self.o
proc-$(CONFIG_PROC_UID)  += uid.o
#Antai <AI_BSP_EFUSE> <luobowen> <2020-07-21> add for  secure_boot begin
proc-$(CONFIG_AI_BSP_EFUSE_SBC_FLAG)	+= sbcflag.o
proc-$(CONFIG_AI_BSP_EFUSE_BLOWER)	+= efuse_blower.o
#Antai <AI_BSP_EFUSE> <luobowen> <2020-07-21> add for  secure_boot end
proc-$(CONFIG_PROC_SYSCTL)	+= proc_sysctl.o
proc-$(CONFIG_NET)		+= proc_net.o
proc-$(CONFIG_PROC_KCORE)	+= kcore.o
proc-$(CONFIG_PROC_VMCORE)	+= vmcore.o
proc-$(CONFIG_PRINTK)	+= kmsg.o
proc-$(CONFIG_PROC_PAGE_MONITOR)	+= page.o
#Antai <AI_BSP_PROC> <luobowen> <2021-03-17> add for  proc node begin
proc-$(CONFIG_AI_BSP_UFS_ID)	+= ufs_id.o
proc-$(CONFIG_AI_BSP_MEMORY_INFO)	+= antaimeminfo.o
#Antai <AI_BSP_STORAGE> <luobowen> <2020-12-28> add for ata test  end
