#include <linux/oem/im.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include "../kernel/sched/sched.h"
#include "../kernel/sched/walt.h"


/* default list, empty string means no need to check */
static char target[IM_ID_MAX][64] = {
	"surfaceflinger",
	"",
	"logd",
	"logcat",
	"",
	"",
	"",
	"",
	"composer-servic",
	"HwBinder:",
	"Binder:",
	"",
	"",
	"",
	"CrRendererMain",
};

/* ignore list, not set any im_flag */
static char target_ignore_prefix[IM_IG_MAX][64] = {
	"Prober_",
	"DispSync",
	"app",
	"sf",
	"ScreenShotThrea",
	"DPPS_THREAD",
	"LTM_THREAD",
};

static inline bool im_ignore(struct task_struct *task, int idx)
{
	size_t tlen = 0, len = 0;

	tlen = strlen(target_ignore_prefix[idx]);
	if (tlen == 0)
		return false;

	/* NOTE: task->comm has only 16 bytes */
	len = strlen(task->comm);
	if (len < tlen)
		return false;

	if (!strncmp(task->comm, target_ignore_prefix[idx], tlen)) {
		task->im_flag = 0;
		return true;
	}
	return false;
}

static inline void im_tagging(struct task_struct *task, int idx)
{
	size_t tlen = 0, len = 0;

	tlen = strlen(target[idx]);
	if (tlen == 0)
		return;

	/* NOTE: task->comm has only 16 bytes */
	len = strlen(task->comm);

	/* non restrict tagging for some prefixed tasks*/
	if (len < tlen)
		return;
	if (!strncmp(task->comm, target[idx], tlen)) {
		switch (idx) {
		case IM_ID_HWBINDER:
			task->im_flag |= IM_HWBINDER;
			break;
		case IM_ID_BINDER:
			task->im_flag |= IM_BINDER;
			break;
		}
	}

	/* restrict tagging for specific identical tasks */
	if (len != tlen)
		return;
	if (!strncmp(task->comm, target[idx], len)) {
		switch (idx) {
		case IM_ID_SURFACEFLINGER:
			task->im_flag |= IM_SURFACEFLINGER;
			break;
		case IM_ID_LOGD:
			task->im_flag |= IM_LOGD;
			break;
		case IM_ID_LOGCAT:
			task->im_flag |= IM_LOGCAT;
			break;
		case IM_ID_HWC:
			task->im_flag |= IM_HWC;
			break;
		case IM_ID_RENDER:
			task->im_flag |= IM_RENDER;
			break;
		case IM_ID_CRENDER:
			/*mark this to save power for DOU test*/
			//task->im_flag |= IM_CRENDER;
			break;
		}
	}
}

void im_wmi(struct task_struct *task)
{
	int i = 0;
	struct task_struct *leader = task->group_leader;

	/* check for ignore */
	for (i = 0; i < IM_IG_MAX; ++i)
		if (im_ignore(task, i))
			return;

	/* do the check and initial */
	task->im_flag = 0;
	for (i = 0; i < IM_ID_MAX; ++i)
		im_tagging(task, i);

	/* for hwc cases */
	if (im_hwc(leader)) {
		struct task_struct *p;
		rcu_read_lock();
		for_each_thread(task, p) {
			if (im_binder_related(p))
				p->im_flag |= IM_HWC;
		}
		rcu_read_unlock();
	}

	/* for sf cases */
	if (im_sf(leader) && im_binder_related(task))
		task->im_flag |= IM_SURFACEFLINGER;
}

void im_wmi_current(void)
{
	int i = 0;
	struct task_struct *leader = current->group_leader;

	/* check for ignore */
	for (i = 0; i < IM_IG_MAX; ++i)
		if (im_ignore(current, i))
			return;

	/* do the check and initial */
	current->im_flag = 0;
	for (i = 0; i < IM_ID_MAX; ++i)
		im_tagging(current, i);

	/* for hwc cases */
	if (im_hwc(leader)) {
		struct task_struct *p;
		rcu_read_lock();
		for_each_thread(current, p) {
			if (im_binder_related(p))
				p->im_flag |= IM_HWC;
		}
		rcu_read_unlock();
	}

	/* for sf cases */
	if (im_sf(leader) && im_binder_related(current))
		current->im_flag |= IM_SURFACEFLINGER;
}

void im_set_flag(struct task_struct *task, int flag)
{
	struct task_struct *leader = task->group_leader;

	task->im_flag |= flag;
	if (flag == IM_ENQUEUE) {
		if (leader)
			im_set_flag(leader, IM_MAIN);
	}
}

void im_set_flag_current(int flag)
{
	struct task_struct *leader = current->group_leader;

	current->im_flag |= flag;
	if (flag == IM_ENQUEUE) {
		if (leader)
			im_set_flag(leader, IM_MAIN);
	}
}

void im_unset_flag(struct task_struct *task, int flag)
{
	task->im_flag &= ~flag;
}

void im_unset_flag_current(int flag)
{
	current->im_flag &= ~flag;
}

void im_reset_flag(struct task_struct *task)
{
	task->im_flag = 0;
}

void im_reset_flag_current(void)
{
	current->im_flag = 0;
}
