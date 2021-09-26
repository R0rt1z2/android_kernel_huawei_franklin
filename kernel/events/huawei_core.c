/*
 * huawei_core.c
 *
 * huawei perf event core
 *
 * Copyright (c) 2012-2020 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#if defined CONFIG_HOTPLUG_CPU || defined CONFIG_KEXEC_CORE
static void perf_prepare_install_in_context(struct perf_event *event)
{
	spin_lock(&dormant_event_list_lock);
	if (event->state == PERF_EVENT_STATE_DORMANT)
		goto out;

	event->state = PERF_EVENT_STATE_DORMANT;
	list_add_tail(&event->dormant_event_entry, &dormant_event_list);
out:
	spin_unlock(&dormant_event_list_lock);
}

static void perf_deferred_install_in_context(int cpu)
{
	struct perf_event *event = NULL;
	struct perf_event *tmp = NULL;
	struct perf_event_context *ctx = NULL;

	spin_lock(&dormant_event_list_lock);
	list_for_each_entry_safe(event, tmp, &dormant_event_list,
						dormant_event_entry) {
		if (cpu != event->cpu)
			continue;

		list_del(&event->dormant_event_entry);
		event->state = PERF_EVENT_STATE_INACTIVE;
		spin_unlock(&dormant_event_list_lock);

		ctx = event->ctx;

		mutex_lock(&ctx->mutex);
		perf_install_in_context(ctx, event, cpu);
		mutex_unlock(&ctx->mutex);

		spin_lock(&dormant_event_list_lock);
	}
	spin_unlock(&dormant_event_list_lock);
}
#endif

static int
perf_event_delete_kernel_shared(struct perf_event *event)
{
	int rc = -1, cpu = event->cpu;
	struct shared_events_str *shrd_events = NULL;
	unsigned long idx;

	if (!shared_events || (u32)cpu >= nr_cpu_ids)
		return 0;

	shrd_events = per_cpu_ptr(shared_events, cpu);

	mutex_lock(&shrd_events->list_mutex);

	for_each_set_bit(idx, shrd_events->used_mask, SHARED_EVENTS_MAX) {
		if (shrd_events->events[idx] == event) {
			if (atomic_dec_and_test(&shrd_events->refcount[idx])) {
				clear_bit(idx, shrd_events->used_mask);
				shrd_events->events[idx] = NULL;
			}
			rc = (int)atomic_read(&shrd_events->refcount[idx]);
			break;
		}
	}

	mutex_unlock(&shrd_events->list_mutex);
	return rc;
}

static int __perf_event_release_kernel(struct perf_event *event)
{
	struct perf_event_context *ctx = event->ctx;
	struct perf_event *child = NULL;
	struct perf_event *tmp = NULL;

#if defined CONFIG_HOTPLUG_CPU || defined CONFIG_KEXEC_CORE
	if (event->cpu != -1) {
		spin_lock(&dormant_event_list_lock);
		if (event->state == PERF_EVENT_STATE_DORMANT)
			list_del(&event->dormant_event_entry);
		spin_unlock(&dormant_event_list_lock);
	}
#endif

	/*
	 * If we got here through err_file: fput(event_file); we will not have
	 * attached to a context yet.
	 */
	if (!ctx) {
		WARN_ON_ONCE(event->attach_state &
				(PERF_ATTACH_CONTEXT|PERF_ATTACH_GROUP));
		goto no_ctx;
	}

	if (!is_kernel_event(event))
		perf_remove_from_owner(event);

	ctx = perf_event_ctx_lock(event);
	WARN_ON_ONCE(ctx->parent_ctx);
	perf_remove_from_context(event, DETACH_GROUP);

	if (perf_event_delete_kernel_shared(event) > 0) {
		perf_event__state_init(event);
		perf_install_in_context(ctx, event, event->cpu);

		perf_event_ctx_unlock(event, ctx);

		perf_event_enable(event);

		return 0;
	}

	raw_spin_lock_irq(&ctx->lock);
	/*
	 * Mark this event as STATE_DEAD, there is no external reference to it
	 * anymore.
	 *
	 * Anybody acquiring event->child_mutex after the below loop _must_
	 * also see this, most importantly inherit_event() which will avoid
	 * placing more children on the list.
	 *
	 * Thus this guarantees that we will in fact observe and kill _ALL_
	 * child events.
	 */
	event->state = PERF_EVENT_STATE_DEAD;
	raw_spin_unlock_irq(&ctx->lock);

	perf_event_ctx_unlock(event, ctx);

again:
	mutex_lock(&event->child_mutex);
	list_for_each_entry(child, &event->child_list, child_list) {

		/*
		 * Cannot change, child events are not migrated, see the
		 * comment with perf_event_ctx_lock_nested().
		 */
		ctx = READ_ONCE(child->ctx);
		/*
		 * Since child_mutex nests inside ctx::mutex, we must jump
		 * through hoops. We start by grabbing a reference on the ctx.
		 *
		 * Since the event cannot get freed while we hold the
		 * child_mutex, the context must also exist and have a !0
		 * reference count.
		 */
		get_ctx(ctx);

		/*
		 * Now that we have a ctx ref, we can drop child_mutex, and
		 * acquire ctx::mutex without fear of it going away. Then we
		 * can re-acquire child_mutex.
		 */
		mutex_unlock(&event->child_mutex);
		mutex_lock(&ctx->mutex);
		mutex_lock(&event->child_mutex);

		/*
		 * Now that we hold ctx::mutex and child_mutex, revalidate our
		 * state, if child is still the first entry, it didn't get freed
		 * and we can continue doing so.
		 */
		tmp = list_first_entry_or_null(&event->child_list,
					       struct perf_event, child_list);
		if (tmp == child) {
			perf_remove_from_context(child, DETACH_GROUP);
			list_del(&child->child_list);
			free_event(child);
			/*
			 * This matches the refcount bump in inherit_event();
			 * this can't be the last reference.
			 */
			put_event(event);
		}

		mutex_unlock(&event->child_mutex);
		mutex_unlock(&ctx->mutex);
		put_ctx(ctx);
		goto again;
	}
	mutex_unlock(&event->child_mutex);

no_ctx:
	put_event(event); /* Must be the 'last' reference */
	return 0;
}

int perf_event_release_kernel(struct perf_event *event)
{
	int ret;

	mutex_lock(&pmus_lock);
	ret = __perf_event_release_kernel(event);
	mutex_unlock(&pmus_lock);

	return ret;
}

static struct perf_event *
perf_event_create_kernel_shared_check(struct perf_event_attr *attr, int cpu,
		struct task_struct *task,
		perf_overflow_handler_t overflow_handler,
		struct perf_event *group_leader)
{
	unsigned long idx;
	struct perf_event *event = NULL;
	struct shared_events_str *shrd_events = NULL;

	/*
	 * Have to be per cpu events for sharing
	 */
	if (!shared_events || (u32)cpu >= nr_cpu_ids)
		return NULL;

	/*
	 * Can't handle these type requests for sharing right now.
	 */
	if (task || overflow_handler || attr->sample_period ||
	    (attr->type != PERF_TYPE_HARDWARE &&
	     attr->type != PERF_TYPE_RAW)) {
		return NULL;
	}

	/*
	 * Using per_cpu_ptr (or could do cross cpu call which is what most of
	 * perf does to access per cpu data structures
	 */
	shrd_events = per_cpu_ptr(shared_events, cpu);

	mutex_lock(&shrd_events->list_mutex);

	for_each_set_bit(idx, shrd_events->used_mask, SHARED_EVENTS_MAX) {
		/* Do the comparisons field by field on the attr structure.
		 * This is because the user-space and kernel-space might
		 * be using different versions of perf. As a result,
		 * the fields' position in the memory and the size might not
		 * be the same. Hence memcmp() is not the best way to
		 * compare.
		 */
		if (attr->type == shrd_events->attr[idx].type &&
			attr->config == shrd_events->attr[idx].config) {

			event = shrd_events->events[idx];

			/* Do not change the group for this shared event */
			if (group_leader && event->group_leader != event) {
				event = NULL;
				continue;
			}

			event->shared = true;
			atomic_inc(&shrd_events->refcount[idx]);
			break;
		}
	}
	mutex_unlock(&shrd_events->list_mutex);

	return event;
}

static void
perf_event_create_kernel_shared_add(struct perf_event_attr *attr, int cpu,
				 struct task_struct *task,
				 perf_overflow_handler_t overflow_handler,
				 void *context,
				 struct perf_event *event)
{
	unsigned long idx;
	struct shared_events_str *shrd_events = NULL;

	/*
	 * Have to be per cpu events for sharing
	 */
	if (!shared_events || (u32)cpu >= nr_cpu_ids)
		return;

	/*
	 * Can't handle these type requests for sharing right now.
	 */
	if (overflow_handler || attr->sample_period ||
	    (attr->type != PERF_TYPE_HARDWARE &&
	     attr->type != PERF_TYPE_RAW)) {
		return;
	}

	/*
	 * Using per_cpu_ptr (or could do cross cpu call which is what most of
	 * perf does to access per cpu data structures
	 */
	shrd_events = per_cpu_ptr(shared_events, cpu);

	mutex_lock(&shrd_events->list_mutex);

	/*
	 * If we are in this routine, we know that this event isn't already in
	 * the shared list. Check if slot available in shared list
	 */
	idx = find_first_zero_bit(shrd_events->used_mask, SHARED_EVENTS_MAX);

	if (idx >= SHARED_EVENTS_MAX)
		goto out;

	/*
	 * The event isn't in the list and there is an empty slot so add it.
	 */
	shrd_events->attr[idx]   = *attr;
	shrd_events->events[idx] = event;
	set_bit(idx, shrd_events->used_mask);
	atomic_set(&shrd_events->refcount[idx], 1);
out:
	mutex_unlock(&shrd_events->list_mutex);
}

#ifdef CONFIG_PERF_USER_SHARE
static void perf_group_shared_event(struct perf_event *event,
		struct perf_event *group_leader)
{
	if (!event->shared || !group_leader)
		return;

	/* Do not attempt to change the group for this shared event */
	if (event->group_leader != event)
		return;

	/*
	 * Single events have the group leaders as themselves.
	 * As we now have a new group to attach to, remove from
	 * the previous group and attach it to the new group.
	 */
	perf_remove_from_context(event, DETACH_GROUP);

	event->group_leader	= group_leader;
	perf_event__state_init(event);

	perf_install_in_context(group_leader->ctx, event, event->cpu);
}
#endif

int perf_event_restart_events(unsigned int cpu)
{
	mutex_lock(&pmus_lock);
	per_cpu(is_hotplugging, cpu) = false;
	perf_deferred_install_in_context(cpu);
	mutex_unlock(&pmus_lock);

	return 0;
}

static int event_idle_notif(struct notifier_block *nb, unsigned long action,
							void *data)
{
	switch (action) {
	case IDLE_START:
		__this_cpu_write(is_idle, true);
		break;
	case IDLE_END:
		__this_cpu_write(is_idle, false);
		break;
	}

	return NOTIFY_OK;
}

struct notifier_block perf_event_idle_nb = {
	.notifier_call = event_idle_notif,
};

