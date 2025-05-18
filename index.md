# BitMap Queue CPU 调度器

## 1. 项目背景

BMQ 调度器作为 PDS 的演进版本，为 Linux 内核提供一个简洁、高效且具备良好拓展性的 CPU 调度方案。BMQ 采用每 CPU 本地运行队列，并为每个队列配备自旋锁来保护任务插入与调度操作。这种设计虽然简洁，但锁的粒度依然较粗：任何对统一 CPU 运行队列的并发访问都必须串行化，导致多核环境下竞争严重。此外，BMQ 当前并未引入全局运行队列或统一的任务调度视图，跨 CPU 的任务迁移依赖于各 CPU 主动从其他 CPU 的运行队列中“窃取”任务。这一过程通过加锁访问远程 runqueue 实现，且只在特定条件下被动触发，缺乏统一的负载均衡机制。因此，当前 BMQ 的任务迁移机制在多核高负载场景下存在调度粒度粗、迁移效率低的问题。

## 2. 现有BMQ核心机制分析

### 2.1 核心数据结构和基本组件

为实现ELF BMQ中“每个CPU的无锁抢占队列”和“优先级级别的全局运行队列锁”，数据结构的设计至关重要。核心代码位于 `kernel/sched/bmq_sched.h` 中。

#### 2.1.1 CPU 运行队列

`struct rq` 代表了每个CPU独立的运行队列，它是CPU进行任务调度的核心上下文。该结构体汇集了当前CPU上所有可运行任务、调度所需的各种状态信息、统计数据以及同步原语。

```c
struct rq {
    // [关键成员 - 针对“优先级级别的全局运行队列锁”]
    raw_spinlock_t lock;        // 本CPU运行队列的自旋锁，用于保护队列内部数据结构在并发访问时的一致性。
                                // 全局范围的锁协调（若涉及跨CPU）则依赖于更高层面的设计或此锁在特定场景下的扩展应用。

    // [关键成员 - 任务状态与管理]
    struct task_struct __rcu *curr; // 指向当前在此CPU上执行的任务。
    struct task_struct *idle;       // 指向本CPU的空闲任务。
    struct task_struct *stop;       // 指向用于停止CPU的任务（例如CPU hotplug场景）。
    struct task_struct *skip;       // 指向一个在下次调度时应被暂时跳过的任务。
                                    // 此机制与 sched_yield() 的特定行为相关，影响本地抢占队列的任务选择。

    struct mm_struct *prev_mm;      // 上一个运行任务的内存描述符，用于优化上下文切换。

    // [关键成员 - 针对“每个CPU的无锁抢占队列”]
    struct bmq queue;             // 内嵌的BMQ位图队列结构，是实现本CPU高效、低延迟任务管理的核心。
                                  // 其设计直接支撑了无锁或极低锁竞争的抢占队列操作。

    unsigned long watermark;      // 运行队列的水位标记，用于快速判断队列的繁忙程度，
                                  // 可辅助负载均衡决策和优化任务迁移。

    // [关键成员 - 统计与状态]
    u64 nr_switches;              // 本CPU上发生的上下文切换总次数。
    atomic_t nr_iowait;           // 在本CPU上，处于不可中断睡眠状态且等待I/O的任务数量。
    int cpu;                      // 本运行队列所属的CPU ID。

    unsigned long nr_running;     // 本运行队列中可运行（处于TASK_RUNNING状态且不在CPU上运行）的任务数量。
                                  // 此字段对于本地队列是否为空、是否可以无锁操作、
                                  // 以及在 take_other_rq_tasks 中进行负载均衡判断至关重要。

    u64 clock;                    // 运行队列自身的时钟，通常由 sched_clock_cpu() 更新。
    u64 clock_task;               // 用于精确追踪任务在CPU上运行的时间。
    u64 last_ts_switch;           // 上一次上下文切换时的时间戳。
    // ... 其他统计和管理字段 ...
};
```

针对项目目标的现有设计分析：

- 同步原语（`rq->lock`）：
  - 在现有的 BMQ 设计中，`rq->lock` 是一个原始自旋锁，用于保护单个CPU运行队列中的所有核心数据结构在并发访问时的一致性。几乎所有的本地队列操作，如任务入队（`enqueue_task`）, 出队（`dequeue_task`）, 重新入队（`requeue_task`）, 以及在 `__schedule` 函数中选择下一个任务（`choose_next_task`） 和更新任务状态等过程，都必须持有这个锁。
  - 在SMP 环境下，当一个CPU尝试从另一个CPU窃取任务时（例如在 `take_other_rq_tasks` 函数中），也需要获取目标CPU的 `rq->lock`。
- 任务计数（`nr_running`）：
  - `rq->nr_running` 字段记录了当前运行队列中可运行状态的任务数量。
  - 当前的任务计数并不是原子类型的，是因为它目前受到 `rq->lock`的保护，在设计无锁抢占队列当中需要设置为原子类型。
  
- 当前任务指针（`curr`）：
  - 指向当前在此CPU上执行的任务。它需要重新设计，是另一个任务的内容，这里不做分析。

- 水位标记（`watermark`）与 SMP 相关的掩码：
  - `rq->watermark` 记录了当前 `rq` 中（通过 `find_first_bit` 查找到的）最高优先级任务的级别。 此外，还有一些全局位图如 `sched_rq_watermark` 和 `sched_sg_idle_mask` 用于SMP环境下的负载均衡和任务选择。
  - 这些状态的更新目前也间接或直接地依赖于 `rq->lock` 的保护，以确保在修改位图后原子地更新这些聚合状态。


#### 2.1.2  位图优先级队列

`struct bmq` 是 `struct rq` 的核心组成部分，它直接负责组织和管理CPU本地的可运行任务。该结构通过位图和优先级链表数组，实现了高效的任务查找和特定优先级任务的队列管理。

```c
// 定义了BMQ中支持的总优先级位数。
// 计算方式为：MAX_RT_PRIO (实时优先级数量) + NICE_WIDTH (普通任务nice值范围)
//              + 2 * MAX_PRIORITY_ADJ (动态优先级调整范围的正负两部分)
//              + 1 (通常为空闲任务保留一个专用位)
// 这个宏精确定义了bitmap的大小，确保能覆盖所有可能的有效任务优先级。
#define bmq_BITS (MAX_RT_PRIO + NICE_WIDTH + 2 * MAX_PRIORITY_ADJ + 1)
#define IDLE_TASK_SCHED_PRIO	(bmq_BITS - 1) // 空闲任务的调度优先级，通常是bitmap的最后一位

struct bmq {
    // [关键成员 - 针对“每个CPU的无锁抢占队列”]
    DECLARE_BITMAP(bitmap, bmq_BITS); // 一个位图，每一位对应一个优先级。
                                      // 若某位为1，则表示对应优先级的任务队列 (heads数组中的对应项) 非空。
                                      // 这是实现 O(1) 复杂度查找最高优先级非空队列的关键，
                                      // 极大提升了任务选择效率，是无锁或低锁操作的基础。

    struct list_head heads[bmq_BITS]; // 任务队列数组，数组的每个元素是一个双向链表头，
                                      // 代表一个特定优先级的可运行任务队列。
                                      // 同一优先级队列中的任务遵循FIFO（先进先出）原则。
};

// 宏定义，用于声明一个指定长度的位图数组
#define DECLARE_BITMAP(name, bits) \
    unsigned long name[BITS_TO_LONGS(bits)]

// 宏定义，计算存储指定位数所需的 unsigned long 数组的长度
#define BITS_TO_LONGS(nr) DIV_ROUND_UP(nr, BITS_PER_TYPE(long))
```

针对项目目标的现有设计分析：

- 当前的设计，可以使得调度器能够非常快速地查找到最高优先级非空队列。
- 虽然查找效率高，但是现有的BMQ中，对 bitmap的修改，例如：`set_bit`，`clear_bit` 以及对 `heads`进行操作仍然是在 `rq->lock` 的保护下完成的。
- 根据项目目标“每个CPU的无锁抢占队列”，这意味着对 `bmq->heads`（即各优先级任务链表）的操作需要设计成无锁队列。同时，对 `bmq->bitmap` 的更新也必须是原子的，以确保其状态与各个无锁队列的状态（是否为空）保持一致。

#### 2.1.3 调度器初始化

调度器的初始化过程主要由 `sched_init()` 函数完成，它负责为系统中每个CPU设置初始的运行队列状态。同时，`init_idle()` 函数专门用于初始化每个CPU的空闲任务。

1. 调度器为系统中每个可能存在的CPU都初始化了一个 `struct rq` 实例。
2. 初始化每个运行队列 `rq` 内的 `bmq` 结构。
3. 设置初始水位标记为 `IDLE_WM`。
4. 初始化任务计数器为 0。
5. 为引导CPU（通常是CPU 0）上的当前任务（即 `init_task`）调用 `init_idle()`，将其设置为空闲任务。

```c
void __init sched_init(void)
{
    int i;
    struct rq *rq;

    // ... (一些全局初始化，如 print_scheduler_version, wait_bit_init) ...

#ifdef CONFIG_SMP
    for (i = 0; i < bmq_BITS; i++)
        cpumask_copy(&sched_rq_watermark[i], cpu_present_mask);
#endif

    // ... (cgroup 初始化) ...

    for_each_possible_cpu(i) { // 遍历系统中所有可能的CPU
        rq = cpu_rq(i);        // 获取该CPU的运行队列指针

        bmq_init(&rq->queue);  // 初始化该运行队列中的bmq结构 (位图和链表头)
        rq->watermark = IDLE_WM; // 设置水位标记为idle任务的优先级
        rq->skip = NULL;         // 初始化skip指针

        raw_spin_lock_init(&rq->lock); // 初始化运行队列的自旋锁 <<--- 关键点
        rq->nr_running = rq->nr_uninterruptible = 0; // 初始化可运行任务和不可中断任务计数
        rq->calc_load_active = 0;
        rq->calc_load_update = jiffies + LOAD_FREQ; // 初始化负载计算相关变量
#ifdef CONFIG_SMP
        rq->online = false;     // 初始时标记CPU为离线 (在后续的smp初始化中会设置为在线)
        rq->cpu = i;            // 设置rq对应的CPU ID

#ifdef CONFIG_SCHED_SMT
        rq->active_balance = 0; // SMT相关的主动负载均衡标记
#endif
#endif
        rq->nr_switches = 0;    // 上下文切换计数器清零
        atomic_set(&rq->nr_iowait, 0); // IO等待任务计数器清零
        hrtick_rq_init(rq);    // 高精度定时器初始化 (如果启用)
    }
#ifdef CONFIG_SMP
    /* Set rq->online for cpu 0 */
    cpu_rq(0)->online = true; // 启动CPU 0通常是第一个上线的
#endif

    // ... (当前任务(init_task)的idle化设置，mmgrab等) ...
    init_idle(current, smp_processor_id()); // 将当前任务(通常是init_task)初始化为启动CPU的idle任务

    calc_load_update = jiffies + LOAD_FREQ; // 全局负载更新时间戳

#ifdef CONFIG_SMP
    // idle_thread_set_boot_cpu(); // 这个函数在BMQ代码中没有，可能是从其他调度器借鉴的注释
    sched_init_topology_cpumask_early(); // 初始化CPU拓扑相关的掩码 (早期)
#endif

    init_schedstats(); // 调度统计相关初始化
    psi_init();        // PSI (Pressure Stall Information) 初始化
}

// 另外，idle任务的初始化在 init_idle 函数中：
void init_idle(struct task_struct *idle, int cpu)
{
    struct rq *rq = cpu_rq(cpu);
    // ...
    raw_spin_lock_irqsave(&idle->pi_lock, flags);
    raw_spin_lock(&rq->lock); // 获取rq锁
    // ...
    bmq_init_idle(&rq->queue, idle); // 初始化bmq中的idle任务队列，并设置bitmap相应位
    // ...
    rcu_assign_pointer(rq->curr, idle); // 设置当前任务为idle
    idle->on_cpu = 1;
    // ...
    raw_spin_unlock(&rq->lock); // 释放rq锁
    raw_spin_unlock_irqrestore(&idle->pi_lock, flags);
    // ...
}
```

#### 2.1.4 调度策略与优先级映射

BMQ调度器支持多种调度策略，并通过一系列内部机制将任务的策略和用户指定的优先级参数（如实时优先级或nice值）转化为其在位图优先级队列（`struct bmq`）中的具体位置。这个映射是调度器正确处理不同类型任务的基础。

1. 系统调用入口与参数处理

用户空间通过 `sched_setscheduler()` 系统调用来设定任务的调度策略和相关参数。

```c
// kernel/sched/bmq.c
SYSCALL_DEFINE3(sched_setscheduler, pid_t, pid, int, policy, struct sched_param __user *, param)
{
    if (policy < 0) // policy 值不能为负
        return -EINVAL;

    return do_sched_setscheduler(pid, policy, param); // 调用核心处理函数
}

// kernel/sched/bmq.c
static int
do_sched_setscheduler(pid_t pid, int policy, struct sched_param __user *param)
{
    struct sched_param lparam; // 内核空间的参数副本
    struct task_struct *p;
    int retval;

    if (!param || pid < 0)
        return -EINVAL;
    // 从用户空间拷贝参数到内核空间
    if (copy_from_user(&lparam, param, sizeof(struct sched_param)))
        return -EFAULT;

    rcu_read_lock();
    retval = -ESRCH;
    p = find_process_by_pid(pid); // 查找目标任务
    if (likely(p))
        get_task_struct(p); // 增加任务引用计数，防止任务消失
    rcu_read_unlock();

    if (likely(p)) {
        // 调用实际的设置函数
        retval = sched_setscheduler(p, policy, &lparam);
        put_task_struct(p); // 减少任务引用计数
    }

    return retval;
}
```

`sched_setscheduler` 最终会调用 `__sched_setscheduler` 函数，它是 BMQ 调度器内部处理策略和优先级变更的核心。

```c
// kernel/sched/bmq.c
static int __sched_setscheduler(struct task_struct *p,
				const struct sched_attr *attr, // 使用 sched_attr 结构，更通用
				bool user, bool pi)
{
    // ... (SCHED_DEADLINE 转换为 SCHED_FIFO 的处理) ...
    // ... (权限检查和参数有效性验证) ...

    raw_spin_lock_irqsave(&p->pi_lock, flags); // 获取任务的pi_lock
    rq = __task_access_lock(p, &lock);        // 获取任务所在rq的锁

    // ... (检查是否真的需要改变，如果策略和优先级都未变则提前返回) ...

    // 如果策略或优先级发生变化:
    p->sched_reset_on_fork = reset_on_fork;

    // 如果启用了PI (Priority Inheritance) 且任务是PI相关的，
    // 可能会有特殊的优先级计算，如果最终有效优先级不变，则只更新参数，不立即调整队列位置
    if (pi) {
        if (rt_effective_prio(p, newprio) == p->prio) { // newprio 是根据 attr 计算出的目标优先级
            __setscheduler_params(p, attr);
            retval = 0;
            goto unlock;
        }
    }

    // 实际更新任务的policy, prio, normal_prio等，并基于此计算新的p->prio
    __setscheduler(rq, p, attr, pi);

    // 检查任务计算出的BMQ索引是否变化，如果变化则重新入队
    check_task_changed(rq, p); // <<--- 核心：将任务放到新的BMQ优先级队列

    // ... (解锁) ...
    return 0;

unlock:
    __task_access_unlock(p, lock);
    raw_spin_unlock_irqrestore(&p->pi_lock, flags);
    // ...
    return retval;
}

// kernel/sched/bmq.c
// __setscheduler_params 仅更新 task_struct 中的策略和基础优先级参数
static void __setscheduler_params(struct task_struct *p,
		const struct sched_attr *attr)
{
	int policy = attr->sched_policy;

	if (policy == SETPARAM_POLICY) // SETPARAM_POLICY (-1) 表示不改变策略
		policy = p->policy;

	p->policy = policy;
	p->static_prio = NICE_TO_PRIO(attr->sched_nice); // nice值转换为内部static_prio
	p->rt_priority = attr->sched_priority;         // 实时任务的优先级
	p->normal_prio = normal_prio(p);               // 计算normal_prio
}

// kernel/sched/bmq.c
// __setscheduler 更新任务的最终prio值
static void __setscheduler(struct rq *rq, struct task_struct *p,
			   const struct sched_attr *attr, bool keep_boost)
{
	__setscheduler_params(p, attr); // 先设置基础参数

	p->prio = normal_prio(p); // 基于新参数计算prio (不含boost_prio, 但已包含rt偏移或nice转换)
	if (keep_boost)         // 如果允许保持之前的boost状态 (例如，非用户主动修改，而是PI)
		p->prio = rt_effective_prio(p, p->prio); // 考虑PI-boosting
    // 注意：最终task_sched_prio()还会结合p->boost_prio (对于普通任务)
}
```

2. BMQ调度器如何处理不同策略的映射

```c
static inline int task_sched_prio(struct task_struct *p)
{
    // 如果任务p的prio值小于MAX_RT_PRIO，说明它是一个实时任务或者被提升到实时优先级的任务。
    // 对于这类任务，其在BMQ中的索引直接由其prio值决定。
    // 否则，任务p是一个普通任务（SCHED_NORMAL, SCHED_BATCH, SCHED_IDLE），
    // 其在BMQ中的索引为其prio值加上动态调整的boost_prio。
    // prio本身已经包含了静态优先级（nice值转换而来）和MAX_RT_PRIO的偏移。
    return (p->prio < MAX_RT_PRIO)? p->prio : p->prio + p->boost_prio;
}
```

- **SCHED_DEADLINE:** 作为最高优先级的任务，放到索引为 0 的队列中。
- **SCHED_FIFO/SCHED_RR**: 其 `p->rt_priority` (0-99) 决定了它们在BMQ索引中的绝对高优先级位置 (通常是 0 到 `MAX_RT_PRIO-1`)。
- **SCHED_NORMAL/SCHED_BATCH/SCHED_IDLE**: 其 `p->static_prio` (来自nice值) 加上 `p->boost_prio` (动态调整) 并经过偏移后，映射到BMQ索引中实时任务之后的位置。
- **ISO**：在 BMQ 队列中不支持，可以使用 nice 为 -20 的 SCHED_NORMAL 策略。

### 2.2 CPU本地队列操作分析

#### 2.2.1 任务创建与初次唤醒

任务的生命周期始于创建（通常通过 `fork()` 或 `clone()` 系统调用），并在其首次变为可运行状态时被调度器接管。在BMQ调度器中，这两个关键的初始化步骤由 `sched_fork()` 和 `wake_up_new_task()` 函数处理。

当一个新任务被创建时，`sched_fork()` 负责进行初步的调度相关的设置。

```c
// kernel/sched/bmq.c
int sched_fork(unsigned long clone_flags, struct task_struct *p)
{
    unsigned long flags;
    struct rq *rq;

    __sched_fork(clone_flags, p); // 1. 执行通用的、与调度类无关的fork初始化
    /*
     * We mark the process as NEW here. This guarantees that
     * nobody will actually run it, and a signal or other external
     * event cannot wake it up and insert it on the runqueue either.
     */
    p->state = TASK_NEW; // 2. 设置新任务状态为 TASK_NEW

    /*
     * Make sure we do not leak PI boosting priority to the child.
     */
    p->prio = current->normal_prio; // 3. 继承父进程的普通优先级作为初始优先级

    /*
     * Revert to default priority/policy on fork if requested.
     */
    if (unlikely(p->sched_reset_on_fork)) { // 4. 如果设置了SCHED_RESET_ON_FORK标志
        if (task_has_rt_policy(p)) {
            p->policy = SCHED_NORMAL;
            p->static_prio = NICE_TO_PRIO(0);
            p->rt_priority = 0;
        } else if (PRIO_TO_NICE(p->static_prio) < 0)
            p->static_prio = NICE_TO_PRIO(0);

        p->prio = p->normal_prio = normal_prio(p); // 重置为SCHED_NORMAL和nice 0

        p->sched_reset_on_fork = 0;
    }

    // 5. 初始化动态优先级调整值 (boost_prio)
    p->boost_prio = (p->boost_prio < 0) ?
        p->boost_prio + MAX_PRIORITY_ADJ : MAX_PRIORITY_ADJ;

    raw_spin_lock_irqsave(&p->pi_lock, flags); // 6. 获取子任务的 pi_lock
    /*
     * Share the timeslice between parent and child, thus the
     * total amount of pending timeslices in the system doesn't change,
     * resulting in more scheduling fairness.
     */
    rq = this_rq(); // 7. 获取当前CPU (父进程所在CPU) 的运行队列
    raw_spin_lock(&rq->lock); // 8. 获取当前CPU运行队列的锁 (rq->lock)
    if (current->time_slice > 1) // BMQ的原始实现中, 父任务时间片大于1才进行分割
        current->time_slice /= 2; // 9. 父任务的时间片减半
    p->time_slice = current->time_slice; // 10. 子任务继承父任务剩余时间片的一半
#ifdef CONFIG_SCHED_HRTICK
    // 如果父任务正在使用高精度定时器，需要为父任务重设计时器
    if (hrtick_enabled(rq) && current == rq->curr && current->time_slice > 1)
        hrtick_start(rq, current->time_slice);
#endif

    // 如果分配的时间片过小，则设置为默认时间片，并标记父进程需要重调度
    if (p->time_slice < RESCHED_NS) {
        p->time_slice = sched_timeslice_ns;
        resched_curr(rq);
    }
    raw_spin_unlock(&rq->lock); // 11. 释放 rq->lock

    /*
     * We're setting the CPU for the first time, we don't migrate,
     * so use __set_task_cpu().
     */
    __set_task_cpu(p, cpu_of(rq)); // 12. 将新任务初始绑定到父进程所在的CPU
    raw_spin_unlock_irqrestore(&p->pi_lock, flags); // 13. 释放子任务的 pi_lock

    // ... (其他非BMQ核心的初始化，如sched_info, preempt_count等) ...
    return 0;
}

// __sched_fork() 是更底层的通用初始化
static inline void __sched_fork(unsigned long clone_flags, struct task_struct *p)
{
    p->on_rq			= 0; // 不在运行队列上
    p->on_cpu			= 0; // 不在CPU上运行
    p->utime			= 0; // 用户态时间
    p->stime			= 0; // 内核态时间
    p->sched_time			= 0; // 总调度时间
    // ...
}
```

当新创建的任务准备好首次运行时，会调用 `wake_up_new_task()` 来将其正式放入运行队列。

```c
// kernel/sched/bmq.c
void wake_up_new_task(struct task_struct *p)
{
    unsigned long flags;
    struct rq *rq;

    raw_spin_lock_irqsave(&p->pi_lock, flags); // 1. 获取新任务的 pi_lock，保护状态变更

    p->state = TASK_RUNNING; // 2. 设置任务状态为 TASK_RUNNING

    rq = cpu_rq(select_task_rq(p)); // 3. 为新任务选择一个目标运行队列 <<--- 关键决策点
#ifdef CONFIG_SMP
    /*
     * Fork balancing, do it here and not earlier because:
     * - cpus_ptr can change in the fork path
     * - any previously selected CPU might disappear through hotplug
     * Use __set_task_cpu() to avoid calling sched_class::migrate_task_rq,
     * as we're not fully set-up yet.
     */
    __set_task_cpu(p, cpu_of(rq)); // 4. 将任务的CPU属主设置为选定rq的CPU
#endif

    raw_spin_lock(&rq->lock); // 5. 获取目标CPU运行队列的锁 (rq->lock)

    update_rq_clock(rq);       // 更新目标rq的时钟
    activate_task(p, rq);      // 6. 激活任务 (内部调用 enqueue_task 将任务加入rq的bmq队列)
    trace_sched_wakeup_new(p); // 调度跟踪事件
    check_preempt_curr(rq);    // 7. 检查是否需要抢占目标CPU上当前正在运行的任务

    raw_spin_unlock(&rq->lock); // 8. 释放目标rq的锁
    raw_spin_unlock_irqrestore(&p->pi_lock, flags); // 9. 释放新任务的 pi_lock
}
```

针对项目目标的现有设计分析：

- `sched_fork()`：修改父任务时间片和为子任务分配时间片的操作在父任务CPU的 `rq->lock` 保护下。
- `wake_up_new_task()`：将任务放入选定的目标CPU队列的操作在目标CPU的 `rq->lock` 保护下。

#### 2.2.2 任务唤醒

当一个已存在但处于睡眠状态的任务被唤醒时，`try_to_wake_up()` (通常由 `wake_up_process()` 调用) 负责将其重新调度。

```c
// kernel/sched/bmq.c
static int try_to_wake_up(struct task_struct *p, unsigned int state,
        int wake_flags)
{
    unsigned long flags; // 用于保存中断状态
    int cpu, success = 0; // cpu: 目标CPU ID; success: 是否成功唤醒

    preempt_disable(); // 1. 禁止抢占，保护后续操作的原子性

    // 2. 特殊情况：唤醒当前任务 (p == current)
    //    这种情况通常发生在当前任务自己改变状态并希望继续运行时。
    //    由于是当前任务，不需要复杂的锁操作，可以直接修改状态。
    if (p == current) {
        if (!(p->state & state)) // 如果任务当前状态不匹配期望被唤醒的状态掩码
            goto out; // 则不进行唤醒

        success = 1;
        cpu = task_cpu(p); // 任务仍在当前CPU
        trace_sched_waking(p); // 跟踪事件：正在唤醒
        p->state = TASK_RUNNING; // 直接设置任务状态为运行
        trace_sched_wakeup(p);  // 跟踪事件：已唤醒
        goto out; // 完成，跳转到末尾
    }

    // 3. 通用情况：唤醒其他任务
    //    需要获取任务的 pi_lock 来保护任务状态 (p->state) 和其他调度相关字段。
    raw_spin_lock_irqsave(&p->pi_lock, flags); // 保存中断状态并禁止中断，然后获取pi_lock
    smp_mb__after_spinlock(); // 确保锁获取后的内存操作顺序

    if (!(p->state & state)) // 再次检查任务状态是否匹配可唤醒状态掩码
        goto unlock; // 如果不匹配，则直接跳转到解锁步骤

    trace_sched_waking(p); // 跟踪事件：正在唤醒

    success = 1; // 标记唤醒成功
    cpu = task_cpu(p); // 获取任务当前（或上次）所在的CPU

    // 4. 处理任务已在运行队列上的情况 (p->on_rq)
    //    smp_rmb() 确保先读取 p->state 再读取 p->on_rq，防止竞态。
    //    如果任务已在队列上 (p->on_rq == 1)，并且 ttwu_remote() 返回true（表示远程唤醒已处理或不需要），
    //    则说明任务只是状态改变，不需要重新入队，直接跳转到解锁。
    smp_rmb(); // 读内存屏障
    if (p->on_rq && ttwu_remote(p, wake_flags))
        goto unlock;

#ifdef CONFIG_SMP
    // 5. SMP环境下的额外同步与决策
    smp_rmb(); // 读内存屏障，确保先读 p->on_rq 再读 p->on_cpu

    // 等待任务完全离开前一个CPU (如果它之前在运行并被换下)
    // smp_cond_load_acquire确保在p->on_cpu变为0（被finish_task设置）之后才继续，
    // 保证了唤醒操作能观察到任务前一次执行的所有状态变更。
    smp_cond_load_acquire(&p->on_cpu, !VAL);

    // 更新任务的负载贡献标记和状态为TASK_WAKING (表示正在被唤醒过程中)
    p->sched_contributes_to_load = !!task_contributes_to_load(p);
    p->state = TASK_WAKING;

    if (p->in_iowait) { // 如果任务之前在等待I/O
        delayacct_blkio_end(p); // 结束块I/O延迟统计
        atomic_dec(&task_rq(p)->nr_iowait); // 减少对应rq的I/O等待计数
    }

    // 6. 优先级提升逻辑 (boost_task)
    //    如果任务从上次运行到现在的间隔超过一个时间片，则提升其动态优先级。
    //    这是为了奖励那些长时间睡眠后被唤醒的交互性任务。
    if(this_rq()->clock_task - p->last_ran > sched_timeslice_ns)
        boost_task(p);

    // 7. 选择目标CPU (`select_task_rq`) <<--- 核心决策点
    //    调用 select_task_rq 为任务 p 选择一个最合适的CPU运行队列。
    cpu = select_task_rq(p);

    // 8. 处理CPU迁移 (如果选定的CPU与任务当前CPU不同)
    if (cpu != task_cpu(p)) {
        wake_flags |= WF_MIGRATED; // 标记唤醒涉及迁移
        psi_ttwu_dequeue(p);       // PSI相关的出队操作（从旧CPU角度）
        set_task_cpu(p, cpu);      // 将任务的CPU属主更新为新的目标CPU `cpu`
    }
#else // 单核 (UP) 环境
    if (p->in_iowait) {
        delayacct_blkio_end(p);
        atomic_dec(&task_rq(p)->nr_iowait);
    }
#endif /* CONFIG_SMP */

    // 9. 将任务放入选定CPU的运行队列
    //    ttwu_queue 会获取目标CPU的 rq->lock，然后调用 activate_task -> enqueue_task
    ttwu_queue(p, cpu, wake_flags);

unlock: // 解锁标签
    raw_spin_unlock_irqrestore(&p->pi_lock, flags); // 释放pi_lock并恢复中断状态
out:    // 末尾标签
    if (success) // 如果唤醒成功
        ttwu_stat(p, cpu, wake_flags); // 更新调度统计
    preempt_enable(); // 恢复抢占（与函数开头的 preempt_disable() 对应）

    return success;
}

// ttwu_remote 辅助函数，用于处理任务已在队列上的远程唤醒情况
static int ttwu_remote(struct task_struct *p, int wake_flags)
{
    struct rq *rq;
    raw_spinlock_t *lock;
    int ret = 0;

    // 尝试安全地获取任务所在rq的锁
    rq = __task_access_lock(p, &lock); // 注意：这里获取的是任务当前所在rq的锁
    if (task_on_rq_queued(p)) { // 如果任务确实还在这个队列上
        ttwu_do_wakeup(rq, p, wake_flags); // 仅将其状态设置为 TASK_RUNNING
        ret = 1;
    }
    __task_access_unlock(p, lock); // 释放获取的锁

    return ret;
}
```

针对项目目标的现有设计分析：

- 这些复杂的同步机制是为了在**现有锁模型**下保证唤醒的正确性。如果ELF BMQ引入了新的锁机制（如全局锁）或无锁操作，这些同步点可能需要重新审视。
- 例如，如果任务从全局队列被唤醒并迁移到本地无锁队列，那么从全局队列取任务（受全局锁保护）到放入本地无锁队列（无锁操作）的过程，需要新的同步协议来保证状态的正确流转。

```c
// kernel/sched/bmq.c
static void ttwu_queue(struct task_struct *p, int cpu, int wake_flags)
{
    struct rq *rq = cpu_rq(cpu); // 获取目标CPU的运行队列

    raw_spin_lock(&rq->lock);   // <<--- 获取运行队列锁
    update_rq_clock(rq);
    ttwu_do_activate(rq, p, wake_flags); // 调用激活函数
    check_preempt_curr(rq);     // 检查是否需要抢占当前任务
    raw_spin_unlock(&rq->lock); // <<--- 释放运行队列锁
}

static inline void ttwu_do_activate(struct rq *rq, struct task_struct *p, int wake_flags)
{
    // ... (一些统计和状态更新) ...
    activate_task(p, rq); // 激活任务
    ttwu_do_wakeup(rq, p, 0); // 标记任务为 TASK_RUNNING
}

static void activate_task(struct task_struct *p, struct rq *rq)
{
    if (task_contributes_to_load(p))
        rq->nr_uninterruptible--; // 如果任务贡献负载，则减少不可中断计数
    enqueue_task(p, rq, ENQUEUE_WAKEUP); // 调用 enqueue_task
    p->on_rq = TASK_ON_RQ_QUEUED;     // 标记任务已在运行队列上
    cpufreq_update_util(rq, 0);       // 更新CPU频率相关的利用率
}
```

这里存在一个重要的辅助函数 `select_task_rq`，会为新任务选择一个“最佳”的初始队列。

```c
static inline int select_task_rq(struct task_struct *p)
{
    cpumask_t chk_mask, tmp; // 用于CPU掩码计算的临时变量

    // 1. 初步筛选：获取任务p允许运行的CPU集合 (p->cpus_ptr) 与当前所有在线CPU (cpu_online_mask) 的交集。
    //    结果存入 chk_mask。如果交集为空（即任务允许运行的CPU当前都不在线），
    //    则调用 select_fallback_rq 寻找一个备选的CPU。
    if (unlikely(!cpumask_and(&chk_mask, p->cpus_ptr, cpu_online_mask)))
        return select_fallback_rq(task_cpu(p), p); // task_cpu(p) 是之前绑定的父进程所在的 CPU

    // 2. 启发式选择：尝试寻找“更优”的CPU。
    //    这个if语句块通过一系列的`cpumask_and`操作，试图找到满足特定“空闲”或“匹配”条件的CPU集合。
    if (
#ifdef CONFIG_SCHED_SMT
        // 条件a: 在允许的在线CPU中 (chk_mask)，是否存在CPU其SMT siblings整体空闲 (sched_sg_idle_mask)？
        cpumask_and(&tmp, &chk_mask, &sched_sg_idle_mask) ||
#endif
        // 条件b: 在允许的在线CPU中 (chk_mask)，是否存在CPU其运行队列处于IDLE水位 (sched_rq_watermark[IDLE_WM])？
        // IDLE_WM 表示该CPU上只有idle任务或非常低优先级的任务。
        cpumask_and(&tmp, &chk_mask, &sched_rq_watermark[IDLE_WM]) ||
        // 条件c: 在允许的在线CPU中 (chk_mask)，是否存在CPU其运行队列的水位线暗示着它可以较好地接纳当前任务？
        // 这里比较的是任务p计算出的调度优先级 (task_sched_prio(p)) 加1之后对应的水位线。
        // sched_rq_watermark[prio]表示哪些CPU上当前运行的最高优先级任务是prio或更低。
        // 检查 prio+1 是为了寻找那些当前最高优先级任务比p略低（或更低）的CPU。
        cpumask_and(&tmp, &chk_mask,
                &sched_rq_watermark[task_sched_prio(p) + 1]))
        // 如果上述任一条件为真，并且找到了这样的CPU集合 (结果在tmp中)，
        // 则调用 best_mask_cpu 从tmp中选择一个最优的CPU。
        // best_mask_cpu会考虑任务p当前所在的CPU (task_cpu(p))，优先选择它（如果它在tmp中），
        // 否则会根据CPU拓扑距离从tmp中选择一个最近的。
        return best_mask_cpu(task_cpu(p), &tmp);

    // 3. 常规选择：如果以上启发式条件都不满足（或者没有找到符合条件的CPU），
    //    则直接从任务允许运行且在线的CPU集合 (chk_mask) 中选择一个最优的CPU。
    //    同样，best_mask_cpu 会优先考虑任务当前的CPU。
    return best_mask_cpu(task_cpu(p), &chk_mask);
}

// 辅助函数 select_fallback_rq (当任务允许的CPU都不在线时调用)
static int select_fallback_rq(int cpu, struct task_struct *p)
{
    // ... (逻辑较为复杂，大致是：)
    // 1. 尝试在任务当前CPU所在NUMA节点中找一个允许的、活跃的CPU。
    // 2. 如果找不到，则在任务所有允许的CPU中（p->cpus_ptr）寻找任何一个 is_cpu_allowed 的CPU。
    // 3. 如果还找不到（可能cpuset设置导致），则尝试放宽cpuset限制（cpuset_cpus_allowed_fallback）。
    // 4. 再找不到，则将任务的允许CPU设置为所有可能的CPU（cpu_possible_mask）。
    // 5. 最终一定会找到一个CPU，否则BUG()。
    // ...
    // (返回找到的 dest_cpu)
}

// 辅助函数 best_mask_cpu (在 bmq_sched.h 中)
static inline int best_mask_cpu(int cpu, const struct cpumask *cpumask)
{
    // 如果cpu在cpumask中，则直接返回cpu (保持缓存热度)
    if (cpumask_test_cpu(cpu, cpumask))
        return cpu;
    // 否则，从cpumask中根据预设的CPU亲和力拓扑级别 (sched_cpu_affinity_masks) 寻找一个最优的CPU
    return __best_mask_cpu(cpu, cpumask, &(per_cpu(sched_cpu_affinity_masks, cpu)[0]));
}
static inline int __best_mask_cpu(int cpu, const struct cpumask *cpumask,
                  const cpumask_t *mask)
{
    // 遍历 per_cpu(sched_cpu_affinity_masks, cpu) 中的各级亲和掩码
    // 找到第一个与 cpumask 有交集的亲和掩码，并返回交集中的第一个CPU
    while ((cpu = cpumask_any_and(cpumask, mask)) >= nr_cpu_ids)
        mask++;
    return cpu;
}
```

针对项目目标的现有设计分析：

- 当前 `select_task_rq` 总是尝试将新任务放置到某个CPU的本地队列。如果所有本地CPU都繁忙，新任务仍然会加入某个本地队列。
- 引入全局运行队列后，那么在 `select_task_rq` 中，当判断所有本地CPU都不适合立即放置新任务时（例如都高度繁忙），可以将新任务放入全局运行队列。这就需要 `select_task_rq` 的逻辑能够与全局队列交互，并受到“优先级级别的全局运行队列锁”的保护。

#### 2.2.3 任务入队

当一个任务变成可运行时（例如，从睡眠中唤醒或刚创建），调度器会根据其调度策略和优先级计算出进入到 BMQ 的哪个优先级队列，然后任务会添加到对应链表中，并且在位图中设置该优先级对应位为 1，表示该队列不为空。

```c
static inline void enqueue_task(struct task_struct *p, struct rq *rq, int flags)
{
    lockdep_assert_held(&rq->lock); // 确认调用时已持有 rq->lock

    WARN_ON_ONCE(task_rq(p) != rq, "bmq: enqueue task reside on cpu%d to cpu%d\n",
                 task_cpu(p), cpu_of(rq));

    __enqueue_task(p, rq, flags); // 调用实际的入队逻辑
    update_sched_rq_watermark(rq); // 更新运行队列的水位标记
    ++rq->nr_running;              // 增加运行队列中的可运行任务数量
#ifdef CONFIG_SMP
    if (2 == rq->nr_running) // 如果有两个任务了，标记该CPU可能有待处理任务 (用于负载均衡)
        cpumask_set_cpu(cpu_of(rq), &sched_rq_pending_mask);
#endif

    sched_update_tick_dependency(rq); // 更新调度tick依赖

    if (p->in_iowait) // 如果任务之前在iowait状态
        cpufreq_update_util(rq, SCHED_CPUFREQ_IOWAIT);
}

static inline void __enqueue_task(struct task_struct *p, struct rq *rq, int flags)
{
    sched_info_queued(rq, p); // 调度信息统计
    psi_enqueue(p, flags);    // PSI 相关入队操作

    p->bmq_idx = task_sched_prio(p); // 计算任务在BMQ中的优先级索引
    list_add_tail(&p->bmq_node, &rq->queue.heads[p->bmq_idx]); // 将任务节点加入到对应优先级链表的尾部
    set_bit(p->bmq_idx, rq->queue.bitmap); // 在位图中标记该优先级队列非空
}
```

#### 2.2.4 任务出队

`dequeue_task` 函数负责将一个任务从其所在的运行队列中移除。它通常在任务不再可运行时被调用。`deactivate_task` (在 `__schedule` 中任务主动睡眠时调用) 和 `move_queued_task` (在任务迁移时调用) 都会间接或直接调用 `dequeue_task`。

```c
// kernel/sched/bmq.c

// 在 __schedule 中，当任务 prev 主动阻塞时：
// deactivate_task(prev, rq);

static inline void deactivate_task(struct task_struct *p, struct rq *rq)
{
    if (task_contributes_to_load(p))
        rq->nr_uninterruptible++;
    dequeue_task(p, rq, DEQUEUE_SLEEP); // 调用 dequeue_task
    p->on_rq = 0; // 标记任务已不在运行队列上
    cpufreq_update_util(rq, 0);
}

// 实际的 dequeue_task 实现
static inline void dequeue_task(struct task_struct *p, struct rq *rq, int flags)
{
    lockdep_assert_held(&rq->lock); // 确认调用时已持有 rq->lock

    WARN_ON_ONCE(task_rq(p) != rq, "bmq: dequeue task reside on cpu%d from cpu%d\n",
                 task_cpu(p), cpu_of(rq));

    psi_dequeue(p, flags & DEQUEUE_SLEEP); // PSI 相关出队操作
    sched_info_dequeued(rq, p);           // 调度信息统计

    list_del(&p->bmq_node); // 1. 从其所在的优先级链表中移除任务节点 <<--- 核心操作
    if (list_empty(&rq->queue.heads[p->bmq_idx])) { // 2. 如果移除后该优先级链表为空
        clear_bit(p->bmq_idx, rq->queue.bitmap); // 3. 则在位图中清除对应优先级的标记 <<--- 核心操作
        update_sched_rq_watermark(rq);           // 4. 更新运行队列的水位标记
    }
    --rq->nr_running; // 5. 减少运行队列中的可运行任务数量 <<--- 核心操作

#ifdef CONFIG_SMP
    // 6. 如果可运行任务只剩下一个 (通常是idle任务或下一个要运行的任务)，
    //    清除该CPU在sched_rq_pending_mask中的标记 (用于负载均衡)
    if (1 == rq->nr_running)
        cpumask_clear_cpu(cpu_of(rq), &sched_rq_pending_mask);
#endif

    sched_update_tick_dependency(rq); // 更新调度tick依赖
}

// 还有一个 __dequeue_task，用于某些特殊场景（如迁移时先dequeue再enqueue，避免nr_running等重复更新）
// 它只做最核心的 list_del 和 clear_bit
static inline void __dequeue_task(struct task_struct *p, struct rq *rq, int flags)
{
    psi_dequeue(p, flags & DEQUEUE_SLEEP);
    sched_info_dequeued(rq, p);

    list_del(&p->bmq_node);
    if (list_empty(&rq->queue.heads[p->bmq_idx]))
        clear_bit(p->bmq_idx, rq->queue.bitmap);
}
```

#### 2.2.5 任务重新入队

在BMQ调度器中，`requeue_task()`函数负责将一个已经在运行队列上的任务重新放置到其优先级链表的尾部，或者在任务优先级发生变化时将其移动到新的优先级链表中。这个操作对于实现时间片轮转（针对非SCHED_FIFO/SCHED_RR策略）和响应任务动态优先级调整至关重要。

```c
// kernel/sched/bmq.c
static inline void requeue_task(struct task_struct *p, struct rq *rq)
{
    int idx = task_sched_prio(p); // 1. 重新计算任务p当前的调度优先级索引

    lockdep_assert_held(&rq->lock); // 2. 断言调用时已持有 rq->lock
    WARN_ONCE(task_rq(p) != rq, "bmq: cpu[%d] requeue task reside on cpu%d\n",
              cpu_of(rq), task_cpu(p)); // 确认任务p确实属于当前rq

    // 3. 从任务p当前所在的（旧）优先级链表中移除
    list_del(&p->bmq_node);

    // 4. 将任务p添加到其新计算出的优先级idx对应的链表的尾部
    list_add_tail(&p->bmq_node, &rq->queue.heads[idx]);

    // 5. 如果任务的优先级索引发生了变化 (idx != p->bmq_idx)
    if (idx != p->bmq_idx) {
        // 5a. 如果任务之前所在的旧优先级队列 (p->bmq_idx) 因此变为空
        if (list_empty(&rq->queue.heads[p->bmq_idx]))
            clear_bit(p->bmq_idx, rq->queue.bitmap); // 则清除旧优先级在bitmap中的标记
        
        p->bmq_idx = idx; // 5b. 更新任务结构体中存储的优先级索引为新的索引idx
        
        set_bit(p->bmq_idx, rq->queue.bitmap); // 5c. 在bitmap中设置新优先级idx的标记 (表示该队列非空)
        
        update_sched_rq_watermark(rq); // 5d. 由于bitmap可能发生变化，更新rq的水位标记
    }
    // 如果idx == p->bmq_idx (优先级未变)，则任务只是被移到同优先级链表的尾部，
    // bitmap和p->bmq_idx本身不需要改变，watermark通常也不需要更新（除非这是唯一的非空队列且之前它是空的，但这种情况比较边缘）。
}
```

针对项目目标的现有设计分析：

- **现有问题：**当前的入队、出队和重新入队都依赖于 `rq->lock` 来保护位图和链表的修改，导致这些操作在 CPU 内部串行化。
- 重新入队操作是最具挑战性的操作之一。它本质上是一个“原子移动”操作：任务需要从一个无锁队列（可能在旧优先级）中移除，并加入到另一个无锁队列（新优先级）中，同时需要原子地更新位图中可能变化的两位（旧优先级位可能变0，新优先级位可能变1）。如果任务的优先级变化也影响其在全局队列中的状态或位置，那么重新入队操作可能还需要与全局队列及其锁进行交互。

### 2.3 核心调度决策与执行

当CPU需要选择下一个要运行的任务时，或者当前任务因特定原因（如阻塞、时间片耗尽）需要放弃CPU时，调度器会介入执行核心的调度决策。这一过程主要围绕 `schedule()` 函数及其核心实现 `__schedule()` 展开，涉及到从运行队列中选择最优任务、执行上下文切换等关键步骤。这些操作的正确性和效率直接关系到系统的响应速度和吞吐量。

#### 2.3.1 任务调度与选择

当任务主动放弃CPU（例如调用 `sleep()` 或等待锁）或被动抢占（例如时间片耗尽、更高优先级任务就绪）时，内核会调用 `schedule()` 函数（或其变体如 `preempt_schedule`）来触发一次调度。`schedule()` 本身是一个上层封装，它会调用核心的 `__schedule()` 函数来执行实际的调度逻辑。

```c
// kernel/sched/bmq.c

asmlinkage __visible void __sched schedule(void)
{
    struct task_struct *tsk = current;

    sched_submit_work(tsk); // 处理worker队列相关工作，以及可能的IO提交
    do {
        preempt_disable();    // 禁止抢占
        __schedule(false);    // 调用核心调度逻辑，false表示非抢占式调用
        sched_preempt_enable_no_resched(); // 尝试重新启用抢占，但不立即重调度
    } while (need_resched()); // 如果调度后仍然需要重调度，则循环
    sched_update_worker(tsk); // 更新worker状态
}

```

`__schedule` 函数是 Linux 内核中负责执行任务切换的**核心函数**。它在需要从当前任务切换到另一个可运行任务时被调用。

```c
// kernel/sched/bmq.c

static void __sched notrace __schedule(bool preempt) // preempt参数指示是否为抢占调用
{
    struct task_struct *prev, *next;
    unsigned long *switch_count;
    struct rq *rq;
    int cpu;

    cpu = smp_processor_id(); // 获取当前CPU ID
    rq = cpu_rq(cpu);         // 获取当前CPU的运行队列
    prev = rq->curr;          // prev 指向当前正在运行的任务

    // ... (schedule_debug, hrtick_clear) ...

    local_irq_disable();      // 关闭本地中断，保护后续操作
    rcu_note_context_switch(preempt);

    raw_spin_lock(&rq->lock); // <<--- 获取运行队列锁
    smp_mb__after_spinlock(); // 内存屏障

    update_rq_clock(rq);      // 更新运行队列时钟

    switch_count = &prev->nivcsw; // 默认指向非自愿上下文切换计数器
    if (!preempt && prev->state) { // 如果不是抢占且任务有状态 (通常意味着任务要睡眠)
        if (signal_pending_state(prev->state, prev)) { // 检查是否有信号可以唤醒任务
            prev->state = TASK_RUNNING; // 如果有，任务保持运行
        } else {
            // 任务主动放弃CPU (睡眠)
            if (rq_switch_time(rq) < boost_threshold(prev)) // 如果任务运行时间很短就放弃了
                boost_task(prev); // 提升其优先级 (鼓励交互性)
            deactivate_task(prev, rq); // 将任务从运行队列中移除 <<--- 关键点: 出队操作
            // ... (IO等待处理) ...
        }
        switch_count = &prev->nvcsw; // 指向自愿上下文切换计数器
    }

    clear_tsk_need_resched(prev); // 清除任务的 TIF_NEED_RESCHED 标志
    clear_preempt_need_resched(); // 清除CPU的抢占标志

    check_curr(prev, rq); // 检查当前任务 prev 的时间片，如果耗尽则重置并可能降级、重新入队

    next = choose_next_task(rq, cpu, prev); // <<--- 选择下一个要运行的任务

    if (likely(prev != next)) { // 如果选出的任务与当前任务不同
        next->last_ran = rq->clock_task;
        rq->last_ts_switch = rq->clock;

        rq->nr_switches++; // 增加上下文切换次数
        RCU_INIT_POINTER(rq->curr, next); // 将 rq->curr 指向新任务 next
        ++*switch_count; // 增加相应的上下文切换计数

        // ... (psi_sched_switch, trace_sched_switch) ...

        rq = context_switch(rq, prev, next); // <<--- 执行上下文切换，此函数会释放 rq->lock
                                             // 并返回新的 this_rq() (通常是同一个，除非有特殊迁移)
    } else { // 如果选出的任务与当前任务相同 (没有其他更合适的任务)
        raw_spin_unlock_irq(&rq->lock); // 直接释放锁 (保持中断关闭状态)
    }

#ifdef CONFIG_SCHED_SMT
    sg_balance_check(rq); // SMT相关的负载均衡检查
#endif
}
```

当前的设计分析：

- `__schedule()`在绝大部分核心逻辑执行期间都持有 `rq->lock`。

`choose_next_task` 是负责在当前 CPU 的运行队列中找到下一个最合适运行的任务。

```c
// kernel/sched/bmq.c

// 辅助函数：找到运行队列中第一个可运行的任务
static inline struct task_struct *rq_first_bmq_task(struct rq *rq)
{
    unsigned long idx = find_first_bit(rq->queue.bitmap, bmq_BITS); // 1. 在位图中查找第一个被设置的位（最低索引，代表最高优先级）
    const struct list_head *head = &rq->queue.heads[idx];          // 2. 获取对应优先级链表的头指针

    return list_first_entry(head, struct task_struct, bmq_node);   // 3. 返回该链表中的第一个任务
}

// 辅助函数：找到指定任务p在运行队列中的下一个可运行任务
static inline struct task_struct *
rq_next_bmq_task(struct task_struct *p, struct rq *rq)
{
    unsigned long idx = p->bmq_idx; // 获取任务p当前的优先级索引
    struct list_head *head = &rq->queue.heads[idx]; // 获取p所在优先级链表的头

    if (list_is_last(&p->bmq_node, head)) { // 如果p是其所在优先级链表的最后一个任务
        idx = find_next_bit(rq->queue.bitmap, bmq_BITS, idx + 1); // 从下一个优先级开始查找非空队列
        head = &rq->queue.heads[idx]; // 获取新找到的优先级链表的头

        return list_first_entry(head, struct task_struct, bmq_node); // 返回新队列的第一个任务
    }

    return list_next_entry(p, bmq_node); // 否则，返回p在当前优先级链表中的下一个任务
}

// 辅助函数：找到运行队列中可运行的任务，会跳过rq->skip指定的任务
static inline struct task_struct *rq_runnable_task(struct rq *rq)
{
    struct task_struct *next = rq_first_bmq_task(rq); // 先找第一个（最高优先级的第一个）

    if (unlikely(next == rq->skip)) // 如果找到的是要被跳过的任务
        next = rq_next_bmq_task(next, rq); // 则找它之后的下一个任务

    return next;
}

// choose_next_task 主函数
static inline struct task_struct *
choose_next_task(struct rq *rq, int cpu, struct task_struct *prev)
{
    struct task_struct *next;

    if (unlikely(rq->skip)) { // 1. 处理 rq->skip 的情况 (通常由 sched_yield 设置)
        next = rq_runnable_task(rq); // 找到可运行任务，跳过 skip 任务
        if (next == rq->idle) { // 如果除了skip任务和idle任务外，没有其他任务
#ifdef CONFIG_SMP
            if (!take_other_rq_tasks(rq, cpu)) { // 尝试从其他CPU窃取任务
#endif
                // 没有窃取到任务，或者是非SMP情况
                rq->skip = NULL; // 清除 skip 标记
                schedstat_inc(rq->sched_goidle); // 统计进入idle状态
                return next; // 返回 idle 任务
#ifdef CONFIG_SMP
            }
            // 窃取到任务了
            next = rq_runnable_task(rq); // 重新获取可运行任务 (此时队列可能非空了)
#endif
        }
        rq->skip = NULL; // 清除 skip 标记
#ifdef CONFIG_SCHED_HRTICK // 注意，源码中使用的是 CONFIG_SCHED_HRTICK，而非文档中的CONFIG_HIGH_RES_TIMERS
        hrtick_start(rq, next->time_slice); // 为选中的任务启动高精度抢占定时器
#endif
        return next;
    }

    // 2. 常规情况：rq->skip 为 NULL
    next = rq_first_bmq_task(rq); // 直接从位图队列中找最高优先级的第一个任务
    if (next == rq->idle) { // 如果本地队列只有idle任务
#ifdef CONFIG_SMP
        if (!take_other_rq_tasks(rq, cpu)) { // 尝试从其他CPU窃取任务
#endif
            // 没有窃取到任务，或者是非SMP情况
            schedstat_inc(rq->sched_goidle); // 统计进入idle状态
            return next; // 返回 idle 任务
#ifdef CONFIG_SMP
        }
        // 窃取到任务了
        next = rq_first_bmq_task(rq); // 再次从本地队列获取任务 (现在应该有实际任务了)
#endif
    }
#ifdef CONFIG_SCHED_HRTICK
    hrtick_start(rq, next->time_slice); // 为选中的任务启动高精度抢占定时器
#endif
    return next;
}
```

针对项目目标的现有设计分析：

- `__schedule` 函数的核心逻辑几乎完全在 `rq->lock` 的保护下执行，包括任务出队 (`deactivate_task`)、时间片检查与重入队 (`check_curr` -> `requeue_task`)、选择下一个任务 (`choose_next_task` 内部的 `rq_first_bmq_task` 等访问BMQ结构) 以及更新 `rq->curr`。`context_switch` 最终会释放这个锁。`choose_next_task` 内部调用的 `take_other_rq_tasks` 会尝试获取其他CPU的 `rq->lock`。

#### 2.3.2 上下文切换

当 `__schedule()` 选定了与当前任务不同的下一个任务后，`context_switch()` 函数负责执行实际的CPU状态切换。这包括保存当前任务的执行环境（寄存器、栈指针等）并加载下一个任务的执行环境。

```c
// kernel/sched/bmq.c
static __always_inline struct rq *
context_switch(struct rq *rq, struct task_struct *prev,
           struct task_struct *next)
{
    // 1. 任务切换前的准备工作
    prepare_task_switch(rq, prev, next);

    /*
     * For paravirt, this is coupled with an exit in switch_to to
     * combine the page table reload and the switch backend into
     * one hypercall.
     */
    arch_start_context_switch(prev);
  
    // 例如，如果 next 是内核线程 (next->mm == NULL)，则可能进入 lazy TLB 模式；
    // 如果 next 是用户线程，则可能需要切换地址空间 (switch_mm)。
    // BMQ的实现与标准调度器类似，会处理 active_mm 的切换和 mmdrop 的延迟处理。
    if (!next->mm) { // to kernel
        enter_lazy_tlb(prev->active_mm, next);
        next->active_mm = prev->active_mm;
        if (prev->mm) // from user
            mmgrab(prev->active_mm);
        else
            prev->active_mm = NULL;
    } else { // to user
        membarrier_switch_mm(rq, prev->active_mm, next->mm);
        switch_mm_irqs_off(prev->active_mm, next->mm, next);
        if (!prev->mm) { // from kernel
            rq->prev_mm = prev->active_mm; // 延迟 prev_mm 的 mmdrop 到 finish_task_switch
            prev->active_mm = NULL;
        }
    }

    // 2. 准备锁的“传递” (主要用于Lockdep)
    prepare_lock_switch(rq, next);

    // 3. 实际的CPU状态切换 (架构相关的汇编代码)
    //    保存 prev 任务的寄存器状态到其栈中，加载 next 任务的寄存器状态。
    //    切换栈指针。CPU的执行流从这里“跳”到 next 任务上次调用 schedule() 时的返回点。
    //    注意：switch_to 宏/函数执行时，本地中断仍然是关闭的 (由 __schedule() 开头的 local_irq_disable() 关闭)。
    switch_to(prev, next, prev); 
    barrier(); // 编译器屏障，防止指令重排跨越 switch_to

    // 4. 任务切换后的清理工作 (在新任务 next 的上下文中执行)
    //    finish_task_switch 会负责释放 rq->lock 并开启中断。
    return finish_task_switch(prev);
}
```

- `rq->lock` 在 `__schedule()` 开始时获取，并在 `context_switch()` 内部，具体是在 `finish_task_switch()` 中的 `finish_lock_switch()` 里通过 `raw_spin_unlock_irq(&rq->lock)` 释放。这意味着从选择下一个任务到实际完成CPU状态切换并准备在新任务上运行的整个关键路径，都受到 `rq->lock` 的保护。

#### 2.3.3 时间片管理与抢占

BMQ调度器通过周期性的调度器tick (`scheduler_tick`) 来管理任务的时间片，并在必要时触发抢占。

周期性调度器 Tick `scheduler_tick`：

```c
// kernel/sched/bmq.c
void scheduler_tick(void)
{
    int cpu __maybe_unused = smp_processor_id();
    struct rq *rq = cpu_rq(cpu);

    // ... (架构相关的频率调整，全局调度时钟tick) ...
    arch_scale_freq_tick();
    sched_clock_tick();

    raw_spin_lock(&rq->lock); // 1. 获取本地运行队列锁
    update_rq_clock(rq);      // 2. 更新运行队列时钟

    scheduler_task_tick(rq);  // 3. 处理当前任务的时间片等
    calc_global_load_tick(rq); // 4. 更新全局负载统计 (BMQ中此函数实现可能较简单)
    psi_task_tick(rq);         // PSI相关的tick处理

    rq->last_tick = rq->clock; // 记录本次tick的时间
    raw_spin_unlock(&rq->lock); // 5. 释放运行队列锁

    perf_event_task_tick(); // perf事件
}
```

处理当前任务的 Tick `scheduler_task_tick`：

```c
// kernel/sched/bmq.c
static inline void scheduler_task_tick(struct rq *rq)
{
    struct task_struct *p = rq->curr;

    if (is_idle_task(p)) // Idle任务不参与时间片计算
        return;

    update_curr(rq, p); // 1. 更新当前任务的已运行时间，并消耗其时间片
    cpufreq_update_util(rq, 0); // 更新CPU频率利用率

    /*
     * Tasks have less than RESCHED_NS of time slice left they will be
     * rescheduled. (如果剩余时间片小于 RESCHED_NS，则标记重调度)
     */
    if (p->time_slice >= RESCHED_NS) // 2. 如果时间片仍然充足
        return;
    
    // 3. 时间片不足，标记任务p需要重调度，并标记CPU需要抢占
    set_tsk_need_resched(p); 
    set_preempt_need_resched();
}
```

更新当前任务运行时间`update_curr`：

```c
// kernel/sched/bmq.c
static inline void update_curr(struct rq *rq, struct task_struct *p)
{
    s64 ns = rq->clock_task - p->last_ran; // 计算自上次更新以来的运行时间

    p->sched_time += ns; // 累加到任务的总调度时间
    account_group_exec_runtime(p, ns); // cgroup相关的运行时间统计

    p->time_slice -= ns; // 从任务的剩余时间片中扣除
    p->last_ran = rq->clock_task; // 更新任务上次运行的时间戳为当前rq的任务时钟
}
```



### 2.4  SMP环境下的协作与任务迁移

在SMP（对称多处理）系统中，负载均衡是确保各个CPU的负载大致相当，从而提高系统整体吞吐量和响应性的关键机制。BMQ调度器中，当一个CPU的运行队列变空（或接近空闲）时，它会尝试从其他繁忙的CPU“窃取”任务来执行。这个过程主要由 `take_other_rq_tasks` 和 `migrate_pending_tasks` 两个函数协同完成。

#### 2.4.1 负载均衡机制

此函数由一个任务较少（通常是即将idle）的CPU（目标CPU `cpu`，其运行队列为 `rq`）调用，目的是从其他可能繁忙的CPU（源CPU）上拉取任务。

```c
// kernel/sched/bmq.c
#ifdef CONFIG_SMP
// 全局位图，标记 nr_running > 1 的CPU，这些CPU的任务队列是可被窃取者
static cpumask_t sched_rq_pending_mask ____cacheline_aligned_in_smp;

// Per-CPU的CPU亲和力掩码数组，用于指导负载均衡时优先从哪些CPU拉取任务
// sched_cpu_affinity_masks[level] 定义了不同拓扑层级的亲和CPU集合
DEFINE_PER_CPU(cpumask_t [NR_CPU_AFFINITY_CHK_LEVEL], sched_cpu_affinity_masks);
DEFINE_PER_CPU(cpumask_t *, sched_cpu_affinity_end_mask); // 指向亲和力掩码数组的末尾

static inline int take_other_rq_tasks(struct rq *rq, int cpu) // rq是目标队列，cpu是目标CPU的ID
{
    struct cpumask *affinity_mask, *end_mask;

    if (unlikely(!rq->online)) // 如果目标CPU的rq不在线，则不进行操作
        return 0;

    // 如果 sched_rq_pending_mask 为空，说明没有CPU报告自己有多个待处理任务，直接返回
    if (cpumask_empty(&sched_rq_pending_mask))
        return 0;

    // 获取目标CPU的亲和力掩码遍历起点和终点
    affinity_mask = &(per_cpu(sched_cpu_affinity_masks, cpu)[0]);
    end_mask = per_cpu(sched_cpu_affinity_end_mask, cpu);

    do { // 遍历不同级别的亲和力掩码 (例如：先看同核心的SMT线程，再看同物理核心的其他逻辑CPU，再看同NUMA节点的等)
        int i;
        // for_each_cpu_and：遍历那些 *同时* 存在于 sched_rq_pending_mask 和当前 affinity_mask 中的CPU
        // 这些CPU是既有任务可供拉取，又与目标CPU有一定亲和关系的源CPU
        for_each_cpu_and(i, &sched_rq_pending_mask, affinity_mask) {
            int nr_migrated;
            struct rq *src_rq;

            src_rq = cpu_rq(i); // 获取源CPU i 的运行队列
            // 尝试获取源运行队列的锁，使用 trylock 避免长时间阻塞造成死锁或性能下降。
            if (!do_raw_spin_trylock(&src_rq->lock))
                continue; // 如果获取锁失败, 跳过这个源CPU, 尝试下一个
            
            // Lockdep相关的断言和记录 (如果获取锁成功)
            spin_acquire(&src_rq->lock.dep_map, SINGLE_DEPTH_NESTING, 1, _RET_IP_);

            // 调用 migrate_pending_tasks 尝试从源运行队列 src_rq 向目标运行队列 rq 迁移任务
            if ((nr_migrated = migrate_pending_tasks(src_rq, rq, cpu))) {
                // 如果成功迁移到任务 (nr_migrated > 0)
                // 更新源运行队列的状态
                src_rq->nr_running -= nr_migrated; // 减少源队列的可运行任务计数
                if (src_rq->nr_running < 2) // 如果源队列任务少于2个, 清除其在 sched_rq_pending_mask 中的标记
                    cpumask_clear_cpu(i, &sched_rq_pending_mask);

                // 更新当前(目标)运行队列的状态
                rq->nr_running += nr_migrated; // 增加目标队列的可运行任务计数
                if (rq->nr_running > 1) // 如果目标队列任务多于1个, 设置其在 sched_rq_pending_mask 中的标记
                    cpumask_set_cpu(cpu, &sched_rq_pending_mask);
                
                update_sched_rq_watermark(rq); // 更新目标队列的水位线
                cpufreq_update_util(rq, 0);    // 更新 CPU频率相关的利用率信息

                // 释放源运行队列的锁,并返回成功标志(1)
                spin_release(&src_rq->lock.dep_map, _RET_IP_);
                do_raw_spin_unlock(&src_rq->lock);
                return 1; // 成功窃取到任务, 返回 1
            }

            // 如果没有从这个 src_rq 迁移到任务, 也要释放其锁
            spin_release(&src_rq->lock.dep_map, _RET_IP_);
            do_raw_spin_unlock(&src_rq->lock);
        }
    } while (++affinity_mask < end_mask); // 继续检查下一组(更远亲和关系的)CPU掩码

    return 0; // 遍历完所有亲和性相关的CPU都没有窃取到任务, 返回0
}
#endif
```

当前的设计分析：

- 这里涉及的同步机制是尝试获取其他CPU的运行队列锁。这里有一个跨 CPU 的锁获取操作，为了放置死锁和性能问题，如果获取要窃取的任务队列锁失败的话，就会放弃这个CPU，选择下一个。

实际的任务选择和迁移逻辑在 `migrate_pending_tasks` 中完成。

```c
// kernel/sched/bmq.c
#ifdef CONFIG_SMP
#define SCHED_RQ_NR_MIGRATION (32UL) // 单次迁移尝试的最大任务数

static inline int
migrate_pending_tasks(struct rq *src_rq, struct rq *dest_rq, const int dest_cpu)
{
    struct task_struct *p, *skip = src_rq->curr; // p:当前考虑迁移的任务, skip:遍历辅助指针,从源队列的当前任务后开始
    int nr_migrated = 0; // 记录成功迁移的任务数量

    // 计算最多尝试迁移的任务数量: 源队列可运行任务数量的一半, 或者最大 SCHED_RQ_NR_MIGRATION 个
    int nr_tries = min_t(int, src_rq->nr_running / 2, SCHED_RQ_NR_MIGRATION);

    // 循环遍历源运行队列(src_rq)中的任务, 尝试迁移
    // 循环条件: skip 不是 idle 任务 (防止无限循环), 还有尝试次数, 并且找到的下一个任务 p 也不是 idle 任务
    while (skip != src_rq->idle && nr_tries &&
           (p = rq_next_bmq_task(skip, src_rq)) != src_rq->idle) { // rq_next_bmq_task会从skip之后开始找
        
        // 在处理当前任务p之前, 先将skip推进到p的下一个任务
        // 这是为了确保即便任务p被迁移(从链表中删除), 下次循环也能从正确的位置继续
        skip = rq_next_bmq_task(p, src_rq);

        // 检查任务p是否允许迁移到目标CPU (dest_cpu)
        if (cpumask_test_cpu(dest_cpu, p->cpus_ptr)) { // 检查任务的CPU亲和性是否包含dest_cpu
            // 如果允许迁移, 执行实际的迁移步骤
            __dequeue_task(p, src_rq, 0);   // 1. 从源运行队列(src_rq)中移除任务 (调用__dequeue_task, 只操作位图和链表)
            set_task_cpu(p, dest_cpu);      // 2. 更新任务p记录的CPU ID为目标CPU
            __enqueue_task(p, dest_rq, 0);  // 3. 将任务p添加到目标运行队列(dest_rq)中 (调用__enqueue_task, 只操作位图和链表)
            nr_migrated++;                  // 成功迁移一个任务, 计数加一
        }
        nr_tries--; // 尝试次数减一
    }
    return nr_migrated; // 返回本次函数调用成功迁移的任务总数
}
#endif
```

#### 2.4.2 CPU亲和性管理与任务迁移

### 2.5 底层同步原语与并发控制策略

## 3. 现有的设计局限性

## 4. ELF BMQ 设计方案


## 项目源码
```c
/*
 *  kernel/sched/bmq.c
 *
 *  BMQ Core kernel scheduler code and related syscalls
 *
 *  Copyright (C) 1991-2002  Linus Torvalds
 *
 *  2009-08-13	Brainfuck deadline scheduling policy by Con Kolivas deletes
 *		a whole lot of those previous things.
 *  2017-09-06	Priority and Deadline based Skip list multiple queue kernel
 *		scheduler by Alfred Chen.
 *  2019-02-20	BMQ(BitMap Queue) kernel scheduler by Alfred Chen.
 */
#include "bmq_sched.h"

#include <linux/sched/rt.h>

#include <linux/context_tracking.h>
#include <linux/compat.h>
#include <linux/blkdev.h>
#include <linux/delayacct.h>
#include <linux/freezer.h>
#include <linux/init_task.h>
#include <linux/kprobes.h>
#include <linux/mmu_context.h>
#include <linux/nmi.h>
#include <linux/profile.h>
#include <linux/rcupdate_wait.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/wait_bit.h>

#include <linux/kcov.h>

#include <asm/switch_to.h>

#include "../workqueue_internal.h"
#include "../../fs/io-wq.h"
#include "../smpboot.h"

#include "pelt.h"

#define CREATE_TRACE_POINTS
#include <trace/events/sched.h>

/* rt_prio(prio) defined in include/linux/sched/rt.h */
#define rt_task(p)		rt_prio((p)->prio)
#define rt_policy(policy)	((policy) == SCHED_FIFO || (policy) == SCHED_RR)
#define task_has_rt_policy(p)	(rt_policy((p)->policy))

#define STOP_PRIO		(MAX_RT_PRIO - 1)

/* Default time slice is 4 in ms, can be set via kernel parameter "bmq.timeslice" */
u64 sched_timeslice_ns __read_mostly = (4 * 1000 * 1000);

static int __init sched_timeslice(char *str)
{
	int timeslice_us;

	get_option(&str, &timeslice_us);
	if (timeslice_us >= 1000)
		sched_timeslice_ns = timeslice_us * 1000;

	return 0;
}
early_param("bmq.timeslice", sched_timeslice);

/* Reschedule if less than this many μs left */
#define RESCHED_NS		(100 * 1000)

static inline void print_scheduler_version(void)
{
	printk(KERN_INFO "bmq: BMQ CPU Scheduler 5.7-r1 by Alfred Chen.\n");
}

/**
 * sched_yield_type - Choose what sort of yield sched_yield will perform.
 * 0: No yield.
 * 1: Deboost and requeue task. (default)
 * 2: Set rq skip task.
 */
int sched_yield_type __read_mostly = 1;

#define rq_switch_time(rq)	((rq)->clock - (rq)->last_ts_switch)
#define boost_threshold(p)	(sched_timeslice_ns >>\
				 (10 - MAX_PRIORITY_ADJ -  (p)->boost_prio))

static inline void boost_task(struct task_struct *p)
{
	int limit;

	switch (p->policy) {
	case SCHED_NORMAL:
		limit = -MAX_PRIORITY_ADJ;
		break;
	case SCHED_BATCH:
	case SCHED_IDLE:
		limit = 0;
		break;
	default:
		return;
	}

	if (p->boost_prio > limit)
		p->boost_prio--;
}

static inline void deboost_task(struct task_struct *p)
{
	if (p->boost_prio < MAX_PRIORITY_ADJ)
		p->boost_prio++;
}

#ifdef CONFIG_SMP
static cpumask_t sched_rq_pending_mask ____cacheline_aligned_in_smp;

DEFINE_PER_CPU(cpumask_t [NR_CPU_AFFINITY_CHK_LEVEL], sched_cpu_affinity_masks);
DEFINE_PER_CPU(cpumask_t *, sched_cpu_affinity_end_mask);
DEFINE_PER_CPU(cpumask_t *, sched_cpu_llc_mask);

#ifdef CONFIG_SCHED_SMT
DEFINE_STATIC_KEY_FALSE(sched_smt_present);
EXPORT_SYMBOL_GPL(sched_smt_present);
#endif

/*
 * Keep a unique ID per domain (we use the first CPUs number in the cpumask of
 * the domain), this allows us to quickly tell if two cpus are in the same cache
 * domain, see cpus_share_cache().
 */
DEFINE_PER_CPU(int, sd_llc_id);
#endif /* CONFIG_SMP */

static DEFINE_MUTEX(sched_hotcpu_mutex);

DEFINE_PER_CPU_SHARED_ALIGNED(struct rq, runqueues);

#ifndef prepare_arch_switch
# define prepare_arch_switch(next)	do { } while (0)
#endif
#ifndef finish_arch_post_lock_switch
# define finish_arch_post_lock_switch()	do { } while (0)
#endif

#define IDLE_WM	(IDLE_TASK_SCHED_PRIO)

static cpumask_t sched_sg_idle_mask ____cacheline_aligned_in_smp;
static cpumask_t sched_rq_watermark[bmq_BITS] ____cacheline_aligned_in_smp;

static inline void update_sched_rq_watermark(struct rq *rq)
{
	unsigned long watermark = find_first_bit(rq->queue.bitmap, bmq_BITS);
	unsigned long last_wm = rq->watermark;
	unsigned long i;
	int cpu;

	if (watermark == last_wm)
		return;

	rq->watermark = watermark;
	cpu = cpu_of(rq);
	if (watermark < last_wm) {
		for (i = watermark + 1; i <= last_wm; i++)
			cpumask_andnot(&sched_rq_watermark[i],
				       &sched_rq_watermark[i], cpumask_of(cpu));
#ifdef CONFIG_SCHED_SMT
		if (!static_branch_likely(&sched_smt_present))
			return;
		if (IDLE_WM == last_wm)
			cpumask_andnot(&sched_sg_idle_mask,
				       &sched_sg_idle_mask, cpu_smt_mask(cpu));
#endif
		return;
	}
	/* last_wm < watermark */
	for (i = last_wm + 1; i <= watermark; i++)
		cpumask_set_cpu(cpu, &sched_rq_watermark[i]);
#ifdef CONFIG_SCHED_SMT
	if (!static_branch_likely(&sched_smt_present))
		return;
	if (IDLE_WM == watermark) {
		cpumask_t tmp;
		cpumask_and(&tmp, cpu_smt_mask(cpu), &sched_rq_watermark[IDLE_WM]);
		if (cpumask_equal(&tmp, cpu_smt_mask(cpu)))
			cpumask_or(&sched_sg_idle_mask, cpu_smt_mask(cpu),
				   &sched_sg_idle_mask);
	}
#endif
}

static inline int task_sched_prio(struct task_struct *p)
{
	return (p->prio < MAX_RT_PRIO)? p->prio : p->prio + p->boost_prio;
}

static inline void bmq_init(struct bmq *q)
{
	int i;

	bitmap_zero(q->bitmap, bmq_BITS);
	for(i = 0; i < bmq_BITS; i++)
		INIT_LIST_HEAD(&q->heads[i]);
}

static inline void bmq_init_idle(struct bmq *q, struct task_struct *idle)
{
	INIT_LIST_HEAD(&q->heads[IDLE_TASK_SCHED_PRIO]);
	list_add(&idle->bmq_node, &q->heads[IDLE_TASK_SCHED_PRIO]);
	set_bit(IDLE_TASK_SCHED_PRIO, q->bitmap);
}

/*
 * This routine used in bmq scheduler only which assume the idle task in the bmq
 */
static inline struct task_struct *rq_first_bmq_task(struct rq *rq)
{
	unsigned long idx = find_first_bit(rq->queue.bitmap, bmq_BITS);
	const struct list_head *head = &rq->queue.heads[idx];

	return list_first_entry(head, struct task_struct, bmq_node);
}

static inline struct task_struct *
rq_next_bmq_task(struct task_struct *p, struct rq *rq)
{
	unsigned long idx = p->bmq_idx;
	struct list_head *head = &rq->queue.heads[idx];

	if (list_is_last(&p->bmq_node, head)) {
		idx = find_next_bit(rq->queue.bitmap, bmq_BITS, idx + 1);
		head = &rq->queue.heads[idx];

		return list_first_entry(head, struct task_struct, bmq_node);
	}

	return list_next_entry(p, bmq_node);
}

static inline struct task_struct *rq_runnable_task(struct rq *rq)
{
	struct task_struct *next = rq_first_bmq_task(rq);

	if (unlikely(next == rq->skip))
		next = rq_next_bmq_task(next, rq);

	return next;
}

/*
 * Context: p->pi_lock
 */
static inline struct rq
*__task_access_lock(struct task_struct *p, raw_spinlock_t **plock)
{
	struct rq *rq;
	for (;;) {
		rq = task_rq(p);
		if (p->on_cpu || task_on_rq_queued(p)) {
			raw_spin_lock(&rq->lock);
			if (likely((p->on_cpu || task_on_rq_queued(p))
				   && rq == task_rq(p))) {
				*plock = &rq->lock;
				return rq;
			}
			raw_spin_unlock(&rq->lock);
		} else if (task_on_rq_migrating(p)) {
			do {
				cpu_relax();
			} while (unlikely(task_on_rq_migrating(p)));
		} else {
			*plock = NULL;
			return rq;
		}
	}
}

static inline void
__task_access_unlock(struct task_struct *p, raw_spinlock_t *lock)
{
	if (NULL != lock)
		raw_spin_unlock(lock);
}

static inline struct rq
*task_access_lock_irqsave(struct task_struct *p, raw_spinlock_t **plock,
			  unsigned long *flags)
{
	struct rq *rq;
	for (;;) {
		rq = task_rq(p);
		if (p->on_cpu || task_on_rq_queued(p)) {
			raw_spin_lock_irqsave(&rq->lock, *flags);
			if (likely((p->on_cpu || task_on_rq_queued(p))
				   && rq == task_rq(p))) {
				*plock = &rq->lock;
				return rq;
			}
			raw_spin_unlock_irqrestore(&rq->lock, *flags);
		} else if (task_on_rq_migrating(p)) {
			do {
				cpu_relax();
			} while (unlikely(task_on_rq_migrating(p)));
		} else {
			raw_spin_lock_irqsave(&p->pi_lock, *flags);
			if (likely(!p->on_cpu && !p->on_rq &&
				   rq == task_rq(p))) {
				*plock = &p->pi_lock;
				return rq;
			}
			raw_spin_unlock_irqrestore(&p->pi_lock, *flags);
		}
	}
}

static inline void
task_access_unlock_irqrestore(struct task_struct *p, raw_spinlock_t *lock,
			      unsigned long *flags)
{
	raw_spin_unlock_irqrestore(lock, *flags);
}

/*
 * __task_rq_lock - lock the rq @p resides on.
 */
struct rq *__task_rq_lock(struct task_struct *p, struct rq_flags *rf)
	__acquires(rq->lock)
{
	struct rq *rq;

	lockdep_assert_held(&p->pi_lock);

	for (;;) {
		rq = task_rq(p);
		raw_spin_lock(&rq->lock);
		if (likely(rq == task_rq(p) && !task_on_rq_migrating(p)))
			return rq;
		raw_spin_unlock(&rq->lock);

		while (unlikely(task_on_rq_migrating(p)))
			cpu_relax();
	}
}

/*
 * task_rq_lock - lock p->pi_lock and lock the rq @p resides on.
 */
struct rq *task_rq_lock(struct task_struct *p, struct rq_flags *rf)
	__acquires(p->pi_lock)
	__acquires(rq->lock)
{
	struct rq *rq;

	for (;;) {
		raw_spin_lock_irqsave(&p->pi_lock, rf->flags);
		rq = task_rq(p);
		raw_spin_lock(&rq->lock);
		/*
		 *	move_queued_task()		task_rq_lock()
		 *
		 *	ACQUIRE (rq->lock)
		 *	[S] ->on_rq = MIGRATING		[L] rq = task_rq()
		 *	WMB (__set_task_cpu())		ACQUIRE (rq->lock);
		 *	[S] ->cpu = new_cpu		[L] task_rq()
		 *					[L] ->on_rq
		 *	RELEASE (rq->lock)
		 *
		 * If we observe the old CPU in task_rq_lock(), the acquire of
		 * the old rq->lock will fully serialize against the stores.
		 *
		 * If we observe the new CPU in task_rq_lock(), the address
		 * dependency headed by '[L] rq = task_rq()' and the acquire
		 * will pair with the WMB to ensure we then also see migrating.
		 */
		if (likely(rq == task_rq(p) && !task_on_rq_migrating(p))) {
			return rq;
		}
		raw_spin_unlock(&rq->lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, rf->flags);

		while (unlikely(task_on_rq_migrating(p)))
			cpu_relax();
	}
}

/*
 * RQ-clock updating methods:
 */

static void update_rq_clock_task(struct rq *rq, s64 delta)
{
/*
 * In theory, the compile should just see 0 here, and optimize out the call
 * to sched_rt_avg_update. But I don't trust it...
 */
	s64 __maybe_unused steal = 0, irq_delta = 0;

#ifdef CONFIG_IRQ_TIME_ACCOUNTING
	irq_delta = irq_time_read(cpu_of(rq)) - rq->prev_irq_time;

	/*
	 * Since irq_time is only updated on {soft,}irq_exit, we might run into
	 * this case when a previous update_rq_clock() happened inside a
	 * {soft,}irq region.
	 *
	 * When this happens, we stop ->clock_task and only update the
	 * prev_irq_time stamp to account for the part that fit, so that a next
	 * update will consume the rest. This ensures ->clock_task is
	 * monotonic.
	 *
	 * It does however cause some slight miss-attribution of {soft,}irq
	 * time, a more accurate solution would be to update the irq_time using
	 * the current rq->clock timestamp, except that would require using
	 * atomic ops.
	 */
	if (irq_delta > delta)
		irq_delta = delta;

	rq->prev_irq_time += irq_delta;
	delta -= irq_delta;
#endif
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
	if (static_key_false((&paravirt_steal_rq_enabled))) {
		steal = paravirt_steal_clock(cpu_of(rq));
		steal -= rq->prev_steal_time_rq;

		if (unlikely(steal > delta))
			steal = delta;

		rq->prev_steal_time_rq += steal;
		delta -= steal;
	}
#endif

	rq->clock_task += delta;

#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	if ((irq_delta + steal))
		update_irq_load_avg(rq, irq_delta + steal);
#endif
}

static inline void update_rq_clock(struct rq *rq)
{
	s64 delta = sched_clock_cpu(cpu_of(rq)) - rq->clock;

	if (unlikely(delta <= 0))
		return;
	rq->clock += delta;
	update_rq_clock_task(rq, delta);
}

#ifdef CONFIG_NO_HZ_FULL
/*
 * Tick may be needed by tasks in the runqueue depending on their policy and
 * requirements. If tick is needed, lets send the target an IPI to kick it out
 * of nohz mode if necessary.
 */
static inline void sched_update_tick_dependency(struct rq *rq)
{
	int cpu;

	if (!tick_nohz_full_enabled())
		return;

	cpu = cpu_of(rq);

	if (!tick_nohz_full_cpu(cpu))
		return;

	if (rq->nr_running < 2)
		tick_nohz_dep_clear_cpu(cpu, TICK_DEP_BIT_SCHED);
	else
		tick_nohz_dep_set_cpu(cpu, TICK_DEP_BIT_SCHED);
}
#else /* !CONFIG_NO_HZ_FULL */
static inline void sched_update_tick_dependency(struct rq *rq) { }
#endif

/*
 * Add/Remove/Requeue task to/from the runqueue routines
 * Context: rq->lock
 */
static inline void __dequeue_task(struct task_struct *p, struct rq *rq, int flags)
{
	psi_dequeue(p, flags & DEQUEUE_SLEEP);
	sched_info_dequeued(rq, p);

	list_del(&p->bmq_node);
	if (list_empty(&rq->queue.heads[p->bmq_idx]))
		clear_bit(p->bmq_idx, rq->queue.bitmap);
}

static inline void dequeue_task(struct task_struct *p, struct rq *rq, int flags)
{
	lockdep_assert_held(&rq->lock);

	WARN_ONCE(task_rq(p) != rq, "bmq: dequeue task reside on cpu%d from cpu%d\n",
		  task_cpu(p), cpu_of(rq));

	psi_dequeue(p, flags & DEQUEUE_SLEEP);
	sched_info_dequeued(rq, p);

	list_del(&p->bmq_node);
	if (list_empty(&rq->queue.heads[p->bmq_idx])) {
		clear_bit(p->bmq_idx, rq->queue.bitmap);
		update_sched_rq_watermark(rq);
	}
	--rq->nr_running;
#ifdef CONFIG_SMP
	if (1 == rq->nr_running)
		cpumask_clear_cpu(cpu_of(rq), &sched_rq_pending_mask);
#endif

	sched_update_tick_dependency(rq);
}

static inline void __enqueue_task(struct task_struct *p, struct rq *rq, int flags)
{
	sched_info_queued(rq, p);
	psi_enqueue(p, flags);

	p->bmq_idx = task_sched_prio(p);
	list_add_tail(&p->bmq_node, &rq->queue.heads[p->bmq_idx]);
	set_bit(p->bmq_idx, rq->queue.bitmap);
}

static inline void enqueue_task(struct task_struct *p, struct rq *rq, int flags)
{
	lockdep_assert_held(&rq->lock);

	WARN_ONCE(task_rq(p) != rq, "bmq: enqueue task reside on cpu%d to cpu%d\n",
		  task_cpu(p), cpu_of(rq));

	__enqueue_task(p, rq, flags);
	update_sched_rq_watermark(rq);
	++rq->nr_running;
#ifdef CONFIG_SMP
	if (2 == rq->nr_running)
		cpumask_set_cpu(cpu_of(rq), &sched_rq_pending_mask);
#endif

	sched_update_tick_dependency(rq);

	/*
	 * If in_iowait is set, the code below may not trigger any cpufreq
	 * utilization updates, so do it here explicitly with the IOWAIT flag
	 * passed.
	 */
	if (p->in_iowait)
		cpufreq_update_util(rq, SCHED_CPUFREQ_IOWAIT);
}

static inline void requeue_task(struct task_struct *p, struct rq *rq)
{
	int idx = task_sched_prio(p);

	lockdep_assert_held(&rq->lock);
	WARN_ONCE(task_rq(p) != rq, "bmq: cpu[%d] requeue task reside on cpu%d\n",
		  cpu_of(rq), task_cpu(p));

	list_del(&p->bmq_node);
	list_add_tail(&p->bmq_node, &rq->queue.heads[idx]);
	if (idx != p->bmq_idx) {
		if (list_empty(&rq->queue.heads[p->bmq_idx]))
			clear_bit(p->bmq_idx, rq->queue.bitmap);
		p->bmq_idx = idx;
		set_bit(p->bmq_idx, rq->queue.bitmap);
		update_sched_rq_watermark(rq);
	}
}

/*
 * cmpxchg based fetch_or, macro so it works for different integer types
 */
#define fetch_or(ptr, mask)						\
	({								\
		typeof(ptr) _ptr = (ptr);				\
		typeof(mask) _mask = (mask);				\
		typeof(*_ptr) _old, _val = *_ptr;			\
									\
		for (;;) {						\
			_old = cmpxchg(_ptr, _val, _val | _mask);	\
			if (_old == _val)				\
				break;					\
			_val = _old;					\
		}							\
	_old;								\
})

#if defined(CONFIG_SMP) && defined(TIF_POLLING_NRFLAG)
/*
 * Atomically set TIF_NEED_RESCHED and test for TIF_POLLING_NRFLAG,
 * this avoids any races wrt polling state changes and thereby avoids
 * spurious IPIs.
 */
static bool set_nr_and_not_polling(struct task_struct *p)
{
	struct thread_info *ti = task_thread_info(p);
	return !(fetch_or(&ti->flags, _TIF_NEED_RESCHED) & _TIF_POLLING_NRFLAG);
}

/*
 * Atomically set TIF_NEED_RESCHED if TIF_POLLING_NRFLAG is set.
 *
 * If this returns true, then the idle task promises to call
 * sched_ttwu_pending() and reschedule soon.
 */
static bool set_nr_if_polling(struct task_struct *p)
{
	struct thread_info *ti = task_thread_info(p);
	typeof(ti->flags) old, val = READ_ONCE(ti->flags);

	for (;;) {
		if (!(val & _TIF_POLLING_NRFLAG))
			return false;
		if (val & _TIF_NEED_RESCHED)
			return true;
		old = cmpxchg(&ti->flags, val, val | _TIF_NEED_RESCHED);
		if (old == val)
			break;
		val = old;
	}
	return true;
}

#else
static bool set_nr_and_not_polling(struct task_struct *p)
{
	set_tsk_need_resched(p);
	return true;
}

#ifdef CONFIG_SMP
static bool set_nr_if_polling(struct task_struct *p)
{
	return false;
}
#endif
#endif

static bool __wake_q_add(struct wake_q_head *head, struct task_struct *task)
{
	struct wake_q_node *node = &task->wake_q;

	/*
	 * Atomically grab the task, if ->wake_q is !nil already it means
	 * its already queued (either by us or someone else) and will get the
	 * wakeup due to that.
	 *
	 * In order to ensure that a pending wakeup will observe our pending
	 * state, even in the failed case, an explicit smp_mb() must be used.
	 */
	smp_mb__before_atomic();
	if (unlikely(cmpxchg_relaxed(&node->next, NULL, WAKE_Q_TAIL)))
		return false;

	/*
	 * The head is context local, there can be no concurrency.
	 */
	*head->lastp = node;
	head->lastp = &node->next;
	return true;
}

/**
 * wake_q_add() - queue a wakeup for 'later' waking.
 * @head: the wake_q_head to add @task to
 * @task: the task to queue for 'later' wakeup
 *
 * Queue a task for later wakeup, most likely by the wake_up_q() call in the
 * same context, _HOWEVER_ this is not guaranteed, the wakeup can come
 * instantly.
 *
 * This function must be used as-if it were wake_up_process(); IOW the task
 * must be ready to be woken at this location.
 */
void wake_q_add(struct wake_q_head *head, struct task_struct *task)
{
	if (__wake_q_add(head, task))
		get_task_struct(task);
}

/**
 * wake_q_add_safe() - safely queue a wakeup for 'later' waking.
 * @head: the wake_q_head to add @task to
 * @task: the task to queue for 'later' wakeup
 *
 * Queue a task for later wakeup, most likely by the wake_up_q() call in the
 * same context, _HOWEVER_ this is not guaranteed, the wakeup can come
 * instantly.
 *
 * This function must be used as-if it were wake_up_process(); IOW the task
 * must be ready to be woken at this location.
 *
 * This function is essentially a task-safe equivalent to wake_q_add(). Callers
 * that already hold reference to @task can call the 'safe' version and trust
 * wake_q to do the right thing depending whether or not the @task is already
 * queued for wakeup.
 */
void wake_q_add_safe(struct wake_q_head *head, struct task_struct *task)
{
	if (!__wake_q_add(head, task))
		put_task_struct(task);
}

void wake_up_q(struct wake_q_head *head)
{
	struct wake_q_node *node = head->first;

	while (node != WAKE_Q_TAIL) {
		struct task_struct *task;

		task = container_of(node, struct task_struct, wake_q);
		BUG_ON(!task);
		/* task can safely be re-inserted now: */
		node = node->next;
		task->wake_q.next = NULL;

		/*
		 * wake_up_process() executes a full barrier, which pairs with
		 * the queueing in wake_q_add() so as not to miss wakeups.
		 */
		wake_up_process(task);
		put_task_struct(task);
	}
}

/*
 * resched_curr - mark rq's current task 'to be rescheduled now'.
 *
 * On UP this means the setting of the need_resched flag, on SMP it
 * might also involve a cross-CPU call to trigger the scheduler on
 * the target CPU.
 */
void resched_curr(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	int cpu;

	lockdep_assert_held(&rq->lock);

	if (test_tsk_need_resched(curr))
		return;

	cpu = cpu_of(rq);
	if (cpu == smp_processor_id()) {
		set_tsk_need_resched(curr);
		set_preempt_need_resched();
		return;
	}

	if (set_nr_and_not_polling(curr))
		smp_send_reschedule(cpu);
	else
		trace_sched_wake_idle_without_ipi(cpu);
}

void resched_cpu(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	raw_spin_lock_irqsave(&rq->lock, flags);
	if (cpu_online(cpu) || cpu == smp_processor_id())
		resched_curr(cpu_rq(cpu));
	raw_spin_unlock_irqrestore(&rq->lock, flags);
}

#ifdef CONFIG_SMP
#ifdef CONFIG_NO_HZ_COMMON
void nohz_balance_enter_idle(int cpu)
{
}

void select_nohz_load_balancer(int stop_tick)
{
}

void set_cpu_sd_state_idle(void) {}

/*
 * In the semi idle case, use the nearest busy CPU for migrating timers
 * from an idle CPU.  This is good for power-savings.
 *
 * We don't do similar optimization for completely idle system, as
 * selecting an idle CPU will add more delays to the timers than intended
 * (as that CPU's timer base may not be uptodate wrt jiffies etc).
 */
int get_nohz_timer_target(void)
{
	int i, cpu = smp_processor_id(), default_cpu = -1;
	struct cpumask *mask;

	if (housekeeping_cpu(cpu, HK_FLAG_TIMER)) {
		if (!idle_cpu(cpu))
			return cpu;
		default_cpu = cpu;
	}

	for (mask = &(per_cpu(sched_cpu_affinity_masks, cpu)[0]);
	     mask < per_cpu(sched_cpu_affinity_end_mask, cpu); mask++)
		for_each_cpu_and(i, mask, housekeeping_cpumask(HK_FLAG_TIMER))
			if (!idle_cpu(i))
				return i;

	if (default_cpu == -1)
		default_cpu = housekeeping_any_cpu(HK_FLAG_TIMER);
	cpu = default_cpu;

	return cpu;
}

/*
 * When add_timer_on() enqueues a timer into the timer wheel of an
 * idle CPU then this timer might expire before the next timer event
 * which is scheduled to wake up that CPU. In case of a completely
 * idle system the next event might even be infinite time into the
 * future. wake_up_idle_cpu() ensures that the CPU is woken up and
 * leaves the inner idle loop so the newly added timer is taken into
 * account when the CPU goes back to idle and evaluates the timer
 * wheel for the next timer event.
 */
static inline void wake_up_idle_cpu(int cpu)
{
	if (cpu == smp_processor_id())
		return;

	set_tsk_need_resched(cpu_rq(cpu)->idle);
	smp_send_reschedule(cpu);
}

static inline bool wake_up_full_nohz_cpu(int cpu)
{
	/*
	 * We just need the target to call irq_exit() and re-evaluate
	 * the next tick. The nohz full kick at least implies that.
	 * If needed we can still optimize that later with an
	 * empty IRQ.
	 */
	if (tick_nohz_full_cpu(cpu)) {
		if (cpu != smp_processor_id() ||
		    tick_nohz_tick_stopped())
			tick_nohz_full_kick_cpu(cpu);
		return true;
	}

	return false;
}

void wake_up_nohz_cpu(int cpu)
{
	if (cpu_online(cpu) && !wake_up_full_nohz_cpu(cpu))
		wake_up_idle_cpu(cpu);
}

#endif /* CONFIG_NO_HZ_COMMON */
#endif /* CONFIG_SMP */

static inline void check_preempt_curr(struct rq *rq)
{
	if (rq_first_bmq_task(rq) != rq->curr)
		resched_curr(rq);
}

#ifdef CONFIG_SCHED_HRTICK
/*
 * Use HR-timers to deliver accurate preemption points.
 */

static void hrtick_clear(struct rq *rq)
{
	if (hrtimer_active(&rq->hrtick_timer))
		hrtimer_cancel(&rq->hrtick_timer);
}

/*
 * High-resolution timer tick.
 * Runs from hardirq context with interrupts disabled.
 */
static enum hrtimer_restart hrtick(struct hrtimer *timer)
{
	struct rq *rq = container_of(timer, struct rq, hrtick_timer);
	struct task_struct *p;

	WARN_ON_ONCE(cpu_of(rq) != smp_processor_id());

	raw_spin_lock(&rq->lock);
	p = rq->curr;
	p->time_slice = 0;
	resched_curr(rq);
	raw_spin_unlock(&rq->lock);

	return HRTIMER_NORESTART;
}

/*
 * Use hrtick when:
 *  - enabled by features
 *  - hrtimer is actually high res
 */
static inline int hrtick_enabled(struct rq *rq)
{
	/**
	 * BMQ doesn't support sched_feat yet
	if (!sched_feat(HRTICK))
		return 0;
	*/
	if (!cpu_active(cpu_of(rq)))
		return 0;
	return hrtimer_is_hres_active(&rq->hrtick_timer);
}

#ifdef CONFIG_SMP

static void __hrtick_restart(struct rq *rq)
{
	struct hrtimer *timer = &rq->hrtick_timer;

	hrtimer_start_expires(timer, HRTIMER_MODE_ABS_PINNED_HARD);
}

/*
 * called from hardirq (IPI) context
 */
static void __hrtick_start(void *arg)
{
	struct rq *rq = arg;

	raw_spin_lock(&rq->lock);
	__hrtick_restart(rq);
	raw_spin_unlock(&rq->lock);
}

/*
 * Called to set the hrtick timer state.
 *
 * called with rq->lock held and irqs disabled
 */
void hrtick_start(struct rq *rq, u64 delay)
{
	struct hrtimer *timer = &rq->hrtick_timer;
	ktime_t time;
	s64 delta;

	/*
	 * Don't schedule slices shorter than 10000ns, that just
	 * doesn't make sense and can cause timer DoS.
	 */
	delta = max_t(s64, delay, 10000LL);
	time = ktime_add_ns(timer->base->get_time(), delta);

	hrtimer_set_expires(timer, time);

	if (rq == this_rq())
		__hrtick_restart(rq);
	else
		smp_call_function_single_async(cpu_of(rq), &rq->hrtick_csd);
}

#else
/*
 * Called to set the hrtick timer state.
 *
 * called with rq->lock held and irqs disabled
 */
void hrtick_start(struct rq *rq, u64 delay)
{
	/*
	 * Don't schedule slices shorter than 10000ns, that just
	 * doesn't make sense. Rely on vruntime for fairness.
	 */
	delay = max_t(u64, delay, 10000LL);
	hrtimer_start(&rq->hrtick_timer, ns_to_ktime(delay),
		      HRTIMER_MODE_REL_PINNED_HARD);
}
#endif /* CONFIG_SMP */

static void hrtick_rq_init(struct rq *rq)
{
#ifdef CONFIG_SMP
	rq->hrtick_csd.flags = 0;
	rq->hrtick_csd.func = __hrtick_start;
	rq->hrtick_csd.info = rq;
#endif

	hrtimer_init(&rq->hrtick_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	rq->hrtick_timer.function = hrtick;
}
#else	/* CONFIG_SCHED_HRTICK */
static inline int hrtick_enabled(struct rq *rq)
{
	return 0;
}

static inline void hrtick_clear(struct rq *rq)
{
}

static inline void hrtick_rq_init(struct rq *rq)
{
}
#endif	/* CONFIG_SCHED_HRTICK */

static inline int normal_prio(struct task_struct *p)
{
	if (task_has_rt_policy(p))
		return MAX_RT_PRIO - 1 - p->rt_priority;

	return p->static_prio + MAX_PRIORITY_ADJ;
}

/*
 * Calculate the current priority, i.e. the priority
 * taken into account by the scheduler. This value might
 * be boosted by RT tasks as it will be RT if the task got
 * RT-boosted. If not then it returns p->normal_prio.
 */
static int effective_prio(struct task_struct *p)
{
	p->normal_prio = normal_prio(p);
	/*
	 * If we are RT tasks or we were boosted to RT priority,
	 * keep the priority unchanged. Otherwise, update priority
	 * to the normal priority:
	 */
	if (!rt_prio(p->prio))
		return p->normal_prio;
	return p->prio;
}

/*
 * activate_task - move a task to the runqueue.
 *
 * Context: rq->lock
 */
static void activate_task(struct task_struct *p, struct rq *rq)
{
	if (task_contributes_to_load(p))
		rq->nr_uninterruptible--;
	enqueue_task(p, rq, ENQUEUE_WAKEUP);
	p->on_rq = TASK_ON_RQ_QUEUED;
	cpufreq_update_util(rq, 0);
}

/*
 * deactivate_task - remove a task from the runqueue.
 *
 * Context: rq->lock
 */
static inline void deactivate_task(struct task_struct *p, struct rq *rq)
{
	if (task_contributes_to_load(p))
		rq->nr_uninterruptible++;
	dequeue_task(p, rq, DEQUEUE_SLEEP);
	p->on_rq = 0;
	cpufreq_update_util(rq, 0);
}

static inline void __set_task_cpu(struct task_struct *p, unsigned int cpu)
{
#ifdef CONFIG_SMP
	/*
	 * After ->cpu is set up to a new value, task_access_lock(p, ...) can be
	 * successfully executed on another CPU. We must ensure that updates of
	 * per-task data have been completed by this moment.
	 */
	smp_wmb();

#ifdef CONFIG_THREAD_INFO_IN_TASK
	WRITE_ONCE(p->cpu, cpu);
#else
	WRITE_ONCE(task_thread_info(p)->cpu, cpu);
#endif
#endif
}

#ifdef CONFIG_SMP
void set_task_cpu(struct task_struct *p, unsigned int new_cpu)
{
#ifdef CONFIG_SCHED_DEBUG
	/*
	 * We should never call set_task_cpu() on a blocked task,
	 * ttwu() will sort out the placement.
	 */
	WARN_ON_ONCE(p->state != TASK_RUNNING && p->state != TASK_WAKING &&
		     !p->on_rq);
#ifdef CONFIG_LOCKDEP
	/*
	 * The caller should hold either p->pi_lock or rq->lock, when changing
	 * a task's CPU. ->pi_lock for waking tasks, rq->lock for runnable tasks.
	 *
	 * sched_move_task() holds both and thus holding either pins the cgroup,
	 * see task_group().
	 */
	WARN_ON_ONCE(debug_locks && !(lockdep_is_held(&p->pi_lock) ||
				      lockdep_is_held(&task_rq(p)->lock)));
#endif
	/*
	 * Clearly, migrating tasks to offline CPUs is a fairly daft thing.
	 */
	WARN_ON_ONCE(!cpu_online(new_cpu));
#endif
	if (task_cpu(p) == new_cpu)
		return;
	trace_sched_migrate_task(p, new_cpu);
	rseq_migrate(p);
	perf_event_task_migrate(p);

	__set_task_cpu(p, new_cpu);
}

static inline bool is_per_cpu_kthread(struct task_struct *p)
{
	return ((p->flags & PF_KTHREAD) && (1 == p->nr_cpus_allowed));
}

/*
 * Per-CPU kthreads are allowed to run on !active && online CPUs, see
 * __set_cpus_allowed_ptr() and select_fallback_rq().
 */
static inline bool is_cpu_allowed(struct task_struct *p, int cpu)
{
	if (!cpumask_test_cpu(cpu, p->cpus_ptr))
		return false;

	if (is_per_cpu_kthread(p))
		return cpu_online(cpu);

	return cpu_active(cpu);
}

/*
 * This is how migration works:
 *
 * 1) we invoke migration_cpu_stop() on the target CPU using
 *    stop_one_cpu().
 * 2) stopper starts to run (implicitly forcing the migrated thread
 *    off the CPU)
 * 3) it checks whether the migrated task is still in the wrong runqueue.
 * 4) if it's in the wrong runqueue then the migration thread removes
 *    it and puts it into the right queue.
 * 5) stopper completes and stop_one_cpu() returns and the migration
 *    is done.
 */

/*
 * move_queued_task - move a queued task to new rq.
 *
 * Returns (locked) new rq. Old rq's lock is released.
 */
static struct rq *move_queued_task(struct rq *rq, struct task_struct *p, int
				   new_cpu)
{
	lockdep_assert_held(&rq->lock);

	WRITE_ONCE(p->on_rq, TASK_ON_RQ_MIGRATING);
	dequeue_task(p, rq, 0);
	set_task_cpu(p, new_cpu);
	raw_spin_unlock(&rq->lock);

	rq = cpu_rq(new_cpu);

	raw_spin_lock(&rq->lock);
	BUG_ON(task_cpu(p) != new_cpu);
	enqueue_task(p, rq, 0);
	p->on_rq = TASK_ON_RQ_QUEUED;
	check_preempt_curr(rq);

	return rq;
}

struct migration_arg {
	struct task_struct *task;
	int dest_cpu;
};

/*
 * Move (not current) task off this CPU, onto the destination CPU. We're doing
 * this because either it can't run here any more (set_cpus_allowed()
 * away from this CPU, or CPU going down), or because we're
 * attempting to rebalance this task on exec (sched_exec).
 *
 * So we race with normal scheduler movements, but that's OK, as long
 * as the task is no longer on this CPU.
 */
static struct rq *__migrate_task(struct rq *rq, struct task_struct *p, int
				 dest_cpu)
{
	/* Affinity changed (again). */
	if (!is_cpu_allowed(p, dest_cpu))
		return rq;

	update_rq_clock(rq);
	return move_queued_task(rq, p, dest_cpu);
}

/*
 * migration_cpu_stop - this will be executed by a highprio stopper thread
 * and performs thread migration by bumping thread off CPU then
 * 'pushing' onto another runqueue.
 */
static int migration_cpu_stop(void *data)
{
	struct migration_arg *arg = data;
	struct task_struct *p = arg->task;
	struct rq *rq = this_rq();

	/*
	 * The original target CPU might have gone down and we might
	 * be on another CPU but it doesn't matter.
	 */
	local_irq_disable();

	raw_spin_lock(&p->pi_lock);
	raw_spin_lock(&rq->lock);
	/*
	 * If task_rq(p) != rq, it cannot be migrated here, because we're
	 * holding rq->lock, if p->on_rq == 0 it cannot get enqueued because
	 * we're holding p->pi_lock.
	 */
	if (task_rq(p) == rq && task_on_rq_queued(p))
		rq = __migrate_task(rq, p, arg->dest_cpu);
	raw_spin_unlock(&rq->lock);
	raw_spin_unlock(&p->pi_lock);

	local_irq_enable();
	return 0;
}

static inline void
set_cpus_allowed_common(struct task_struct *p, const struct cpumask *new_mask)
{
	cpumask_copy(&p->cpus_mask, new_mask);
	p->nr_cpus_allowed = cpumask_weight(new_mask);
}

void do_set_cpus_allowed(struct task_struct *p, const struct cpumask *new_mask)
{
	set_cpus_allowed_common(p, new_mask);
}
#endif

/**
 * task_curr - is this task currently executing on a CPU?
 * @p: the task in question.
 *
 * Return: 1 if the task is currently executing. 0 otherwise.
 */
inline int task_curr(const struct task_struct *p)
{
	return cpu_curr(task_cpu(p)) == p;
}

#ifdef CONFIG_SMP
/*
 * wait_task_inactive - wait for a thread to unschedule.
 *
 * If @match_state is nonzero, it's the @p->state value just checked and
 * not expected to change.  If it changes, i.e. @p might have woken up,
 * then return zero.  When we succeed in waiting for @p to be off its CPU,
 * we return a positive number (its total switch count).  If a second call
 * a short while later returns the same number, the caller can be sure that
 * @p has remained unscheduled the whole time.
 *
 * The caller must ensure that the task *will* unschedule sometime soon,
 * else this function might spin for a *long* time. This function can't
 * be called with interrupts off, or it may introduce deadlock with
 * smp_call_function() if an IPI is sent by the same process we are
 * waiting to become inactive.
 */
unsigned long wait_task_inactive(struct task_struct *p, long match_state)
{
	unsigned long flags;
	bool running, on_rq;
	unsigned long ncsw;
	struct rq *rq;
	raw_spinlock_t *lock;

	for (;;) {
		rq = task_rq(p);

		/*
		 * If the task is actively running on another CPU
		 * still, just relax and busy-wait without holding
		 * any locks.
		 *
		 * NOTE! Since we don't hold any locks, it's not
		 * even sure that "rq" stays as the right runqueue!
		 * But we don't care, since this will return false
		 * if the runqueue has changed and p is actually now
		 * running somewhere else!
		 */
		while (task_running(p) && p == rq->curr) {
			if (match_state && unlikely(p->state != match_state))
				return 0;
			cpu_relax();
		}

		/*
		 * Ok, time to look more closely! We need the rq
		 * lock now, to be *sure*. If we're wrong, we'll
		 * just go back and repeat.
		 */
		task_access_lock_irqsave(p, &lock, &flags);
		trace_sched_wait_task(p);
		running = task_running(p);
		on_rq = p->on_rq;
		ncsw = 0;
		if (!match_state || p->state == match_state)
			ncsw = p->nvcsw | LONG_MIN; /* sets MSB */
		task_access_unlock_irqrestore(p, lock, &flags);

		/*
		 * If it changed from the expected state, bail out now.
		 */
		if (unlikely(!ncsw))
			break;

		/*
		 * Was it really running after all now that we
		 * checked with the proper locks actually held?
		 *
		 * Oops. Go back and try again..
		 */
		if (unlikely(running)) {
			cpu_relax();
			continue;
		}

		/*
		 * It's not enough that it's not actively running,
		 * it must be off the runqueue _entirely_, and not
		 * preempted!
		 *
		 * So if it was still runnable (but just not actively
		 * running right now), it's preempted, and we should
		 * yield - it could be a while.
		 */
		if (unlikely(on_rq)) {
			ktime_t to = NSEC_PER_SEC / HZ;

			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_hrtimeout(&to, HRTIMER_MODE_REL);
			continue;
		}

		/*
		 * Ahh, all good. It wasn't running, and it wasn't
		 * runnable, which means that it will never become
		 * running in the future either. We're all done!
		 */
		break;
	}

	return ncsw;
}

/***
 * kick_process - kick a running thread to enter/exit the kernel
 * @p: the to-be-kicked thread
 *
 * Cause a process which is running on another CPU to enter
 * kernel-mode, without any delay. (to get signals handled.)
 *
 * NOTE: this function doesn't have to take the runqueue lock,
 * because all it wants to ensure is that the remote task enters
 * the kernel. If the IPI races and the task has been migrated
 * to another CPU then no harm is done and the purpose has been
 * achieved as well.
 */
void kick_process(struct task_struct *p)
{
	int cpu;

	preempt_disable();
	cpu = task_cpu(p);
	if ((cpu != smp_processor_id()) && task_curr(p))
		smp_send_reschedule(cpu);
	preempt_enable();
}
EXPORT_SYMBOL_GPL(kick_process);

/*
 * ->cpus_ptr is protected by both rq->lock and p->pi_lock
 *
 * A few notes on cpu_active vs cpu_online:
 *
 *  - cpu_active must be a subset of cpu_online
 *
 *  - on CPU-up we allow per-CPU kthreads on the online && !active CPU,
 *    see __set_cpus_allowed_ptr(). At this point the newly online
 *    CPU isn't yet part of the sched domains, and balancing will not
 *    see it.
 *
 *  - on cpu-down we clear cpu_active() to mask the sched domains and
 *    avoid the load balancer to place new tasks on the to be removed
 *    CPU. Existing tasks will remain running there and will be taken
 *    off.
 *
 * This means that fallback selection must not select !active CPUs.
 * And can assume that any active CPU must be online. Conversely
 * select_task_rq() below may allow selection of !active CPUs in order
 * to satisfy the above rules.
 */
static int select_fallback_rq(int cpu, struct task_struct *p)
{
	int nid = cpu_to_node(cpu);
	const struct cpumask *nodemask = NULL;
	enum { cpuset, possible, fail } state = cpuset;
	int dest_cpu;

	/*
	 * If the node that the CPU is on has been offlined, cpu_to_node()
	 * will return -1. There is no CPU on the node, and we should
	 * select the CPU on the other node.
	 */
	if (nid != -1) {
		nodemask = cpumask_of_node(nid);

		/* Look for allowed, online CPU in same node. */
		for_each_cpu(dest_cpu, nodemask) {
			if (!cpu_active(dest_cpu))
				continue;
			if (cpumask_test_cpu(dest_cpu, p->cpus_ptr))
				return dest_cpu;
		}
	}

	for (;;) {
		/* Any allowed, online CPU? */
		for_each_cpu(dest_cpu, p->cpus_ptr) {
			if (!is_cpu_allowed(p, dest_cpu))
				continue;
			goto out;
		}

		/* No more Mr. Nice Guy. */
		switch (state) {
		case cpuset:
			if (IS_ENABLED(CONFIG_CPUSETS)) {
				cpuset_cpus_allowed_fallback(p);
				state = possible;
				break;
			}
			/* Fall-through */
		case possible:
			do_set_cpus_allowed(p, cpu_possible_mask);
			state = fail;
			break;

		case fail:
			BUG();
			break;
		}
	}

out:
	if (state != cpuset) {
		/*
		 * Don't tell them about moving exiting tasks or
		 * kernel threads (both mm NULL), since they never
		 * leave kernel.
		 */
		if (p->mm && printk_ratelimit()) {
			printk_deferred("process %d (%s) no longer affine to cpu%d\n",
					task_pid_nr(p), p->comm, cpu);
		}
	}

	return dest_cpu;
}

static inline int select_task_rq(struct task_struct *p)
{
	cpumask_t chk_mask, tmp;

	if (unlikely(!cpumask_and(&chk_mask, p->cpus_ptr, cpu_online_mask)))
		return select_fallback_rq(task_cpu(p), p);

	if (
#ifdef CONFIG_SCHED_SMT
	    cpumask_and(&tmp, &chk_mask, &sched_sg_idle_mask) ||
#endif
	    cpumask_and(&tmp, &chk_mask, &sched_rq_watermark[IDLE_WM]) ||
	    cpumask_and(&tmp, &chk_mask,
			&sched_rq_watermark[task_sched_prio(p) + 1]))
		return best_mask_cpu(task_cpu(p), &tmp);

	return best_mask_cpu(task_cpu(p), &chk_mask);
}

void sched_set_stop_task(int cpu, struct task_struct *stop)
{
	struct sched_param stop_param = { .sched_priority = STOP_PRIO };
	struct sched_param start_param = { .sched_priority = 0 };
	struct task_struct *old_stop = cpu_rq(cpu)->stop;

	if (stop) {
		/*
		 * Make it appear like a SCHED_FIFO task, its something
		 * userspace knows about and won't get confused about.
		 *
		 * Also, it will make PI more or less work without too
		 * much confusion -- but then, stop work should not
		 * rely on PI working anyway.
		 */
		sched_setscheduler_nocheck(stop, SCHED_FIFO, &stop_param);
	}

	cpu_rq(cpu)->stop = stop;

	if (old_stop) {
		/*
		 * Reset it back to a normal scheduling policy so that
		 * it can die in pieces.
		 */
		sched_setscheduler_nocheck(old_stop, SCHED_NORMAL, &start_param);
	}
}

/*
 * Change a given task's CPU affinity. Migrate the thread to a
 * proper CPU and schedule it away if the CPU it's executing on
 * is removed from the allowed bitmask.
 *
 * NOTE: the caller must have a valid reference to the task, the
 * task must not exit() & deallocate itself prematurely. The
 * call is not atomic; no spinlocks may be held.
 */
static int __set_cpus_allowed_ptr(struct task_struct *p,
				  const struct cpumask *new_mask, bool check)
{
	const struct cpumask *cpu_valid_mask = cpu_active_mask;
	int dest_cpu;
	unsigned long flags;
	struct rq *rq;
	raw_spinlock_t *lock;
	int ret = 0;

	raw_spin_lock_irqsave(&p->pi_lock, flags);
	rq = __task_access_lock(p, &lock);

	if (p->flags & PF_KTHREAD) {
		/*
		 * Kernel threads are allowed on online && !active CPUs
		 */
		cpu_valid_mask = cpu_online_mask;
	}

	/*
	 * Must re-check here, to close a race against __kthread_bind(),
	 * sched_setaffinity() is not guaranteed to observe the flag.
	 */
	if (check && (p->flags & PF_NO_SETAFFINITY)) {
		ret = -EINVAL;
		goto out;
	}

	if (cpumask_equal(p->cpus_ptr, new_mask))
		goto out;

	dest_cpu = cpumask_any_and(cpu_valid_mask, new_mask);
	if (dest_cpu >= nr_cpu_ids) {
		ret = -EINVAL;
		goto out;
	}

	do_set_cpus_allowed(p, new_mask);

	if (p->flags & PF_KTHREAD) {
		/*
		 * For kernel threads that do indeed end up on online &&
		 * !active we want to ensure they are strict per-CPU threads.
		 */
		WARN_ON(cpumask_intersects(new_mask, cpu_online_mask) &&
			!cpumask_intersects(new_mask, cpu_active_mask) &&
			p->nr_cpus_allowed != 1);
	}

	/* Can the task run on the task's current CPU? If so, we're done */
	if (cpumask_test_cpu(task_cpu(p), new_mask))
		goto out;

	if (task_running(p) || p->state == TASK_WAKING) {
		struct migration_arg arg = { p, dest_cpu };

		/* Need help from migration thread: drop lock and wait. */
		__task_access_unlock(p, lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, flags);
		stop_one_cpu(cpu_of(rq), migration_cpu_stop, &arg);
		return 0;
	}
	if (task_on_rq_queued(p)) {
		/*
		 * OK, since we're going to drop the lock immediately
		 * afterwards anyway.
		 */
		update_rq_clock(rq);
		rq = move_queued_task(rq, p, dest_cpu);
		lock = &rq->lock;
	}

out:
	__task_access_unlock(p, lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

	return ret;
}

int set_cpus_allowed_ptr(struct task_struct *p, const struct cpumask *new_mask)
{
	return __set_cpus_allowed_ptr(p, new_mask, false);
}
EXPORT_SYMBOL_GPL(set_cpus_allowed_ptr);

#else /* CONFIG_SMP */

static inline int select_task_rq(struct task_struct *p)
{
	return 0;
}

static inline int
__set_cpus_allowed_ptr(struct task_struct *p,
		       const struct cpumask *new_mask, bool check)
{
	return set_cpus_allowed_ptr(p, new_mask);
}

#endif /* CONFIG_SMP */

static void
ttwu_stat(struct task_struct *p, int cpu, int wake_flags)
{
	struct rq *rq;

	if (!schedstat_enabled())
		return;

	rq= this_rq();

#ifdef CONFIG_SMP
	if (cpu == rq->cpu)
		__schedstat_inc(rq->ttwu_local);
	else {
		/** BMQ ToDo:
		 * How to do ttwu_wake_remote
		 */
	}
#endif /* CONFIG_SMP */

	__schedstat_inc(rq->ttwu_count);
}

/*
 * Mark the task runnable and perform wakeup-preemption.
 */
static inline void
ttwu_do_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
	p->state = TASK_RUNNING;
	trace_sched_wakeup(p);
}

static inline void
ttwu_do_activate(struct rq *rq, struct task_struct *p, int wake_flags)
{
#ifdef CONFIG_SMP
	if (p->sched_contributes_to_load)
		rq->nr_uninterruptible--;
#endif

	activate_task(p, rq);
	ttwu_do_wakeup(rq, p, 0);
}

static int ttwu_remote(struct task_struct *p, int wake_flags)
{
	struct rq *rq;
	raw_spinlock_t *lock;
	int ret = 0;

	rq = __task_access_lock(p, &lock);
	if (task_on_rq_queued(p)) {
		ttwu_do_wakeup(rq, p, wake_flags);
		ret = 1;
	}
	__task_access_unlock(p, lock);

	return ret;
}

#ifdef CONFIG_SMP
void scheduler_ipi(void)
{
	/*
	 * Fold TIF_NEED_RESCHED into the preempt_count; anybody setting
	 * TIF_NEED_RESCHED remotely (for the first time) will also send
	 * this IPI.
	 */
	preempt_fold_need_resched();

	if (!idle_cpu(smp_processor_id()) || need_resched())
		return;

	irq_enter();
	irq_exit();
}

void wake_up_if_idle(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	rcu_read_lock();

	if (!is_idle_task(rcu_dereference(rq->curr)))
		goto out;

	if (set_nr_if_polling(rq->idle)) {
		trace_sched_wake_idle_without_ipi(cpu);
	} else {
		raw_spin_lock_irqsave(&rq->lock, flags);
		if (is_idle_task(rq->curr))
			smp_send_reschedule(cpu);
		/* Else CPU is not idle, do nothing here */
		raw_spin_unlock_irqrestore(&rq->lock, flags);
	}

out:
	rcu_read_unlock();
}

bool cpus_share_cache(int this_cpu, int that_cpu)
{
	return per_cpu(sd_llc_id, this_cpu) == per_cpu(sd_llc_id, that_cpu);
}
#endif /* CONFIG_SMP */

static inline void ttwu_queue(struct task_struct *p, int cpu, int wake_flags)
{
	struct rq *rq = cpu_rq(cpu);

	raw_spin_lock(&rq->lock);
	update_rq_clock(rq);
	ttwu_do_activate(rq, p, wake_flags);
	check_preempt_curr(rq);
	raw_spin_unlock(&rq->lock);
}

/*
 * Notes on Program-Order guarantees on SMP systems.
 *
 *  MIGRATION
 *
 * The basic program-order guarantee on SMP systems is that when a task [t]
 * migrates, all its activity on its old CPU [c0] happens-before any subsequent
 * execution on its new CPU [c1].
 *
 * For migration (of runnable tasks) this is provided by the following means:
 *
 *  A) UNLOCK of the rq(c0)->lock scheduling out task t
 *  B) migration for t is required to synchronize *both* rq(c0)->lock and
 *     rq(c1)->lock (if not at the same time, then in that order).
 *  C) LOCK of the rq(c1)->lock scheduling in task
 *
 * Transitivity guarantees that B happens after A and C after B.
 * Note: we only require RCpc transitivity.
 * Note: the CPU doing B need not be c0 or c1
 *
 * Example:
 *
 *   CPU0            CPU1            CPU2
 *
 *   LOCK rq(0)->lock
 *   sched-out X
 *   sched-in Y
 *   UNLOCK rq(0)->lock
 *
 *                                   LOCK rq(0)->lock // orders against CPU0
 *                                   dequeue X
 *                                   UNLOCK rq(0)->lock
 *
 *                                   LOCK rq(1)->lock
 *                                   enqueue X
 *                                   UNLOCK rq(1)->lock
 *
 *                   LOCK rq(1)->lock // orders against CPU2
 *                   sched-out Z
 *                   sched-in X
 *                   UNLOCK rq(1)->lock
 *
 *
 *  BLOCKING -- aka. SLEEP + WAKEUP
 *
 * For blocking we (obviously) need to provide the same guarantee as for
 * migration. However the means are completely different as there is no lock
 * chain to provide order. Instead we do:
 *
 *   1) smp_store_release(X->on_cpu, 0)
 *   2) smp_cond_load_acquire(!X->on_cpu)
 *
 * Example:
 *
 *   CPU0 (schedule)  CPU1 (try_to_wake_up) CPU2 (schedule)
 *
 *   LOCK rq(0)->lock LOCK X->pi_lock
 *   dequeue X
 *   sched-out X
 *   smp_store_release(X->on_cpu, 0);
 *
 *                    smp_cond_load_acquire(&X->on_cpu, !VAL);
 *                    X->state = WAKING
 *                    set_task_cpu(X,2)
 *
 *                    LOCK rq(2)->lock
 *                    enqueue X
 *                    X->state = RUNNING
 *                    UNLOCK rq(2)->lock
 *
 *                                          LOCK rq(2)->lock // orders against CPU1
 *                                          sched-out Z
 *                                          sched-in X
 *                                          UNLOCK rq(2)->lock
 *
 *                    UNLOCK X->pi_lock
 *   UNLOCK rq(0)->lock
 *
 *
 * However; for wakeups there is a second guarantee we must provide, namely we
 * must observe the state that lead to our wakeup. That is, not only must our
 * task observe its own prior state, it must also observe the stores prior to
 * its wakeup.
 *
 * This means that any means of doing remote wakeups must order the CPU doing
 * the wakeup against the CPU the task is going to end up running on. This,
 * however, is already required for the regular Program-Order guarantee above,
 * since the waking CPU is the one issueing the ACQUIRE (smp_cond_load_acquire).
 *
 */

/***
 * try_to_wake_up - wake up a thread
 * @p: the thread to be awakened
 * @state: the mask of task states that can be woken
 * @wake_flags: wake modifier flags (WF_*)
 *
 * Put it on the run-queue if it's not already there. The "current"
 * thread is always on the run-queue (except when the actual
 * re-schedule is in progress), and as such you're allowed to do
 * the simpler "current->state = TASK_RUNNING" to mark yourself
 * runnable without the overhead of this.
 *
 * Return: %true if @p was woken up, %false if it was already running.
 * or @state didn't match @p's state.
 */
static int try_to_wake_up(struct task_struct *p, unsigned int state,
			  int wake_flags)
{
	unsigned long flags;
	int cpu, success = 0;

	preempt_disable();
	if (p == current) {
		/*
		 * We're waking current, this means 'p->on_rq' and 'task_cpu(p)
		 * == smp_processor_id()'. Together this means we can special
		 * case the whole 'p->on_rq && ttwu_remote()' case below
		 * without taking any locks.
		 *
		 * In particular:
		 *  - we rely on Program-Order guarantees for all the ordering,
		 *  - we're serialized against set_special_state() by virtue of
		 *    it disabling IRQs (this allows not taking ->pi_lock).
		 */
		if (!(p->state & state))
			goto out;

		success = 1;
		cpu = task_cpu(p);
		trace_sched_waking(p);
		p->state = TASK_RUNNING;
		trace_sched_wakeup(p);
		goto out;
	}

	/*
	 * If we are going to wake up a thread waiting for CONDITION we
	 * need to ensure that CONDITION=1 done by the caller can not be
	 * reordered with p->state check below. This pairs with mb() in
	 * set_current_state() the waiting thread does.
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);
	smp_mb__after_spinlock();
	if (!(p->state & state))
		goto unlock;

	trace_sched_waking(p);

	/* We're going to change ->state: */
	success = 1;
	cpu = task_cpu(p);

	/*
	 * Ensure we load p->on_rq _after_ p->state, otherwise it would
	 * be possible to, falsely, observe p->on_rq == 0 and get stuck
	 * in smp_cond_load_acquire() below.
	 *
	 * sched_ttwu_pending()			try_to_wake_up()
	 *   STORE p->on_rq = 1			  LOAD p->state
	 *   UNLOCK rq->lock
	 *
	 * __schedule() (switch to task 'p')
	 *   LOCK rq->lock			  smp_rmb();
	 *   smp_mb__after_spinlock();
	 *   UNLOCK rq->lock
	 *
	 * [task p]
	 *   STORE p->state = UNINTERRUPTIBLE	  LOAD p->on_rq
	 *
	 * Pairs with the LOCK+smp_mb__after_spinlock() on rq->lock in
	 * __schedule().  See the comment for smp_mb__after_spinlock().
	 */
	smp_rmb();
	if (p->on_rq && ttwu_remote(p, wake_flags))
		goto unlock;

#ifdef CONFIG_SMP
	/*
	 * Ensure we load p->on_cpu _after_ p->on_rq, otherwise it would be
	 * possible to, falsely, observe p->on_cpu == 0.
	 *
	 * One must be running (->on_cpu == 1) in order to remove oneself
	 * from the runqueue.
	 *
	 * __schedule() (switch to task 'p')	try_to_wake_up()
	 *   STORE p->on_cpu = 1		  LOAD p->on_rq
	 *   UNLOCK rq->lock
	 *
	 * __schedule() (put 'p' to sleep)
	 *   LOCK rq->lock			  smp_rmb();
	 *   smp_mb__after_spinlock();
	 *   STORE p->on_rq = 0			  LOAD p->on_cpu
	 *
	 * Pairs with the LOCK+smp_mb__after_spinlock() on rq->lock in
	 * __schedule().  See the comment for smp_mb__after_spinlock().
	 */
	smp_rmb();

	/*
	 * If the owning (remote) CPU is still in the middle of schedule() with
	 * this task as prev, wait until its done referencing the task.
	 *
	 * Pairs with the smp_store_release() in finish_task().
	 *
	 * This ensures that tasks getting woken will be fully ordered against
	 * their previous state and preserve Program Order.
	 */
	smp_cond_load_acquire(&p->on_cpu, !VAL);

	p->sched_contributes_to_load = !!task_contributes_to_load(p);
	p->state = TASK_WAKING;

	if (p->in_iowait) {
		delayacct_blkio_end(p);
		atomic_dec(&task_rq(p)->nr_iowait);
	}

	if(this_rq()->clock_task - p->last_ran > sched_timeslice_ns)
		boost_task(p);

	cpu = select_task_rq(p);

	if (cpu != task_cpu(p)) {
		wake_flags |= WF_MIGRATED;
		psi_ttwu_dequeue(p);
		set_task_cpu(p, cpu);
	}
#else /* CONFIG_SMP */
	if (p->in_iowait) {
		delayacct_blkio_end(p);
		atomic_dec(&task_rq(p)->nr_iowait);
	}
#endif /* CONFIG_SMP */

	ttwu_queue(p, cpu, wake_flags);
unlock:
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);
out:
	if (success)
		ttwu_stat(p, cpu, wake_flags);
	preempt_enable();

	return success;
}

/**
 * wake_up_process - Wake up a specific process
 * @p: The process to be woken up.
 *
 * Attempt to wake up the nominated process and move it to the set of runnable
 * processes.
 *
 * Return: 1 if the process was woken up, 0 if it was already running.
 *
 * This function executes a full memory barrier before accessing the task state.
 */
int wake_up_process(struct task_struct *p)
{
	return try_to_wake_up(p, TASK_NORMAL, 0);
}
EXPORT_SYMBOL(wake_up_process);

int wake_up_state(struct task_struct *p, unsigned int state)
{
	return try_to_wake_up(p, state, 0);
}

/*
 * Perform scheduler related setup for a newly forked process p.
 * p is forked by current.
 *
 * __sched_fork() is basic setup used by init_idle() too:
 */
static inline void __sched_fork(unsigned long clone_flags, struct task_struct *p)
{
	p->on_rq			= 0;
	p->on_cpu			= 0;
	p->utime			= 0;
	p->stime			= 0;
	p->sched_time			= 0;

#ifdef CONFIG_PREEMPT_NOTIFIERS
	INIT_HLIST_HEAD(&p->preempt_notifiers);
#endif

#ifdef CONFIG_COMPACTION
	p->capture_control = NULL;
#endif
}

/*
 * fork()/clone()-time setup:
 */
int sched_fork(unsigned long clone_flags, struct task_struct *p)
{
	unsigned long flags;
	struct rq *rq;

	__sched_fork(clone_flags, p);
	/*
	 * We mark the process as NEW here. This guarantees that
	 * nobody will actually run it, and a signal or other external
	 * event cannot wake it up and insert it on the runqueue either.
	 */
	p->state = TASK_NEW;

	/*
	 * Make sure we do not leak PI boosting priority to the child.
	 */
	p->prio = current->normal_prio;

	/*
	 * Revert to default priority/policy on fork if requested.
	 */
	if (unlikely(p->sched_reset_on_fork)) {
		if (task_has_rt_policy(p)) {
			p->policy = SCHED_NORMAL;
			p->static_prio = NICE_TO_PRIO(0);
			p->rt_priority = 0;
		} else if (PRIO_TO_NICE(p->static_prio) < 0)
			p->static_prio = NICE_TO_PRIO(0);

		p->prio = p->normal_prio = normal_prio(p);

		/*
		 * We don't need the reset flag anymore after the fork. It has
		 * fulfilled its duty:
		 */
		p->sched_reset_on_fork = 0;
	}

	p->boost_prio = (p->boost_prio < 0) ?
		p->boost_prio + MAX_PRIORITY_ADJ : MAX_PRIORITY_ADJ;
	/*
	 * The child is not yet in the pid-hash so no cgroup attach races,
	 * and the cgroup is pinned to this child due to cgroup_fork()
	 * is ran before sched_fork().
	 *
	 * Silence PROVE_RCU.
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);
	/*
	 * Share the timeslice between parent and child, thus the
	 * total amount of pending timeslices in the system doesn't change,
	 * resulting in more scheduling fairness.
	 */
	rq = this_rq();
	raw_spin_lock(&rq->lock);
	rq->curr->time_slice /= 2;
	p->time_slice = rq->curr->time_slice;
#ifdef CONFIG_SCHED_HRTICK
	hrtick_start(rq, rq->curr->time_slice);
#endif

	if (p->time_slice < RESCHED_NS) {
		p->time_slice = sched_timeslice_ns;
		resched_curr(rq);
	}
	raw_spin_unlock(&rq->lock);

	/*
	 * We're setting the CPU for the first time, we don't migrate,
	 * so use __set_task_cpu().
	 */
	__set_task_cpu(p, cpu_of(rq));
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

#ifdef CONFIG_SCHED_INFO
	if (unlikely(sched_info_on()))
		memset(&p->sched_info, 0, sizeof(p->sched_info));
#endif
	init_task_preempt_count(p);

	return 0;
}

#ifdef CONFIG_SCHEDSTATS

DEFINE_STATIC_KEY_FALSE(sched_schedstats);
static bool __initdata __sched_schedstats = false;

static void set_schedstats(bool enabled)
{
	if (enabled)
		static_branch_enable(&sched_schedstats);
	else
		static_branch_disable(&sched_schedstats);
}

void force_schedstat_enabled(void)
{
	if (!schedstat_enabled()) {
		pr_info("kernel profiling enabled schedstats, disable via kernel.sched_schedstats.\n");
		static_branch_enable(&sched_schedstats);
	}
}

static int __init setup_schedstats(char *str)
{
	int ret = 0;
	if (!str)
		goto out;

	/*
	 * This code is called before jump labels have been set up, so we can't
	 * change the static branch directly just yet.  Instead set a temporary
	 * variable so init_schedstats() can do it later.
	 */
	if (!strcmp(str, "enable")) {
		__sched_schedstats = true;
		ret = 1;
	} else if (!strcmp(str, "disable")) {
		__sched_schedstats = false;
		ret = 1;
	}
out:
	if (!ret)
		pr_warn("Unable to parse schedstats=\n");

	return ret;
}
__setup("schedstats=", setup_schedstats);

static void __init init_schedstats(void)
{
	set_schedstats(__sched_schedstats);
}

#ifdef CONFIG_PROC_SYSCTL
int sysctl_schedstats(struct ctl_table *table, int write,
			 void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct ctl_table t;
	int err;
	int state = static_branch_likely(&sched_schedstats);

	if (write && !capable(CAP_SYS_ADMIN))
		return -EPERM;

	t = *table;
	t.data = &state;
	err = proc_dointvec_minmax(&t, write, buffer, lenp, ppos);
	if (err < 0)
		return err;
	if (write)
		set_schedstats(state);
	return err;
}
#endif /* CONFIG_PROC_SYSCTL */
#else  /* !CONFIG_SCHEDSTATS */
static inline void init_schedstats(void) {}
#endif /* CONFIG_SCHEDSTATS */

/*
 * wake_up_new_task - wake up a newly created task for the first time.
 *
 * This function will do some initial scheduler statistics housekeeping
 * that must be done for every newly created context, then puts the task
 * on the runqueue and wakes it.
 */
void wake_up_new_task(struct task_struct *p)
{
	unsigned long flags;
	struct rq *rq;

	raw_spin_lock_irqsave(&p->pi_lock, flags);

	p->state = TASK_RUNNING;

	rq = cpu_rq(select_task_rq(p));
#ifdef CONFIG_SMP
	/*
	 * Fork balancing, do it here and not earlier because:
	 * - cpus_ptr can change in the fork path
	 * - any previously selected CPU might disappear through hotplug
	 * Use __set_task_cpu() to avoid calling sched_class::migrate_task_rq,
	 * as we're not fully set-up yet.
	 */
	__set_task_cpu(p, cpu_of(rq));
#endif

	raw_spin_lock(&rq->lock);

	update_rq_clock(rq);
	activate_task(p, rq);
	trace_sched_wakeup_new(p);
	check_preempt_curr(rq);

	raw_spin_unlock(&rq->lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);
}

#ifdef CONFIG_PREEMPT_NOTIFIERS

static DEFINE_STATIC_KEY_FALSE(preempt_notifier_key);

void preempt_notifier_inc(void)
{
	static_branch_inc(&preempt_notifier_key);
}
EXPORT_SYMBOL_GPL(preempt_notifier_inc);

void preempt_notifier_dec(void)
{
	static_branch_dec(&preempt_notifier_key);
}
EXPORT_SYMBOL_GPL(preempt_notifier_dec);

/**
 * preempt_notifier_register - tell me when current is being preempted & rescheduled
 * @notifier: notifier struct to register
 */
void preempt_notifier_register(struct preempt_notifier *notifier)
{
	if (!static_branch_unlikely(&preempt_notifier_key))
		WARN(1, "registering preempt_notifier while notifiers disabled\n");

	hlist_add_head(&notifier->link, &current->preempt_notifiers);
}
EXPORT_SYMBOL_GPL(preempt_notifier_register);

/**
 * preempt_notifier_unregister - no longer interested in preemption notifications
 * @notifier: notifier struct to unregister
 *
 * This is *not* safe to call from within a preemption notifier.
 */
void preempt_notifier_unregister(struct preempt_notifier *notifier)
{
	hlist_del(&notifier->link);
}
EXPORT_SYMBOL_GPL(preempt_notifier_unregister);

static void __fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
	struct preempt_notifier *notifier;

	hlist_for_each_entry(notifier, &curr->preempt_notifiers, link)
		notifier->ops->sched_in(notifier, raw_smp_processor_id());
}

static __always_inline void fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
	if (static_branch_unlikely(&preempt_notifier_key))
		__fire_sched_in_preempt_notifiers(curr);
}

static void
__fire_sched_out_preempt_notifiers(struct task_struct *curr,
				   struct task_struct *next)
{
	struct preempt_notifier *notifier;

	hlist_for_each_entry(notifier, &curr->preempt_notifiers, link)
		notifier->ops->sched_out(notifier, next);
}

static __always_inline void
fire_sched_out_preempt_notifiers(struct task_struct *curr,
				 struct task_struct *next)
{
	if (static_branch_unlikely(&preempt_notifier_key))
		__fire_sched_out_preempt_notifiers(curr, next);
}

#else /* !CONFIG_PREEMPT_NOTIFIERS */

static inline void fire_sched_in_preempt_notifiers(struct task_struct *curr)
{
}

static inline void
fire_sched_out_preempt_notifiers(struct task_struct *curr,
				 struct task_struct *next)
{
}

#endif /* CONFIG_PREEMPT_NOTIFIERS */

static inline void prepare_task(struct task_struct *next)
{
	/*
	 * Claim the task as running, we do this before switching to it
	 * such that any running task will have this set.
	 */
	next->on_cpu = 1;
}

static inline void finish_task(struct task_struct *prev)
{
#ifdef CONFIG_SMP
	/*
	 * After ->on_cpu is cleared, the task can be moved to a different CPU.
	 * We must ensure this doesn't happen until the switch is completely
	 * finished.
	 *
	 * In particular, the load of prev->state in finish_task_switch() must
	 * happen before this.
	 *
	 * Pairs with the smp_cond_load_acquire() in try_to_wake_up().
	 */
	smp_store_release(&prev->on_cpu, 0);
#else
	prev->on_cpu = 0;
#endif
}

static inline void
prepare_lock_switch(struct rq *rq, struct task_struct *next)
{
	/*
	 * Since the runqueue lock will be released by the next
	 * task (which is an invalid locking op but in the case
	 * of the scheduler it's an obvious special-case), so we
	 * do an early lockdep release here:
	 */
	spin_release(&rq->lock.dep_map, _THIS_IP_);
#ifdef CONFIG_DEBUG_SPINLOCK
	/* this is a valid case when another task releases the spinlock */
	rq->lock.owner = next;
#endif
}

static inline void finish_lock_switch(struct rq *rq)
{
	/*
	 * If we are tracking spinlock dependencies then we have to
	 * fix up the runqueue lock - which gets 'carried over' from
	 * prev into current:
	 */
	spin_acquire(&rq->lock.dep_map, 0, 0, _THIS_IP_);
	raw_spin_unlock_irq(&rq->lock);
}

/**
 * prepare_task_switch - prepare to switch tasks
 * @rq: the runqueue preparing to switch
 * @next: the task we are going to switch to.
 *
 * This is called with the rq lock held and interrupts off. It must
 * be paired with a subsequent finish_task_switch after the context
 * switch.
 *
 * prepare_task_switch sets up locking and calls architecture specific
 * hooks.
 */
static inline void
prepare_task_switch(struct rq *rq, struct task_struct *prev,
		    struct task_struct *next)
{
	kcov_prepare_switch(prev);
	sched_info_switch(rq, prev, next);
	perf_event_task_sched_out(prev, next);
	rseq_preempt(prev);
	fire_sched_out_preempt_notifiers(prev, next);
	prepare_task(next);
	prepare_arch_switch(next);
}

/**
 * finish_task_switch - clean up after a task-switch
 * @rq: runqueue associated with task-switch
 * @prev: the thread we just switched away from.
 *
 * finish_task_switch must be called after the context switch, paired
 * with a prepare_task_switch call before the context switch.
 * finish_task_switch will reconcile locking set up by prepare_task_switch,
 * and do any other architecture-specific cleanup actions.
 *
 * Note that we may have delayed dropping an mm in context_switch(). If
 * so, we finish that here outside of the runqueue lock.  (Doing it
 * with the lock held can cause deadlocks; see schedule() for
 * details.)
 *
 * The context switch have flipped the stack from under us and restored the
 * local variables which were saved when this task called schedule() in the
 * past. prev == current is still correct but we need to recalculate this_rq
 * because prev may have moved to another CPU.
 */
static struct rq *finish_task_switch(struct task_struct *prev)
	__releases(rq->lock)
{
	struct rq *rq = this_rq();
	struct mm_struct *mm = rq->prev_mm;
	long prev_state;

	/*
	 * The previous task will have left us with a preempt_count of 2
	 * because it left us after:
	 *
	 *	schedule()
	 *	  preempt_disable();			// 1
	 *	  __schedule()
	 *	    raw_spin_lock_irq(&rq->lock)	// 2
	 *
	 * Also, see FORK_PREEMPT_COUNT.
	 */
	if (WARN_ONCE(preempt_count() != 2*PREEMPT_DISABLE_OFFSET,
		      "corrupted preempt_count: %s/%d/0x%x\n",
		      current->comm, current->pid, preempt_count()))
		preempt_count_set(FORK_PREEMPT_COUNT);

	rq->prev_mm = NULL;

	/*
	 * A task struct has one reference for the use as "current".
	 * If a task dies, then it sets TASK_DEAD in tsk->state and calls
	 * schedule one last time. The schedule call will never return, and
	 * the scheduled task must drop that reference.
	 *
	 * We must observe prev->state before clearing prev->on_cpu (in
	 * finish_task), otherwise a concurrent wakeup can get prev
	 * running on another CPU and we could rave with its RUNNING -> DEAD
	 * transition, resulting in a double drop.
	 */
	prev_state = prev->state;
	vtime_task_switch(prev);
	perf_event_task_sched_in(prev, current);
	finish_task(prev);
	finish_lock_switch(rq);
	finish_arch_post_lock_switch();
	kcov_finish_switch(current);

	fire_sched_in_preempt_notifiers(current);
	/*
	 * When switching through a kernel thread, the loop in
	 * membarrier_{private,global}_expedited() may have observed that
	 * kernel thread and not issued an IPI. It is therefore possible to
	 * schedule between user->kernel->user threads without passing though
	 * switch_mm(). Membarrier requires a barrier after storing to
	 * rq->curr, before returning to userspace, so provide them here:
	 *
	 * - a full memory barrier for {PRIVATE,GLOBAL}_EXPEDITED, implicitly
	 *   provided by mmdrop(),
	 * - a sync_core for SYNC_CORE.
	 */
	if (mm) {
		membarrier_mm_sync_core_before_usermode(mm);
		mmdrop(mm);
	}
	if (unlikely(prev_state == TASK_DEAD)) {
		/*
		 * Remove function-return probe instances associated with this
		 * task and put them back on the free list.
		 */
		kprobe_flush_task(prev);

		/* Task is done with its stack. */
		put_task_stack(prev);

		put_task_struct_rcu_user(prev);
	}

	tick_nohz_task_switch();
	return rq;
}

/**
 * schedule_tail - first thing a freshly forked thread must call.
 * @prev: the thread we just switched away from.
 */
asmlinkage __visible void schedule_tail(struct task_struct *prev)
	__releases(rq->lock)
{
	struct rq *rq;

	/*
	 * New tasks start with FORK_PREEMPT_COUNT, see there and
	 * finish_task_switch() for details.
	 *
	 * finish_task_switch() will drop rq->lock() and lower preempt_count
	 * and the preempt_enable() will end up enabling preemption (on
	 * PREEMPT_COUNT kernels).
	 */

	rq = finish_task_switch(prev);
	preempt_enable();

	if (current->set_child_tid)
		put_user(task_pid_vnr(current), current->set_child_tid);

	calculate_sigpending();
}

/*
 * context_switch - switch to the new MM and the new thread's register state.
 */
static __always_inline struct rq *
context_switch(struct rq *rq, struct task_struct *prev,
	       struct task_struct *next)
{
	prepare_task_switch(rq, prev, next);

	/*
	 * For paravirt, this is coupled with an exit in switch_to to
	 * combine the page table reload and the switch backend into
	 * one hypercall.
	 */
	arch_start_context_switch(prev);

	/*
	 * kernel -> kernel   lazy + transfer active
	 *   user -> kernel   lazy + mmgrab() active
	 *
	 * kernel ->   user   switch + mmdrop() active
	 *   user ->   user   switch
	 */
	if (!next->mm) {                                // to kernel
		enter_lazy_tlb(prev->active_mm, next);

		next->active_mm = prev->active_mm;
		if (prev->mm)                           // from user
			mmgrab(prev->active_mm);
		else
			prev->active_mm = NULL;
	} else {                                        // to user
		membarrier_switch_mm(rq, prev->active_mm, next->mm);
		/*
		 * sys_membarrier() requires an smp_mb() between setting
		 * rq->curr / membarrier_switch_mm() and returning to userspace.
		 *
		 * The below provides this either through switch_mm(), or in
		 * case 'prev->active_mm == next->mm' through
		 * finish_task_switch()'s mmdrop().
		 */
		switch_mm_irqs_off(prev->active_mm, next->mm, next);

		if (!prev->mm) {                        // from kernel
			/* will mmdrop() in finish_task_switch(). */
			rq->prev_mm = prev->active_mm;
			prev->active_mm = NULL;
		}
	}

	prepare_lock_switch(rq, next);

	/* Here we just switch the register state and the stack. */
	switch_to(prev, next, prev);
	barrier();

	return finish_task_switch(prev);
}

/*
 * nr_running, nr_uninterruptible and nr_context_switches:
 *
 * externally visible scheduler statistics: current number of runnable
 * threads, total number of context switches performed since bootup.
 */
unsigned long nr_running(void)
{
	unsigned long i, sum = 0;

	for_each_online_cpu(i)
		sum += cpu_rq(i)->nr_running;

	return sum;
}

/*
 * Check if only the current task is running on the CPU.
 *
 * Caution: this function does not check that the caller has disabled
 * preemption, thus the result might have a time-of-check-to-time-of-use
 * race.  The caller is responsible to use it correctly, for example:
 *
 * - from a non-preemptible section (of course)
 *
 * - from a thread that is bound to a single CPU
 *
 * - in a loop with very short iterations (e.g. a polling loop)
 */
bool single_task_running(void)
{
	return raw_rq()->nr_running == 1;
}
EXPORT_SYMBOL(single_task_running);

unsigned long long nr_context_switches(void)
{
	int i;
	unsigned long long sum = 0;

	for_each_possible_cpu(i)
		sum += cpu_rq(i)->nr_switches;

	return sum;
}

/*
 * Consumers of these two interfaces, like for example the cpuidle menu
 * governor, are using nonsensical data. Preferring shallow idle state selection
 * for a CPU that has IO-wait which might not even end up running the task when
 * it does become runnable.
 */

unsigned long nr_iowait_cpu(int cpu)
{
	return atomic_read(&cpu_rq(cpu)->nr_iowait);
}

/*
 * IO-wait accounting, and how its mostly bollocks (on SMP).
 *
 * The idea behind IO-wait account is to account the idle time that we could
 * have spend running if it were not for IO. That is, if we were to improve the
 * storage performance, we'd have a proportional reduction in IO-wait time.
 *
 * This all works nicely on UP, where, when a task blocks on IO, we account
 * idle time as IO-wait, because if the storage were faster, it could've been
 * running and we'd not be idle.
 *
 * This has been extended to SMP, by doing the same for each CPU. This however
 * is broken.
 *
 * Imagine for instance the case where two tasks block on one CPU, only the one
 * CPU will have IO-wait accounted, while the other has regular idle. Even
 * though, if the storage were faster, both could've ran at the same time,
 * utilising both CPUs.
 *
 * This means, that when looking globally, the current IO-wait accounting on
 * SMP is a lower bound, by reason of under accounting.
 *
 * Worse, since the numbers are provided per CPU, they are sometimes
 * interpreted per CPU, and that is nonsensical. A blocked task isn't strictly
 * associated with any one particular CPU, it can wake to another CPU than it
 * blocked on. This means the per CPU IO-wait number is meaningless.
 *
 * Task CPU affinities can make all that even more 'interesting'.
 */

unsigned long nr_iowait(void)
{
	unsigned long i, sum = 0;

	for_each_possible_cpu(i)
		sum += nr_iowait_cpu(i);

	return sum;
}

#ifdef CONFIG_SMP

/*
 * sched_exec - execve() is a valuable balancing opportunity, because at
 * this point the task has the smallest effective memory and cache
 * footprint.
 */
void sched_exec(void)
{
	struct task_struct *p = current;
	int dest_cpu;

	if (task_rq(p)->nr_running < 2)
		return;

	dest_cpu = cpumask_any_and(p->cpus_ptr, &sched_rq_watermark[IDLE_WM]);
	if ( dest_cpu < nr_cpu_ids) {
#ifdef CONFIG_SCHED_SMT
		int smt = cpumask_any_and(p->cpus_ptr, &sched_sg_idle_mask);
		if (smt < nr_cpu_ids)
			dest_cpu = smt;
#endif
		if (likely(cpu_active(dest_cpu))) {
			struct migration_arg arg = { p, dest_cpu };

			stop_one_cpu(task_cpu(p), migration_cpu_stop, &arg);
			return;
		}
	}
}

#endif

DEFINE_PER_CPU(struct kernel_stat, kstat);
DEFINE_PER_CPU(struct kernel_cpustat, kernel_cpustat);

EXPORT_PER_CPU_SYMBOL(kstat);
EXPORT_PER_CPU_SYMBOL(kernel_cpustat);

static inline void update_curr(struct rq *rq, struct task_struct *p)
{
	s64 ns = rq->clock_task - p->last_ran;

	p->sched_time += ns;
	account_group_exec_runtime(p, ns);

	p->time_slice -= ns;
	p->last_ran = rq->clock_task;
}

/*
 * Return accounted runtime for the task.
 * Return separately the current's pending runtime that have not been
 * accounted yet.
 */
unsigned long long task_sched_runtime(struct task_struct *p)
{
	unsigned long flags;
	struct rq *rq;
	raw_spinlock_t *lock;
	u64 ns;

#if defined(CONFIG_64BIT) && defined(CONFIG_SMP)
	/*
	 * 64-bit doesn't need locks to atomically read a 64-bit value.
	 * So we have a optimization chance when the task's delta_exec is 0.
	 * Reading ->on_cpu is racy, but this is ok.
	 *
	 * If we race with it leaving CPU, we'll take a lock. So we're correct.
	 * If we race with it entering CPU, unaccounted time is 0. This is
	 * indistinguishable from the read occurring a few cycles earlier.
	 * If we see ->on_cpu without ->on_rq, the task is leaving, and has
	 * been accounted, so we're correct here as well.
	 */
	if (!p->on_cpu || !task_on_rq_queued(p))
		return tsk_seruntime(p);
#endif

	rq = task_access_lock_irqsave(p, &lock, &flags);
	/*
	 * Must be ->curr _and_ ->on_rq.  If dequeued, we would
	 * project cycles that may never be accounted to this
	 * thread, breaking clock_gettime().
	 */
	if (p == rq->curr && task_on_rq_queued(p)) {
		update_rq_clock(rq);
		update_curr(rq, p);
	}
	ns = tsk_seruntime(p);
	task_access_unlock_irqrestore(p, lock, &flags);

	return ns;
}

DEFINE_PER_CPU(unsigned long, thermal_pressure);

void arch_set_thermal_pressure(struct cpumask *cpus,
			       unsigned long th_pressure)
{
	int cpu;

	for_each_cpu(cpu, cpus)
		WRITE_ONCE(per_cpu(thermal_pressure, cpu), th_pressure);
}

/* This manages tasks that have run out of timeslice during a scheduler_tick */
static inline void scheduler_task_tick(struct rq *rq)
{
	struct task_struct *p = rq->curr;

	if (is_idle_task(p))
		return;

	update_curr(rq, p);
	cpufreq_update_util(rq, 0);

	/*
	 * Tasks have less than RESCHED_NS of time slice left they will be
	 * rescheduled.
	 */
	if (p->time_slice >= RESCHED_NS)
		return;
	set_tsk_need_resched(p);
	set_preempt_need_resched();
}

/*
 * This function gets called by the timer code, with HZ frequency.
 * We call it with interrupts disabled.
 */
void scheduler_tick(void)
{
	int cpu __maybe_unused = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);

	arch_scale_freq_tick();
	sched_clock_tick();

	raw_spin_lock(&rq->lock);
	update_rq_clock(rq);

	scheduler_task_tick(rq);
	calc_global_load_tick(rq);
	psi_task_tick(rq);

	rq->last_tick = rq->clock;
	raw_spin_unlock(&rq->lock);

	perf_event_task_tick();
}

#ifdef CONFIG_SCHED_SMT
static inline int active_load_balance_cpu_stop(void *data)
{
	struct rq *rq = this_rq();
	struct task_struct *p = data;
	cpumask_t tmp;
	unsigned long flags;

	local_irq_save(flags);

	raw_spin_lock(&p->pi_lock);
	raw_spin_lock(&rq->lock);

	rq->active_balance = 0;
	/* _something_ may have changed the task, double check again */
	if (task_on_rq_queued(p) && task_rq(p) == rq &&
	    cpumask_and(&tmp, p->cpus_ptr, &sched_sg_idle_mask)) {
		int cpu = cpu_of(rq);
		int dcpu = __best_mask_cpu(cpu, &tmp,
					   per_cpu(sched_cpu_llc_mask, cpu));
		rq = move_queued_task(rq, p, dcpu);
	}

	raw_spin_unlock(&rq->lock);
	raw_spin_unlock(&p->pi_lock);

	local_irq_restore(flags);

	return 0;
}

/* sg_balance_trigger - trigger slibing group balance for @cpu */
static inline int sg_balance_trigger(const int cpu)
{
	struct rq *rq= cpu_rq(cpu);
	unsigned long flags;
	struct task_struct *curr;
	int res;

	if (!raw_spin_trylock_irqsave(&rq->lock, flags))
		return 0;
	curr = rq->curr;
	res = (!is_idle_task(curr)) && (1 == rq->nr_running) &&\
	      cpumask_intersects(curr->cpus_ptr, &sched_sg_idle_mask) &&\
	      (!rq->active_balance);

	if (res)
		rq->active_balance = 1;

	raw_spin_unlock_irqrestore(&rq->lock, flags);

	if (res)
		stop_one_cpu_nowait(cpu, active_load_balance_cpu_stop,
				    curr, &rq->active_balance_work);
	return res;
}

/*
 * sg_balance_check - slibing group balance check for run queue @rq
 */
static inline void sg_balance_check(struct rq *rq)
{
	cpumask_t chk;
	int cpu;

	/* exit when no sg in idle */
	if (cpumask_empty(&sched_sg_idle_mask))
		return;

	cpu = cpu_of(rq);
	/*
	 * Only cpu in slibing idle group will do the checking and then
	 * find potential cpus which can migrate the current running task
	 */
	if (cpumask_test_cpu(cpu, &sched_sg_idle_mask) &&
	    cpumask_andnot(&chk, cpu_online_mask, &sched_rq_pending_mask) &&
	    cpumask_andnot(&chk, &chk, &sched_rq_watermark[IDLE_WM])) {
		int i, tried = 0;

		for_each_cpu_wrap(i, &chk, cpu) {
			if (cpumask_subset(cpu_smt_mask(i), &chk)) {
				if (sg_balance_trigger(i))
					return;
				if (tried)
					return;
				tried++;
			}
		}
	}
}
#endif /* CONFIG_SCHED_SMT */

#ifdef CONFIG_NO_HZ_FULL

struct tick_work {
	int			cpu;
	atomic_t		state;
	struct delayed_work	work;
};
/* Values for ->state, see diagram below. */
#define TICK_SCHED_REMOTE_OFFLINE	0
#define TICK_SCHED_REMOTE_OFFLINING	1
#define TICK_SCHED_REMOTE_RUNNING	2

/*
 * State diagram for ->state:
 *
 *
 *          TICK_SCHED_REMOTE_OFFLINE
 *                    |   ^
 *                    |   |
 *                    |   | sched_tick_remote()
 *                    |   |
 *                    |   |
 *                    +--TICK_SCHED_REMOTE_OFFLINING
 *                    |   ^
 *                    |   |
 * sched_tick_start() |   | sched_tick_stop()
 *                    |   |
 *                    V   |
 *          TICK_SCHED_REMOTE_RUNNING
 *
 *
 * Other transitions get WARN_ON_ONCE(), except that sched_tick_remote()
 * and sched_tick_start() are happy to leave the state in RUNNING.
 */

static struct tick_work __percpu *tick_work_cpu;

static void sched_tick_remote(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tick_work *twork = container_of(dwork, struct tick_work, work);
	int cpu = twork->cpu;
	struct rq *rq = cpu_rq(cpu);
	struct task_struct *curr;
	unsigned long flags;
	u64 delta;
	int os;

	/*
	 * Handle the tick only if it appears the remote CPU is running in full
	 * dynticks mode. The check is racy by nature, but missing a tick or
	 * having one too much is no big deal because the scheduler tick updates
	 * statistics and checks timeslices in a time-independent way, regardless
	 * of when exactly it is running.
	 */
	if (!tick_nohz_tick_stopped_cpu(cpu))
		goto out_requeue;

	raw_spin_lock_irqsave(&rq->lock, flags);
	curr = rq->curr;
	if (cpu_is_offline(cpu))
		goto out_unlock;

	update_rq_clock(rq);
	if (!is_idle_task(curr)) {
		/*
		 * Make sure the next tick runs within a reasonable
		 * amount of time.
		 */
		delta = rq_clock_task(rq) - curr->last_ran;
		WARN_ON_ONCE(delta > (u64)NSEC_PER_SEC * 3);
	}
	scheduler_task_tick(rq);

	calc_load_nohz_remote(rq);
out_unlock:
	raw_spin_unlock_irqrestore(&rq->lock, flags);

out_requeue:
	/*
	 * Run the remote tick once per second (1Hz). This arbitrary
	 * frequency is large enough to avoid overload but short enough
	 * to keep scheduler internal stats reasonably up to date.  But
	 * first update state to reflect hotplug activity if required.
	 */
	os = atomic_fetch_add_unless(&twork->state, -1, TICK_SCHED_REMOTE_RUNNING);
	WARN_ON_ONCE(os == TICK_SCHED_REMOTE_OFFLINE);
	if (os == TICK_SCHED_REMOTE_RUNNING)
		queue_delayed_work(system_unbound_wq, dwork, HZ);
}

static void sched_tick_start(int cpu)
{
	int os;
	struct tick_work *twork;

	if (housekeeping_cpu(cpu, HK_FLAG_TICK))
		return;

	WARN_ON_ONCE(!tick_work_cpu);

	twork = per_cpu_ptr(tick_work_cpu, cpu);
	os = atomic_xchg(&twork->state, TICK_SCHED_REMOTE_RUNNING);
	WARN_ON_ONCE(os == TICK_SCHED_REMOTE_RUNNING);
	if (os == TICK_SCHED_REMOTE_OFFLINE) {
		twork->cpu = cpu;
		INIT_DELAYED_WORK(&twork->work, sched_tick_remote);
		queue_delayed_work(system_unbound_wq, &twork->work, HZ);
	}
}

#ifdef CONFIG_HOTPLUG_CPU
static void sched_tick_stop(int cpu)
{
	struct tick_work *twork;

	if (housekeeping_cpu(cpu, HK_FLAG_TICK))
		return;

	WARN_ON_ONCE(!tick_work_cpu);

	twork = per_cpu_ptr(tick_work_cpu, cpu);
	cancel_delayed_work_sync(&twork->work);
}
#endif /* CONFIG_HOTPLUG_CPU */

int __init sched_tick_offload_init(void)
{
	tick_work_cpu = alloc_percpu(struct tick_work);
	BUG_ON(!tick_work_cpu);
	return 0;
}

#else /* !CONFIG_NO_HZ_FULL */
static inline void sched_tick_start(int cpu) { }
static inline void sched_tick_stop(int cpu) { }
#endif

#if defined(CONFIG_PREEMPTION) && (defined(CONFIG_DEBUG_PREEMPT) || \
				defined(CONFIG_PREEMPT_TRACER))
/*
 * If the value passed in is equal to the current preempt count
 * then we just disabled preemption. Start timing the latency.
 */
static inline void preempt_latency_start(int val)
{
	if (preempt_count() == val) {
		unsigned long ip = get_lock_parent_ip();
#ifdef CONFIG_DEBUG_PREEMPT
		current->preempt_disable_ip = ip;
#endif
		trace_preempt_off(CALLER_ADDR0, ip);
	}
}

void preempt_count_add(int val)
{
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	if (DEBUG_LOCKS_WARN_ON((preempt_count() < 0)))
		return;
#endif
	__preempt_count_add(val);
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Spinlock count overflowing soon?
	 */
	DEBUG_LOCKS_WARN_ON((preempt_count() & PREEMPT_MASK) >=
				PREEMPT_MASK - 10);
#endif
	preempt_latency_start(val);
}
EXPORT_SYMBOL(preempt_count_add);
NOKPROBE_SYMBOL(preempt_count_add);

/*
 * If the value passed in equals to the current preempt count
 * then we just enabled preemption. Stop timing the latency.
 */
static inline void preempt_latency_stop(int val)
{
	if (preempt_count() == val)
		trace_preempt_on(CALLER_ADDR0, get_lock_parent_ip());
}

void preempt_count_sub(int val)
{
#ifdef CONFIG_DEBUG_PREEMPT
	/*
	 * Underflow?
	 */
	if (DEBUG_LOCKS_WARN_ON(val > preempt_count()))
		return;
	/*
	 * Is the spinlock portion underflowing?
	 */
	if (DEBUG_LOCKS_WARN_ON((val < PREEMPT_MASK) &&
			!(preempt_count() & PREEMPT_MASK)))
		return;
#endif

	preempt_latency_stop(val);
	__preempt_count_sub(val);
}
EXPORT_SYMBOL(preempt_count_sub);
NOKPROBE_SYMBOL(preempt_count_sub);

#else
static inline void preempt_latency_start(int val) { }
static inline void preempt_latency_stop(int val) { }
#endif

static inline unsigned long get_preempt_disable_ip(struct task_struct *p)
{
#ifdef CONFIG_DEBUG_PREEMPT
	return p->preempt_disable_ip;
#else
	return 0;
#endif
}

/*
 * Print scheduling while atomic bug:
 */
static noinline void __schedule_bug(struct task_struct *prev)
{
	/* Save this before calling printk(), since that will clobber it */
	unsigned long preempt_disable_ip = get_preempt_disable_ip(current);

	if (oops_in_progress)
		return;

	printk(KERN_ERR "BUG: scheduling while atomic: %s/%d/0x%08x\n",
		prev->comm, prev->pid, preempt_count());

	debug_show_held_locks(prev);
	print_modules();
	if (irqs_disabled())
		print_irqtrace_events(prev);
	if (IS_ENABLED(CONFIG_DEBUG_PREEMPT)
	    && in_atomic_preempt_off()) {
		pr_err("Preemption disabled at:");
		print_ip_sym(preempt_disable_ip);
		pr_cont("\n");
	}
	if (panic_on_warn)
		panic("scheduling while atomic\n");

	dump_stack();
	add_taint(TAINT_WARN, LOCKDEP_STILL_OK);
}

/*
 * Various schedule()-time debugging checks and statistics:
 */
static inline void schedule_debug(struct task_struct *prev, bool preempt)
{
#ifdef CONFIG_SCHED_STACK_END_CHECK
	if (task_stack_end_corrupted(prev))
		panic("corrupted stack end detected inside scheduler\n");
#endif

#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
	if (!preempt && prev->state && prev->non_block_count) {
		printk(KERN_ERR "BUG: scheduling in a non-blocking section: %s/%d/%i\n",
			prev->comm, prev->pid, prev->non_block_count);
		dump_stack();
		add_taint(TAINT_WARN, LOCKDEP_STILL_OK);
	}
#endif

	if (unlikely(in_atomic_preempt_off())) {
		__schedule_bug(prev);
		preempt_count_set(PREEMPT_DISABLED);
	}
	rcu_sleep_check();

	profile_hit(SCHED_PROFILING, __builtin_return_address(0));

	schedstat_inc(this_rq()->sched_count);
}

#ifdef	CONFIG_SMP

#define SCHED_RQ_NR_MIGRATION (32UL)
/*
 * Migrate pending tasks in @rq to @dest_cpu
 * Will try to migrate mininal of half of @rq nr_running tasks and
 * SCHED_RQ_NR_MIGRATION to @dest_cpu
 */
static inline int
migrate_pending_tasks(struct rq *rq, struct rq *dest_rq, const int dest_cpu)
{
	struct task_struct *p, *skip = rq->curr;
	int nr_migrated = 0;
	int nr_tries = min(rq->nr_running / 2, SCHED_RQ_NR_MIGRATION);

	while (skip != rq->idle && nr_tries &&
	       (p = rq_next_bmq_task(skip, rq)) != rq->idle) {
		skip = rq_next_bmq_task(p, rq);
		if (cpumask_test_cpu(dest_cpu, p->cpus_ptr)) {
			__dequeue_task(p, rq, 0);
			set_task_cpu(p, dest_cpu);
			__enqueue_task(p, dest_rq, 0);
			nr_migrated++;
		}
		nr_tries--;
	}

	return nr_migrated;
}

static inline int take_other_rq_tasks(struct rq *rq, int cpu)
{
	struct cpumask *affinity_mask, *end_mask;

	if (unlikely(!rq->online))
		return 0;

	if (cpumask_empty(&sched_rq_pending_mask))
		return 0;

	affinity_mask = &(per_cpu(sched_cpu_affinity_masks, cpu)[0]);
	end_mask = per_cpu(sched_cpu_affinity_end_mask, cpu);
	do {
		int i;
		for_each_cpu_and(i, &sched_rq_pending_mask, affinity_mask) {
			int nr_migrated;
			struct rq *src_rq;

			src_rq = cpu_rq(i);
			if (!do_raw_spin_trylock(&src_rq->lock))
				continue;
			spin_acquire(&src_rq->lock.dep_map,
				     SINGLE_DEPTH_NESTING, 1, _RET_IP_);

			if ((nr_migrated = migrate_pending_tasks(src_rq, rq, cpu))) {
				src_rq->nr_running -= nr_migrated;
#ifdef CONFIG_SMP
				if (src_rq->nr_running < 2)
					cpumask_clear_cpu(i, &sched_rq_pending_mask);
#endif
				rq->nr_running += nr_migrated;
#ifdef CONFIG_SMP
				if (rq->nr_running > 1)
					cpumask_set_cpu(cpu, &sched_rq_pending_mask);
#endif
				update_sched_rq_watermark(rq);
				cpufreq_update_util(rq, 0);

				spin_release(&src_rq->lock.dep_map, _RET_IP_);
				do_raw_spin_unlock(&src_rq->lock);

				return 1;
			}

			spin_release(&src_rq->lock.dep_map, _RET_IP_);
			do_raw_spin_unlock(&src_rq->lock);
		}
	} while (++affinity_mask < end_mask);

	return 0;
}
#endif

/*
 * Timeslices below RESCHED_NS are considered as good as expired as there's no
 * point rescheduling when there's so little time left.
 */
static inline void check_curr(struct task_struct *p, struct rq *rq)
{
	if (unlikely(rq->idle == p))
		return;

	update_curr(rq, p);

	if (p->time_slice < RESCHED_NS) {
		p->time_slice = sched_timeslice_ns;
		if (SCHED_FIFO != p->policy && task_on_rq_queued(p)) {
			if (SCHED_RR != p->policy)
				deboost_task(p);
			requeue_task(p, rq);
		}
	}
}

static inline struct task_struct *
choose_next_task(struct rq *rq, int cpu, struct task_struct *prev)
{
	struct task_struct *next;

	if (unlikely(rq->skip)) {
		next = rq_runnable_task(rq);
		if (next == rq->idle) {
#ifdef	CONFIG_SMP
			if (!take_other_rq_tasks(rq, cpu)) {
#endif
				rq->skip = NULL;
				schedstat_inc(rq->sched_goidle);
				return next;
#ifdef	CONFIG_SMP
			}
			next = rq_runnable_task(rq);
#endif
		}
		rq->skip = NULL;
#ifdef CONFIG_HIGH_RES_TIMERS
		hrtick_start(rq, next->time_slice);
#endif
		return next;
	}

	next = rq_first_bmq_task(rq);
	if (next == rq->idle) {
#ifdef	CONFIG_SMP
		if (!take_other_rq_tasks(rq, cpu)) {
#endif
			schedstat_inc(rq->sched_goidle);
			return next;
#ifdef	CONFIG_SMP
		}
		next = rq_first_bmq_task(rq);
#endif
	}
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtick_start(rq, next->time_slice);
#endif
	return next;
}

/*
 * schedule() is the main scheduler function.
 *
 * The main means of driving the scheduler and thus entering this function are:
 *
 *   1. Explicit blocking: mutex, semaphore, waitqueue, etc.
 *
 *   2. TIF_NEED_RESCHED flag is checked on interrupt and userspace return
 *      paths. For example, see arch/x86/entry_64.S.
 *
 *      To drive preemption between tasks, the scheduler sets the flag in timer
 *      interrupt handler scheduler_tick().
 *
 *   3. Wakeups don't really cause entry into schedule(). They add a
 *      task to the run-queue and that's it.
 *
 *      Now, if the new task added to the run-queue preempts the current
 *      task, then the wakeup sets TIF_NEED_RESCHED and schedule() gets
 *      called on the nearest possible occasion:
 *
 *       - If the kernel is preemptible (CONFIG_PREEMPTION=y):
 *
 *         - in syscall or exception context, at the next outmost
 *           preempt_enable(). (this might be as soon as the wake_up()'s
 *           spin_unlock()!)
 *
 *         - in IRQ context, return from interrupt-handler to
 *           preemptible context
 *
 *       - If the kernel is not preemptible (CONFIG_PREEMPTION is not set)
 *         then at the next:
 *
 *          - cond_resched() call
 *          - explicit schedule() call
 *          - return from syscall or exception to user-space
 *          - return from interrupt-handler to user-space
 *
 * WARNING: must be called with preemption disabled!
 */
static void __sched notrace __schedule(bool preempt)
{
	struct task_struct *prev, *next;
	unsigned long *switch_count;
	struct rq *rq;
	int cpu;

	cpu = smp_processor_id();
	rq = cpu_rq(cpu);
	prev = rq->curr;

	schedule_debug(prev, preempt);

	/* by passing sched_feat(HRTICK) checking which BMQ doesn't support */
	hrtick_clear(rq);

	local_irq_disable();
	rcu_note_context_switch(preempt);

	/*
	 * Make sure that signal_pending_state()->signal_pending() below
	 * can't be reordered with __set_current_state(TASK_INTERRUPTIBLE)
	 * done by the caller to avoid the race with signal_wake_up().
	 *
	 * The membarrier system call requires a full memory barrier
	 * after coming from user-space, before storing to rq->curr.
	 */
	raw_spin_lock(&rq->lock);
	smp_mb__after_spinlock();

	update_rq_clock(rq);

	switch_count = &prev->nivcsw;
	if (!preempt && prev->state) {
		if (signal_pending_state(prev->state, prev)) {
			prev->state = TASK_RUNNING;
		} else {
			if (rq_switch_time(rq) < boost_threshold(prev))
				boost_task(prev);
			deactivate_task(prev, rq);

			if (prev->in_iowait) {
				atomic_inc(&rq->nr_iowait);
				delayacct_blkio_start();
			}
		}
		switch_count = &prev->nvcsw;
	}

	clear_tsk_need_resched(prev);
	clear_preempt_need_resched();

	check_curr(prev, rq);

	next = choose_next_task(rq, cpu, prev);

	if (likely(prev != next)) {
		next->last_ran = rq->clock_task;
		rq->last_ts_switch = rq->clock;

		rq->nr_switches++;
		/*
		 * RCU users of rcu_dereference(rq->curr) may not see
		 * changes to task_struct made by pick_next_task().
		 */
		RCU_INIT_POINTER(rq->curr, next);
		/*
		 * The membarrier system call requires each architecture
		 * to have a full memory barrier after updating
		 * rq->curr, before returning to user-space.
		 *
		 * Here are the schemes providing that barrier on the
		 * various architectures:
		 * - mm ? switch_mm() : mmdrop() for x86, s390, sparc, PowerPC.
		 *   switch_mm() rely on membarrier_arch_switch_mm() on PowerPC.
		 * - finish_lock_switch() for weakly-ordered
		 *   architectures where spin_unlock is a full barrier,
		 * - switch_to() for arm64 (weakly-ordered, spin_unlock
		 *   is a RELEASE barrier),
		 */
		++*switch_count;

		psi_sched_switch(prev, next, !task_on_rq_queued(prev));

		trace_sched_switch(preempt, prev, next);

		/* Also unlocks the rq: */
		rq = context_switch(rq, prev, next);
	} else
		raw_spin_unlock_irq(&rq->lock);

#ifdef CONFIG_SCHED_SMT
	sg_balance_check(rq);
#endif
}

void __noreturn do_task_dead(void)
{
	/* Causes final put_task_struct in finish_task_switch(): */
	set_special_state(TASK_DEAD);

	/* Tell freezer to ignore us: */
	current->flags |= PF_NOFREEZE;

	__schedule(false);
	BUG();

	/* Avoid "noreturn function does return" - but don't continue if BUG() is a NOP: */
	for (;;)
		cpu_relax();
}

static inline void sched_submit_work(struct task_struct *tsk)
{
	if (!tsk->state)
		return;

	/*
	 * If a worker went to sleep, notify and ask workqueue whether
	 * it wants to wake up a task to maintain concurrency.
	 * As this function is called inside the schedule() context,
	 * we disable preemption to avoid it calling schedule() again
	 * in the possible wakeup of a kworker and because wq_worker_sleeping()
	 * requires it.
	 */
	if (tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER)) {
		preempt_disable();
		if (tsk->flags & PF_WQ_WORKER)
			wq_worker_sleeping(tsk);
		else
			io_wq_worker_sleeping(tsk);
		preempt_enable_no_resched();
	}

	if (tsk_is_pi_blocked(tsk))
		return;

	/*
	 * If we are going to sleep and we have plugged IO queued,
	 * make sure to submit it to avoid deadlocks.
	 */
	if (blk_needs_flush_plug(tsk))
		blk_schedule_flush_plug(tsk);
}

static void sched_update_worker(struct task_struct *tsk)
{
	if (tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER)) {
		if (tsk->flags & PF_WQ_WORKER)
			wq_worker_running(tsk);
		else
			io_wq_worker_running(tsk);
	}
}

asmlinkage __visible void __sched schedule(void)
{
	struct task_struct *tsk = current;

	sched_submit_work(tsk);
	do {
		preempt_disable();
		__schedule(false);
		sched_preempt_enable_no_resched();
	} while (need_resched());
	sched_update_worker(tsk);
}
EXPORT_SYMBOL(schedule);

/*
 * synchronize_rcu_tasks() makes sure that no task is stuck in preempted
 * state (have scheduled out non-voluntarily) by making sure that all
 * tasks have either left the run queue or have gone into user space.
 * As idle tasks do not do either, they must not ever be preempted
 * (schedule out non-voluntarily).
 *
 * schedule_idle() is similar to schedule_preempt_disable() except that it
 * never enables preemption because it does not call sched_submit_work().
 */
void __sched schedule_idle(void)
{
	/*
	 * As this skips calling sched_submit_work(), which the idle task does
	 * regardless because that function is a nop when the task is in a
	 * TASK_RUNNING state, make sure this isn't used someplace that the
	 * current task can be in any other state. Note, idle is always in the
	 * TASK_RUNNING state.
	 */
	WARN_ON_ONCE(current->state);
	do {
		__schedule(false);
	} while (need_resched());
}

#ifdef CONFIG_CONTEXT_TRACKING
asmlinkage __visible void __sched schedule_user(void)
{
	/*
	 * If we come here after a random call to set_need_resched(),
	 * or we have been woken up remotely but the IPI has not yet arrived,
	 * we haven't yet exited the RCU idle mode. Do it here manually until
	 * we find a better solution.
	 *
	 * NB: There are buggy callers of this function.  Ideally we
	 * should warn if prev_state != CONTEXT_USER, but that will trigger
	 * too frequently to make sense yet.
	 */
	enum ctx_state prev_state = exception_enter();
	schedule();
	exception_exit(prev_state);
}
#endif

/**
 * schedule_preempt_disabled - called with preemption disabled
 *
 * Returns with preemption disabled. Note: preempt_count must be 1
 */
void __sched schedule_preempt_disabled(void)
{
	sched_preempt_enable_no_resched();
	schedule();
	preempt_disable();
}

static void __sched notrace preempt_schedule_common(void)
{
	do {
		/*
		 * Because the function tracer can trace preempt_count_sub()
		 * and it also uses preempt_enable/disable_notrace(), if
		 * NEED_RESCHED is set, the preempt_enable_notrace() called
		 * by the function tracer will call this function again and
		 * cause infinite recursion.
		 *
		 * Preemption must be disabled here before the function
		 * tracer can trace. Break up preempt_disable() into two
		 * calls. One to disable preemption without fear of being
		 * traced. The other to still record the preemption latency,
		 * which can also be traced by the function tracer.
		 */
		preempt_disable_notrace();
		preempt_latency_start(1);
		__schedule(true);
		preempt_latency_stop(1);
		preempt_enable_no_resched_notrace();

		/*
		 * Check again in case we missed a preemption opportunity
		 * between schedule and now.
		 */
	} while (need_resched());
}

#ifdef CONFIG_PREEMPTION
/*
 * This is the entry point to schedule() from in-kernel preemption
 * off of preempt_enable.
 */
asmlinkage __visible void __sched notrace preempt_schedule(void)
{
	/*
	 * If there is a non-zero preempt_count or interrupts are disabled,
	 * we do not want to preempt the current task. Just return..
	 */
	if (likely(!preemptible()))
		return;

	preempt_schedule_common();
}
NOKPROBE_SYMBOL(preempt_schedule);
EXPORT_SYMBOL(preempt_schedule);

/**
 * preempt_schedule_notrace - preempt_schedule called by tracing
 *
 * The tracing infrastructure uses preempt_enable_notrace to prevent
 * recursion and tracing preempt enabling caused by the tracing
 * infrastructure itself. But as tracing can happen in areas coming
 * from userspace or just about to enter userspace, a preempt enable
 * can occur before user_exit() is called. This will cause the scheduler
 * to be called when the system is still in usermode.
 *
 * To prevent this, the preempt_enable_notrace will use this function
 * instead of preempt_schedule() to exit user context if needed before
 * calling the scheduler.
 */
asmlinkage __visible void __sched notrace preempt_schedule_notrace(void)
{
	enum ctx_state prev_ctx;

	if (likely(!preemptible()))
		return;

	do {
		/*
		 * Because the function tracer can trace preempt_count_sub()
		 * and it also uses preempt_enable/disable_notrace(), if
		 * NEED_RESCHED is set, the preempt_enable_notrace() called
		 * by the function tracer will call this function again and
		 * cause infinite recursion.
		 *
		 * Preemption must be disabled here before the function
		 * tracer can trace. Break up preempt_disable() into two
		 * calls. One to disable preemption without fear of being
		 * traced. The other to still record the preemption latency,
		 * which can also be traced by the function tracer.
		 */
		preempt_disable_notrace();
		preempt_latency_start(1);
		/*
		 * Needs preempt disabled in case user_exit() is traced
		 * and the tracer calls preempt_enable_notrace() causing
		 * an infinite recursion.
		 */
		prev_ctx = exception_enter();
		__schedule(true);
		exception_exit(prev_ctx);

		preempt_latency_stop(1);
		preempt_enable_no_resched_notrace();
	} while (need_resched());
}
EXPORT_SYMBOL_GPL(preempt_schedule_notrace);

#endif /* CONFIG_PREEMPTION */

/*
 * This is the entry point to schedule() from kernel preemption
 * off of irq context.
 * Note, that this is called and return with irqs disabled. This will
 * protect us against recursive calling from irq.
 */
asmlinkage __visible void __sched preempt_schedule_irq(void)
{
	enum ctx_state prev_state;

	/* Catch callers which need to be fixed */
	BUG_ON(preempt_count() || !irqs_disabled());

	prev_state = exception_enter();

	do {
		preempt_disable();
		local_irq_enable();
		__schedule(true);
		local_irq_disable();
		sched_preempt_enable_no_resched();
	} while (need_resched());

	exception_exit(prev_state);
}

int default_wake_function(wait_queue_entry_t *curr, unsigned mode, int wake_flags,
			  void *key)
{
	return try_to_wake_up(curr->private, mode, wake_flags);
}
EXPORT_SYMBOL(default_wake_function);

static inline void check_task_changed(struct rq *rq, struct task_struct *p)
{
	/* Trigger resched if task sched_prio has been modified. */
	if (task_on_rq_queued(p) && task_sched_prio(p) != p->bmq_idx) {
		requeue_task(p, rq);
		check_preempt_curr(rq);
	}
}

#ifdef CONFIG_RT_MUTEXES

static inline int __rt_effective_prio(struct task_struct *pi_task, int prio)
{
	if (pi_task)
		prio = min(prio, pi_task->prio);

	return prio;
}

static inline int rt_effective_prio(struct task_struct *p, int prio)
{
	struct task_struct *pi_task = rt_mutex_get_top_task(p);

	return __rt_effective_prio(pi_task, prio);
}

/*
 * rt_mutex_setprio - set the current priority of a task
 * @p: task to boost
 * @pi_task: donor task
 *
 * This function changes the 'effective' priority of a task. It does
 * not touch ->normal_prio like __setscheduler().
 *
 * Used by the rt_mutex code to implement priority inheritance
 * logic. Call site only calls if the priority of the task changed.
 */
void rt_mutex_setprio(struct task_struct *p, struct task_struct *pi_task)
{
	int prio;
	struct rq *rq;
	raw_spinlock_t *lock;

	/* XXX used to be waiter->prio, not waiter->task->prio */
	prio = __rt_effective_prio(pi_task, p->normal_prio);

	/*
	 * If nothing changed; bail early.
	 */
	if (p->pi_top_task == pi_task && prio == p->prio)
		return;

	rq = __task_access_lock(p, &lock);
	/*
	 * Set under pi_lock && rq->lock, such that the value can be used under
	 * either lock.
	 *
	 * Note that there is loads of tricky to make this pointer cache work
	 * right. rt_mutex_slowunlock()+rt_mutex_postunlock() work together to
	 * ensure a task is de-boosted (pi_task is set to NULL) before the
	 * task is allowed to run again (and can exit). This ensures the pointer
	 * points to a blocked task -- which guaratees the task is present.
	 */
	p->pi_top_task = pi_task;

	/*
	 * For FIFO/RR we only need to set prio, if that matches we're done.
	 */
	if (prio == p->prio)
		goto out_unlock;

	/*
	 * Idle task boosting is a nono in general. There is one
	 * exception, when PREEMPT_RT and NOHZ is active:
	 *
	 * The idle task calls get_next_timer_interrupt() and holds
	 * the timer wheel base->lock on the CPU and another CPU wants
	 * to access the timer (probably to cancel it). We can safely
	 * ignore the boosting request, as the idle CPU runs this code
	 * with interrupts disabled and will complete the lock
	 * protected section without being interrupted. So there is no
	 * real need to boost.
	 */
	if (unlikely(p == rq->idle)) {
		WARN_ON(p != rq->curr);
		WARN_ON(p->pi_blocked_on);
		goto out_unlock;
	}

	trace_sched_pi_setprio(p, pi_task);
	p->prio = prio;

	check_task_changed(rq, p);
out_unlock:
	__task_access_unlock(p, lock);
}
#else
static inline int rt_effective_prio(struct task_struct *p, int prio)
{
	return prio;
}
#endif

void set_user_nice(struct task_struct *p, long nice)
{
	unsigned long flags;
	struct rq *rq;
	raw_spinlock_t *lock;

	if (task_nice(p) == nice || nice < MIN_NICE || nice > MAX_NICE)
		return;
	/*
	 * We have to be careful, if called from sys_setpriority(),
	 * the task might be in the middle of scheduling on another CPU.
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);
	rq = __task_access_lock(p, &lock);

	p->static_prio = NICE_TO_PRIO(nice);
	/*
	 * The RT priorities are set via sched_setscheduler(), but we still
	 * allow the 'normal' nice value to be set - but as expected
	 * it wont have any effect on scheduling until the task is
	 * not SCHED_NORMAL/SCHED_BATCH:
	 */
	if (task_has_rt_policy(p))
		goto out_unlock;

	p->prio = effective_prio(p);
	check_task_changed(rq, p);
out_unlock:
	__task_access_unlock(p, lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);
}
EXPORT_SYMBOL(set_user_nice);

/*
 * can_nice - check if a task can reduce its nice value
 * @p: task
 * @nice: nice value
 */
int can_nice(const struct task_struct *p, const int nice)
{
	/* Convert nice value [19,-20] to rlimit style value [1,40] */
	int nice_rlim = nice_to_rlimit(nice);

	return (nice_rlim <= task_rlimit(p, RLIMIT_NICE) ||
		capable(CAP_SYS_NICE));
}

#ifdef __ARCH_WANT_SYS_NICE

/*
 * sys_nice - change the priority of the current process.
 * @increment: priority increment
 *
 * sys_setpriority is a more generic, but much slower function that
 * does similar things.
 */
SYSCALL_DEFINE1(nice, int, increment)
{
	long nice, retval;

	/*
	 * Setpriority might change our priority at the same moment.
	 * We don't have to worry. Conceptually one call occurs first
	 * and we have a single winner.
	 */

	increment = clamp(increment, -NICE_WIDTH, NICE_WIDTH);
	nice = task_nice(current) + increment;

	nice = clamp_val(nice, MIN_NICE, MAX_NICE);
	if (increment < 0 && !can_nice(current, nice))
		return -EPERM;

	retval = security_task_setnice(current, nice);
	if (retval)
		return retval;

	set_user_nice(current, nice);
	return 0;
}

#endif

/**
 * task_prio - return the priority value of a given task.
 * @p: the task in question.
 *
 * Return: The priority value as seen by users in /proc.
 * RT tasks are offset by -100. Normal tasks are centered around 1, value goes
 * from 0(SCHED_ISO) up to 82 (nice +19 SCHED_IDLE).
 */
int task_prio(const struct task_struct *p)
{
	if (p->prio < MAX_RT_PRIO)
		return (p->prio - MAX_RT_PRIO);
	return (p->prio - MAX_RT_PRIO + p->boost_prio);
}

/**
 * idle_cpu - is a given CPU idle currently?
 * @cpu: the processor in question.
 *
 * Return: 1 if the CPU is currently idle. 0 otherwise.
 */
int idle_cpu(int cpu)
{
	return cpu_curr(cpu) == cpu_rq(cpu)->idle;
}

/**
 * idle_task - return the idle task for a given CPU.
 * @cpu: the processor in question.
 *
 * Return: The idle task for the cpu @cpu.
 */
struct task_struct *idle_task(int cpu)
{
	return cpu_rq(cpu)->idle;
}

/**
 * find_process_by_pid - find a process with a matching PID value.
 * @pid: the pid in question.
 *
 * The task of @pid, if found. %NULL otherwise.
 */
static inline struct task_struct *find_process_by_pid(pid_t pid)
{
	return pid ? find_task_by_vpid(pid) : current;
}

/*
 * sched_setparam() passes in -1 for its policy, to let the functions
 * it calls know not to change it.
 */
#define SETPARAM_POLICY -1

static void __setscheduler_params(struct task_struct *p,
		const struct sched_attr *attr)
{
	int policy = attr->sched_policy;

	if (policy == SETPARAM_POLICY)
		policy = p->policy;

	p->policy = policy;

	/*
	 * allow normal nice value to be set, but will not have any
	 * effect on scheduling until the task not SCHED_NORMAL/
	 * SCHED_BATCH
	 */
	p->static_prio = NICE_TO_PRIO(attr->sched_nice);

	/*
	 * __sched_setscheduler() ensures attr->sched_priority == 0 when
	 * !rt_policy. Always setting this ensures that things like
	 * getparam()/getattr() don't report silly values for !rt tasks.
	 */
	p->rt_priority = attr->sched_priority;
	p->normal_prio = normal_prio(p);
}

/* Actually do priority change: must hold rq lock. */
static void __setscheduler(struct rq *rq, struct task_struct *p,
			   const struct sched_attr *attr, bool keep_boost)
{
	__setscheduler_params(p, attr);

	/*
	 * Keep a potential priority boosting if called from
	 * sched_setscheduler().
	 */
	p->prio = normal_prio(p);
	if (keep_boost)
		p->prio = rt_effective_prio(p, p->prio);
}

/*
 * check the target process has a UID that matches the current process's
 */
static bool check_same_owner(struct task_struct *p)
{
	const struct cred *cred = current_cred(), *pcred;
	bool match;

	rcu_read_lock();
	pcred = __task_cred(p);
	match = (uid_eq(cred->euid, pcred->euid) ||
		 uid_eq(cred->euid, pcred->uid));
	rcu_read_unlock();
	return match;
}

static int __sched_setscheduler(struct task_struct *p,
				const struct sched_attr *attr,
				bool user, bool pi)
{
	const struct sched_attr dl_squash_attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_FIFO,
		.sched_nice	= 0,
		.sched_priority = 99,
	};
	int newprio = MAX_RT_PRIO - 1 - attr->sched_priority;
	int retval, oldpolicy = -1;
	int policy = attr->sched_policy;
	unsigned long flags;
	struct rq *rq;
	int reset_on_fork;
	raw_spinlock_t *lock;

	/* The pi code expects interrupts enabled */
	BUG_ON(pi && in_interrupt());

	/*
	 * BMQ supports SCHED_DEADLINE by squash it as prio 0 SCHED_FIFO
	 */
	if (unlikely(SCHED_DEADLINE == policy)) {
		attr = &dl_squash_attr;
		policy = attr->sched_policy;
		newprio = MAX_RT_PRIO - 1 - attr->sched_priority;
	}
recheck:
	/* Double check policy once rq lock held */
	if (policy < 0) {
		reset_on_fork = p->sched_reset_on_fork;
		policy = oldpolicy = p->policy;
	} else {
		reset_on_fork = !!(attr->sched_flags & SCHED_RESET_ON_FORK);

		if (policy > SCHED_IDLE)
			return -EINVAL;
	}

	if (attr->sched_flags & ~(SCHED_FLAG_ALL))
		return -EINVAL;

	/*
	 * Valid priorities for SCHED_FIFO and SCHED_RR are
	 * 1..MAX_USER_RT_PRIO-1, valid priority for SCHED_NORMAL and
	 * SCHED_BATCH and SCHED_IDLE is 0.
	 */
	if (attr->sched_priority < 0 ||
	    (p->mm && attr->sched_priority > MAX_USER_RT_PRIO - 1) ||
	    (!p->mm && attr->sched_priority > MAX_RT_PRIO - 1))
		return -EINVAL;
	if ((SCHED_RR == policy || SCHED_FIFO == policy) !=
	    (attr->sched_priority != 0))
		return -EINVAL;

	/*
	 * Allow unprivileged RT tasks to decrease priority:
	 */
	if (user && !capable(CAP_SYS_NICE)) {
		if (SCHED_FIFO == policy || SCHED_RR == policy) {
			unsigned long rlim_rtprio =
					task_rlimit(p, RLIMIT_RTPRIO);

			/* Can't set/change the rt policy */
			if (policy != p->policy && !rlim_rtprio)
				return -EPERM;

			/* Can't increase priority */
			if (attr->sched_priority > p->rt_priority &&
			    attr->sched_priority > rlim_rtprio)
				return -EPERM;
		}

		/* Can't change other user's priorities */
		if (!check_same_owner(p))
			return -EPERM;

		/* Normal users shall not reset the sched_reset_on_fork flag */
		if (p->sched_reset_on_fork && !reset_on_fork)
			return -EPERM;
	}

	if (user) {
		retval = security_task_setscheduler(p);
		if (retval)
			return retval;
	}

	if (pi)
		cpuset_read_lock();

	/*
	 * Make sure no PI-waiters arrive (or leave) while we are
	 * changing the priority of the task:
	 */
	raw_spin_lock_irqsave(&p->pi_lock, flags);

	/*
	 * To be able to change p->policy safely, task_access_lock()
	 * must be called.
	 * IF use task_access_lock() here:
	 * For the task p which is not running, reading rq->stop is
	 * racy but acceptable as ->stop doesn't change much.
	 * An enhancemnet can be made to read rq->stop saftly.
	 */
	rq = __task_access_lock(p, &lock);

	/*
	 * Changing the policy of the stop threads its a very bad idea
	 */
	if (p == rq->stop) {
		retval = -EINVAL;
		goto unlock;
	}

	/*
	 * If not changing anything there's no need to proceed further:
	 */
	if (unlikely(policy == p->policy)) {
		if (rt_policy(policy) && attr->sched_priority != p->rt_priority)
			goto change;
		if (!rt_policy(policy) &&
		    NICE_TO_PRIO(attr->sched_nice) != p->static_prio)
			goto change;

		p->sched_reset_on_fork = reset_on_fork;
		retval = 0;
		goto unlock;
	}
change:

	/* Re-check policy now with rq lock held */
	if (unlikely(oldpolicy != -1 && oldpolicy != p->policy)) {
		policy = oldpolicy = -1;
		__task_access_unlock(p, lock);
		raw_spin_unlock_irqrestore(&p->pi_lock, flags);
		if (pi)
			cpuset_read_unlock();
		goto recheck;
	}

	p->sched_reset_on_fork = reset_on_fork;

	if (pi) {
		/*
		 * Take priority boosted tasks into account. If the new
		 * effective priority is unchanged, we just store the new
		 * normal parameters and do not touch the scheduler class and
		 * the runqueue. This will be done when the task deboost
		 * itself.
		 */
		if (rt_effective_prio(p, newprio) == p->prio) {
			__setscheduler_params(p, attr);
			retval = 0;
			goto unlock;
		}
	}

	__setscheduler(rq, p, attr, pi);

	check_task_changed(rq, p);

	/* Avoid rq from going away on us: */
	preempt_disable();
	__task_access_unlock(p, lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);

	if (pi) {
		cpuset_read_unlock();
		rt_mutex_adjust_pi(p);
	}

	preempt_enable();

	return 0;

unlock:
	__task_access_unlock(p, lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, flags);
	if (pi)
		cpuset_read_unlock();
	return retval;
}

static int _sched_setscheduler(struct task_struct *p, int policy,
			       const struct sched_param *param, bool check)
{
	struct sched_attr attr = {
		.sched_policy   = policy,
		.sched_priority = param->sched_priority,
		.sched_nice     = PRIO_TO_NICE(p->static_prio),
	};

	/* Fixup the legacy SCHED_RESET_ON_FORK hack. */
	if ((policy != SETPARAM_POLICY) && (policy & SCHED_RESET_ON_FORK)) {
		attr.sched_flags |= SCHED_FLAG_RESET_ON_FORK;
		policy &= ~SCHED_RESET_ON_FORK;
		attr.sched_policy = policy;
	}

	return __sched_setscheduler(p, &attr, check, true);
}

/**
 * sched_setscheduler - change the scheduling policy and/or RT priority of a thread.
 * @p: the task in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 *
 * Return: 0 on success. An error code otherwise.
 *
 * NOTE that the task may be already dead.
 */
int sched_setscheduler(struct task_struct *p, int policy,
		       const struct sched_param *param)
{
	return _sched_setscheduler(p, policy, param, true);
}

EXPORT_SYMBOL_GPL(sched_setscheduler);

int sched_setattr(struct task_struct *p, const struct sched_attr *attr)
{
	return __sched_setscheduler(p, attr, true, true);
}
EXPORT_SYMBOL_GPL(sched_setattr);

int sched_setattr_nocheck(struct task_struct *p, const struct sched_attr *attr)
{
	return __sched_setscheduler(p, attr, false, true);
}

/**
 * sched_setscheduler_nocheck - change the scheduling policy and/or RT priority of a thread from kernelspace.
 * @p: the task in question.
 * @policy: new policy.
 * @param: structure containing the new RT priority.
 *
 * Just like sched_setscheduler, only don't bother checking if the
 * current context has permission.  For example, this is needed in
 * stop_machine(): we create temporary high priority worker threads,
 * but our caller might not have that capability.
 *
 * Return: 0 on success. An error code otherwise.
 */
int sched_setscheduler_nocheck(struct task_struct *p, int policy,
			       const struct sched_param *param)
{
	return _sched_setscheduler(p, policy, param, false);
}
EXPORT_SYMBOL_GPL(sched_setscheduler_nocheck);

static int
do_sched_setscheduler(pid_t pid, int policy, struct sched_param __user *param)
{
	struct sched_param lparam;
	struct task_struct *p;
	int retval;

	if (!param || pid < 0)
		return -EINVAL;
	if (copy_from_user(&lparam, param, sizeof(struct sched_param)))
		return -EFAULT;

	rcu_read_lock();
	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (likely(p))
		get_task_struct(p);
	rcu_read_unlock();

	if (likely(p)) {
		retval = sched_setscheduler(p, policy, &lparam);
		put_task_struct(p);
	}

	return retval;
}

/*
 * Mimics kernel/events/core.c perf_copy_attr().
 */
static int sched_copy_attr(struct sched_attr __user *uattr, struct sched_attr *attr)
{
	u32 size;
	int ret;

	/* Zero the full structure, so that a short copy will be nice: */
	memset(attr, 0, sizeof(*attr));

	ret = get_user(size, &uattr->size);
	if (ret)
		return ret;

	/* ABI compatibility quirk: */
	if (!size)
		size = SCHED_ATTR_SIZE_VER0;

	if (size < SCHED_ATTR_SIZE_VER0 || size > PAGE_SIZE)
		goto err_size;

	ret = copy_struct_from_user(attr, sizeof(*attr), uattr, size);
	if (ret) {
		if (ret == -E2BIG)
			goto err_size;
		return ret;
	}

	/*
	 * XXX: Do we want to be lenient like existing syscalls; or do we want
	 * to be strict and return an error on out-of-bounds values?
	 */
	attr->sched_nice = clamp(attr->sched_nice, -20, 19);

	/* sched/core.c uses zero here but we already know ret is zero */
	return 0;

err_size:
	put_user(sizeof(*attr), &uattr->size);
	return -E2BIG;
}

/**
 * sys_sched_setscheduler - set/change the scheduler policy and RT priority
 * @pid: the pid in question.
 * @policy: new policy.
 *
 * Return: 0 on success. An error code otherwise.
 * @param: structure containing the new RT priority.
 */
SYSCALL_DEFINE3(sched_setscheduler, pid_t, pid, int, policy, struct sched_param __user *, param)
{
	if (policy < 0)
		return -EINVAL;

	return do_sched_setscheduler(pid, policy, param);
}

/**
 * sys_sched_setparam - set/change the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the new RT priority.
 *
 * Return: 0 on success. An error code otherwise.
 */
SYSCALL_DEFINE2(sched_setparam, pid_t, pid, struct sched_param __user *, param)
{
	return do_sched_setscheduler(pid, SETPARAM_POLICY, param);
}

/**
 * sys_sched_setattr - same as above, but with extended sched_attr
 * @pid: the pid in question.
 * @uattr: structure containing the extended parameters.
 */
SYSCALL_DEFINE3(sched_setattr, pid_t, pid, struct sched_attr __user *, uattr,
			       unsigned int, flags)
{
	struct sched_attr attr;
	struct task_struct *p;
	int retval;

	if (!uattr || pid < 0 || flags)
		return -EINVAL;

	retval = sched_copy_attr(uattr, &attr);
	if (retval)
		return retval;

	if ((int)attr.sched_policy < 0)
		return -EINVAL;

	rcu_read_lock();
	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (p != NULL)
		retval = sched_setattr(p, &attr);
	rcu_read_unlock();

	return retval;
}

/**
 * sys_sched_getscheduler - get the policy (scheduling class) of a thread
 * @pid: the pid in question.
 *
 * Return: On success, the policy of the thread. Otherwise, a negative error
 * code.
 */
SYSCALL_DEFINE1(sched_getscheduler, pid_t, pid)
{
	struct task_struct *p;
	int retval = -EINVAL;

	if (pid < 0)
		goto out_nounlock;

	retval = -ESRCH;
	rcu_read_lock();
	p = find_process_by_pid(pid);
	if (p) {
		retval = security_task_getscheduler(p);
		if (!retval)
			retval = p->policy;
	}
	rcu_read_unlock();

out_nounlock:
	return retval;
}

/**
 * sys_sched_getscheduler - get the RT priority of a thread
 * @pid: the pid in question.
 * @param: structure containing the RT priority.
 *
 * Return: On success, 0 and the RT priority is in @param. Otherwise, an error
 * code.
 */
SYSCALL_DEFINE2(sched_getparam, pid_t, pid, struct sched_param __user *, param)
{
	struct sched_param lp = { .sched_priority = 0 };
	struct task_struct *p;
	int retval = -EINVAL;

	if (!param || pid < 0)
		goto out_nounlock;

	rcu_read_lock();
	p = find_process_by_pid(pid);
	retval = -ESRCH;
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	if (task_has_rt_policy(p))
		lp.sched_priority = p->rt_priority;
	rcu_read_unlock();

	/*
	 * This one might sleep, we cannot do it with a spinlock held ...
	 */
	retval = copy_to_user(param, &lp, sizeof(*param)) ? -EFAULT : 0;

out_nounlock:
	return retval;

out_unlock:
	rcu_read_unlock();
	return retval;
}

/*
 * Copy the kernel size attribute structure (which might be larger
 * than what user-space knows about) to user-space.
 *
 * Note that all cases are valid: user-space buffer can be larger or
 * smaller than the kernel-space buffer. The usual case is that both
 * have the same size.
 */
static int
sched_attr_copy_to_user(struct sched_attr __user *uattr,
			struct sched_attr *kattr,
			unsigned int usize)
{
	unsigned int ksize = sizeof(*kattr);

	if (!access_ok(uattr, usize))
		return -EFAULT;

	/*
	 * sched_getattr() ABI forwards and backwards compatibility:
	 *
	 * If usize == ksize then we just copy everything to user-space and all is good.
	 *
	 * If usize < ksize then we only copy as much as user-space has space for,
	 * this keeps ABI compatibility as well. We skip the rest.
	 *
	 * If usize > ksize then user-space is using a newer version of the ABI,
	 * which part the kernel doesn't know about. Just ignore it - tooling can
	 * detect the kernel's knowledge of attributes from the attr->size value
	 * which is set to ksize in this case.
	 */
	kattr->size = min(usize, ksize);

	if (copy_to_user(uattr, kattr, kattr->size))
		return -EFAULT;

	return 0;
}

/**
 * sys_sched_getattr - similar to sched_getparam, but with sched_attr
 * @pid: the pid in question.
 * @uattr: structure containing the extended parameters.
 * @usize: sizeof(attr) for fwd/bwd comp.
 * @flags: for future extension.
 */
SYSCALL_DEFINE4(sched_getattr, pid_t, pid, struct sched_attr __user *, uattr,
		unsigned int, usize, unsigned int, flags)
{
	struct sched_attr kattr = { };
	struct task_struct *p;
	int retval;

	if (!uattr || pid < 0 || usize > PAGE_SIZE ||
	    usize < SCHED_ATTR_SIZE_VER0 || flags)
		return -EINVAL;

	rcu_read_lock();
	p = find_process_by_pid(pid);
	retval = -ESRCH;
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	kattr.sched_policy = p->policy;
	if (p->sched_reset_on_fork)
		kattr.sched_flags |= SCHED_FLAG_RESET_ON_FORK;
	if (task_has_rt_policy(p))
		kattr.sched_priority = p->rt_priority;
	else
		kattr.sched_nice = task_nice(p);

#ifdef CONFIG_UCLAMP_TASK
	kattr.sched_util_min = p->uclamp_req[UCLAMP_MIN].value;
	kattr.sched_util_max = p->uclamp_req[UCLAMP_MAX].value;
#endif

	rcu_read_unlock();

	return sched_attr_copy_to_user(uattr, &kattr, usize);

out_unlock:
	rcu_read_unlock();
	return retval;
}

long sched_setaffinity(pid_t pid, const struct cpumask *in_mask)
{
	cpumask_var_t cpus_allowed, new_mask;
	struct task_struct *p;
	int retval;

	get_online_cpus();
	rcu_read_lock();

	p = find_process_by_pid(pid);
	if (!p) {
		rcu_read_unlock();
		put_online_cpus();
		return -ESRCH;
	}

	/* Prevent p going away */
	get_task_struct(p);
	rcu_read_unlock();

	if (p->flags & PF_NO_SETAFFINITY) {
		retval = -EINVAL;
		goto out_put_task;
	}
	if (!alloc_cpumask_var(&cpus_allowed, GFP_KERNEL)) {
		retval = -ENOMEM;
		goto out_put_task;
	}
	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL)) {
		retval = -ENOMEM;
		goto out_free_cpus_allowed;
	}
	retval = -EPERM;
	if (!check_same_owner(p)) {
		rcu_read_lock();
		if (!ns_capable(__task_cred(p)->user_ns, CAP_SYS_NICE)) {
			rcu_read_unlock();
			goto out_unlock;
		}
		rcu_read_unlock();
	}

	retval = security_task_setscheduler(p);
	if (retval)
		goto out_unlock;

	cpuset_cpus_allowed(p, cpus_allowed);
	cpumask_and(new_mask, in_mask, cpus_allowed);
again:
	retval = __set_cpus_allowed_ptr(p, new_mask, true);

	if (!retval) {
		cpuset_cpus_allowed(p, cpus_allowed);
		if (!cpumask_subset(new_mask, cpus_allowed)) {
			/*
			 * We must have raced with a concurrent cpuset
			 * update. Just reset the cpus_allowed to the
			 * cpuset's cpus_allowed
			 */
			cpumask_copy(new_mask, cpus_allowed);
			goto again;
		}
	}
out_unlock:
	free_cpumask_var(new_mask);
out_free_cpus_allowed:
	free_cpumask_var(cpus_allowed);
out_put_task:
	put_task_struct(p);
	put_online_cpus();
	return retval;
}

static int get_user_cpu_mask(unsigned long __user *user_mask_ptr, unsigned len,
			     struct cpumask *new_mask)
{
	if (len < cpumask_size())
		cpumask_clear(new_mask);
	else if (len > cpumask_size())
		len = cpumask_size();

	return copy_from_user(new_mask, user_mask_ptr, len) ? -EFAULT : 0;
}

/**
 * sys_sched_setaffinity - set the CPU affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to the new CPU mask
 *
 * Return: 0 on success. An error code otherwise.
 */
SYSCALL_DEFINE3(sched_setaffinity, pid_t, pid, unsigned int, len,
		unsigned long __user *, user_mask_ptr)
{
	cpumask_var_t new_mask;
	int retval;

	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL))
		return -ENOMEM;

	retval = get_user_cpu_mask(user_mask_ptr, len, new_mask);
	if (retval == 0)
		retval = sched_setaffinity(pid, new_mask);
	free_cpumask_var(new_mask);
	return retval;
}

long sched_getaffinity(pid_t pid, cpumask_t *mask)
{
	struct task_struct *p;
	raw_spinlock_t *lock;
	unsigned long flags;
	int retval;

	rcu_read_lock();

	retval = -ESRCH;
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;

	task_access_lock_irqsave(p, &lock, &flags);
	cpumask_and(mask, &p->cpus_mask, cpu_active_mask);
	task_access_unlock_irqrestore(p, lock, &flags);

out_unlock:
	rcu_read_unlock();

	return retval;
}

/**
 * sys_sched_getaffinity - get the CPU affinity of a process
 * @pid: pid of the process
 * @len: length in bytes of the bitmask pointed to by user_mask_ptr
 * @user_mask_ptr: user-space pointer to hold the current CPU mask
 *
 * Return: size of CPU mask copied to user_mask_ptr on success. An
 * error code otherwise.
 */
SYSCALL_DEFINE3(sched_getaffinity, pid_t, pid, unsigned int, len,
		unsigned long __user *, user_mask_ptr)
{
	int ret;
	cpumask_var_t mask;

	if ((len * BITS_PER_BYTE) < nr_cpu_ids)
		return -EINVAL;
	if (len & (sizeof(unsigned long)-1))
		return -EINVAL;

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	ret = sched_getaffinity(pid, mask);
	if (ret == 0) {
		unsigned int retlen = min_t(size_t, len, cpumask_size());

		if (copy_to_user(user_mask_ptr, mask, retlen))
			ret = -EFAULT;
		else
			ret = retlen;
	}
	free_cpumask_var(mask);

	return ret;
}

/**
 * sys_sched_yield - yield the current processor to other threads.
 *
 * This function yields the current CPU to other tasks. It does this by
 * scheduling away the current task. If it still has the earliest deadline
 * it will be scheduled again as the next task.
 *
 * Return: 0.
 */
static void do_sched_yield(void)
{
	struct rq *rq;
	struct rq_flags rf;

	if (!sched_yield_type)
		return;

	rq = this_rq_lock_irq(&rf);

	schedstat_inc(rq->yld_count);

	if (1 == sched_yield_type) {
		if (!rt_task(current)) {
			current->boost_prio = MAX_PRIORITY_ADJ;
			requeue_task(current, rq);
		}
	} else if (2 == sched_yield_type) {
		if (rq->nr_running > 1)
			rq->skip = current;
	}

	/*
	 * Since we are going to call schedule() anyway, there's
	 * no need to preempt or enable interrupts:
	 */
	preempt_disable();
	raw_spin_unlock(&rq->lock);
	sched_preempt_enable_no_resched();

	schedule();
}

SYSCALL_DEFINE0(sched_yield)
{
	do_sched_yield();
	return 0;
}

#ifndef CONFIG_PREEMPTION
int __sched _cond_resched(void)
{
	if (should_resched(0)) {
		preempt_schedule_common();
		return 1;
	}
	rcu_all_qs();
	return 0;
}
EXPORT_SYMBOL(_cond_resched);
#endif

/*
 * __cond_resched_lock() - if a reschedule is pending, drop the given lock,
 * call schedule, and on return reacquire the lock.
 *
 * This works OK both with and without CONFIG_PREEMPTION.  We do strange low-level
 * operations here to prevent schedule() from being called twice (once via
 * spin_unlock(), once by hand).
 */
int __cond_resched_lock(spinlock_t *lock)
{
	int resched = should_resched(PREEMPT_LOCK_OFFSET);
	int ret = 0;

	lockdep_assert_held(lock);

	if (spin_needbreak(lock) || resched) {
		spin_unlock(lock);
		if (resched)
			preempt_schedule_common();
		else
			cpu_relax();
		ret = 1;
		spin_lock(lock);
	}
	return ret;
}
EXPORT_SYMBOL(__cond_resched_lock);

/**
 * yield - yield the current processor to other threads.
 *
 * Do not ever use this function, there's a 99% chance you're doing it wrong.
 *
 * The scheduler is at all times free to pick the calling task as the most
 * eligible task to run, if removing the yield() call from your code breaks
 * it, its already broken.
 *
 * Typical broken usage is:
 *
 * while (!event)
 * 	yield();
 *
 * where one assumes that yield() will let 'the other' process run that will
 * make event true. If the current task is a SCHED_FIFO task that will never
 * happen. Never use yield() as a progress guarantee!!
 *
 * If you want to use yield() to wait for something, use wait_event().
 * If you want to use yield() to be 'nice' for others, use cond_resched().
 * If you still want to use yield(), do not!
 */
void __sched yield(void)
{
	set_current_state(TASK_RUNNING);
	do_sched_yield();
}
EXPORT_SYMBOL(yield);

/**
 * yield_to - yield the current processor to another thread in
 * your thread group, or accelerate that thread toward the
 * processor it's on.
 * @p: target task
 * @preempt: whether task preemption is allowed or not
 *
 * It's the caller's job to ensure that the target task struct
 * can't go away on us before we can do any checks.
 *
 * In BMQ, yield_to is not supported.
 *
 * Return:
 *	true (>0) if we indeed boosted the target task.
 *	false (0) if we failed to boost the target.
 *	-ESRCH if there's no task to yield to.
 */
int __sched yield_to(struct task_struct *p, bool preempt)
{
	return 0;
}
EXPORT_SYMBOL_GPL(yield_to);

int io_schedule_prepare(void)
{
	int old_iowait = current->in_iowait;

	current->in_iowait = 1;
	blk_schedule_flush_plug(current);

	return old_iowait;
}

void io_schedule_finish(int token)
{
	current->in_iowait = token;
}

/*
 * This task is about to go to sleep on IO.  Increment rq->nr_iowait so
 * that process accounting knows that this is a task in IO wait state.
 *
 * But don't do that if it is a deliberate, throttling IO wait (this task
 * has set its backing_dev_info: the queue against which it should throttle)
 */

long __sched io_schedule_timeout(long timeout)
{
	int token;
	long ret;

	token = io_schedule_prepare();
	ret = schedule_timeout(timeout);
	io_schedule_finish(token);

	return ret;
}
EXPORT_SYMBOL(io_schedule_timeout);

void __sched io_schedule(void)
{
	int token;

	token = io_schedule_prepare();
	schedule();
	io_schedule_finish(token);
}
EXPORT_SYMBOL(io_schedule);

/**
 * sys_sched_get_priority_max - return maximum RT priority.
 * @policy: scheduling class.
 *
 * Return: On success, this syscall returns the maximum
 * rt_priority that can be used by a given scheduling class.
 * On failure, a negative error code is returned.
 */
SYSCALL_DEFINE1(sched_get_priority_max, int, policy)
{
	int ret = -EINVAL;

	switch (policy) {
	case SCHED_FIFO:
	case SCHED_RR:
		ret = MAX_USER_RT_PRIO-1;
		break;
	case SCHED_NORMAL:
	case SCHED_BATCH:
	case SCHED_IDLE:
		ret = 0;
		break;
	}
	return ret;
}

/**
 * sys_sched_get_priority_min - return minimum RT priority.
 * @policy: scheduling class.
 *
 * Return: On success, this syscall returns the minimum
 * rt_priority that can be used by a given scheduling class.
 * On failure, a negative error code is returned.
 */
SYSCALL_DEFINE1(sched_get_priority_min, int, policy)
{
	int ret = -EINVAL;

	switch (policy) {
	case SCHED_FIFO:
	case SCHED_RR:
		ret = 1;
		break;
	case SCHED_NORMAL:
	case SCHED_BATCH:
	case SCHED_IDLE:
		ret = 0;
		break;
	}
	return ret;
}

static int sched_rr_get_interval(pid_t pid, struct timespec64 *t)
{
	struct task_struct *p;
	int retval;

	if (pid < 0)
		return -EINVAL;

	retval = -ESRCH;
	rcu_read_lock();
	p = find_process_by_pid(pid);
	if (!p)
		goto out_unlock;

	retval = security_task_getscheduler(p);
	if (retval)
		goto out_unlock;
	rcu_read_unlock();

	*t = ns_to_timespec64(sched_timeslice_ns);
	return 0;

out_unlock:
	rcu_read_unlock();
	return retval;
}

/**
 * sys_sched_rr_get_interval - return the default timeslice of a process.
 * @pid: pid of the process.
 * @interval: userspace pointer to the timeslice value.
 *
 *
 * Return: On success, 0 and the timeslice is in @interval. Otherwise,
 * an error code.
 */
SYSCALL_DEFINE2(sched_rr_get_interval, pid_t, pid,
		struct __kernel_timespec __user *, interval)
{
	struct timespec64 t;
	int retval = sched_rr_get_interval(pid, &t);

	if (retval == 0)
		retval = put_timespec64(&t, interval);

	return retval;
}

#ifdef CONFIG_COMPAT_32BIT_TIME
SYSCALL_DEFINE2(sched_rr_get_interval_time32, pid_t, pid,
		struct old_timespec32 __user *, interval)
{
	struct timespec64 t;
	int retval = sched_rr_get_interval(pid, &t);

	if (retval == 0)
		retval = put_old_timespec32(&t, interval);
	return retval;
}
#endif

void sched_show_task(struct task_struct *p)
{
	unsigned long free = 0;
	int ppid;

	if (!try_get_task_stack(p))
		return;

	printk(KERN_INFO "%-15.15s %c", p->comm, task_state_to_char(p));

	if (p->state == TASK_RUNNING)
		printk(KERN_CONT "  running task    ");
#ifdef CONFIG_DEBUG_STACK_USAGE
	free = stack_not_used(p);
#endif
	ppid = 0;
	rcu_read_lock();
	if (pid_alive(p))
		ppid = task_pid_nr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	printk(KERN_CONT "%5lu %5d %6d 0x%08lx\n", free,
		task_pid_nr(p), ppid,
		(unsigned long)task_thread_info(p)->flags);

	print_worker_info(KERN_INFO, p);
	show_stack(p, NULL);
	put_task_stack(p);
}
EXPORT_SYMBOL_GPL(sched_show_task);

static inline bool
state_filter_match(unsigned long state_filter, struct task_struct *p)
{
	/* no filter, everything matches */
	if (!state_filter)
		return true;

	/* filter, but doesn't match */
	if (!(p->state & state_filter))
		return false;

	/*
	 * When looking for TASK_UNINTERRUPTIBLE skip TASK_IDLE (allows
	 * TASK_KILLABLE).
	 */
	if (state_filter == TASK_UNINTERRUPTIBLE && p->state == TASK_IDLE)
		return false;

	return true;
}


void show_state_filter(unsigned long state_filter)
{
	struct task_struct *g, *p;

#if BITS_PER_LONG == 32
	printk(KERN_INFO
		"  task                PC stack   pid father\n");
#else
	printk(KERN_INFO
		"  task                        PC stack   pid father\n");
#endif
	rcu_read_lock();
	for_each_process_thread(g, p) {
		/*
		 * reset the NMI-timeout, listing all files on a slow
		 * console might take a lot of time:
		 * Also, reset softlockup watchdogs on all CPUs, because
		 * another CPU might be blocked waiting for us to process
		 * an IPI.
		 */
		touch_nmi_watchdog();
		touch_all_softlockup_watchdogs();
		if (state_filter_match(state_filter, p))
			sched_show_task(p);
	}

#ifdef CONFIG_SCHED_DEBUG
	/* TODO: BMQ should support this
	if (!state_filter)
		sysrq_sched_debug_show();
	*/
#endif
	rcu_read_unlock();
	/*
	 * Only show locks if all tasks are dumped:
	 */
	if (!state_filter)
		debug_show_all_locks();
}

void dump_cpu_task(int cpu)
{
	pr_info("Task dump for CPU %d:\n", cpu);
	sched_show_task(cpu_curr(cpu));
}

/**
 * init_idle - set up an idle thread for a given CPU
 * @idle: task in question
 * @cpu: CPU the idle task belongs to
 *
 * NOTE: this function does not set the idle thread's NEED_RESCHED
 * flag, to make booting more robust.
 */
void init_idle(struct task_struct *idle, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	__sched_fork(0, idle);

	raw_spin_lock_irqsave(&idle->pi_lock, flags);
	raw_spin_lock(&rq->lock);
	update_rq_clock(rq);

	idle->last_ran = rq->clock_task;
	idle->state = TASK_RUNNING;
	idle->flags |= PF_IDLE;
	idle->bmq_idx = IDLE_TASK_SCHED_PRIO;
	bmq_init_idle(&rq->queue, idle);

	kasan_unpoison_task_stack(idle);

#ifdef CONFIG_SMP
	/*
	 * It's possible that init_idle() gets called multiple times on a task,
	 * in that case do_set_cpus_allowed() will not do the right thing.
	 *
	 * And since this is boot we can forgo the serialisation.
	 */
	set_cpus_allowed_common(idle, cpumask_of(cpu));
#endif

	/* Silence PROVE_RCU */
	rcu_read_lock();
	__set_task_cpu(idle, cpu);
	rcu_read_unlock();

	rq->idle = idle;
	rcu_assign_pointer(rq->curr, idle);
	idle->on_cpu = 1;

	raw_spin_unlock(&rq->lock);
	raw_spin_unlock_irqrestore(&idle->pi_lock, flags);

	/* Set the preempt count _outside_ the spinlocks! */
	init_idle_preempt_count(idle, cpu);

	ftrace_graph_init_idle_task(idle, cpu);
	vtime_init_idle(idle, cpu);
#ifdef CONFIG_SMP
	sprintf(idle->comm, "%s/%d", INIT_TASK_COMM, cpu);
#endif
}

#ifdef CONFIG_SMP

int cpuset_cpumask_can_shrink(const struct cpumask __maybe_unused *cur,
			      const struct cpumask __maybe_unused *trial)
{
	return 1;
}

int task_can_attach(struct task_struct *p,
		    const struct cpumask *cs_cpus_allowed)
{
	int ret = 0;

	/*
	 * Kthreads which disallow setaffinity shouldn't be moved
	 * to a new cpuset; we don't want to change their CPU
	 * affinity and isolating such threads by their set of
	 * allowed nodes is unnecessary.  Thus, cpusets are not
	 * applicable for such threads.  This prevents checking for
	 * success of set_cpus_allowed_ptr() on all attached tasks
	 * before cpus_mask may be changed.
	 */
	if (p->flags & PF_NO_SETAFFINITY)
		ret = -EINVAL;

	return ret;
}

bool sched_smp_initialized __read_mostly;

#ifdef CONFIG_HOTPLUG_CPU
/*
 * Ensures that the idle task is using init_mm right before its CPU goes
 * offline.
 */
void idle_task_exit(void)
{
	struct mm_struct *mm = current->active_mm;

	BUG_ON(cpu_online(smp_processor_id()));

	if (mm != &init_mm) {
		switch_mm(mm, &init_mm, current);
		current->active_mm = &init_mm;
		finish_arch_post_lock_switch();
	}
	mmdrop(mm);
}

/*
 * Migrate all tasks from the rq, sleeping tasks will be migrated by
 * try_to_wake_up()->select_task_rq().
 *
 * Called with rq->lock held even though we'er in stop_machine() and
 * there's no concurrency possible, we hold the required locks anyway
 * because of lock validation efforts.
 */
static void migrate_tasks(struct rq *dead_rq)
{
	struct rq *rq = dead_rq;
	struct task_struct *p, *stop = rq->stop;
	int count = 0;

	/*
	 * Fudge the rq selection such that the below task selection loop
	 * doesn't get stuck on the currently eligible stop task.
	 *
	 * We're currently inside stop_machine() and the rq is either stuck
	 * in the stop_machine_cpu_stop() loop, or we're executing this code,
	 * either way we should never end up calling schedule() until we're
	 * done here.
	 */
	rq->stop = NULL;

	p = rq_first_bmq_task(rq);
	while (p != rq->idle) {
		int dest_cpu;

		/* skip the running task */
		if (task_running(p) || 1 == p->nr_cpus_allowed) {
			p = rq_next_bmq_task(p, rq);
			continue;
		}

		/*
		 * Rules for changing task_struct::cpus_allowed are holding
		 * both pi_lock and rq->lock, such that holding either
		 * stabilizes the mask.
		 *
		 * Drop rq->lock is not quite as disastrous as it usually is
		 * because !cpu_active at this point, which means load-balance
		 * will not interfere. Also, stop-machine.
		 */
		raw_spin_unlock(&rq->lock);
		raw_spin_lock(&p->pi_lock);
		raw_spin_lock(&rq->lock);

		/*
		 * Since we're inside stop-machine, _nothing_ should have
		 * changed the task, WARN if weird stuff happened, because in
		 * that case the above rq->lock drop is a fail too.
		 */
		if (WARN_ON(task_rq(p) != rq || !task_on_rq_queued(p))) {
			raw_spin_unlock(&p->pi_lock);
			p = rq_next_bmq_task(p, rq);
			continue;
		}

		count++;
		/* Find suitable destination for @next, with force if needed. */
		dest_cpu = select_fallback_rq(dead_rq->cpu, p);
		rq = __migrate_task(rq, p, dest_cpu);
		raw_spin_unlock(&rq->lock);
		raw_spin_unlock(&p->pi_lock);

		rq = dead_rq;
		raw_spin_lock(&rq->lock);
		/* Check queued task all over from the header again */
		p = rq_first_bmq_task(rq);
	}

	rq->stop = stop;
}

static void set_rq_offline(struct rq *rq)
{
	if (rq->online)
		rq->online = false;
}
#endif /* CONFIG_HOTPLUG_CPU */

static void set_rq_online(struct rq *rq)
{
	if (!rq->online)
		rq->online = true;
}

/*
 * used to mark begin/end of suspend/resume:
 */
static int num_cpus_frozen;

/*
 * Update cpusets according to cpu_active mask.  If cpusets are
 * disabled, cpuset_update_active_cpus() becomes a simple wrapper
 * around partition_sched_domains().
 *
 * If we come here as part of a suspend/resume, don't touch cpusets because we
 * want to restore it back to its original state upon resume anyway.
 */
static void cpuset_cpu_active(void)
{
	if (cpuhp_tasks_frozen) {
		/*
		 * num_cpus_frozen tracks how many CPUs are involved in suspend
		 * resume sequence. As long as this is not the last online
		 * operation in the resume sequence, just build a single sched
		 * domain, ignoring cpusets.
		 */
		partition_sched_domains(1, NULL, NULL);
		if (--num_cpus_frozen)
			return;
		/*
		 * This is the last CPU online operation. So fall through and
		 * restore the original sched domains by considering the
		 * cpuset configurations.
		 */
		cpuset_force_rebuild();
	}

	cpuset_update_active_cpus();
}

static int cpuset_cpu_inactive(unsigned int cpu)
{
	if (!cpuhp_tasks_frozen) {
		cpuset_update_active_cpus();
	} else {
		num_cpus_frozen++;
		partition_sched_domains(1, NULL, NULL);
	}
	return 0;
}

int sched_cpu_activate(unsigned int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

#ifdef CONFIG_SCHED_SMT
	/*
	 * When going up, increment the number of cores with SMT present.
	 */
	if (cpumask_weight(cpu_smt_mask(cpu)) == 2)
		static_branch_inc_cpuslocked(&sched_smt_present);
#endif
	set_cpu_active(cpu, true);

	if (sched_smp_initialized)
		cpuset_cpu_active();

	/*
	 * Put the rq online, if not already. This happens:
	 *
	 * 1) In the early boot process, because we build the real domains
	 *    after all cpus have been brought up.
	 *
	 * 2) At runtime, if cpuset_cpu_active() fails to rebuild the
	 *    domains.
	 */
	raw_spin_lock_irqsave(&rq->lock, flags);
	set_rq_online(rq);
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	return 0;
}

int sched_cpu_deactivate(unsigned int cpu)
{
	int ret;

	set_cpu_active(cpu, false);
	/*
	 * We've cleared cpu_active_mask, wait for all preempt-disabled and RCU
	 * users of this state to go away such that all new such users will
	 * observe it.
	 *
	 * Do sync before park smpboot threads to take care the rcu boost case.
	 */
	synchronize_rcu();

#ifdef CONFIG_SCHED_SMT
	/*
	 * When going down, decrement the number of cores with SMT present.
	 */
	if (cpumask_weight(cpu_smt_mask(cpu)) == 2) {
		static_branch_dec_cpuslocked(&sched_smt_present);
		if (!static_branch_likely(&sched_smt_present))
			cpumask_clear(&sched_sg_idle_mask);
	}
#endif

	if (!sched_smp_initialized)
		return 0;

	ret = cpuset_cpu_inactive(cpu);
	if (ret) {
		set_cpu_active(cpu, true);
		return ret;
	}
	return 0;
}

static void sched_rq_cpu_starting(unsigned int cpu)
{
	struct rq *rq = cpu_rq(cpu);

	rq->calc_load_update = calc_load_update;
}

int sched_cpu_starting(unsigned int cpu)
{
	sched_rq_cpu_starting(cpu);
	sched_tick_start(cpu);
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
int sched_cpu_dying(unsigned int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

	sched_tick_stop(cpu);
	raw_spin_lock_irqsave(&rq->lock, flags);
	set_rq_offline(rq);
	migrate_tasks(rq);
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	hrtick_clear(rq);
	return 0;
}
#endif

#ifdef CONFIG_SMP
static void sched_init_topology_cpumask_early(void)
{
	int cpu, level;
	cpumask_t *tmp;

	for_each_possible_cpu(cpu) {
		for (level = 0; level < NR_CPU_AFFINITY_CHK_LEVEL; level++) {
			tmp = &(per_cpu(sched_cpu_affinity_masks, cpu)[level]);
			cpumask_copy(tmp, cpu_possible_mask);
			cpumask_clear_cpu(cpu, tmp);
		}
		per_cpu(sched_cpu_llc_mask, cpu) =
			&(per_cpu(sched_cpu_affinity_masks, cpu)[0]);
		per_cpu(sched_cpu_affinity_end_mask, cpu) =
			&(per_cpu(sched_cpu_affinity_masks, cpu)[1]);
		per_cpu(sd_llc_id, cpu) = cpu;
	}
}

#define TOPOLOGY_CPUMASK(name, mask, last) \
	if (cpumask_and(chk, chk, mask))					\
		printk(KERN_INFO "bmq: cpu#%02d affinity mask: 0x%08lx - "#name,\
		       cpu, (chk++)->bits[0]);					\
	if (!last)								\
		cpumask_complement(chk, mask)

static void sched_init_topology_cpumask(void)
{
	int cpu;
	cpumask_t *chk;

	for_each_online_cpu(cpu) {
		/* take chance to reset time slice for idle tasks */
		cpu_rq(cpu)->idle->time_slice = sched_timeslice_ns;

		chk = &(per_cpu(sched_cpu_affinity_masks, cpu)[0]);

		cpumask_complement(chk, cpumask_of(cpu));
#ifdef CONFIG_SCHED_SMT
		TOPOLOGY_CPUMASK(smt, topology_sibling_cpumask(cpu), false);
#endif
		per_cpu(sd_llc_id, cpu) = cpumask_first(cpu_coregroup_mask(cpu));
		per_cpu(sched_cpu_llc_mask, cpu) = chk;
		TOPOLOGY_CPUMASK(coregroup, cpu_coregroup_mask(cpu), false);

		TOPOLOGY_CPUMASK(core, topology_core_cpumask(cpu), false);

		TOPOLOGY_CPUMASK(others, cpu_online_mask, true);

		per_cpu(sched_cpu_affinity_end_mask, cpu) = chk;
		printk(KERN_INFO "bmq: cpu#%02d llc_id = %d, llc_mask idx = %d\n",
		       cpu, per_cpu(sd_llc_id, cpu),
		       (int) (per_cpu(sched_cpu_llc_mask, cpu) -
			      &(per_cpu(sched_cpu_affinity_masks, cpu)[0])));
	}
}
#endif

void __init sched_init_smp(void)
{
	/* Move init over to a non-isolated CPU */
	if (set_cpus_allowed_ptr(current, housekeeping_cpumask(HK_FLAG_DOMAIN)) < 0)
		BUG();

	sched_init_topology_cpumask();

	sched_smp_initialized = true;
}
#else
void __init sched_init_smp(void)
{
	cpu_rq(0)->idle->time_slice = sched_timeslice_ns;
}
#endif /* CONFIG_SMP */

int in_sched_functions(unsigned long addr)
{
	return in_lock_functions(addr) ||
		(addr >= (unsigned long)__sched_text_start
		&& addr < (unsigned long)__sched_text_end);
}

#ifdef CONFIG_CGROUP_SCHED
/* task group related information */
struct task_group {
	struct cgroup_subsys_state css;

	struct rcu_head rcu;
	struct list_head list;

	struct task_group *parent;
	struct list_head siblings;
	struct list_head children;
};

/*
 * Default task group.
 * Every task in system belongs to this group at bootup.
 */
struct task_group root_task_group;
LIST_HEAD(task_groups);

/* Cacheline aligned slab cache for task_group */
static struct kmem_cache *task_group_cache __read_mostly;
#endif /* CONFIG_CGROUP_SCHED */

void __init sched_init(void)
{
	int i;
	struct rq *rq;

	print_scheduler_version();

	wait_bit_init();

#ifdef CONFIG_SMP
	for (i = 0; i < bmq_BITS; i++)
		cpumask_copy(&sched_rq_watermark[i], cpu_present_mask);
#endif

#ifdef CONFIG_CGROUP_SCHED
	task_group_cache = KMEM_CACHE(task_group, 0);

	list_add(&root_task_group.list, &task_groups);
	INIT_LIST_HEAD(&root_task_group.children);
	INIT_LIST_HEAD(&root_task_group.siblings);
#endif /* CONFIG_CGROUP_SCHED */
	for_each_possible_cpu(i) {
		rq = cpu_rq(i);

		bmq_init(&rq->queue);
		rq->watermark = IDLE_WM;
		rq->skip = NULL;

		raw_spin_lock_init(&rq->lock);
		rq->nr_running = rq->nr_uninterruptible = 0;
		rq->calc_load_active = 0;
		rq->calc_load_update = jiffies + LOAD_FREQ;
#ifdef CONFIG_SMP
		rq->online = false;
		rq->cpu = i;

#ifdef CONFIG_SCHED_SMT
		rq->active_balance = 0;
#endif
#endif
		rq->nr_switches = 0;
		atomic_set(&rq->nr_iowait, 0);
		hrtick_rq_init(rq);
	}
#ifdef CONFIG_SMP
	/* Set rq->online for cpu 0 */
	cpu_rq(0)->online = true;
#endif

	/*
	 * The boot idle thread does lazy MMU switching as well:
	 */
	mmgrab(&init_mm);
	enter_lazy_tlb(&init_mm, current);

	/*
	 * Make us the idle thread. Technically, schedule() should not be
	 * called from this thread, however somewhere below it might be,
	 * but because we are the idle thread, we just pick up running again
	 * when this runqueue becomes "idle".
	 */
	init_idle(current, smp_processor_id());

	calc_load_update = jiffies + LOAD_FREQ;

#ifdef CONFIG_SMP
	idle_thread_set_boot_cpu();

	sched_init_topology_cpumask_early();
#endif /* SMP */

	init_schedstats();

	psi_init();
}

#ifdef CONFIG_DEBUG_ATOMIC_SLEEP
static inline int preempt_count_equals(int preempt_offset)
{
	int nested = preempt_count() + rcu_preempt_depth();

	return (nested == preempt_offset);
}

void __might_sleep(const char *file, int line, int preempt_offset)
{
	/*
	 * Blocking primitives will set (and therefore destroy) current->state,
	 * since we will exit with TASK_RUNNING make sure we enter with it,
	 * otherwise we will destroy state.
	 */
	WARN_ONCE(current->state != TASK_RUNNING && current->task_state_change,
			"do not call blocking ops when !TASK_RUNNING; "
			"state=%lx set at [<%p>] %pS\n",
			current->state,
			(void *)current->task_state_change,
			(void *)current->task_state_change);

	___might_sleep(file, line, preempt_offset);
}
EXPORT_SYMBOL(__might_sleep);

void ___might_sleep(const char *file, int line, int preempt_offset)
{
	/* Ratelimiting timestamp: */
	static unsigned long prev_jiffy;

	unsigned long preempt_disable_ip;

	/* WARN_ON_ONCE() by default, no rate limit required: */
	rcu_sleep_check();

	if ((preempt_count_equals(preempt_offset) && !irqs_disabled() &&
	     !is_idle_task(current) && !current->non_block_count) ||
	    system_state == SYSTEM_BOOTING || system_state > SYSTEM_RUNNING ||
	    oops_in_progress)
		return;
	if (time_before(jiffies, prev_jiffy + HZ) && prev_jiffy)
		return;
	prev_jiffy = jiffies;

	/* Save this before calling printk(), since that will clobber it: */
	preempt_disable_ip = get_preempt_disable_ip(current);

	printk(KERN_ERR
		"BUG: sleeping function called from invalid context at %s:%d\n",
			file, line);
	printk(KERN_ERR
		"in_atomic(): %d, irqs_disabled(): %d, non_block: %d, pid: %d, name: %s\n",
			in_atomic(), irqs_disabled(), current->non_block_count,
			current->pid, current->comm);

	if (task_stack_end_corrupted(current))
		printk(KERN_EMERG "Thread overran stack, or stack corrupted\n");

	debug_show_held_locks(current);
	if (irqs_disabled())
		print_irqtrace_events(current);
#ifdef CONFIG_DEBUG_PREEMPT
	if (!preempt_count_equals(preempt_offset)) {
		pr_err("Preemption disabled at:");
		print_ip_sym(preempt_disable_ip);
		pr_cont("\n");
	}
#endif
	dump_stack();
	add_taint(TAINT_WARN, LOCKDEP_STILL_OK);
}
EXPORT_SYMBOL(___might_sleep);

void __cant_sleep(const char *file, int line, int preempt_offset)
{
	static unsigned long prev_jiffy;

	if (irqs_disabled())
		return;

	if (!IS_ENABLED(CONFIG_PREEMPT_COUNT))
		return;

	if (preempt_count() > preempt_offset)
		return;

	if (time_before(jiffies, prev_jiffy + HZ) && prev_jiffy)
		return;
	prev_jiffy = jiffies;

	printk(KERN_ERR "BUG: assuming atomic context at %s:%d\n", file, line);
	printk(KERN_ERR "in_atomic(): %d, irqs_disabled(): %d, pid: %d, name: %s\n",
			in_atomic(), irqs_disabled(),
			current->pid, current->comm);

	debug_show_held_locks(current);
	dump_stack();
	add_taint(TAINT_WARN, LOCKDEP_STILL_OK);
}
EXPORT_SYMBOL_GPL(__cant_sleep);
#endif

#ifdef CONFIG_MAGIC_SYSRQ
void normalize_rt_tasks(void)
{
	struct task_struct *g, *p;
	struct sched_attr attr = {
		.sched_policy = SCHED_NORMAL,
	};

	read_lock(&tasklist_lock);
	for_each_process_thread(g, p) {
		/*
		 * Only normalize user tasks:
		 */
		if (p->flags & PF_KTHREAD)
			continue;

		if (!rt_task(p)) {
			/*
			 * Renice negative nice level userspace
			 * tasks back to 0:
			 */
			if (task_nice(p) < 0)
				set_user_nice(p, 0);
			continue;
		}

		__sched_setscheduler(p, &attr, false, false);
	}
	read_unlock(&tasklist_lock);
}
#endif /* CONFIG_MAGIC_SYSRQ */

#if defined(CONFIG_IA64) || defined(CONFIG_KGDB_KDB)
/*
 * These functions are only useful for the IA64 MCA handling, or kdb.
 *
 * They can only be called when the whole system has been
 * stopped - every CPU needs to be quiescent, and no scheduling
 * activity can take place. Using them for anything else would
 * be a serious bug, and as a result, they aren't even visible
 * under any other configuration.
 */

/**
 * curr_task - return the current task for a given CPU.
 * @cpu: the processor in question.
 *
 * ONLY VALID WHEN THE WHOLE SYSTEM IS STOPPED!
 *
 * Return: The current task for @cpu.
 */
struct task_struct *curr_task(int cpu)
{
	return cpu_curr(cpu);
}

#endif /* defined(CONFIG_IA64) || defined(CONFIG_KGDB_KDB) */

#ifdef CONFIG_IA64
/**
 * ia64_set_curr_task - set the current task for a given CPU.
 * @cpu: the processor in question.
 * @p: the task pointer to set.
 *
 * Description: This function must only be used when non-maskable interrupts
 * are serviced on a separate stack.  It allows the architecture to switch the
 * notion of the current task on a CPU in a non-blocking manner.  This function
 * must be called with all CPU's synchronised, and interrupts disabled, the
 * and caller must save the original value of the current task (see
 * curr_task() above) and restore that value before reenabling interrupts and
 * re-starting the system.
 *
 * ONLY VALID WHEN THE WHOLE SYSTEM IS STOPPED!
 */
void ia64_set_curr_task(int cpu, struct task_struct *p)
{
	cpu_curr(cpu) = p;
}

#endif

#ifdef CONFIG_CGROUP_SCHED
static void sched_free_group(struct task_group *tg)
{
	kmem_cache_free(task_group_cache, tg);
}

/* allocate runqueue etc for a new task group */
struct task_group *sched_create_group(struct task_group *parent)
{
	struct task_group *tg;

	tg = kmem_cache_alloc(task_group_cache, GFP_KERNEL | __GFP_ZERO);
	if (!tg)
		return ERR_PTR(-ENOMEM);

	return tg;
}

void sched_online_group(struct task_group *tg, struct task_group *parent)
{
}

/* rcu callback to free various structures associated with a task group */
static void sched_free_group_rcu(struct rcu_head *rhp)
{
	/* Now it should be safe to free those cfs_rqs */
	sched_free_group(container_of(rhp, struct task_group, rcu));
}

void sched_destroy_group(struct task_group *tg)
{
	/* Wait for possible concurrent references to cfs_rqs complete */
	call_rcu(&tg->rcu, sched_free_group_rcu);
}

void sched_offline_group(struct task_group *tg)
{
}

static inline struct task_group *css_tg(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct task_group, css) : NULL;
}

static struct cgroup_subsys_state *
cpu_cgroup_css_alloc(struct cgroup_subsys_state *parent_css)
{
	struct task_group *parent = css_tg(parent_css);
	struct task_group *tg;

	if (!parent) {
		/* This is early initialization for the top cgroup */
		return &root_task_group.css;
	}

	tg = sched_create_group(parent);
	if (IS_ERR(tg))
		return ERR_PTR(-ENOMEM);
	return &tg->css;
}

/* Expose task group only after completing cgroup initialization */
static int cpu_cgroup_css_online(struct cgroup_subsys_state *css)
{
	struct task_group *tg = css_tg(css);
	struct task_group *parent = css_tg(css->parent);

	if (parent)
		sched_online_group(tg, parent);
	return 0;
}

static void cpu_cgroup_css_released(struct cgroup_subsys_state *css)
{
	struct task_group *tg = css_tg(css);

	sched_offline_group(tg);
}

static void cpu_cgroup_css_free(struct cgroup_subsys_state *css)
{
	struct task_group *tg = css_tg(css);

	/*
	 * Relies on the RCU grace period between css_released() and this.
	 */
	sched_free_group(tg);
}

static void cpu_cgroup_fork(struct task_struct *task)
{
}

static int cpu_cgroup_can_attach(struct cgroup_taskset *tset)
{
	return 0;
}

static void cpu_cgroup_attach(struct cgroup_taskset *tset)
{
}

static struct cftype cpu_legacy_files[] = {
	{ }	/* Terminate */
};

static struct cftype cpu_files[] = {
	{ }	/* terminate */
};

static int cpu_extra_stat_show(struct seq_file *sf,
			       struct cgroup_subsys_state *css)
{
	return 0;
}

struct cgroup_subsys cpu_cgrp_subsys = {
	.css_alloc	= cpu_cgroup_css_alloc,
	.css_online	= cpu_cgroup_css_online,
	.css_released	= cpu_cgroup_css_released,
	.css_free	= cpu_cgroup_css_free,
	.css_extra_stat_show = cpu_extra_stat_show,
	.fork		= cpu_cgroup_fork,
	.can_attach	= cpu_cgroup_can_attach,
	.attach		= cpu_cgroup_attach,
	.legacy_cftypes	= cpu_files,
	.legacy_cftypes	= cpu_legacy_files,
	.dfl_cftypes	= cpu_files,
	.early_init	= true,
	.threaded	= true,
};
#endif	/* CONFIG_CGROUP_SCHED */

#undef CREATE_TRACE_POINTS

```

```c
#ifndef BMQ_SCHED_H
#define BMQ_SCHED_H

#include <linux/sched.h>

#include <linux/sched/clock.h>
#include <linux/sched/cpufreq.h>
#include <linux/sched/cputime.h>
#include <linux/sched/debug.h>
#include <linux/sched/init.h>
#include <linux/sched/isolation.h>
#include <linux/sched/loadavg.h>
#include <linux/sched/mm.h>
#include <linux/sched/nohz.h>
#include <linux/sched/signal.h>
#include <linux/sched/stat.h>
#include <linux/sched/sysctl.h>
#include <linux/sched/task.h>
#include <linux/sched/topology.h>
#include <linux/sched/wake_q.h>

#include <uapi/linux/sched/types.h>

#include <linux/cgroup.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/cpuset.h>
#include <linux/ctype.h>
#include <linux/kthread.h>
#include <linux/livepatch.h>
#include <linux/membarrier.h>
#include <linux/proc_fs.h>
#include <linux/psi.h>
#include <linux/slab.h>
#include <linux/stop_machine.h>
#include <linux/suspend.h>
#include <linux/swait.h>
#include <linux/syscalls.h>
#include <linux/tsacct_kern.h>

#include <asm/tlb.h>

#ifdef CONFIG_PARAVIRT
# include <asm/paravirt.h>
#endif

#include "cpupri.h"

/* task_struct::on_rq states: */
#define TASK_ON_RQ_QUEUED	1
#define TASK_ON_RQ_MIGRATING	2

static inline int task_on_rq_queued(struct task_struct *p)
{
	return p->on_rq == TASK_ON_RQ_QUEUED;
}

static inline int task_on_rq_migrating(struct task_struct *p)
{
	return READ_ONCE(p->on_rq) == TASK_ON_RQ_MIGRATING;
}

/*
 * wake flags
 */
#define WF_SYNC		0x01		/* waker goes to sleep after wakeup */
#define WF_FORK		0x02		/* child wakeup after fork */
#define WF_MIGRATED	0x04		/* internal use, task got migrated */

/* bits:
 * RT(0-99), Low prio adj range, nice width, high prio adj range, cpu idle task */
#define bmq_BITS	(MAX_RT_PRIO + NICE_WIDTH + 2 * MAX_PRIORITY_ADJ + 1)
#define IDLE_TASK_SCHED_PRIO	(bmq_BITS - 1)

struct bmq {
	DECLARE_BITMAP(bitmap, bmq_BITS);
	struct list_head heads[bmq_BITS];
};

/*
 * This is the main, per-CPU runqueue data structure.
 * This data should only be modified by the local cpu.
 */
struct rq {
	/* runqueue lock: */
	raw_spinlock_t lock;

	struct task_struct __rcu *curr;
	struct task_struct *idle, *stop, *skip;
	struct mm_struct *prev_mm;

	struct bmq queue;
	unsigned long watermark;

	/* switch count */
	u64 nr_switches;

	atomic_t nr_iowait;

#ifdef CONFIG_MEMBARRIER
	int membarrier_state;
#endif

#ifdef CONFIG_SMP
	int cpu;		/* cpu of this runqueue */
	bool online;

#ifdef CONFIG_HAVE_SCHED_AVG_IRQ
	struct sched_avg	avg_irq;
#endif

#ifdef CONFIG_SCHED_SMT
	int active_balance;
	struct cpu_stop_work active_balance_work;
#endif
#endif /* CONFIG_SMP */
#ifdef CONFIG_IRQ_TIME_ACCOUNTING
	u64 prev_irq_time;
#endif /* CONFIG_IRQ_TIME_ACCOUNTING */
#ifdef CONFIG_PARAVIRT
	u64 prev_steal_time;
#endif /* CONFIG_PARAVIRT */
#ifdef CONFIG_PARAVIRT_TIME_ACCOUNTING
	u64 prev_steal_time_rq;
#endif /* CONFIG_PARAVIRT_TIME_ACCOUNTING */

	/* calc_load related fields */
	unsigned long calc_load_update;
	long calc_load_active;

	u64 clock, last_tick;
	u64 last_ts_switch;
	u64 clock_task;

	unsigned long nr_running;
	unsigned long nr_uninterruptible;

#ifdef CONFIG_SCHED_HRTICK
#ifdef CONFIG_SMP
	call_single_data_t hrtick_csd;
#endif
	struct hrtimer hrtick_timer;
#endif

#ifdef CONFIG_SCHEDSTATS

	/* latency stats */
	struct sched_info rq_sched_info;
	unsigned long long rq_cpu_time;
	/* could above be rq->cfs_rq.exec_clock + rq->rt_rq.rt_runtime ? */

	/* sys_sched_yield() stats */
	unsigned int yld_count;

	/* schedule() stats */
	unsigned int sched_switch;
	unsigned int sched_count;
	unsigned int sched_goidle;

	/* try_to_wake_up() stats */
	unsigned int ttwu_count;
	unsigned int ttwu_local;
#endif /* CONFIG_SCHEDSTATS */
#ifdef CONFIG_CPU_IDLE
	/* Must be inspected within a rcu lock section */
	struct cpuidle_state *idle_state;
#endif
};

extern unsigned long calc_load_update;
extern atomic_long_t calc_load_tasks;

extern void calc_global_load_tick(struct rq *this_rq);
extern long calc_load_fold_active(struct rq *this_rq, long adjust);

DECLARE_PER_CPU_SHARED_ALIGNED(struct rq, runqueues);
#define cpu_rq(cpu)		(&per_cpu(runqueues, (cpu)))
#define this_rq()		this_cpu_ptr(&runqueues)
#define task_rq(p)		cpu_rq(task_cpu(p))
#define cpu_curr(cpu)		(cpu_rq(cpu)->curr)
#define raw_rq()		raw_cpu_ptr(&runqueues)

#ifdef CONFIG_SMP
#if defined(CONFIG_SCHED_DEBUG) && defined(CONFIG_SYSCTL)
void register_sched_domain_sysctl(void);
void unregister_sched_domain_sysctl(void);
#else
static inline void register_sched_domain_sysctl(void)
{
}
static inline void unregister_sched_domain_sysctl(void)
{
}
#endif

extern bool sched_smp_initialized;

enum {
	BASE_CPU_AFFINITY_CHK_LEVEL = 1,
#ifdef CONFIG_SCHED_SMT
	SMT_CPU_AFFINITY_CHK_LEVEL_SPACE_HOLDER,
#endif
#ifdef CONFIG_SCHED_MC
	MC_CPU_AFFINITY_CHK_LEVEL_SPACE_HOLDER,
#endif
	NR_CPU_AFFINITY_CHK_LEVEL
};

DECLARE_PER_CPU(cpumask_t [NR_CPU_AFFINITY_CHK_LEVEL], sched_cpu_affinity_masks);

static inline int __best_mask_cpu(int cpu, const cpumask_t *cpumask,
				  const cpumask_t *mask)
{
	while ((cpu = cpumask_any_and(cpumask, mask)) >= nr_cpu_ids)
		mask++;
	return cpu;
}

static inline int best_mask_cpu(int cpu, const cpumask_t *cpumask)
{
	return cpumask_test_cpu(cpu, cpumask)? cpu :
		__best_mask_cpu(cpu, cpumask, &(per_cpu(sched_cpu_affinity_masks, cpu)[0]));
}

#endif /* CONFIG_SMP */

#ifndef arch_scale_freq_tick
static __always_inline
void arch_scale_freq_tick(void)
{
}
#endif

#ifndef arch_scale_freq_capacity
static __always_inline
unsigned long arch_scale_freq_capacity(int cpu)
{
	return SCHED_CAPACITY_SCALE;
}
#endif

static inline u64 __rq_clock_broken(struct rq *rq)
{
	return READ_ONCE(rq->clock);
}

static inline u64 rq_clock(struct rq *rq)
{
	/*
	 * Relax lockdep_assert_held() checking as in VRQ, call to
	 * sched_info_xxxx() may not held rq->lock
	 * lockdep_assert_held(&rq->lock);
	 */
	return rq->clock;
}

static inline u64 rq_clock_task(struct rq *rq)
{
	/*
	 * Relax lockdep_assert_held() checking as in VRQ, call to
	 * sched_info_xxxx() may not held rq->lock
	 * lockdep_assert_held(&rq->lock);
	 */
	return rq->clock_task;
}

/*
 * {de,en}queue flags:
 *
 * DEQUEUE_SLEEP  - task is no longer runnable
 * ENQUEUE_WAKEUP - task just became runnable
 *
 */

#define DEQUEUE_SLEEP		0x01

#define ENQUEUE_WAKEUP		0x01


/*
 * Below are scheduler API which using in other kernel code
 * It use the dummy rq_flags
 * ToDo : BMQ need to support these APIs for compatibility with mainline
 * scheduler code.
 */
struct rq_flags {
	unsigned long flags;
};

struct rq *__task_rq_lock(struct task_struct *p, struct rq_flags *rf)
	__acquires(rq->lock);

struct rq *task_rq_lock(struct task_struct *p, struct rq_flags *rf)
	__acquires(p->pi_lock)
	__acquires(rq->lock);

static inline void __task_rq_unlock(struct rq *rq, struct rq_flags *rf)
	__releases(rq->lock)
{
	raw_spin_unlock(&rq->lock);
}

static inline void
task_rq_unlock(struct rq *rq, struct task_struct *p, struct rq_flags *rf)
	__releases(rq->lock)
	__releases(p->pi_lock)
{
	raw_spin_unlock(&rq->lock);
	raw_spin_unlock_irqrestore(&p->pi_lock, rf->flags);
}

static inline void
rq_unlock_irq(struct rq *rq, struct rq_flags *rf)
	__releases(rq->lock)
{
	raw_spin_unlock_irq(&rq->lock);
}

static inline struct rq *
this_rq_lock_irq(struct rq_flags *rf)
	__acquires(rq->lock)
{
	struct rq *rq;

	local_irq_disable();
	rq = this_rq();
	raw_spin_lock(&rq->lock);

	return rq;
}

static inline int task_current(struct rq *rq, struct task_struct *p)
{
	return rq->curr == p;
}

static inline bool task_running(struct task_struct *p)
{
	return p->on_cpu;
}

extern struct static_key_false sched_schedstats;

static inline void sched_ttwu_pending(void) { }

#ifdef CONFIG_CPU_IDLE
static inline void idle_set_state(struct rq *rq,
				  struct cpuidle_state *idle_state)
{
	rq->idle_state = idle_state;
}

static inline struct cpuidle_state *idle_get_state(struct rq *rq)
{
	WARN_ON(!rcu_read_lock_held());
	return rq->idle_state;
}
#else
static inline void idle_set_state(struct rq *rq,
				  struct cpuidle_state *idle_state)
{
}

static inline struct cpuidle_state *idle_get_state(struct rq *rq)
{
	return NULL;
}
#endif

static inline int cpu_of(const struct rq *rq)
{
#ifdef CONFIG_SMP
	return rq->cpu;
#else
	return 0;
#endif
}

#include "stats.h"

#ifdef CONFIG_IRQ_TIME_ACCOUNTING
struct irqtime {
	u64			total;
	u64			tick_delta;
	u64			irq_start_time;
	struct u64_stats_sync	sync;
};

DECLARE_PER_CPU(struct irqtime, cpu_irqtime);

/*
 * Returns the irqtime minus the softirq time computed by ksoftirqd.
 * Otherwise ksoftirqd's sum_exec_runtime is substracted its own runtime
 * and never move forward.
 */
static inline u64 irq_time_read(int cpu)
{
	struct irqtime *irqtime = &per_cpu(cpu_irqtime, cpu);
	unsigned int seq;
	u64 total;

	do {
		seq = __u64_stats_fetch_begin(&irqtime->sync);
		total = irqtime->total;
	} while (__u64_stats_fetch_retry(&irqtime->sync, seq));

	return total;
}
#endif /* CONFIG_IRQ_TIME_ACCOUNTING */

#ifdef CONFIG_CPU_FREQ
DECLARE_PER_CPU(struct update_util_data __rcu *, cpufreq_update_util_data);

/**
 * cpufreq_update_util - Take a note about CPU utilization changes.
 * @rq: Runqueue to carry out the update for.
 * @flags: Update reason flags.
 *
 * This function is called by the scheduler on the CPU whose utilization is
 * being updated.
 *
 * It can only be called from RCU-sched read-side critical sections.
 *
 * The way cpufreq is currently arranged requires it to evaluate the CPU
 * performance state (frequency/voltage) on a regular basis to prevent it from
 * being stuck in a completely inadequate performance level for too long.
 * That is not guaranteed to happen if the updates are only triggered from CFS
 * and DL, though, because they may not be coming in if only RT tasks are
 * active all the time (or there are RT tasks only).
 *
 * As a workaround for that issue, this function is called periodically by the
 * RT sched class to trigger extra cpufreq updates to prevent it from stalling,
 * but that really is a band-aid.  Going forward it should be replaced with
 * solutions targeted more specifically at RT tasks.
 */
static inline void cpufreq_update_util(struct rq *rq, unsigned int flags)
{
	struct update_util_data *data;

	data = rcu_dereference_sched(*this_cpu_ptr(&cpufreq_update_util_data));
	if (data)
		data->func(data, rq_clock(rq), flags);
}
#else
static inline void cpufreq_update_util(struct rq *rq, unsigned int flags) {}
#endif /* CONFIG_CPU_FREQ */

#ifdef CONFIG_NO_HZ_FULL
extern int __init sched_tick_offload_init(void);
#else
static inline int sched_tick_offload_init(void) { return 0; }
#endif

#ifdef arch_scale_freq_capacity
#ifndef arch_scale_freq_invariant
#define arch_scale_freq_invariant()	(true)
#endif
#else /* arch_scale_freq_capacity */
#define arch_scale_freq_invariant()	(false)
#endif

extern void schedule_idle(void);

/*
 * !! For sched_setattr_nocheck() (kernel) only !!
 *
 * This is actually gross. :(
 *
 * It is used to make schedutil kworker(s) higher priority than SCHED_DEADLINE
 * tasks, but still be able to sleep. We need this on platforms that cannot
 * atomically change clock frequency. Remove once fast switching will be
 * available on such platforms.
 *
 * SUGOV stands for SchedUtil GOVernor.
 */
#define SCHED_FLAG_SUGOV	0x10000000

#ifdef CONFIG_MEMBARRIER
/*
 * The scheduler provides memory barriers required by membarrier between:
 * - prior user-space memory accesses and store to rq->membarrier_state,
 * - store to rq->membarrier_state and following user-space memory accesses.
 * In the same way it provides those guarantees around store to rq->curr.
 */
static inline void membarrier_switch_mm(struct rq *rq,
					struct mm_struct *prev_mm,
					struct mm_struct *next_mm)
{
	int membarrier_state;

	if (prev_mm == next_mm)
		return;

	membarrier_state = atomic_read(&next_mm->membarrier_state);
	if (READ_ONCE(rq->membarrier_state) == membarrier_state)
		return;

	WRITE_ONCE(rq->membarrier_state, membarrier_state);
}
#else
static inline void membarrier_switch_mm(struct rq *rq,
					struct mm_struct *prev_mm,
					struct mm_struct *next_mm)
{
}
#endif

static inline int task_running_nice(struct task_struct *p)
{
	return (p->prio + p->boost_prio > DEFAULT_PRIO + MAX_PRIORITY_ADJ);
}

#ifdef CONFIG_NUMA
extern int sched_numa_find_closest(const struct cpumask *cpus, int cpu);
#else
static inline int sched_numa_find_closest(const struct cpumask *cpus, int cpu)
{
	return nr_cpu_ids;
}
#endif

void swake_up_all_locked(struct swait_queue_head *q);
void __prepare_to_swait(struct swait_queue_head *q, struct swait_queue *wait);

#endif /* BMQ_SCHED_H */
```
%%  %%
