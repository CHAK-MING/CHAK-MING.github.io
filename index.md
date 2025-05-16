# BitMap Queue CPU 调度器

## 🔗 项目源码链接

- [bmq.c](https://github.com/CHAK-MING/CHAK-MING.github.io/blob/master/bmq.c)
- [bmq_debug.c](https://github.com/CHAK-MING/CHAK-MING.github.io/blob/master/bmq_debug.c)
- [bmq_sched.h](https://github.com/CHAK-MING/CHAK-MING.github.io/blob/master/bmq_sched.h)

## 1. 项目背景



## 2. 现有BMQ核心机制分析

### 2.1 核心数据结构

为实现ELF BMQ中“每个CPU的无锁抢占队列”和“优先级级别的全局运行队列锁”，数据结构的设计至关重要。核心代码位于 `kernel/sched/bmq_sched.h` 中，主要围绕以下两个结构体展开：`struct rq` (运行队列) 和 `struct bmq` (位图队列)。

#### 2.1.1 struct rq

`struct rq` 代表了每个CPU独立的运行队列。其关键成员与本次选题目标的关联如下：

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

结合项目目标对源码剖析：

- 同步原语（rq->lock）：
  - `lock` 是确保对运行队列数据进行独占性访问的主要机制。例如添加任务、移除任务、选择下一个任务以及更新等操作都会持有这个锁。
  - 
- 

#### 2.1.2  struct bmq

`struct bmq` 是 `struct rq` 的核心组成部分，直接负责可运行任务的组织和优先级管理。

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

### 2.2 CPU本地队列操作分析

#### 2.2.1 调度器初始化

初始化工作主要做了以下操作：

1. 调度器为系统中每个可能存在的CPU都初始化了一个 `struct rq` 实例。
2. 初始化每个CPU运行队列的自旋锁。
3. 对位图清零，将idle任务放入到特定优先级的链表中。
4. 全局状态初始化。

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

#### 2.2.2 任务入队

当一个任务变成可运行时（例如，从睡眠中唤醒或刚创建），调度器会根据其调度策略和优先级计算出进入到 BMQ 的哪个优先级队列，然后任务会添加到对应链表中，并且在位图中设置该优先级对应位为 1，表示该队列不为空。

例如，任务唤醒路径通常是 `wake_up_process()` -> `try_to_wake_up()` -> `ttwu_queue()`

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

// 这是实际的 enqueue_task 实现
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

分析 `enqueue_task`及其调用链：

- 从 `ttwu_queue` 开始，整个任务唤醒到最终入队的路径中，锁都被持有。

#### 2.2.3 任务出队

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

#### 2.2.4 任务选择与调度

任务会因为多种原因从运行队列中移除，比如阻塞（等待 I/O、锁等）、任务结束，或被迁移到其他 CPU。总体上可以分为两类情况：

- 一类是任务主动放弃 CPU 的情况，例如调用 `sleep()` 或 `wait_event()`等阻塞操作。
- 另一类是抢占式调度，例如时间片耗尽。

当任务的时间片耗尽时，系统定时器会触发一个时钟中断，进入中断处理程序。在处理过程中，调度器调用 `scheduler_tick()` 函数，该函数进一步调用 `scheduler_task_tick(rq)` 来处理当前任务的调度状态。

如果当前任务已经执行了足够长时间，调度器会调用 `resched_curr()`，设置该任务的 `TIF_NEED_RESCHED` 标志，表示需要重新调度。

 中断处理完成后，在从中断返回到内核或用户空间的路径上（例如 x86 架构中的 entry_64.S），内核会检查当前任务的 `_TIF_NEED_RESCHED` 标志。如果被设置，就会触发调度器，执行  `preempt_schedule_irq`，然后调用 `__schedule(true)`，将当前任务从运行队列中移除，切换到另一个任务。

第一类任务调用 `schedule()`，属于主动放弃 CPU 的情况。

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
- 这里有一点需要注意的是，`context_switch` 函数负责保存 `prev` 任务的上下文，加载 `next` 任务的上下文，完成CPU状态的切换。`context_switch` 函数末尾会调用 `finish_task_switch`，而 `finish_task_switch` 会负责释放 `rq->lock` （通过 `raw_spin_unlock_irq(&rq->lock); `）这个操作会将之前关闭中断的操作也同时开启。
- 

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

## 2.3 CPU间协作与负载均衡分析

在SMP（对称多处理）系统中，负载均衡是确保各个CPU的负载大致相当，从而提高系统整体吞吐量和响应性的关键机制。BMQ调度器中，当一个CPU的运行队列变空（或接近空闲）时，它会尝试从其他繁忙的CPU“窃取”任务来执行。这个过程主要由 `take_other_rq_tasks` 和 `migrate_pending_tasks` 两个函数协同完成。

### 2.3.1 窃取任务

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

## 3.现有的设计局限性

## 4. ELF BMQ 设计方案



