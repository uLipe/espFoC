# 10. OS abstraction and trace

This chapter covers two small modules that sit beside the motor control. The
**OS abstraction layer** (OSAL) is the only part of espFoC that talks to the
real-time operating system: tasks, sleeps, events, critical sections, mutexes
and the time base. You need it to write portable application code and to wake
a task from an interrupt. The **hot-path trace** is a ring buffer that records
what the control interrupts did, with timing; you need it when you debug loop
timing or a fault that happens faster than any log line.

## Part 1 — The OS abstraction layer

### What it does

ESP-IDF runs on FreeRTOS, but the control code does not depend on it. Every
RTOS service the component uses goes through one header,
[`esp_foc_osal.h`](../../include/espFoC/osal/esp_foc_osal.h), and one port
file, `source/osal/esp_foc_osal_idf.c`. That file is the only place in the
component that includes `freertos/*.h` or calls FreeRTOS functions. The
motor-control code, the drivers and the examples call `esp_foc_*` functions
only.

This gives a clear portability boundary: moving espFoC to another RTOS means
writing a new port of one header, not editing the control loops. It also
keeps the portable core free of the RTOS entirely: the host build (target
`linux`) compiles the math, estimators and trace without the OSAL, the
drivers or the stacks.

![Layered architecture](../diagram/architecture.png)

### How it works

The IDF port maps each service onto a FreeRTOS or IDF primitive:

| OSAL service | IDF / FreeRTOS primitive |
|---|---|
| tasks | `xTaskCreate`, `vTaskDelete`, `taskYIELD`, `eTaskGetState` |
| sleep, ticks | `vTaskDelay`, `pdMS_TO_TICKS` |
| time base | `esp_timer_get_time()` (µs since boot) |
| events | task notifications (`xTaskNotifyGive`, `vTaskNotifyGiveFromISR`, `ulTaskNotifyTake`) |
| critical sections | one `portMUX_TYPE` spinlock with `portENTER_CRITICAL` / `portEXIT_CRITICAL` |
| mutexes | `xSemaphoreCreateMutex` from a static pool |
| reboot | `esp_restart()` |

An **event** in the OSAL is not a separate object. It is the notification
counter of a task: the handle you post to is the task handle, obtained by the
task itself with `esp_foc_event_handle_self()`. Posts are counted, and each
wait consumes one post.

### Services

#### Tasks

```c
typedef void (*esp_foc_task_fn_t)(void *arg);
typedef void *esp_foc_task_handle_t;

int esp_foc_task_max_priority(void);
int esp_foc_task_spawn(esp_foc_task_fn_t fn, void *arg, const char *name,
                       size_t stack_bytes, int priority,
                       esp_foc_task_handle_t *out_handle_opt);
bool esp_foc_task_is_alive(esp_foc_task_handle_t handle);
void esp_foc_task_yield(void);
void esp_foc_task_delete_self(void);
```

- `esp_foc_task_max_priority()` returns `configMAX_PRIORITIES - 1`, the
  highest usable priority. The stacks place their supervisors relative to it.
- `esp_foc_task_spawn()` takes the stack size in **bytes** and a FreeRTOS
  priority, clamped to `0 .. max`. A `NULL` or empty name becomes `"espfoc"`.
  It returns 0 on success, -1 for a `NULL` function or a zero stack, -2 when
  the RTOS cannot create the task. `out_handle_opt` may be `NULL`.
- A task function must not return; end it with `esp_foc_task_delete_self()`.

#### Time

```c
void esp_foc_sleep_ms(uint32_t ms);
uint32_t esp_foc_ms_to_ticks(uint32_t ms);
uint64_t esp_foc_now_us(void);
```

- `esp_foc_sleep_ms(0)` returns at once. Any other value sleeps at least one
  tick, so with a 100 Hz tick `esp_foc_sleep_ms(1)` sleeps 10 ms.
- `esp_foc_ms_to_ticks()` rounds down; it returns 0 when the duration is
  shorter than one tick. Phase discovery uses `esp_foc_ms_to_ticks(1) == 0` to
  refuse a tick coarser than 1 ms.
- `esp_foc_now_us()` is the 64-bit microsecond time base. The stacks time
  their waits on it rather than on counted ticks, so a wait cannot stretch
  when the bridge stops ticking.

#### Context

```c
bool esp_foc_in_task_context(void);
bool esp_foc_in_isr_context(void);
void esp_foc_reboot(void);
```

- `esp_foc_in_task_context()` is true when the scheduler is running, there is
  a current task and the CPU is not in an interrupt. The stack `init()`
  functions and the I2C bus driver use it to refuse being called from the
  wrong place.
- `esp_foc_reboot()` resets the chip and does not return.

#### Critical sections

```c
void esp_foc_critical_enter(void);
void esp_foc_critical_leave(void);
```

One spinlock covers the whole component. The stacks use it so that a group of
values shared with the TEZ interrupt is read or written together, for example
the status snapshot or the d/q voltage feedforward pair. Keep the section to a
few loads and stores: no blocking calls, no logging, no float math you can do
outside.

#### Events

```c
typedef void *esp_foc_event_handle_t;

esp_foc_event_handle_t esp_foc_event_handle_self(void);
void esp_foc_event_wait(void);
bool esp_foc_event_wait_ms(uint32_t timeout_ms);
void esp_foc_event_clear(void);
void esp_foc_event_post(esp_foc_event_handle_t handle);
void esp_foc_event_post_from_isr(esp_foc_event_handle_t handle);
void esp_foc_event_post_auto(esp_foc_event_handle_t handle);
```

- `esp_foc_event_wait()` blocks until a post, forever.
- `esp_foc_event_wait_ms()` returns `true` when woken by a post and `false`
  on timeout. `UINT32_MAX` waits forever; a timeout shorter than one tick
  waits one tick.
- `esp_foc_event_clear()` drains pending posts without blocking.
- `esp_foc_event_post()` is for tasks, `esp_foc_event_post_from_isr()` for
  interrupts (it requests a context switch on exit when the woken task has a
  higher priority), and `esp_foc_event_post_auto()` picks one of the two from
  the current context. A `NULL` handle is ignored.

#### Mutexes

```c
typedef struct esp_foc_mutex esp_foc_mutex_t;

int esp_foc_mutex_create(esp_foc_mutex_t **out);
void esp_foc_mutex_destroy(esp_foc_mutex_t *mutex);
void esp_foc_mutex_lock(esp_foc_mutex_t *mutex);
void esp_foc_mutex_unlock(esp_foc_mutex_t *mutex);
```

- Mutexes come from a static pool of `CONFIG_ESP_FOC_MAX_MUTEXES` slots.
  `esp_foc_mutex_create()` returns 0, -1 for a `NULL` `out`, -2 when the RTOS
  object cannot be created, -3 when the pool is full.
- The RTOS mutex is created on the first use of a slot and kept; `destroy`
  only returns the slot to the pool.
- `esp_foc_mutex_lock()` waits forever. Lock and unlock ignore `NULL`.
- The stacks and drivers do not use mutexes; the pool is for your code.

### How to use it

**Pace an application loop.** Sleep between setpoint updates instead of
spinning, as every example does:

```c
/* Sleeps in short steps so a trip is noticed within RAMP_STEP_MS. */
static bool hold_for(uint32_t duration_ms)
{
    for (uint32_t elapsed_ms = 0; elapsed_ms < duration_ms; elapsed_ms += RAMP_STEP_MS) {
        if (controller_tripped) {
            return false;
        }
        esp_foc_sleep_ms(RAMP_STEP_MS);
    }
    return !controller_tripped;
}
```

**Wake a task from an interrupt.** The task publishes its handle, clears old
posts, starts the hardware, and waits with a timeout. The interrupt posts.
This is the pattern the I2C bus driver uses for each encoder transfer.
`start_transfer()` stands for whatever starts your peripheral:

```c
static volatile esp_foc_event_handle_t waiter;

static void my_isr(void *arg)
{
    (void)arg;
    esp_foc_event_post_from_isr(waiter);
}

static bool transfer_and_wait(void)
{
    waiter = esp_foc_event_handle_self();
    esp_foc_event_clear();
    start_transfer();
    return esp_foc_event_wait_ms(5);
}
```

**Create a task.** Keep application tasks below the supervisor of the stack
(see [How espFoC runs](02_execution_model.md)):

```c
static void report_task(void *arg)
{
    (void)arg;
    for (;;) {
        esp_foc_sensored_status_t st;
        esp_foc_sensored_get_status(0, &st);
        printf("iq %.3f A\n", (double)st.iq_a);
        esp_foc_sleep_ms(100);
    }
}

if (esp_foc_task_spawn(report_task, NULL, "report", 4096, 2, NULL) != 0) {
    printf("report task not created\n");
}
```

**Share a resource between tasks.**

```c
static esp_foc_mutex_t *log_lock;

if (esp_foc_mutex_create(&log_lock) != 0) {
    /* pool full: raise CONFIG_ESP_FOC_MAX_MUTEXES */
}
esp_foc_mutex_lock(log_lock);
/* ... */
esp_foc_mutex_unlock(log_lock);
```

### Which calls are safe from an interrupt

| Call | Interrupt | Task |
|---|---|---|
| `esp_foc_event_post_from_isr` | yes | no |
| `esp_foc_event_post_auto` | yes | yes |
| `esp_foc_in_isr_context`, `esp_foc_in_task_context` | yes | yes |
| `esp_foc_event_post` | no | yes |
| `esp_foc_event_wait*`, `esp_foc_event_clear`, `esp_foc_event_handle_self` | no | yes |
| `esp_foc_sleep_ms`, `esp_foc_task_*`, `esp_foc_mutex_*` | no | yes |

Driver interrupts that wake a task (encoder transfer done, stack fault
callback) go through `esp_foc_event_post_auto()` or
`esp_foc_event_post_from_isr()`.

### Configuration

| Symbol | Default | Meaning |
|---|---|---|
| `CONFIG_ESP_FOC_MAX_MUTEXES` | 2 | OSAL mutex pool size (range 1–8) |
| `CONFIG_FREERTOS_HZ` (IDF) | IDF default | tick rate; the examples set 1000, phase discovery requires a tick of 1 ms or shorter |

### Use cases

- **Portable application.** Write the application loop with
  `esp_foc_sleep_ms()`, `esp_foc_task_spawn()` and the events, as in
  [`foc_sensored_velocity`](../../examples/foc_sensored_velocity/README.md),
  and it needs no FreeRTOS includes.
- **Inverter-only test.** When you drive the inverter without a stack
  ([Inverter driver](03_inverter_driver.md)), the TEZ callback you install
  with `set_pwm_callback()` counts periods and posts a worker task with
  `esp_foc_event_post_from_isr()` every N periods; the task does the printing.
- **Coordinated tasks.** A console task and a logging task share the UART
  through an OSAL mutex.

### Limits and pitfalls

- **One event per task.** Every post to a task lands in the same counter.
  Code that clears the event before a wait (as the I2C driver does) also
  discards posts meant for something else on that task. Design waits so that
  a lost wake-up only delays work: the stack supervisors, for example,
  re-check their request flags at least every 20 ms.
- **Ticks round up.** Short sleeps and timeouts are rounded to whole ticks.
  Use a 1000 Hz tick.
- **Critical sections are global.** A long section delays every other user of
  the lock, including the stacks' setters and getters.
- **No lock timeout.** `esp_foc_mutex_lock()` waits forever; do not take a
  mutex in a path that must keep running, such as an event callback.

## Part 2 — The hot-path trace

### What it does

The control interrupts run 20000 times per second; a `printf` there would
break the timing. The trace instead stores small binary records in a ring
buffer in RAM, in constant time, from interrupt or task context. You read the
ring later, from a task or a debugger. It answers questions like "how long
does the TEZ take?" and "what happened in the microseconds before the bridge
tripped?".

The API is in
[`esp_foc_trace.h`](../../include/espFoC/debug/esp_foc_trace.h).

### How it works

Each record is 12 bytes:

```c
typedef struct {
    uint16_t magic;     /* 0xE5F0 */
    uint16_t type;      /* esp_foc_trace_type_t */
    uint32_t seq;
    int32_t  a;
    int32_t  b;
} esp_foc_trace_rec_t;
```

`seq` increases by one per record, so gaps show missing records. `a` and `b`
depend on the type. The ring holds `CONFIG_ESP_FOC_TRACE_DEPTH` records (a
power of two, checked at build time); when it is full the oldest record is
overwritten. `esp_foc_trace_push()` takes no lock and does no allocation.

The MCPWM inverter pushes these records:

| Type | Pushed when | `a` | `b` |
|---|---|---|---|
| `ESP_FOC_TRACE_TEZ_ENTER` | TEZ interrupt entry | 0 | 0 |
| `ESP_FOC_TRACE_TEZ_EXIT` | TEZ interrupt exit | duration in µs | duration in CPU cycles |
| `ESP_FOC_TRACE_DUTY` | every accepted `set_duties()` | duty U (Q16) | duty V (Q16) |
| `ESP_FOC_TRACE_DMA_EOF` | end of the DMA interrupt | duration in µs | duration in CPU cycles |
| `ESP_FOC_TRACE_FAULT_ILIMIT_HIT` | current limit or frozen sense detected | `ESP_FOC_FAULT_ILIMIT` or `ESP_FOC_FAULT_SENSE_STALE` | peak current (Q16 A) or stale frame count |
| `ESP_FOC_TRACE_FAULT_GPIO_IRQ` | fault pin interrupt | `ESP_FOC_FAULT_GPIO` | interrupt status |
| `ESP_FOC_TRACE_FAULT_SOFT_REQ` | `soft_trip()` | `ESP_FOC_FAULT_SOFT_TRIP` | 0 |
| `ESP_FOC_TRACE_FAULT_TRIP` | trip latched | reason | 1 |
| `ESP_FOC_TRACE_FAULT_TRIP_IGN` | trip while already faulted | new reason | latched reason |
| `ESP_FOC_TRACE_FAULT_OST` | PWM one-shot brake applied | reason | 1 if the hardware braked |
| `ESP_FOC_TRACE_FAULT_EN_OFF` | bridge enable pin released | reason | enable GPIO |
| `ESP_FOC_TRACE_FAULT_CB` | before the fault callback | reason | 0 |
| `ESP_FOC_TRACE_FAULT_DUTY_IGN` | first duty write refused while faulted | reason | 0 |
| `ESP_FOC_TRACE_FAULT_CLEAR_REQ` / `_OK` / `_REJ` | `clear_fault()` asked / done / refused | previous reason | 0, or `ESP_ERR_INVALID_STATE` |
| `ESP_FOC_TRACE_FAULT_ENABLE_REJ` | `enable()` refused while faulted | reason | 0 |

The TEZ duration covers the whole interrupt, including the stack's callback
and the `TEZ_ENTER` push. The DMA duration covers the ADC parse, the inverter
callback and the rearm. `ESP_FOC_TRACE_USER_MARK` is not pushed by the
component; it is free for your own marks.

When a trip latches, the inverter stops the TEZ and DMA interrupts, so the
fault sequence stays at the end of the ring instead of being overwritten by
new periods.

### How to use it

1. **Enable it.** `CONFIG_ESP_FOC_TRACE_ENABLE` is `y` by default in Kconfig,
   but the examples turn it off in `sdkconfig.defaults`
   (`CONFIG_ESP_FOC_TRACE_ENABLE=n`). Set it back to `y` in your project and
   choose `CONFIG_ESP_FOC_TRACE_DEPTH`. With the option off, every trace call
   is an empty stub: `snapshot` returns 0 and the counters read 0.
2. **Initialisation is automatic.** `esp_foc_inverter_mcpwm_init()` calls
   `esp_foc_trace_init()`, which clears the ring. Call
   `esp_foc_trace_reset()` to empty it before a test.
3. **Read it from a task** with `esp_foc_trace_snapshot()`. It copies the
   newest records, oldest first, up to `max_out`, and returns how many it
   copied. Use a static buffer: 256 records are 3 KiB, too much for a small
   task stack.

```c
#include <stdio.h>
#include "espFoC/debug/esp_foc_trace.h"

static esp_foc_trace_rec_t trace_copy[128];

static void print_tez_budget(void)
{
    const size_t n = esp_foc_trace_snapshot(trace_copy, 128);
    int32_t worst_us = 0;
    int32_t worst_cycles = 0;
    for (size_t i = 0; i < n; i++) {
        const esp_foc_trace_rec_t *r = &trace_copy[i];
        if ((r->type == ESP_FOC_TRACE_TEZ_EXIT) && (r->b > worst_cycles)) {
            worst_us = r->a;
            worst_cycles = r->b;
        }
    }
    printf("TEZ worst %d us (%d cycles) over %u records, %u overwritten\n",
           (int)worst_us, (int)worst_cycles, (unsigned)n,
           (unsigned)esp_foc_trace_dropped());
}
```

4. **Mark your own events** to line them up with the hot path:

```c
esp_foc_trace_push(ESP_FOC_TRACE_USER_MARK, 1, (int32_t)step);
```

`esp_foc_trace_count()` returns how many records the ring holds (at most the
depth). `esp_foc_trace_dropped()` counts records overwritten since the last
init or reset; in steady state it grows all the time, because the ring wraps
every few milliseconds.

With a JTAG debugger attached you can also read the ring directly: it is the
static array `s_ring` in `source/debug/esp_foc_trace.c`.

### Configuration

| Symbol | Default | Meaning |
|---|---|---|
| `CONFIG_ESP_FOC_TRACE_ENABLE` | y | build the trace ring; off makes every call a stub |
| `CONFIG_ESP_FOC_TRACE_DEPTH` | 256 | records in the ring, power of two (range 16–4096) |

### Use cases

- **Loop time budget.** Run the motor, take a snapshot, and compare the worst
  `TEZ_EXIT` duration with the PWM period (50 µs at 20 kHz). Add the
  `DMA_EOF` duration: both interrupts share the same core.
- **Fault post-mortem.** After the application sees a `..._EV_FAULT` event,
  read the ring. The tail shows the trigger and the shutdown order, for
  example `FAULT_ILIMIT_HIT` with the peak current, then `FAULT_TRIP`,
  `FAULT_OST`, `FAULT_EN_OFF` and `FAULT_CB`. The `DUTY` records before it
  show what the loop was commanding.
- **Aligning application steps.** Push `USER_MARK` records at setpoint
  changes and look for the `DUTY` response after them.

### Limits and pitfalls

- **Short history.** The inverter pushes several records per PWM period, so
  a 256-record ring covers only the last few milliseconds while the bridge
  runs. Take the snapshot right after the event of interest, or stop the
  bridge first.
- **The snapshot does not stop the writers.** While the interrupts run, the
  ring can move during the copy. Check `seq` for gaps, or read after a trip,
  when the interrupts are stopped.
- **A second inverter clears the ring.** Each `esp_foc_inverter_mcpwm_init()`
  calls `esp_foc_trace_init()`.
- **Cost.** Each push is a few stores inside the interrupt. Measure with the
  trace on, then decide whether to ship with it off, as the examples do.

## Other debug option: ADC software trigger

`CONFIG_ESP_FOC_ADC_DIGI_SW_TRIGGER` (default `n`) is described in Kconfig as
a debug option for DMA bring-up: the ADC digital controller's own timer
starts conversions instead of relying only on the ETM start from the PWM. The
current sources do not read this symbol, so setting it has no effect in this
version; leave it at `n`.
