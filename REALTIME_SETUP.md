# Real-Time Setup (NUC host)

SOBIT HOME runs on a NUC with a `PREEMPT_RT` kernel and cores 2-7 isolated for control
loops (`isolcpus=domain,managed_irq,2,3,4,5,6,7 nohz_full=2,3,4,5,6,7 rcu_nocbs=2,3,4,5,6,7`
in the boot cmdline). That isolation is only useful if the rest of the stack — USB
interrupts, CPU frequency scaling, DDS socket buffers, and the actual control-loop
processes — cooperates with it. This document covers what's already in place, what
was found to undercut it, and how the fixes are applied.

This is **host-level** setup, done once per NUC, outside the Docker container.


## Already in place

- Kernel: `PREEMPT_RT` (`uname -r` ends in `-realtime`; `/sys/kernel/realtime` reads `1`).
- GRUB: cores 2-7 isolated and kept off the scheduler/IRQ/RCU housekeeping paths; cores
  0-1 remain for the OS, DDS discovery, and best-effort work.
- `irqbalance` disabled (correct pairing with manual IRQ pinning below).
- `docker/docker-compose.yml`: `privileged: true`, `cap_add: SYS_NICE`, and
  `ulimits: rtprio=98, rttime=-1, memlock=unlimited` — this is what lets
  `ros2_control`/`realtime_tools` auto-promote control threads to `SCHED_FIFO` inside
  the container, and what `chrt -f 80` below relies on.
- `/dev/bus/usb`, `/dev/serial/by-id`, `/dev/v4l/by-id`, `/dev/cpu_dma_latency` are all
  passed through to the container.
- FTDI serial adapters (the Dynamixel buses) already ship with `latency_timer=1` via
  `/etc/udev/rules.d/99-usb-latency.rules` on the host. Confirmed this generalizes to
  new adapters too: the `BestTechnology E160` hand-bus dongle added later enumerates
  under the same `ftdi_sio` driver and already reads `latency_timer=1` - no new
  host-side rule needed for it.
- `rmw_cyclonedds_cpp` is the selected RMW, and `mode_ctr.sh` keeps DDS discovery on
  loopback by default (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), switching to
  `SUBNET` only when talking to the real robot (`ROS_DOMAIN_ID=80`).


## Gaps found and fixed

### 1. Unmanaged device IRQs landing on isolated cores

`isolcpus=...,managed_irq,...` only auto-excludes *managed* IRQs (e.g. NVMe's
per-CPU queues) from the isolated cores. Legacy/single-vector IRQs, and drivers that
request one queue per CPU regardless of isolation, aren't covered. Found pinned to
isolated cores: `xhci_hcd` (both controllers — bus 003 carries every Dynamixel/FTDI
serial adapter, the CAN adapter, and both wrist cameras), the `igc` Ethernet NIC,
every `iwlwifi` WiFi queue, and the `i915` iGPU.

**Fix:** `sobit-irq-affinity.service` sweeps every IRQ in `/proc/interrupts` and
forces it onto the housekeeping cores (0-1). Genuinely managed IRQs (NVMe) reject the
write with `EIO` and are left alone automatically — no driver allowlist needed.

One boot-ordering gotcha this caught: `igc` doesn't request its per-queue IRQ vectors
until the interface is brought up (`ndo_open`), which NetworkManager does lazily
during its own activation — on a fresh boot, the sweep ran at `sysinit.target` before
those IRQs existed yet, so `enp86s0` silently kept the unrestricted default. Fixed by
also ordering `After=network-online.target` (waits for NetworkManager to report
devices as actually activated, not just `network.target`'s pass-through signal).

Verify (after a real reboot, not just `systemctl restart` on an already-up system —
that's what hid the ordering bug the first time):
```sh
for name in enp86s0 iwlwifi i915 xhci_hcd; do
    awk -F: "/$name/ {print \$1}" /proc/interrupts | while read -r irq; do
        echo "IRQ ${irq// /} ($name) -> $(cat /proc/irq/${irq// /}/smp_affinity_list)"
    done
done
```

**Tested and ruled out:** the bus-003 sharing noted above (Dynamixel/FTDI adapters +
CAN + wrist cameras) looks like an obvious suspect for the intermittent "Incorrect
status packet" errors seen on the hand Dynamixel bus. It isn't — disabling the wrist
cameras entirely made no measurable difference to the error rate or tail latency
(same ~90-120 errors/~200s either way). That bus's problem turned out to be
device-count-per-transaction on the hand chain itself, fixed by giving it its own
dongle (see `controllers.urdf.xacro`), not an IRQ/bus-sharing issue.

### 2. EPP biased away from low-latency ramp-up

The active power profile was `balanced`. Under a *sustained* synthetic load,
ground-truth `perf stat` cycle counts show cores 2-7 reach full clock either way —
HWP's autonomous P-state selection isn't actually gated on OS ticks, only its
`scaling_cur_freq` *reporting* is (see the verify note below, this took a live test to
tell apart). What the `balanced` EPP hint does bias is how eagerly the hardware ramps
for short, bursty, low-duty-cycle wakeups — exactly the shape of a periodic control
loop sitting idle between cycles. The `performance` EPP hint removes that bias.

**Fix:** `sobit-cpu-performance.service` runs `powerprofilesctl set performance` after
`power-profiles-daemon` starts (its profile choice does **not** persist across
reboots on its own, hence the service). This applies system-wide rather than fighting
`power-profiles-daemon` with a raw `cpupower` call on a subset of cores.

Verify: `powerprofilesctl get` should print `performance`. Don't verify with
`scaling_cur_freq` on cores 2-7 — its refresh is tick-driven and gets starved by
`nohz_full`, so it can read stuck at the 400MHz floor even when the core is genuinely
running at full speed. Ground truth is a hardware counter, e.g.:
```sh
taskset -c 3 bash -c 'x=0; while true; do x=$((x+1)); done' &
sleep 0.5 && sudo perf stat -C 3 -- sleep 1
kill %1
```
(confirmed 3.48GHz actual on cpu3 under load, governor label notwithstanding).

Revert: `powerprofilesctl set balanced` (or disable the service).

### 3. `/dev/cpu_dma_latency` mounted but unused

The device was already passed into the container, but nothing opened it — so nothing
was stopping the CPUs from entering deep C-states, which adds wake-up latency.

**Fix:** `sobit-cpu-dma-latency.service` holds the device open at 0us for as long as
the machine is up.

Verify: `systemctl is-active sobit-cpu-dma-latency.service` should print `active`.

### 4. No CPU pinning for the actual control-loop processes

Kernel isolation only helps if the RT-critical processes are actually placed on those
cores. Nothing in `docker-compose.yml` or the launch files enforced this — the
container's cpuset was the full `0-15`, so the control loop could just as easily land
on a housekeeping core (or share a core with perception/camera work) as on an isolated
one.

**Fix:** in [`robot.launch.py`](sobit_home_bringup/launch/robot.launch.py), the
`controller_manager` node (`ros2_control_node`, the actual hardware I/O loop) now
launches with `prefix='taskset -c 2-7 chrt -f 80'`, and `swerve_controller_node`
(which runs its own periodic `control_timer_`) launches with `prefix='taskset -c
2-7'`. Same for the standalone
[`swerve_controller.launch.py`](sobit_home_control/launch/swerve_controller.launch.py).
Container-wide `cpuset_cpus` was deliberately *not* used in `docker-compose.yml`,
since that would also confine perception/DDS work meant to stay on the housekeeping
cores.

### 5. DDS UDP buffers at the Ubuntu default

`net.core.rmem_max`/`wmem_max` were at Ubuntu's default (~208KB), well under what
CycloneDDS' own tuning guide recommends. Too small a buffer causes silent "sample
lost" drops under load from the head camera (Orbbec Gemini 336L) or merged lidar
scans, even with discovery kept on loopback/subnet.

**Fix:** `60-sobit-dds.conf` raises `net.core.{rmem,wmem}_{max,default}` to 16MB.

Verify: `sysctl net.core.rmem_max net.core.wmem_max` should print `16777216`.

### uirobot gateway (CP2102N): no working latency tuning exists — not fixed

An existing host-level rule (`/etc/udev/rules.d/99-usb-latency.rules`) tries
`ATTR{latency_timer}="1"` on the uirobot gateway's Silicon Labs CP2102N (`UM_PORT` in
`install.sh`). That's dead: `latency_timer` is an `ftdi_sio`-only sysfs attribute,
`cp210x` never creates it, so the rule silently no-ops.

The obvious next thing to try, `low_latency` (`ASYNC_LOW_LATENCY`) via `setserial`,
is *also* a dead end — confirmed with a raw `TIOCSSERIAL` ioctl test, bypassing
`setserial` entirely: the call returns success, but the flag reads back `0` even
within the same file descriptor, immediately after. The generic `usb-serial` core
implements a compatibility shim for `TIOCGSERIAL`/`TIOCSSERIAL` that reports success
without actually wiring the `flags` field to anything, for drivers that don't
implement it themselves — and `cp210x` doesn't. `ftdi_sio` is the one driver in this
robot's USB tree that actually does something with it.

There is currently no known kernel-level latency tuning knob for `cp210x` on Linux.
Left alone. Before spending more time on this: measure actual round-trip latency on
that channel first — CP210x chips don't share FTDI's known-bad 16ms default buffering
interval, so the `UM_PORT` may already be fine without any tuning at all.


## Applying the host setup

One-time, per NUC, with sudo:

```sh
cd ~/colcon_ws/src/sobit_home/realtime_setup
sudo bash setup.sh
```

This installs and enables:

| Unit | Type | What it does |
| --- | --- | --- |
| `sobit-irq-affinity.service` | oneshot | Pins `xhci_hcd` IRQs to cores 0-1 |
| `sobit-cpu-performance.service` | oneshot | Sets the `performance` power profile |
| `sobit-cpu-dma-latency.service` | long-running | Holds `/dev/cpu_dma_latency` at 0 |
| `/etc/sysctl.d/60-sobit-dds.conf` | sysctl | Raises DDS UDP buffer limits |

The script is idempotent — safe to re-run after a kernel/GRUB change or if a unit gets
edited.


## Not changed

- **Swap** (8GB, `swappiness=60`) was left as-is — low risk today with >20GB free RAM,
  and any `mlockall`'d RT thread memory is already protected by the container's
  `memlock=unlimited` ulimit. Worth revisiting (`vm.swappiness`) if memory pressure
  becomes an issue.
- **`docker-compose.yml`** cpuset was left unrestricted on purpose — see gap #4 above.

