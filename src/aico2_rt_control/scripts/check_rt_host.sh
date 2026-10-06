#!/usr/bin/env bash
# Is this host capable of the 1 kHz RT loop Flexiv requires?
#
# Flexiv's guidance (2026-10-05) lists three RT prerequisites: C++, an RDK
# Professional licence, and "a real-time capable Linux PC ... with a wired
# connection to the robot". The licence is confirmed on both arms. This script
# answers the host half, which is the open risk: the Jetson also carries Nav2,
# RTAB-Map and two camera pipelines, and in RT a missed deadline is not covered
# by any motion generator.
#
# Read-only. Touches no robot and needs no RDK. Run it on the Jetson:
#   bash src/aico2_rt_control/scripts/check_rt_host.sh

set -uo pipefail
ROBOTS=("192.168.1.100" "192.168.1.101")

hdr() { printf '\n=== %s ===\n' "$1"; }
say() { printf '%-22s %s\n' "$1" "$2"; }

hdr "Host"
say "hostname" "$(hostname)"
model="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null)"
[[ -z "$model" && -r /proc/device-tree/model ]] \
    && model="$(tr -d '\0' < /proc/device-tree/model)"
say "model" "${model:-unknown}"
say "cpu" "$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | sed 's/^ *//' \
    || echo unknown)"
say "arch" "$(uname -m)"

hdr "Kernel"
say "uname -r" "$(uname -r)"
say "uname -v" "$(uname -v)"

# /sys/kernel/realtime exists and reads 1 only on a PREEMPT_RT kernel. This is
# the single most load-bearing line in the whole script.
if [[ -r /sys/kernel/realtime ]]; then
    say "/sys/kernel/realtime" "$(cat /sys/kernel/realtime)  <-- PREEMPT_RT present"
else
    say "/sys/kernel/realtime" "absent  <-- NOT a PREEMPT_RT kernel"
fi

preempt="$(uname -v | grep -o 'PREEMPT[_A-Z]*' | head -1)"
say "preempt flavour" "${preempt:-none advertised}"
RT_OK=0
case "$preempt" in
    PREEMPT_RT) say "verdict" "full RT preemption"; RT_OK=1 ;;
    PREEMPT_DYNAMIC)
        say "verdict" "DYNAMIC: runtime-selectable, check preempt= below"
        [[ -r /sys/kernel/debug/sched/preempt ]] \
            && say "  sched/preempt" "$(cat /sys/kernel/debug/sched/preempt 2>/dev/null)"
        ;;
    PREEMPT) say "verdict" "low-latency, not PREEMPT_RT" ;;
    *)       say "verdict" "no preemption advertised" ;;
esac

for cfg in /boot/config-"$(uname -r)" /proc/config.gz; do
    [[ -r "$cfg" ]] || continue
    reader=cat; [[ "$cfg" == *.gz ]] && reader=zcat
    line="$($reader "$cfg" 2>/dev/null | grep -E '^CONFIG_PREEMPT(_RT|_NONE|_VOLUNTARY|_DYNAMIC)?=' | tr '\n' ' ')"
    [[ -n "$line" ]] && say "$(basename "$cfg")" "$line"
done

hdr "CPU and isolation"
say "cores" "$(nproc)"
say "/proc/cmdline" "$(tr ' ' '\n' < /proc/cmdline | grep -E 'isolcpus|nohz|rcu_nocbs|threadirqs|preempt=' | tr '\n' ' ')"
[[ -z "$(tr ' ' '\n' < /proc/cmdline | grep -E 'isolcpus')" ]] \
    && say "isolcpus" "not set -- the RT thread shares every core with ROS 2"
govs="$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | sort -u | tr '\n' ' ')"
say "cpufreq governor" "${govs:-unavailable}"
[[ "$govs" == *powersave* || "$govs" == *schedutil* ]] \
    && say "  note" "a scaling governor adds wake-up latency; 'performance' is the RT choice"

if command -v nvpmodel >/dev/null 2>&1; then
    say "nvpmodel" "$(nvpmodel -q 2>/dev/null | tr '\n' ' ')"
    say "  note" "MAXN plus jetson_clocks removes DVFS latency"
fi

hdr "Real-time scheduling permission"
say "ulimit -r (rtprio)" "$(ulimit -r 2>/dev/null || echo 'unavailable')"
say "running as" "$(id -un) (uid $(id -u))"
[[ "$(id -u)" -ne 0 ]] && say "  note" "non-root needs an rtprio limit in /etc/security/limits.conf for SCHED_FIFO"
say "sched_rt_runtime_us" "$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || echo unavailable)"
say "  note" "950000 of 1000000 is the default RT throttle; -1 disables it"

hdr "Link to the arms (Flexiv requires wired)"
for ip in "${ROBOTS[@]}"; do
    dev="$(ip -o route get "$ip" 2>/dev/null | grep -o 'dev [^ ]*' | awk '{print $2}')"
    if [[ -z "$dev" ]]; then
        say "$ip" "no route"
        continue
    fi
    kind="wired"
    [[ -d "/sys/class/net/$dev/wireless" || "$dev" == wl* ]] && kind="WIRELESS -- not supported for RT"
    speed="$(cat "/sys/class/net/$dev/speed" 2>/dev/null)"
    say "$ip" "via $dev ($kind${speed:+, ${speed}Mb/s})"
    if command -v ping >/dev/null 2>&1; then
        rtt="$(ping -c 20 -i 0.01 -q "$ip" 2>/dev/null | tail -1)"
        say "  ping x20" "${rtt:-failed}"
    fi
done

hdr "Load that will compete with the loop"
say "loadavg" "$(cut -d' ' -f1-3 /proc/loadavg)"
say "ros2 processes" "$(pgrep -c -f 'ros2|rclpy|component_container' 2>/dev/null | head -1)"
printf '%-22s\n' "top cpu consumers"
ps -eo pcpu,pid,comm --sort=-pcpu 2>/dev/null | head -6 | sed 's/^/  /'
printf '%-22s %s\n' "existing RT threads" \
    "$(ps -eo pid,cls,rtprio,comm 2>/dev/null | awk '$2=="FF"||$2=="RR"' | wc -l) (FIFO/RR)"
ps -eo pid,cls,rtprio,comm 2>/dev/null | awk '$2=="FF"||$2=="RR"' | head -5 | sed 's/^/  /' 

hdr "Reading this"
if [[ "$RT_OK" -eq 1 ]]; then
    cat <<'TXT'
PREEMPT_RT is present, so Flexiv's host requirement is met. That removes the
main risk, but it does not make the loop free: measure it. Install the C++ RDK
and run rt_hold_probe twice, once with the stack stopped and once with Nav2 and
the cameras running. The gap between those two numbers is what matters.

If misses appear only under load, pin the RT process to a core the rest of the
stack does not use (taskset, or isolcpus on the kernel command line). With
PREEMPT_RT plus a performance governor that is often unnecessary, which is why
it is worth measuring before changing the boot configuration.
TXT
else
    cat <<'TXT'
PREEMPT_RT is absent. That is not a hard stop, but it means the 1 kHz loop is
best-effort and Flexiv's stated requirement is unmet. Measure it rather than
assume: install the C++ RDK and run rt_hold_probe twice, once with the stack
stopped and once with Nav2 and the cameras running.

If the misses are load-dependent rather than kernel-dependent, isolating a core
(isolcpus + taskset) may be enough without changing kernels. If they persist on
an idle host, the arms want either an RT kernel or a different machine.
TXT
fi
