# Doosan getter/feedback instrumentation implementation

Date: 2026-10-02

## Scope and safety

Only diagnostic instrumentation source was added. No build, process attach,
ROS command, getter/service request, controller operation, driver restart, or
robot operation was performed.

The editable trace code is disabled unless `DSR_TRACE_ENABLE=1`. It records a
monotonic timestamp, realtime timestamp, Linux thread ID, sequence number,
event, function, and duration. Output uses a direct `write(2)` syscall so that
the diagnostic path does not add a C stdio mutex dependency.

## Instrumented paths

| Requested point | Implementation | Status |
|---|---|---|
| `get_current_posx_cb` | Source `Scope` plus request/result records | Implemented |
| `CDRFLEx::_get_current_posx` | Header wrapper plus actual `_get_current_posx` entry/return uprobe | Implemented |
| `SendCurrentTaskPoseCommand` / 0x0472 send | Function entry/return uprobe | Implemented |
| 0x0472 response dispatch | Handler offsets for dispatch start, Event set, dispatch end | Implemented, binary-version pinned |
| `CNDKHandler::run` | Entry/return and queue mutex uprobes | Implemented |
| `CNDKHandlerUDP::run` | Entry/return and queue mutex uprobes | Implemented |
| `DRHWInterface::read` | Source entry/exit and RT read boundary records | Implemented |
| `read_data_rt` / `_read_data_rt` | Wrapper records plus actual function entry/return uprobe | Implemented |
| JointState publish 직전 | `JointStateBroadcaster::update()+0x820` uprobe | Implemented, binary-version pinned |
| TCP/UDP receive boundary | `CNDKClientSocket[UDP]::OnReceive` uprobes | Added to distinguish receive from dispatch |

Mutex probes emit exactly `LOCK_WAIT_START`, `LOCK_ACQUIRED`, and
`LOCK_RELEASE` for the 0x0472 response-state mutex and TCP/UDP handler queue
mutexes.

## A/B/C discrimination

- A candidate: 0x0472 send is observed, but no corresponding TCP receive and
  no `RESPONSE_0472_DISPATCH_START` appears.
- B candidate: TCP receive activity appears after the send, but the 0x0472
  dispatch markers do not complete. A generic TCP receive alone cannot prove
  that the received packet itself was 0x0472; packet-level capture or an
  additional decoded command-ID hook would be needed for that final proof.
- C candidate: compare UDP receive, `_read_data_rt`, `DRHWInterface::read`, and
  `JOINT_STATE_PRE_PUBLISH` timelines while the getter path is blocked. Their
  independent stopping point identifies RT receive, ros2_control read, or
  broadcaster-side stall.

## Validation performed

- `git diff --check`: PASS.
- Standalone C++17 syntax check of `drfl_instrumentation.hpp`: PASS.
- Full workspace build: NOT EXECUTED.
- bpftrace parse/attach: NOT EXECUTED; `bpftrace` is not installed in the
  current environment.
- Runtime reproduction: NOT EXECUTED.

The uprobe offsets are valid only for the documented installed binaries. They
must be revalidated after any rebuild or library replacement.

## Execution counters

- Robot command: 0
- Getter/service request: 0
- Motion service/action: 0
- Gripper/Home/trajectory: 0
- Hold/E-stop: 0
- Controller/driver restart: 0
- Instrumentation runtime attach: 0

