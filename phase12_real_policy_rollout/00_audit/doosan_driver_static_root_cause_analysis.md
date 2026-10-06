# Doosan real driver static root-cause analysis

Date: 2026-10-02 (Asia/Seoul)  
Scope: source, static library, disassembly, and existing-log analysis only  
Safety: no ROS graph query, getter/service call, controller transition, driver restart, or robot command was performed.

## Executive finding

The strongest proven defect is an unbounded synchronous wait in the proprietary DRFL request path used by `get_current_posx`. The ROS service callback calls `CDRFLEx::get_current_posx()`, which sends normal-channel command `0x0472`, then waits in 10 ms increments until a response handler clears an internal flag. There is no total timeout or cancellation condition in that loop. The observed 3.000606 s is the Phase 12 ROS client timeout, not a DRFL timeout; after the client abandons the request, the server callback can remain blocked indefinitely.

The two approximately 100% CPU threads are DRFL message-dispatch threads: the normal TCP `CNDKHandler::run()` and RT UDP `CNDKHandlerUDP::run()`. Their loops are queue/semaphore driven, not simple unconditional `while {}` loops: both call `Poco::SemaphoreImpl::waitImpl()` or its timed form before dequeuing and dispatching packets. The associated socket threads poll sockets with a one-second `Poco::Timespan`. Therefore static code does not prove idle busy-waiting. Sustained 100% CPU inside both handlers is instead consistent with continuously-ready queues, excessive packet delivery/reprocessing, or a timeout/semaphore configuration that makes the wait continuously return. Runtime evidence is required to select among these.

The normal getter and RT JointState path share the same top-level global `CDRFLEx Drfl`, but use distinct internal handles: `_rbtCtrl` for the normal TCP request/monitoring channel and `_rbtCtrlUDP` for RT UDP. Static analysis proves shared wrapper lifetime and process, but does not prove that the two handles share a socket or a specific internal mutex. Thus `GET_CURRENT_POSX_TIMEOUT_CORRELATED_WITH_FEEDBACK_LOSS` remains the correct causal classification.

## Audited artifacts

- Repository HEAD: `86eaa9632d651eb907332334d02f32c1461850d7`
- The repository was dirty before this audit; no pre-existing change was reset or modified.
- `libDRFL.a` SHA-256: `5744d74c8f7455990a1326e0fa425a01d9911031e090e4c0ddd2fa6809ce61a6`
- `dsr_hw_interface2.cpp` SHA-256: `9cc833839fdd9c209b2f4719b02ad14653c326812864dcc6553a8b7d1474af35`
- `dsr_controller2.cpp` SHA-256: `327df9ac6b296a8dd497037bf6ca89759d85714ec74479040aa598fc77fd3cf0`
- `DRFLEx.h` SHA-256: `12d7bc3bc94e7dc57005afae9c52aca2fe224d1507947eeb17aa3ef3a1def131`
- Current driver log inspected: `/home/ubuntu/.ros/log/ros2_control_node_2873331_1790902417863.log`

The implementations of the two handler `run()` methods are not present as C++ source in the workspace. They were recovered to assembly from these members of `libDRFL.a`:

- `NDKHandler.cpp.o`
- `NDKHandlerUDP.cpp.o`
- Supporting socket objects: `NDKSocket.cpp.o`, `UDPSocket.cpp.o`
- Public API bridge: `DRFLEx.cpp.o`

## 1. Exact roles of the two high-CPU threads

### `DRAFramework::CNDKHandler::run()`

Normal DRFL/TCP response and monitoring-message dispatcher. It:

1. waits on a `Poco::SemaphoreImpl` (indefinite or configured timed wait),
2. dequeues a packet from a mutex-protected ring buffer,
3. switches on its command/message ID,
4. copies response or monitoring payloads into internal buffers,
5. clears per-request wait flags and signals `Poco::EventImpl`, or invokes registered monitoring callbacks,
6. returns to the top of the loop while its running flag remains true.

Response command `0x0472` is handled in this loop: it copies the current task pose into the shared response buffer, clears the wait flag at the matching internal offset, and signals the matching Event.

### `DRAFramework::CNDKHandlerUDP::run()`

RT UDP packet dispatcher. Its structure mirrors the normal handler: semaphore wait, mutex-protected dequeue, command dispatch, internal RT output-buffer update, and optional callback. It owns the response buffer returned by `SendReadDataRTCommand()`.

### Socket receive threads

The handlers do not directly call `recv`. Separate socket worker loops do so:

- `CNDKSocket::run()` polls a TCP socket through a virtual socket poll/readiness function using a one-second `Poco::Timespan`, then calls the virtual receive handler.
- `CNDKSocketUDP::run()` does the same for UDP, with connection/start-state handling.
- Underlying socket objects import `StreamSocket::receiveBytes`, `DatagramSocket::receiveBytes`, and `DatagramSocket::receiveFrom`.

## 2. Busy-loop assessment

| Item | Static result |
|---|---|
| Outer loop | Present in both handler and socket workers |
| Sleep/usleep | No sleep in the handler dispatch loops; supporting socket code imports `Poco::ThreadImpl::sleepImpl` in other paths |
| Condition/semaphore | Both handler loops use `Poco::SemaphoreImpl::waitImpl()` / `waitImpl(long)` |
| Socket receive | Performed by separate TCP/UDP socket workers |
| Socket polling timeout | Socket run loops construct a one-second `Poco::Timespan` |
| Mutex | Ring-buffer dequeue and payload updates use pthread/Poco mutexes |
| Proven idle busy-spin | No |
| Busy execution under continuously-signaled queue | Yes, possible and consistent with perf |

The handler logic can run continuously when its semaphore is continuously posted or its configured timed wait immediately expires. Static disassembly does not reveal the runtime value of the handler timeout member. Consequently the observed two-core saturation is strong evidence of continuously active DRFL communication dispatch, but not by itself proof of a missing wait instruction.

## 3. Complete `get_current_posx` call path

```text
/dsr01/aux_control/get_current_posx
  -> RobotController::get_current_posx_cb
     dsr_controller2.cpp:983-1003
  -> shared Drfl->get_current_posx(ref)
     DRFLEx.h:957
  -> _get_current_posx(_rbtCtrl, ref)
     DRFLEx.cpp.o
  -> CNDKHandler::SendCurrentTaskPoseCommand(payload, 1 or 4)
     command ID 0x0472
  -> SplitPacket(0x0472, ...)
  -> CNDKClient::SendMessageToServer(...)
  -> internal response flag remains set
  -> loop: EventImpl::waitImpl(10 ms), re-check flag, repeat without total deadline
  -> CNDKHandler::run() receives response 0x0472
     copy pose -> clear flag -> EventImpl::setImpl()
  -> return internal LPROBOT_TASK_POSE pointer
  -> populate ROS response
```

If the response is never dispatched, the callback does not return. It also does not return `nullptr`; the `nullptr` check in the callback is unreachable until the SDK call itself returns.

## 4. Complete JointState generation/update path

```text
robot RT channel (default port 12347)
  -> CNDKSocketUDP receive worker
  -> CNDKHandlerUDP::run()
  -> RT output buffer at CNDKHandlerUDP internal storage
  -> ros2_control update loop
  -> DRHWInterface::read()
     dsr_hw_interface2.cpp:350-357
  -> Drfl.read_data_rt()
  -> _read_data_rt(_rbtCtrlUDP)
  -> CNDKHandlerUDP::SendReadDataRTCommand()
     (returns pointer to latest buffered RT data; it sends no request in this build)
  -> convert actual_joint_position/velocity degrees to radians
  -> exported state interfaces
  -> joint_state_broadcaster
  -> /dsr01/joint_states
```

`SendReadDataRTCommand()` is only an address calculation/return in this library. Therefore `DRHWInterface::read()` should not wait for a network round-trip. However, it dereferences the returned data pointer without a null/freshness check.

## 5. Shared objects, mutexes, and sockets

Proven:

- `dsr_hw_interface2.cpp:31` defines one global `CDRFLEx Drfl`.
- `get_drfl()` returns its address.
- `dsr_controller2.cpp:30-31` stores that exact address as `CDRFLEx *Drfl`.
- Consequently hardware read and all controller callbacks share the same wrapper object and process.
- The wrapper constructs two handles: `_rbtCtrl = _CreateRobotControl()` and `_rbtCtrlUDP = _create_robot_control_udp()`.
- Normal getter/monitoring uses `_rbtCtrl`; RT read uses `_rbtCtrlUDP`.
- Normal and UDP handler objects each contain their own visible ring-buffer mutexes, semaphores, flags, and response buffers.

Not proven:

- that `_rbtCtrl` and `_rbtCtrlUDP` share a lower-level socket, mutex, connection state, or controller-side lock;
- that the normal getter holds a mutex needed by `CNDKHandlerUDP::run()`;
- that the ROS service callback executes on the same thread as the ros2_control update loop.

## 6. Source of the three-second timeout

The exact 3.000606 s duration is client-side:

- `phase12_real_policy_rollout/03_shadow_mode/query_doosan_state_once.py:18` defaults `timeout_sec=3.0`.
- line 40 exposes `--timeout`, also defaulting to 3.0.
- No 3000 ms or 3.0 s total timeout was found in `_get_current_posx` or `SendCurrentTaskPoseCommand`.
- `SendCurrentTaskPoseCommand` instead performs repeated `EventImpl::waitImpl(10)` calls with no total-count/deadline check.
- `DRFC.h` value `OPERATION_SERVER_START = 3000` is an enum value, not a timeout.

Thus the client reports timeout at three seconds while the server-side DRFL call may remain blocked. A separate DRFL/controller internal timeout cannot be excluded elsewhere in proprietary code, but it is not the observed three-second boundary.

## 7. Mutex-cost explanation

The disassembly explains the high `pthread_mutex_lock/unlock` samples:

- every normal packet dispatch locks/unlocks ring-buffer head/tail/data mutexes;
- every RT UDP packet dispatch does the same;
- additional payload/callback state uses `Poco::MutexImpl::lockImpl/unlockImpl`;
- request construction also briefly locks an internal response-state mutex.

At high RT packet rates, and especially if queues are continuously active, this creates many lock/unlock calls even without high contention. Perf percentages alone cannot distinguish uncontended lock frequency from lock contention. Mutex wait time/futex evidence was not captured.

## 8. Most likely path explaining timeout plus feedback loss

### Proven portion

1. `get_current_posx_cb` enters a synchronous SDK call.
2. SDK sends command `0x0472` on the normal channel.
3. It waits indefinitely for `CNDKHandler::run()` to dispatch the matching reply.
4. The Phase 12 client stops waiting at 3 s; the server callback is not thereby cancelled.
5. JointState depends on the ros2_control update loop and the separate RT UDP buffer.
6. The actual experiment saw no JointState samples for 15.054 s after the timed-out getter while its publisher endpoint remained present.

### Highest-probability explanation

The normal-channel request failed to receive/dispatch its reply, leaving one executor callback blocked indefinitely. At the same time, both DRFL handlers were consuming nearly two CPU cores, and controller-manager services were already very slow. This combination can starve or block ros2_control process work sufficiently that the update loop/joint_state_broadcaster stops publishing, even though DDS endpoints and process objects remain alive. An alternative is a controller/DRFL shared internal state transition caused by the failed normal request that also stops or invalidates RT UDP updates.

The first mechanism (process/executor/update-loop starvation or blocking) is supported by high CPU, slow controller-manager replies, and the persistent callback wait. The second (cross-channel SDK/controller state coupling) is plausible because both channels belong to one DRFL/controller session, but cannot be proven without vendor source or runtime instrumentation.

It is not justified to claim that a specific shared mutex directly blocks RT feedback.

## 9. Log findings

The current log contains:

- 3,696 `[REAL] Skip dt=...` warnings.
- A dense burst around timestamp `1790902464.20`, predominantly about 10-20 microseconds instead of the expected 10 ms.
- Immediately following that burst, repeated controller-manager `switch_controller` response timeouts and strict-switch aborts.
- One `get_current_posx_cb` entry at `1790903402.503641334`, with no explicit exit log.
- No DRFL disconnect/monitor/socket diagnostic that proves the channel failure.

The tiny `dt` values come from the period supplied by controller_manager (`dt.seconds()`), not from the separately calculated `real_loop_dt`, which is computed but not used for the real-mode filter. They show a controller-manager scheduling/catch-up anomaly and cause severe WARN logging amplification. They do not by themselves explain the later getter failure, because the large burst precedes the recorded getter by about 938 seconds.

The same log also contains historical state-changing calls (`set_robot_mode`, `movej`) made before this static audit. This audit did not issue them.

## 10. Remaining unproven points

1. Whether the controller ever transmitted a `0x0472` response.
2. Whether TCP socket receive got the response but the normal handler failed to dispatch it.
3. Runtime value and semantics of the handler timed-semaphore member.
4. Whether normal and RT handles share a proprietary mutex/state below the exposed classes.
5. Whether the service callback blocked the controller-manager executor, update thread, or only one callback worker.
6. Whether RT UDP packets stopped arriving, kept arriving but were not dispatched, or were dispatched while ROS publication stopped.
7. Whether the two hot threads were processing genuine packet load, an erroneous queue flood, or immediate timeout wakeups.
8. Mutex contention versus high-frequency uncontended locking.
9. Controller-side behavior after an unanswered current-pose request.

## 11. Required runtime instrumentation (future controlled diagnostic)

No instrumentation was activated in this audit. For a later, separately approved diagnostic, add monotonic timestamp, thread ID, sequence/counter, and duration at:

1. `get_current_posx_cb`: entry and immediately before/after `Drfl->get_current_posx`.
2. `_get_current_posx`: entry/exit (wrapper or uprobe if vendor object cannot be rebuilt).
3. `CNDKHandler::SendCurrentTaskPoseCommand`: entry; before/after `SplitPacket`; every Nth wait iteration; response-flag transition; exit.
4. `CNDKHandler::run`: dequeue counter; packet command ID; command `0x0472` receipt; flag clear/Event set; queue depth; timed-wait return reason.
5. TCP socket worker: poll start/end, readiness, receive byte count/error/errno, connection state.
6. `CNDKHandlerUDP::run`: dequeue counter, RT packet ID/sequence, queue depth, semaphore return reason, latest-packet timestamp.
7. UDP socket worker: poll/receive entry/exit, byte count/error/errno, packet sequence.
8. `DRHWInterface::read`: entry; before/after `read_data_rt`; returned pointer; RT source sequence/timestamp if available; exit.
9. ros2_control update loop: cycle start/end, supplied period, read/update/write durations, deadline miss count.
10. joint_state_broadcaster update/publish: entry/exit and publish sequence.
11. `OnMonitoringDataExCB`, `OnMonitoringStateCB`, and `OnDisconnected`: lightweight atomic counters and last monotonic timestamp only.
12. Mutex analysis: eBPF/perf lock or futex tracing keyed by call site; do not add blocking log I/O inside RT/SDK callbacks.

The first decisive diagnostic is a four-way timeline: command `0x0472` sent, TCP bytes received/response dispatched, UDP RT packets received/dispatched, and ros2_control read/publish cycles. It distinguishes network/controller failure, DRFL dispatch failure, shared-state interference, and ROS update-loop starvation.

## Safety/accounting

During this audit:

- Robot command: 0
- Getter/service call: 0
- Motion service/action: 0
- Gripper/Home/trajectory: 0
- Hold/E-stop: 0
- Controller restart/switch/state change: 0
- Driver restart: 0

The current abnormal state was not altered.
