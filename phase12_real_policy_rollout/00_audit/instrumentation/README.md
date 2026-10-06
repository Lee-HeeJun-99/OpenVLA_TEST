# Doosan diagnostic instrumentation

Instrumentation code only; it has not been built or executed.

- Editable callback/wrapper/read paths use compile-time `DSR_TRACE` records.
- `doosan_vendor_trace.bt` supplies uprobes for proprietary DRFL functions whose C++ source is absent.
- The 0x0472 dispatch and mutex offsets were verified against the archived object and installed shared object.
- `JOINT_STATE_PRE_PUBLISH` probes the point in `JointStateBroadcaster::update()` immediately before its realtime publisher is unlocked/notified. The system library is not modified.

The instrumentation has no publisher, ROS client, service call, motion API, or controller-management operation. `DSR_TRACE_ENABLE` defaults to disabled. Do not build, load, or run it until a separate diagnostic procedure is approved.

## Binary version pin

- `libDRFL.a` SHA-256: `5744d74c8f7455990a1326e0fa425a01d9911031e090e4c0ddd2fa6809ce61a6`
- Installed symbols: normal handler `0x0be030`, UDP handler `0x0c2480`, send function `0x0b9760`
- JointState broadcaster update symbol: `0x24050`

Rebuilds can change offsets. Revalidate hashes, symbols, and disassembly before later use. A future invocation must attach with `bpftrace -p <ros2_control_node_pid>` so unrelated processes are excluded.

## Diagnostic interpretation

- No `RESPONSE_0472_DISPATCH_START`: distinguish missing/unreceived controller response from dispatch failure with TCP socket receive instrumentation.
- Dispatch start without Event set/end: handler dispatch failure.
- UDP/read/pre-publish events stop independently of the blocked getter: concurrent RT or ros2_control stall.
- UDP continues while `DRHWInterface::read` stops: ros2_control update-loop stall.
- `read` continues while pre-publish stops: broadcaster path stall.
