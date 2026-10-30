# State of the task controller, published (latched) on every change.

# phase: where the current cycle is.
string PHASE_IDLE=idle                      # waiting for a request or a tool on the table
string PHASE_HOMING=homing                  # joint move to the rest position
string PHASE_PICKING=picking                # from the magazine (handover) or the table (return)
string PHASE_HANDING_OVER=handing_over      # holding the instrument out to the surgeon
string PHASE_PLACING=placing                # putting the instrument back into the magazine
string PHASE_RETURNING_HOME=returning_home  # back to the rest position after a cycle

# kind: which cycle.
string KIND_NONE=
string KIND_HANDOVER=handover               # requested by voice or the dashboard
string KIND_RETURN=return                   # tool detected in the return area

builtin_interfaces/Time stamp
uint32 cycle_id      # counts up from 1 for each handover or return; 0 before the first
string kind
string tool_id       # A..F, empty outside a cycle
string tool_name     # e.g. "scalpel"
string phase
string description   # for people, e.g. "Handing over the scalpel"
uint16 step          # 1-based motion/gripper step within the cycle, 0 outside a cycle
uint16 step_count
bool paused          # no new cycle starts until resumed
bool aborted         # stopped by abort; needs recover before resume
string error         # why the last cycle failed, empty if it did not
string[] queue       # requested tool ids waiting, oldest first
