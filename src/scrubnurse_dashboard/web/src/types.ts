// The JSON that the dashboard node sends (GET /api/state and /ws/state).

export type Action = 'pause' | 'resume' | 'abort' | 'recover' | 'request'

export interface Tool { tool_id: string; tool_name: string }

export interface Cycle {
  cycle_id: number
  kind: '' | 'handover' | 'return'
  tool_id: string
  tool_name: string
  phase: 'idle' | 'homing' | 'picking' | 'handing_over' | 'placing' | 'returning_home'
  description: string
  step: number
  step_count: number
  paused: boolean
  aborted: boolean
  error: string
  queue: Tool[]
}

export interface Freshness { age: number | null; stale: boolean }

export interface Health {
  mode: string
  node: boolean
  controller: boolean
  voice: boolean
  camera_node: boolean
  topics: Record<'joint_states' | 'camera' | 'detections', Freshness>
}

export interface Detection extends Tool {
  color: string  // '#rrggbb', same as the box in the camera stream
  score: number
  x: number
  y: number
  width: number
  height: number
  theta: number | null
  position: [number, number] | null  // robot base frame, m
}

export interface VoiceEvent extends Tool {
  time: number
  kind: 'ready' | 'heard' | 'error'
  text: string
  wake_word: boolean
}

export interface VoiceStatus {
  state: 'listening' | 'muted' | 'command' | 'processing' | 'idle' | 'error'
  message: string
  live: boolean
  muted: boolean
  input_device: string  // '' = system default
  devices: string[]
  wake_words: string[]
}

export interface CameraStatus {
  serial: string             // streaming now
  configured_serial: string  // the calibrated overhead camera
  serials: string[]
}

export interface LogEvent {
  time: number
  level: 'info' | 'warn' | 'error'
  source: string
  text: string
}

export interface CycleRow {  // one handover or return of a recorded run (metrics.py)
  cycle: number
  kind: 'handover' | 'return'
  tool: string
  start: number           // s from the start of the run
  seconds: number | null  // null: unfinished
  result: string          // 'ok', 'aborted', 'unfinished' or 'failed: …'
}

export interface Replay {
  start: number     // s since the epoch, as the event times
  duration: number  // s
  time: number      // s from the start
  paused: boolean
  rate: number
  cycles: CycleRow[]
}

export interface State {
  mode: string
  health: Health
  cycle: Cycle | null
  allowed: Record<Action, string | null>  // why the action is refused, or null
  arm: { names: string[]; positions: number[]; moving: boolean } | null
  detections: Detection[]
  voice: VoiceEvent[]
  voice_status: VoiceStatus | null    // null: voice node not running
  camera_status: CameraStatus | null  // null: no Azure Kinect node (e.g. in the sim)
  events: LogEvent[]
  tools: Tool[]
  replay: Replay | null  // null: live
}

// GET /api/calibration (all positions in pixels or metres).
export interface CalibrationPair {
  u: number
  v: number
  x: number
  y: number
  error?: number         // against the fit of all pairs
  unseen_error?: number  // against a fit without this pair (leave-one-out)
}

export interface CalibrationState {
  pairs: CalibrationPair[]
  pending: { u: number; v: number; frames: number; spread: number } | null
  fit: {
    homography: number[]
    yaml: string
    mean: number
    max: number
    unseen_mean: number | null
    unseen_max: number | null
  } | null
  fit_error: string | null
  files: string[]
  target: { u: number; v: number; corners: [number, number][] } | { error: string }
  tool0: { x: number; y: number } | null
  image_size: [number, number] | null
  saved?: [string, string]
}
