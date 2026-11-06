import { useEffect, useState } from 'react'
import CalibrationPage from './Calibration'
import type { Action, CameraStatus, Cycle, Detection, Health, LogEvent, Replay, State, Tool, VoiceEvent, VoiceStatus } from './types'

const UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

const CYCLE_STEPS: Record<string, [string, string][]> = {
  handover: [['picking', 'Pick from magazine'], ['handing_over', 'Hand over'],
    ['returning_home', 'Back to rest']],
  return: [['picking', 'Pick up from table'], ['placing', 'Place in magazine'],
    ['returning_home', 'Back to rest']],
}

/** POST JSON to the dashboard; resolves to a line for the operator either way. */
async function post(path: string, body?: object): Promise<{ ok: boolean; text: string }> {
  try {
    const response = await fetch(path, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: body ? JSON.stringify(body) : undefined,
    })
    const data = await response.json().catch(() => ({}))
    return response.ok ? { ok: true, text: data.message || 'done' }
      : { ok: false, text: typeof data.detail === 'string' ? data.detail : response.statusText }
  } catch {
    return { ok: false, text: 'dashboard not reachable' }
  }
}

const clock = (t: number) => new Date(t * 1000).toLocaleTimeString([], { hour12: false })
const minutes = (s: number) => `${Math.floor(s / 60)}:${Math.floor(s % 60).toString().padStart(2, '0')}`

/** Latest state from /ws/state; reconnects every second while the dashboard node is down. */
function useDashboardState() {
  const [state, setState] = useState<State | null>(null)
  const [connected, setConnected] = useState(false)
  const [connections, setConnections] = useState(0)
  useEffect(() => {
    let socket: WebSocket
    let retry: number
    let closed = false
    const connect = () => {
      socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/state`)
      socket.onopen = () => { setConnected(true); setConnections(n => n + 1) }
      socket.onmessage = e => setState(JSON.parse(e.data))
      socket.onclose = () => {
        setConnected(false)
        if (!closed) retry = window.setTimeout(connect, 1000)
      }
    }
    connect()
    return () => { closed = true; clearTimeout(retry); socket.close() }
  }, [])
  return { state, connected, connections }
}

/** The page in the URL hash: #calibration, or the operation page. */
function usePage() {
  const [page, setPage] = useState(location.hash)
  useEffect(() => {
    const changed = () => setPage(location.hash)
    window.addEventListener('hashchange', changed)
    return () => window.removeEventListener('hashchange', changed)
  }, [])
  return page === '#calibration' ? 'calibration' : 'operation'
}

export default function App() {
  const { state, connected, connections } = useDashboardState()
  const page = usePage()
  return (
    <>
      <Header state={state} connected={connected} page={page} />
      {page === 'calibration' ? <CalibrationPage /> : !state ? <p className="waiting">Connecting to the dashboard node…</p> : (
        <main>
          {state.replay && <Timeline replay={state.replay} />}
          <div className="column">
            <StatusPanel cycle={state.cycle} />
            {!state.replay && <Controls state={state} />}
            <VoicePanel voice={state.voice} status={state.voice_status} replay={!!state.replay} />
          </div>
          <div className="column">
            <CameraPanel health={state.health} detections={state.detections} streamKey={connections}
              camera={state.camera_status} replay={!!state.replay} />
            {state.mode === 'sim' && !state.health.topics.camera.stale && (  // both sim cameras need camera:=true
              <section>
                <h2>Simulation</h2>
                <div className="camera">
                  <img key={connections} src="/stream/scene" alt="Side view of the simulated cell" />
                </div>
              </section>
            )}
          </div>
          <ArmPanel arm={state.arm} health={state.health} />
          <EventLog events={state.events} replay={state.replay} />
        </main>
      )}
    </>
  )
}

function Header({ state, connected, page }: { state: State | null; connected: boolean; page: string }) {
  const health = state?.health
  return (
    <header>
      <h1>Scrubnurse</h1>
      {state && <span className={`badge mode-${state.mode}`}>{state.mode.toUpperCase()}</span>}
      <nav>
        <a href="#" aria-current={page === 'operation' ? 'page' : undefined}>Operation</a>
        <a href="#calibration" aria-current={page === 'calibration' ? 'page' : undefined}>Calibration</a>
      </nav>
      <div className="health">
        <Light ok={connected} label={connected ? 'Dashboard connected' : 'Dashboard disconnected'} />
        {health && <>
          <Light ok={health.controller} label="Controller" />
          <Light ok={health.voice} label="Voice" />
          <Light ok={!health.topics.joint_states.stale} label="Arm" />
          <Light ok={!health.topics.camera.stale} label="Camera" />
          <Light ok={!health.topics.detections.stale} label="Detection" />
        </>}
      </div>
    </header>
  )
}

const RATES = [0.25, 0.5, 1, 2, 4]

/** Play, pause, jump and speed of a recorded run; the marks are its handovers and returns. */
function Timeline({ replay }: { replay: Replay }) {
  const [dragged, setDragged] = useState<number | null>(null)  // slider position while moving it
  const [result, setResult] = useState<{ ok: boolean; text: string } | null>(null)
  const send = async (path: string, body?: object) => {
    const r = await post(`/api/replay/${path}`, body)
    setResult(r.ok ? null : r)
  }
  const seek = (time: number) => send('seek', { time })
  const commit = () => { if (dragged !== null) { seek(dragged); setDragged(null) } }
  const time = dragged ?? replay.time
  const at = (t: number) => `${(100 * t / replay.duration).toFixed(2)}%`
  return (
    <section className="timeline">
      <div className="buttons">
        <button onClick={() => send(replay.paused ? 'resume' : 'pause')}>{replay.paused ? 'Play' : 'Pause'}</button>
        <span className="clock">{minutes(time)} / {minutes(replay.duration)}</span>
        <label>Speed{' '}
          <select value={replay.rate} onChange={e => send('rate', { rate: Number(e.target.value) })}>
            {RATES.map(r => <option key={r} value={r}>{r}×</option>)}
          </select>
        </label>
      </div>
      <div className="track">
        {replay.cycles.map(c => (
          <button key={c.cycle} className={`mark ${c.result === 'ok' ? 'ok' : c.result === 'unfinished' ? '' : 'alert'}`}
            style={{ left: at(c.start), width: c.seconds ? at(c.seconds) : undefined }}
            title={`${c.kind} of the ${c.tool}: ${c.result}${c.seconds ? `, ${c.seconds} s` : ''}`}
            onClick={() => seek(c.start)}>{c.kind === 'handover' ? 'H' : 'R'}</button>
        ))}
      </div>
      <input type="range" min={0} max={replay.duration} step={0.1} value={time} aria-label="Replay position"
        onChange={e => setDragged(Number(e.target.value))} onPointerUp={commit} onKeyUp={commit} onBlur={commit} />
      {result && <p className="alert" role="status">{result.text}</p>}
    </section>
  )
}

function Light({ ok, label }: { ok: boolean; label: string }) {
  return <span className={`light ${ok ? 'ok' : 'bad'}`}>{label}{ok ? '' : ' ✕'}</span>
}

function StatusPanel({ cycle }: { cycle: Cycle | null }) {
  if (!cycle) return <section><h2>Status</h2><p className="muted">No status from the controller yet.</p></section>
  const steps = CYCLE_STEPS[cycle.kind] ?? []
  const current = steps.findIndex(([phase]) => phase === cycle.phase)
  return (
    <section>
      <h2>Status {cycle.cycle_id > 0 && <span className="muted">· cycle {cycle.cycle_id}</span>}</h2>
      <p className={`description ${cycle.aborted || cycle.error ? 'alert' : ''}`} aria-live="polite">
        {cycle.description}
      </p>
      {steps.length > 0 && (
        <>
          <ol className="steps">
            {steps.map(([phase, label], i) => (
              <li key={phase} className={i < current ? 'done' : i === current ? 'current' : ''}>{label}</li>
            ))}
          </ol>
          <progress max={cycle.step_count} value={cycle.step} aria-label="Steps of this cycle" />
          <p className="muted">Step {cycle.step} of {cycle.step_count}</p>
        </>
      )}
      <p className="flags">
        {cycle.aborted ? <span className="tag alert">Aborted</span>
          : cycle.paused ? <span className="tag warn">{cycle.kind ? 'Pausing after this cycle' : 'Paused'}</span>
          : <span className="tag ok">Running</span>}
        {cycle.error && <span className="tag alert">Error: {cycle.error}</span>}
      </p>
      <p className="muted">
        Waiting: {cycle.queue.length ? cycle.queue.map(t => t.tool_name || t.tool_id).join(', ') : 'nothing'}
      </p>
    </section>
  )
}

function Controls({ state }: { state: State }) {
  const [busy, setBusy] = useState(false)
  const [result, setResult] = useState<{ ok: boolean; text: string } | null>(null)

  async function send(action: Action, body?: object) {
    setBusy(true)
    try {
      const response = await fetch(`/api/${action}`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: body ? JSON.stringify(body) : undefined,
      })
      const data = await response.json().catch(() => ({}))
      setResult(response.ok
        ? { ok: true, text: `${action}: ${data.message || 'done'}` }
        : { ok: false, text: `${action} failed: ${typeof data.detail === 'string' ? data.detail : response.statusText}` })
    } catch (e) {
      setResult({ ok: false, text: `${action} failed: dashboard not reachable` })
    } finally {
      setBusy(false)
    }
  }

  const button = (action: Action, label: string, onClick: () => void, className = '') => (
    <button key={label} className={className} disabled={busy || !!state.allowed[action]}
      title={state.allowed[action] ?? undefined} onClick={onClick}>{label}</button>
  )
  return (
    <section>
      <h2>Controls</h2>
      <div className="buttons">
        {button('resume', 'Resume', () => send('resume'))}
        {button('pause', 'Pause', () => send('pause'))}
        {button('abort', 'Abort', () => {
          if (confirm('Abort: the arm stops where it is and keeps holding the instrument. Abort now?')) send('abort')
        }, 'danger')}
        {button('recover', 'Recover', () => {
          if (confirm('Recover moves the arm to the rest position. Is the arm clear of people and objects?'))
            send('recover', { arm_clear: true })
        })}
      </div>
      <h3>Request an instrument</h3>
      <div className="buttons">
        {state.tools.map((tool: Tool) => button('request', tool.tool_name,
          () => send('request', { instrument: tool.tool_id })))}
      </div>
      {state.allowed.request && <p className="muted">Requests: {state.allowed.request}</p>}
      {result && <p className={result.ok ? 'result' : 'result alert'} role="status">{result.text}</p>}
    </section>
  )
}

function ArmPanel({ arm, health }: { arm: State['arm']; health: Health }) {
  const joints = arm ? UR_JOINTS.map(name => [name, arm.positions[arm.names.indexOf(name)]] as const)
    .filter(([, value]) => value !== undefined) : []
  return (
    <section>
      <h2>Arm {arm && !health.topics.joint_states.stale &&
        <span className={`tag ${arm.moving ? 'warn' : 'ok'}`}>{arm.moving ? 'Moving' : 'Still'}</span>}</h2>
      {health.topics.joint_states.stale ? <p className="alert">No joint states from the robot</p> : (
        <table className="joints">
          <tbody>
            {joints.map(([name, value]) => (
              <tr key={name}><td>{name.replace('_joint', '').replace('_', ' ')}</td>
                <td>{(value * 180 / Math.PI).toFixed(1)}°</td></tr>
            ))}
          </tbody>
        </table>
      )}
    </section>
  )
}

function CameraPanel({ health, detections, streamKey, camera: status, replay }:
  { health: Health; detections: Detection[]; streamKey: number; camera: CameraStatus | null; replay: boolean }) {
  const camera = health.topics.camera
  const detector = health.topics.detections
  return (
    <section>
      <h2>Camera and detection</h2>
      {status && (replay ? <p className="muted">Camera {status.serial}
        {status.serial === status.configured_serial ? ' (calibrated, overhead)' : ''}</p> : <CameraChoice status={status} />)}
      <div className="camera">
        <img key={streamKey} src="/stream/camera" alt="Overhead camera with the detected instruments" />
        {camera.stale && (
          <div className="overlay">
            {camera.age === null ? `No camera image${health.mode === 'sim' ? ' (start the sim with camera:=true)' : ''}`
              : `Camera stale: last image ${camera.age.toFixed(0)} s ago`}
          </div>
        )}
      </div>
      {detector.stale ? <p className="alert">Detection {detector.age === null ? 'not running' : 'stale'}</p> : (
        <table className="detections">
          <thead><tr><th>Instrument</th><th>Confidence</th><th>Position (x, y)</th></tr></thead>
          <tbody>
            {detections.length === 0 && <tr><td colSpan={3} className="muted">Nothing on the table</td></tr>}
            {detections.map((d, i) => (
              <tr key={i}>
                <td><span className="swatch" style={{ background: d.color }} aria-hidden="true" />{d.tool_name || d.tool_id}</td>
                <td>{(d.score * 100).toFixed(0)} %</td>
                <td>{d.position ? `${(d.position[0] * 1000).toFixed(0)}, ${(d.position[1] * 1000).toFixed(0)} mm` : '–'}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </section>
  )
}

function CameraChoice({ status }: { status: CameraStatus }) {
  const [busy, setBusy] = useState(false)
  const [result, setResult] = useState<{ ok: boolean; text: string } | null>(null)
  async function choose(serial: string) {
    if (serial !== status.configured_serial && !confirm(
      `Camera ${serial} is not the calibrated overhead camera (${status.configured_serial}). `
      + 'Detected positions will be wrong and picks will miss. Switch anyway?')) return
    setBusy(true)
    setResult(await post('/api/camera', { serial }))
    setBusy(false)
  }
  const calibrated = status.serial === status.configured_serial || !status.configured_serial
  return (
    <div className="device-choice">
      <label>
        Camera{' '}
        <select value={status.serial} disabled={busy} onChange={e => choose(e.target.value)}>
          {status.serials.map(s => (
            <option key={s} value={s}>{s}{s === status.configured_serial ? ' (calibrated, overhead)' : ''}</option>
          ))}
        </select>
      </label>
      {busy && <span className="muted"> switching…</span>}
      {!calibrated && <p className="alert">Not the calibrated camera: picks will miss.</p>}
      {result && !result.ok && <p className="alert" role="status">{result.text}</p>}
    </div>
  )
}

const VOICE_STATES: Record<VoiceStatus['state'], [string, string]> = {
  listening: ['ok', 'Listening for the wake word'],
  muted: ['warn', 'Muted'],
  command: ['ok', 'Say the command…'],
  processing: ['ok', 'Transcribing…'],
  idle: ['ok', 'Push to talk to give a command'],
  error: ['alert', 'Microphone error'],
}

/** The device name without the ALSA card number, which changes between boots. */
const stableName = (device: string) => device.replace(/\s*\(hw:[^)]*\)$/, '')

function VoicePanel({ voice, status, replay }: { voice: VoiceEvent[]; status: VoiceStatus | null; replay: boolean }) {
  const [busy, setBusy] = useState(false)
  const [result, setResult] = useState<{ ok: boolean; text: string } | null>(null)
  async function send(path: string, body?: object) {
    setBusy(true)
    setResult(await post(path, body))
    setBusy(false)
  }
  const [tagClass, tagText] = status ? VOICE_STATES[status.state] : ['alert', 'Voice node not running']
  const devices = status ? [...status.devices] : []
  if (status?.input_device && !devices.includes(status.input_device)) devices.unshift(status.input_device)
  return (
    <section>
      <h2>Voice <span className={`tag ${tagClass}`} aria-live="polite">{tagText}</span></h2>
      {status?.state === 'error' && <p className="alert">{status.message}</p>}
      {status && replay && <p className="muted">Microphone: {status.input_device || 'system default'}</p>}
      {status && !replay && (
        <>
          <div className="buttons">
            <button disabled={busy || status.state === 'command' || status.state === 'processing'}
              onClick={() => send('/api/voice/listen')}>Push to talk</button>
            {status.live && (
              <button disabled={busy} onClick={() => send('/api/voice/mute', { muted: !status.muted })}>
                {status.muted ? 'Unmute' : 'Mute'}</button>
            )}
          </div>
          <div className="device-choice">
            <label>
              Microphone{' '}
              <select value={status.input_device} disabled={busy}
                onChange={e => send('/api/voice/microphone', { name: e.target.value })}>
                <option value="">System default</option>
                {devices.map(d => <option key={d} value={d}>{d}</option>)}
              </select>
            </label>
            {status.input_device && status.devices.includes(status.input_device) && (
              <p className="muted">Not saved. To keep it, set <code>input_device: "{stableName(status.input_device)}"</code> under
                {' '}<code>voice_command</code> in scrubnurse.yaml.</p>
            )}
          </div>
          {status.live && <p className="muted">Say “{status.wake_words[0]}, scalpel”, or press Push to talk and say “scalpel”.</p>}
          {result && !result.ok && <p className="alert" role="status">{result.text}</p>}
        </>
      )}
      {voice.filter(v => v.kind === 'heard').length === 0 ? <p className="muted">Nothing heard yet.</p> : (
        <ul className="voice">
          {voice.filter(v => v.kind === 'heard').map((v, i) => (
            <li key={i}>
              <span className="time">{clock(v.time)}</span> “{v.text}”{' '}
              {v.tool_id ? <span className="tag ok">→ {v.tool_name}</span>
                : v.wake_word ? <span className="tag warn">no instrument recognised</span>
                : <span className="tag muted">no wake word</span>}
            </li>
          ))}
        </ul>
      )}
    </section>
  )
}

const LEVELS: LogEvent['level'][] = ['info', 'warn', 'error']

/** Newest first; in a replay the run's log up to now, and a click jumps to the line. */
function EventLog({ events, replay }: { events: LogEvent[]; replay: Replay | null }) {
  const [level, setLevel] = useState<LogEvent['level']>('info')
  const [source, setSource] = useState('')
  const sources = [...new Set(events.map(e => e.source))].sort()
  const shown = events.filter(e => LEVELS.indexOf(e.level) >= LEVELS.indexOf(level)
    && (!source || e.source === source))
  return (
    <section className="log">
      <h2>{replay ? 'Log' : 'Event log'}</h2>
      <div className="device-choice">
        <label>Level{' '}
          <select value={level} onChange={e => setLevel(e.target.value as LogEvent['level'])}>
            <option value="info">all</option>
            <option value="warn">warnings and errors</option>
            <option value="error">errors</option>
          </select>
        </label>{' '}
        <label>From{' '}
          <select value={source} onChange={e => setSource(e.target.value)}>
            <option value="">everyone</option>
            {sources.map(s => <option key={s} value={s}>{s}</option>)}
          </select>
        </label>
      </div>
      <ul>
        {shown.length === 0 && <li className="muted">Nothing logged{level !== 'info' || source ? ' with this filter' : ''}.</li>}
        {shown.map((e, i) => (
          <li key={shown.length - i} className={e.level}>
            {replay ? <button className="time link" title="Replay from 2 s before this line"
              onClick={() => post('/api/replay/seek', { time: Math.max(0, e.time - replay.start - 2) })}>
              {clock(e.time)}</button> : <span className="time">{clock(e.time)}</span>}
            <span className="source">{e.source}</span>
            {e.text}
          </li>
        ))}
      </ul>
    </section>
  )
}
