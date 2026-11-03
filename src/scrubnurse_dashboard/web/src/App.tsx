import { useEffect, useState } from 'react'
import CalibrationPage from './Calibration'
import type { Action, Cycle, Detection, Health, LogEvent, State, Tool, VoiceEvent } from './types'

const UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

const CYCLE_STEPS: Record<string, [string, string][]> = {
  handover: [['picking', 'Pick from magazine'], ['handing_over', 'Hand over'],
    ['returning_home', 'Back to rest']],
  return: [['picking', 'Pick up from table'], ['placing', 'Place in magazine'],
    ['returning_home', 'Back to rest']],
}

const clock = (t: number) => new Date(t * 1000).toLocaleTimeString([], { hour12: false })

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
          <div className="column">
            <StatusPanel cycle={state.cycle} />
            <Controls state={state} />
            <ArmPanel arm={state.arm} health={state.health} />
          </div>
          <div className="column">
            <CameraPanel health={state.health} detections={state.detections} streamKey={connections} />
            <VoicePanel voice={state.voice} running={state.health.voice} />
          </div>
          <EventLog events={state.events} />
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

function CameraPanel({ health, detections, streamKey }:
  { health: Health; detections: Detection[]; streamKey: number }) {
  const camera = health.topics.camera
  const detector = health.topics.detections
  return (
    <section>
      <h2>Camera and detection</h2>
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

function VoicePanel({ voice, running }: { voice: VoiceEvent[]; running: boolean }) {
  const last = voice[0]
  const status = !running ? 'Voice node not running'
    : last?.kind === 'error' ? `Error: ${last.text}` : 'Listening'
  return (
    <section>
      <h2>Voice <span className={`tag ${running && last?.kind !== 'error' ? 'ok' : 'alert'}`}>{status}</span></h2>
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

function EventLog({ events }: { events: LogEvent[] }) {
  return (
    <section className="log">
      <h2>Event log</h2>
      <ul>
        {events.map((e, i) => (
          <li key={events.length - i} className={e.level}>
            <span className="time">{clock(e.time)}</span>
            <span className="source">{e.source}</span>
            {e.text}
          </li>
        ))}
      </ul>
    </section>
  )
}
