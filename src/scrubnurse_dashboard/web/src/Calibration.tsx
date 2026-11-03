import { useEffect, useState } from 'react'
import type { CalibrationPair, CalibrationState } from './types'

const mm = (metres: number) => (metres * 1000).toFixed(1)
const errorClass = (e?: number) => e === undefined ? '' : e > 0.01 ? 'alert' : e > 0.005 ? 'warn' : 'ok'

/** Camera-to-table calibration with the 40 x 40 x 10 mm block and a pointer in the gripper. */
export default function CalibrationPage() {
  const [state, setState] = useState<CalibrationState | null>(null)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null)
  const [file, setFile] = useState('')

  useEffect(() => {
    const poll = () => fetch('/api/calibration').then(r => r.json()).then(setState).catch(() => {})
    poll()
    const timer = window.setInterval(poll, 500)
    return () => clearInterval(timer)
  }, [])

  async function send(method: string, path: string, done: string, body?: object) {
    setBusy(true)
    try {
      const response = await fetch(`/api/calibration${path}`, {
        method, headers: { 'Content-Type': 'application/json' },
        body: body ? JSON.stringify(body) : undefined,
      })
      const data = await response.json().catch(() => ({}))
      if (response.ok) {
        setState(data)
        setMessage({ ok: true, text: data.saved ? `Saved ${data.saved.join(' and ')}` : done })
      } else {
        setMessage({ ok: false, text: typeof data.detail === 'string' ? data.detail : response.statusText })
      }
    } catch {
      setMessage({ ok: false, text: 'dashboard not reachable' })
    } finally {
      setBusy(false)
    }
  }

  if (!state) return <p className="waiting">Loading calibration…</p>
  const found = 'u' in state.target
  return (
    <main className="calibration">
      <section>
        <h2>Camera</h2>
        <CameraOverlay state={state} />
        <p className={found ? 'result' : 'alert'}>
          {found ? 'Block found' : `Block: ${(state.target as { error: string }).error}`}
        </p>
      </section>

      <section>
        <h2>Pairs <span className="muted">({state.pairs.length})</span></h2>
        <PairTable pairs={state.pairs} busy={busy}
          onRemove={i => send('DELETE', `/pairs/${i}`, `Pair ${i + 1} removed`)} />
        {state.pending && (
          <p className="note warn">
            Pixel ({state.pending.u.toFixed(1)}, {state.pending.v.toFixed(1)}) waiting for its reference
            position · {state.pending.frames} frames, spread {state.pending.spread.toFixed(1)} px
          </p>
        )}
      </section>

      <section>
        <h2>Result</h2>
        <p>tool0: {state.tool0 ? `x ${mm(state.tool0.x)}, y ${mm(state.tool0.y)} mm` : <span className="alert">no position (robot driver?)</span>}</p>
        {state.fit_error && <p className="alert">No fit: {state.fit_error}</p>}
        {!state.fit ? !state.fit_error && <p className="muted">The fit appears after 4 pairs.</p> : (
          <>
            <p>Fit error: mean {mm(state.fit.mean)} mm, max {mm(state.fit.max)} mm</p>
            <p>
              On unseen points: {state.fit.unseen_mean === null ? 'needs 5 pairs'
                : <span className={errorClass(state.fit.unseen_mean)}>mean {mm(state.fit.unseen_mean)} mm, max {mm(state.fit.unseen_max!)} mm</span>}
            </p>
            <h3>tool_localizer parameters</h3>
            <pre className="yaml">{state.fit.yaml}</pre>
          </>
        )}
      </section>

      <section className="calibration-buttons">
        <div className="buttons">
          <button disabled={busy || !found} title={found ? undefined : 'no block in view'}
            onClick={() => send('POST', '/pixel', 'Pixel position recorded')}>
            {busy ? 'Working…' : 'Select pixel position'}</button>
          <button disabled={busy || !state.pending || !state.tool0}
            title={!state.pending ? 'select the pixel position first' : undefined}
            onClick={() => send('POST', '/reference', 'Reference position recorded')}>Select reference position</button>
          <button disabled={busy || !state.pending}
            onClick={() => send('DELETE', '/pixel', 'Pixel position cleared')}>Clear pixel position</button>
          <button disabled={busy || !state.fit}
            onClick={() => send('POST', '/save', 'Saved')}>Save pairs and homography</button>
          <select value={file} onChange={e => setFile(e.target.value)} aria-label="Saved pairs">
            <option value="">Saved pairs…</option>
            {state.files.map(f => <option key={f} value={f}>{f}</option>)}
          </select>
          <button disabled={busy || !file}
            onClick={() => confirm(`Replace the current pairs with ${file}?`) && send('POST', '/load', `Loaded ${file}`, { name: file })}>Load</button>
        </div>
        {message && <p className={message.ok ? 'result' : 'result alert'} role="status">{message.text}</p>}
        <ol className="muted steps-help">
          <li>Put the block on the table, the arm out of the camera view, then <b>Select pixel position</b>.</li>
          <li>Jog the pointer onto the block centre with the pendant, then <b>Select reference position</b>.</li>
          <li>Repeat over the whole return area, corners included (20+ pairs). Remove pairs with large errors.</li>
          <li><b>Save</b>, then copy the parameters into <code>tool_localizer</code> in scrubnurse.yaml.</li>
        </ol>
      </section>
    </main>
  )
}

function CameraOverlay({ state }: { state: CalibrationState }) {
  const [w, h] = state.image_size ?? [1920, 1080]
  const target = 'u' in state.target ? state.target : null
  return (
    <div className="camera">
      <img src="/stream/camera" alt="Overhead camera with the calibration block and the recorded points" />
      <svg viewBox={`0 0 ${w} ${h}`} className="overlay-svg" aria-hidden="true">
        {target && <polygon points={target.corners.map(p => p.join(',')).join(' ')} className="target" />}
        {state.pairs.map((p, i) => (
          <g key={i} className={`point ${errorClass(p.unseen_error ?? p.error)}`}>
            <circle cx={p.u} cy={p.v} r={14} />
            <text x={p.u + 18} y={p.v - 14}>{i + 1}</text>
          </g>
        ))}
        {state.pending && <path className="pending"
          d={`M${state.pending.u - 18},${state.pending.v}h36M${state.pending.u},${state.pending.v - 18}v36`} />}
      </svg>
    </div>
  )
}

function PairTable({ pairs, busy, onRemove }:
  { pairs: CalibrationPair[]; busy: boolean; onRemove: (index: number) => void }) {
  if (pairs.length === 0) return <p className="muted">No pairs yet.</p>
  return (
    <div className="pairs">
      <table>
        <thead><tr><th>#</th><th>Pixel (u, v)</th><th>Robot (x, y) mm</th><th>Error</th><th>Unseen</th><th /></tr></thead>
        <tbody>
          {pairs.map((p, i) => (
            <tr key={i}>
              <td>{i + 1}</td>
              <td>{p.u.toFixed(1)}, {p.v.toFixed(1)}</td>
              <td>{mm(p.x)}, {mm(p.y)}</td>
              <td className={errorClass(p.error)}>{p.error === undefined ? '–' : mm(p.error)}</td>
              <td className={errorClass(p.unseen_error)}>{p.unseen_error === undefined ? '–' : mm(p.unseen_error)}</td>
              <td><button className="small" disabled={busy} aria-label={`Remove pair ${i + 1}`}
                onClick={() => onRemove(i)}>Remove</button></td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
