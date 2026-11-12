import React, { useEffect, useState } from 'react';
import { HilController } from '../hil/HilController';
import { webSerialAvailable } from '../hil/webserial';

interface Props {
  hil: HilController;
}

/**
 * Hardware-in-the-loop panel. Connects to an ESP32 running the esp32_hil
 * firmware build over WebSerial and shows what the board reports about
 * itself. While connected, the board is flying: the 3D drone is drawn from
 * the HIL plant, and the browser's own controller and autopilot stand down.
 */
export const HilPanel: React.FC<Props> = ({ hil }) => {
  const [, setTick] = useState(0);
  const [busy, setBusy] = useState(false);

  // The session mutates in place; repaint the readout a few times a second.
  useEffect(() => {
    const id = setInterval(() => setTick((t) => t + 1), 250);
    return () => clearInterval(id);
  }, []);

  if (!webSerialAvailable()) {
    return (
      <div className="glass-panel hil-panel">
        <div className="panel-title">HARDWARE IN THE LOOP</div>
        <div className="hil-note">Needs WebSerial: Chrome or Edge on desktop.</div>
      </div>
    );
  }

  const s = hil.session;
  const st = s?.stats;
  const v = s?.board.values ?? {};

  const onClick = async () => {
    setBusy(true);
    if (hil.active) await hil.disconnect();
    else await hil.connect();
    setBusy(false);
  };

  return (
    <div className="glass-panel hil-panel">
      <div className="panel-title">
        HARDWARE IN THE LOOP <span className="panel-title-tag">{hil.active ? 'BOARD FLYING' : 'OFF'}</span>
      </div>
      <button className="hil-button" onClick={onClick} disabled={busy}>
        {hil.active ? 'Disconnect board' : 'Connect ESP32 (esp32_hil build)'}
      </button>
      {hil.error && <div className="hil-error">{hil.error}</div>}
      {s && st && (
        <div className="hil-grid">
          <span>state</span><b>{s.board.state}</b>
          <span>fault</span><b>{s.board.fault}</b>
          <span>RC link</span><b>{s.linkUp ? 'up' : 'CUT (L)'}</b>
          <span>sim time</span><b>{st.simTime.toFixed(1)} s</b>
          <span>round trip</span><b>{st.roundTripMs.toFixed(2)} ms</b>
          <span>timeouts</span><b>{st.timeouts}</b>
          <span>rejected</span><b>{st.crcErrors}</b>
          <span>board jitter</span><b>{v['loop_jit'] !== undefined ? `${v['loop_jit'].toFixed(0)} us` : '-'}</b>
          <span>altitude</span><b>{s.plant.z.toFixed(2)} m</b>
        </div>
      )}
      <div className="hil-note">
        Hold X 2 s to arm, Space/Shift throttle, WASD attitude, Q/E yaw, L cuts the RC link.
        {hil.active && hil.lag > 0.5 ? ` Link slower than real time: ${hil.lag.toFixed(1)} s behind.` : ''}
      </div>
    </div>
  );
};
