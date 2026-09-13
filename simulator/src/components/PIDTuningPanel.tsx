import React from 'react';
import { PIDGains, TelemetryData } from '../types';
import { formatNumber } from '../utils/helpers';

interface Props {
  rollPID: PIDGains;
  pitchPID: PIDGains;
  yawPID: PIDGains;
  telemetry: TelemetryData | null;
  onChange: (axis: 'roll' | 'pitch' | 'yaw', gains: PIDGains) => void;
  onReset: () => void;
}

/** Signed bar, zero in the middle, for a controller output that swings both ways. */
const OutputBar: React.FC<{ value: number; limit: number; color: string }> = ({ value, limit, color }) => {
  const ratio = Math.max(-1, Math.min(1, value / limit));
  const width = Math.abs(ratio) * 50;
  const left = ratio >= 0 ? 50 : 50 - width;

  return (
    <div className="output-track">
      <div className="output-zero" />
      <div className="output-fill" style={{ left: `${left}%`, width: `${width}%`, background: color }} />
    </div>
  );
};

const PIDSection: React.FC<{
  title: string;
  color: string;
  gains: PIDGains;
  output: number;
  outputLimit: number;
  onChange: (gains: PIDGains) => void;
}> = ({ title, color, gains, output, outputLimit, onChange }) => {
  const rows: [keyof PIDGains, string, number][] = [
    ['kp', 'Kp', 5],
    ['ki', 'Ki', 2],
    ['kd', 'Kd', 2],
  ];

  return (
    <div className="pid-section">
      <h3 style={{ color }}>{title}</h3>

      <div className="pid-output">
        <span>OUTPUT</span>
        <b>{output >= 0 ? '+' : ''}{formatNumber(output, 2)}</b>
      </div>
      <OutputBar value={output} limit={outputLimit} color={color} />

      {rows.map(([key, label, max]) => (
        <div className="slider-group" key={key}>
          <div className="slider-header">
            <span>{label}</span>
            <span>{formatNumber(gains[key], 2)}</span>
          </div>
          <input
            type="range"
            className="pid-slider"
            min="0"
            max={max}
            step="0.01"
            value={gains[key]}
            onChange={(e) => onChange({ ...gains, [key]: parseFloat(e.target.value) })}
          />
        </div>
      ))}
    </div>
  );
};

export const PIDTuningPanel: React.FC<Props> = ({
  rollPID,
  pitchPID,
  yawPID,
  telemetry,
  onChange,
  onReset,
}) => {
  const outputs = telemetry?.pidOutputs ?? { roll: 0, pitch: 0, yaw: 0 };

  return (
    <div className="pid-panel glass-panel">
      <div className="panel-title">
        <span>ATTITUDE PID</span>
        <span className="panel-title-tag">LIVE</span>
      </div>
      <p className="pid-note">
        These are the same gains as the C++ controller in <code>firmware/</code>.
        Tuning here changes both manual flight and the autopilot's inner loop.
      </p>

      <PIDSection
        title="ROLL" color="var(--primary)" gains={rollPID}
        output={outputs.roll} outputLimit={2}
        onChange={(gains) => onChange('roll', gains)}
      />
      <PIDSection
        title="PITCH" color="var(--success)" gains={pitchPID}
        output={outputs.pitch} outputLimit={2}
        onChange={(gains) => onChange('pitch', gains)}
      />
      <PIDSection
        title="YAW RATE" color="var(--accent)" gains={yawPID}
        output={outputs.yaw} outputLimit={1}
        onChange={(gains) => onChange('yaw', gains)}
      />

      <button className="reset-btn" onClick={onReset}>Reset to Defaults</button>
    </div>
  );
};
