import React from 'react';
import { TelemetryData } from '../types';
import { formatNumber } from '../utils/helpers';

interface Props {
  telemetry: TelemetryData;
}

export const TelemetryPanel: React.FC<Props> = ({ telemetry }) => {
  return (
    <div className="telemetry-panel glass-panel">
      <div className="telemetry-row">
        <span className="telemetry-label">ALT:</span>
        <span>{formatNumber(telemetry.altitude, 1)} m</span>
      </div>
      <div className="telemetry-row">
        <span className="telemetry-label">SPD:</span>
        <span>{formatNumber(telemetry.speed, 1)} m/s</span>
      </div>
      <div className="telemetry-row">
        <span className="telemetry-label">HDG:</span>
        <span>{formatNumber(telemetry.heading, 0)}°</span>
      </div>
      <div className="telemetry-row">
        <span className="telemetry-label">BAT:</span>
        <span style={{ color: telemetry.battery < 20 ? 'var(--danger)' : 'var(--success)' }}>
          {formatNumber(telemetry.battery, 0)}%
        </span>
      </div>

      <div style={{ marginTop: 15, fontSize: 12, color: 'var(--text-muted)' }}>MOTOR RPM</div>
      <div className="motor-bars">
        <div>
          <div style={{ fontSize: 10, marginBottom: 2 }}>FL (Cyan)</div>
          <div className="motor-bar-container">
            <div className="motor-bar-fill" style={{ width: `${(telemetry.motorRPMs[0] / 10000) * 100}%`, background: 'var(--primary)' }} />
          </div>
        </div>
        <div>
          <div style={{ fontSize: 10, marginBottom: 2 }}>FR (Green)</div>
          <div className="motor-bar-container">
            <div className="motor-bar-fill" style={{ width: `${(telemetry.motorRPMs[1] / 10000) * 100}%`, background: 'var(--success)' }} />
          </div>
        </div>
        <div>
          <div style={{ fontSize: 10, marginBottom: 2 }}>RL (Yellow)</div>
          <div className="motor-bar-container">
            <div className="motor-bar-fill" style={{ width: `${(telemetry.motorRPMs[2] / 10000) * 100}%`, background: 'var(--warning)' }} />
          </div>
        </div>
        <div>
          <div style={{ fontSize: 10, marginBottom: 2 }}>RR (Red)</div>
          <div className="motor-bar-container">
            <div className="motor-bar-fill" style={{ width: `${(telemetry.motorRPMs[3] / 10000) * 100}%`, background: 'var(--danger)' }} />
          </div>
        </div>
      </div>
    </div>
  );
};
