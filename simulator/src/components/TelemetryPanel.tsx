import React from 'react';
import { AutopilotStatus, FlightMode, TelemetryData } from '../types';
import { formatNumber, toCompass } from '../utils/helpers';

interface Props {
  telemetry: TelemetryData;
  status: AutopilotStatus;
  mode: FlightMode;
}

const MOTORS: { label: string; color: string }[] = [
  { label: 'FL', color: 'var(--primary)' },
  { label: 'FR', color: 'var(--success)' },
  { label: 'RL', color: 'var(--warning)' },
  { label: 'RR', color: 'var(--danger)' },
];

const MAX_RPM = 11000;

export const TelemetryPanel: React.FC<Props> = ({ telemetry, status, mode }) => {
  const climbing = telemetry.verticalSpeed > 0.25;
  const descending = telemetry.verticalSpeed < -0.25;

  const batteryColor =
    telemetry.battery < 15 ? 'var(--danger)' : telemetry.battery < 35 ? 'var(--warning)' : 'var(--success)';

  return (
    <div className="telemetry-panel glass-panel">
      <div className="panel-title">
        <span>TELEMETRY</span>
        <span className="panel-title-tag">{mode === 'AUTO' ? 'AP LINK' : 'RC LINK'}</span>
      </div>

      <div className="telemetry-grid">
        <div className="telemetry-cell">
          <div className="cell-label">ALTITUDE</div>
          <div className="cell-value">
            {formatNumber(telemetry.altitude, 1)}<span className="cell-unit">m</span>
          </div>
        </div>
        <div className="telemetry-cell">
          <div className="cell-label">GROUND SPD</div>
          <div className="cell-value">
            {formatNumber(telemetry.speed, 1)}<span className="cell-unit">m/s</span>
          </div>
        </div>
        <div className="telemetry-cell">
          <div className="cell-label">HEADING</div>
          <div className="cell-value">
            {formatNumber(toCompass(telemetry.heading * (Math.PI / 180)), 0)}<span className="cell-unit">°</span>
          </div>
        </div>
        <div className="telemetry-cell">
          <div className="cell-label">V-SPEED</div>
          <div className="cell-value" style={{ color: climbing ? 'var(--success)' : descending ? 'var(--warning)' : undefined }}>
            {telemetry.verticalSpeed >= 0 ? '+' : ''}{formatNumber(telemetry.verticalSpeed, 1)}
            <span className="cell-unit">m/s</span>
          </div>
        </div>
      </div>

      <div className="telemetry-row">
        <span className="telemetry-label">BATTERY</span>
        <span style={{ color: batteryColor }}>{formatNumber(telemetry.battery, 0)}%</span>
      </div>
      <div className="battery-track">
        <div
          className="battery-fill"
          style={{ width: `${Math.max(0, Math.min(100, telemetry.battery))}%`, background: batteryColor }}
        />
      </div>

      <div className="section-label">MOTOR OUTPUT (RPM)</div>
      <div className="motor-bars">
        {MOTORS.map((motor, i) => (
          <div key={motor.label}>
            <div className="motor-head">
              <span style={{ color: motor.color }}>●</span> {motor.label}
              <b>{formatNumber(telemetry.motorRPMs[i], 0)}</b>
            </div>
            <div className="motor-bar-container">
              <div
                className="motor-bar-fill"
                style={{
                  width: `${Math.min(100, (telemetry.motorRPMs[i] / MAX_RPM) * 100)}%`,
                  background: motor.color,
                }}
              />
            </div>
          </div>
        ))}
      </div>

      {status.engaged && (
        <>
          <div className="section-label">MISSION</div>
          <div className="telemetry-row">
            <span className="telemetry-label">NEXT GATE</span>
            <span>#{status.waypointIndex + 1}</span>
          </div>
          <div className="telemetry-row">
            <span className="telemetry-label">RANGE</span>
            <span>{formatNumber(status.distanceToWaypoint, 1)} m</span>
          </div>
          <div className="telemetry-row">
            <span className="telemetry-label">ALT ERROR</span>
            <span style={{ color: Math.abs(status.altitudeError) > 3 ? 'var(--warning)' : 'var(--success)' }}>
              {status.altitudeError >= 0 ? '+' : ''}{formatNumber(status.altitudeError, 1)} m
            </span>
          </div>
        </>
      )}
    </div>
  );
};
