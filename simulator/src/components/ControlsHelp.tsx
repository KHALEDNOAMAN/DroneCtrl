import React from 'react';
import { FlightMode } from '../types';

interface Props {
  mode: FlightMode;
}

const FLIGHT_BINDINGS: [string, string][] = [
  ['Throttle', 'Space / Shift'],
  ['Pitch', 'W / S'],
  ['Roll', 'A / D'],
  ['Yaw', 'Q / E'],
];

const SYSTEM_BINDINGS: [string, string][] = [
  ['Autopilot', 'M'],
  ['Camera', 'C'],
  ['Wind', 'R'],
  ['PID tuning', 'P'],
  ['Hide help', 'H'],
];

export const ControlsHelp: React.FC<Props> = ({ mode }) => {
  return (
    <div className="controls-help glass-panel">
      <h4>CONTROLS</h4>

      {mode === 'AUTO' && (
        <div className="takeover-hint">Touch any stick to take over</div>
      )}

      {FLIGHT_BINDINGS.map(([action, key]) => (
        <div className="key-binding" key={action}>
          <span>{action}</span>
          <span className="key-cap">{key}</span>
        </div>
      ))}

      <div className="help-divider" />

      {SYSTEM_BINDINGS.map(([action, key]) => (
        <div className="key-binding" key={action}>
          <span>{action}</span>
          <span className="key-cap">{key}</span>
        </div>
      ))}
    </div>
  );
};
