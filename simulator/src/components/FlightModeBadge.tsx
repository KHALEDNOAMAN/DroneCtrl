import React from 'react';
import { AutopilotStatus, FlightMode, GameState } from '../types';

interface Props {
  mode: FlightMode;
  status: AutopilotStatus;
  windEnabled: boolean;
  cameraMode: GameState['cameraMode'];
  showPID: boolean;
  onToggleMode: () => void;
  onTogglePID: () => void;
}

const CAMERA_LABEL: Record<GameState['cameraMode'], string> = {
  chase: 'CHASE',
  cinematic: 'CINEMATIC',
  fpv: 'FPV',
  'top-down': 'TOP-DOWN',
};

export const FlightModeBadge: React.FC<Props> = ({
  mode,
  status,
  windEnabled,
  cameraMode,
  showPID,
  onToggleMode,
  onTogglePID,
}) => {
  const auto = mode === 'AUTO';

  return (
    <div className="mode-cluster">
      <button
        className={`mode-badge ${auto ? 'mode-badge--auto' : 'mode-badge--manual'}`}
        onClick={onToggleMode}
        title="Toggle autopilot (M)"
      >
        <span className="mode-dot" />
        <span className="mode-text">{auto ? 'AUTOPILOT' : 'MANUAL'}</span>
        <span className="mode-phase">{auto ? status.phase : 'PILOT'}</span>
      </button>

      <div className="mode-meta glass-panel">
        <div className="mode-meta-row">
          <span>CAM</span>
          <b>{CAMERA_LABEL[cameraMode]}</b>
        </div>
        <div className="mode-meta-row">
          <span>WIND</span>
          <b className={windEnabled ? 'value-warn' : ''}>{windEnabled ? 'GUSTING' : 'CALM'}</b>
        </div>
        {auto && (
          <div className="mode-meta-row">
            <span>LAP</span>
            <b>{status.lap + 1}</b>
          </div>
        )}
      </div>

      <button className="ghost-btn" onClick={onTogglePID} title="PID tuning (P)">
        {showPID ? 'Hide PID' : 'Tune PID'}
      </button>
    </div>
  );
};
