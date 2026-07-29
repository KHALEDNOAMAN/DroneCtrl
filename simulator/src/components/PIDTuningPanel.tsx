import React from 'react';
import { PIDGains } from '../types';
import { DEFAULT_ROLL_PID, DEFAULT_PITCH_PID, DEFAULT_YAW_PID } from '../utils/constants';
import { formatNumber } from '../utils/helpers';

interface Props {
  rollPID: PIDGains;
  pitchPID: PIDGains;
  yawPID: PIDGains;
  onChange: (axis: 'roll' | 'pitch' | 'yaw', gains: PIDGains) => void;
  onReset: () => void;
}

const PIDSection: React.FC<{
  title: string;
  gains: PIDGains;
  onChange: (gains: PIDGains) => void;
}> = ({ title, gains, onChange }) => {
  return (
    <div className="pid-section">
      <h3>{title}</h3>
      <div className="slider-group">
        <div className="slider-header">
          <span>Kp</span>
          <span>{formatNumber(gains.kp, 2)}</span>
        </div>
        <input 
          type="range" className="pid-slider" min="0" max="5" step="0.01" 
          value={gains.kp} onChange={(e) => onChange({...gains, kp: parseFloat(e.target.value)})} 
        />
      </div>
      <div className="slider-group">
        <div className="slider-header">
          <span>Ki</span>
          <span>{formatNumber(gains.ki, 2)}</span>
        </div>
        <input 
          type="range" className="pid-slider" min="0" max="2" step="0.01" 
          value={gains.ki} onChange={(e) => onChange({...gains, ki: parseFloat(e.target.value)})} 
        />
      </div>
      <div className="slider-group">
        <div className="slider-header">
          <span>Kd</span>
          <span>{formatNumber(gains.kd, 2)}</span>
        </div>
        <input 
          type="range" className="pid-slider" min="0" max="2" step="0.01" 
          value={gains.kd} onChange={(e) => onChange({...gains, kd: parseFloat(e.target.value)})} 
        />
      </div>
    </div>
  );
};

export const PIDTuningPanel: React.FC<Props> = ({ rollPID, pitchPID, yawPID, onChange, onReset }) => {
  return (
    <div className="pid-panel glass-panel">
      <PIDSection title="Roll PID" gains={rollPID} onChange={(gains) => onChange('roll', gains)} />
      <PIDSection title="Pitch PID" gains={pitchPID} onChange={(gains) => onChange('pitch', gains)} />
      <PIDSection title="Yaw PID" gains={yawPID} onChange={(gains) => onChange('yaw', gains)} />
      <button className="reset-btn" onClick={onReset}>Reset to Defaults</button>
    </div>
  );
};
