import React, { useState, useEffect } from 'react';

export const ControlsHelp: React.FC = () => {
  const [visible, setVisible] = useState(true);

  useEffect(() => {
    const handleKey = (e: KeyboardEvent) => {
      if (e.key === '?') setVisible(v => !v);
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, []);

  if (!visible) return null;

  return (
    <div className="controls-help glass-panel">
      <h4>FLIGHT CONTROLS</h4>
      <div className="key-binding">
        <span>Throttle Up</span> <span className="key-cap">Space</span>
      </div>
      <div className="key-binding">
        <span>Throttle Down</span> <span className="key-cap">Shift</span>
      </div>
      <div className="key-binding">
        <span>Pitch (Fwd/Back)</span> <span className="key-cap">W / S</span>
      </div>
      <div className="key-binding">
        <span>Roll (Left/Right)</span> <span className="key-cap">A / D</span>
      </div>
      <div className="key-binding">
        <span>Yaw (Rotate)</span> <span className="key-cap">Q / E</span>
      </div>
      <div style={{ borderTop: '1px solid rgba(255,255,255,0.1)', margin: '10px 0' }} />
      <div className="key-binding">
        <span>Toggle Camera</span> <span className="key-cap">C</span>
      </div>
      <div className="key-binding">
        <span>Toggle Wind</span> <span className="key-cap">R</span>
      </div>
      <div className="key-binding">
        <span>Hide Help</span> <span className="key-cap">?</span>
      </div>
    </div>
  );
};
