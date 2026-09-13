import React from 'react';
import { AutopilotStatus, GameState } from '../types';

interface Props {
  gameState: GameState;
  status: AutopilotStatus;
  totalGates: number;
}

export const ScoreDisplay: React.FC<Props> = ({ gameState, status, totalGates }) => {
  const gate = gameState.checkpointsHit;

  return (
    <div className="score-display">
      <div className="score-value">
        {String(gate + 1).padStart(2, '0')}
        <span className="score-total">/{totalGates}</span>
      </div>
      <div className="score-sub">GATE · LAP {gameState.lap + 1}</div>

      <div className="gate-pips">
        {Array.from({ length: totalGates }).map((_, i) => (
          <span
            key={i}
            className={`gate-pip ${i < gate ? 'gate-pip--done' : i === gate ? 'gate-pip--active' : ''}`}
          />
        ))}
      </div>

      {status.engaged && (
        <div className="score-range">{status.distanceToWaypoint.toFixed(0)} m to gate</div>
      )}

      {gameState.crashCount > 0 && (
        <div className="score-crash">GROUND CONTACT × {gameState.crashCount}</div>
      )}
    </div>
  );
};
