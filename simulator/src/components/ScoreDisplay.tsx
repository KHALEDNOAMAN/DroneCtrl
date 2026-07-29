import React from 'react';
import { GameState } from '../types';

interface Props {
  gameState: GameState;
}

export const ScoreDisplay: React.FC<Props> = ({ gameState }) => {
  return (
    <div className="score-display glass-panel" style={{ background: 'transparent', border: 'none', boxShadow: 'none' }}>
      <div className="score-value">{gameState.checkpointsHit} / 10</div>
      <div className="score-sub">CHECKPOINTS</div>
      {gameState.crashCount > 0 && (
        <div style={{ color: 'var(--danger)', fontSize: 12, marginTop: 8 }}>
          CRASHES: {gameState.crashCount}
        </div>
      )}
    </div>
  );
};
