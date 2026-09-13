import React from 'react';
import { TelemetryData } from '../types';

interface Props {
  telemetry: TelemetryData;
}

const SIZE = 116;
const RADIUS = SIZE / 2 - 3;

/**
 * Artificial horizon, the way a ground station draws one: the horizon line
 * rolls opposite to the airframe and slides vertically with pitch, so the
 * little aircraft glyph in the middle stays fixed and the world moves.
 */
export const AttitudeIndicator: React.FC<Props> = ({ telemetry }) => {
  const rollDeg = (telemetry.roll * 180) / Math.PI;
  const pitchDeg = (telemetry.pitch * 180) / Math.PI;

  // Horizon travel per degree of pitch, clamped so an aggressive attitude does
  // not slide the ground band completely out of the instrument.
  const PIXELS_PER_DEGREE = 1.3;
  const pitchOffset = Math.max(-RADIUS, Math.min(RADIUS, pitchDeg * PIXELS_PER_DEGREE));

  return (
    <div className="attitude-indicator">
      <svg width={SIZE} height={SIZE} viewBox={`0 0 ${SIZE} ${SIZE}`}>
        <defs>
          <clipPath id="adi-clip">
            <circle cx={SIZE / 2} cy={SIZE / 2} r={RADIUS} />
          </clipPath>
          <linearGradient id="adi-sky" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#0e7490" />
            <stop offset="100%" stopColor="#22d3ee" />
          </linearGradient>
          <linearGradient id="adi-ground" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#a16207" />
            <stop offset="100%" stopColor="#422006" />
          </linearGradient>
        </defs>

        <g clipPath="url(#adi-clip)">
          <g transform={`rotate(${-rollDeg} ${SIZE / 2} ${SIZE / 2}) translate(0 ${pitchOffset})`}>
            <rect x={-SIZE} y={-SIZE} width={SIZE * 3} height={SIZE * 1.5 + SIZE / 2} fill="url(#adi-sky)" />
            <rect x={-SIZE} y={SIZE / 2} width={SIZE * 3} height={SIZE * 2} fill="url(#adi-ground)" />
            <line x1={-SIZE} y1={SIZE / 2} x2={SIZE * 2} y2={SIZE / 2} stroke="#f8fafc" strokeWidth="1.6" />

            {/* Pitch ladder, every 10 degrees */}
            {[-30, -20, -10, 10, 20, 30].map(deg => {
              const y = SIZE / 2 - deg * 2;
              const half = deg % 20 === 0 ? 17 : 10;
              return (
                <line
                  key={deg}
                  x1={SIZE / 2 - half}
                  y1={y}
                  x2={SIZE / 2 + half}
                  y2={y}
                  stroke="#e2e8f0"
                  strokeWidth="1"
                  opacity="0.75"
                />
              );
            })}
          </g>
        </g>

        {/* Fixed aircraft reference */}
        <g stroke="#fbbf24" strokeWidth="2.4" fill="none" strokeLinecap="round">
          <line x1={SIZE / 2 - 26} y1={SIZE / 2} x2={SIZE / 2 - 9} y2={SIZE / 2} />
          <line x1={SIZE / 2 + 9} y1={SIZE / 2} x2={SIZE / 2 + 26} y2={SIZE / 2} />
        </g>
        <circle cx={SIZE / 2} cy={SIZE / 2} r="2.2" fill="#fbbf24" />

        {/* Bezel and roll pointer */}
        <circle cx={SIZE / 2} cy={SIZE / 2} r={RADIUS} fill="none" stroke="rgba(255,255,255,0.28)" strokeWidth="2" />
        <polygon
          points={`${SIZE / 2},${SIZE / 2 - RADIUS + 2} ${SIZE / 2 - 5},${SIZE / 2 - RADIUS + 11} ${SIZE / 2 + 5},${SIZE / 2 - RADIUS + 11}`}
          fill="#fbbf24"
          transform={`rotate(${-rollDeg} ${SIZE / 2} ${SIZE / 2})`}
        />
      </svg>

      <div className="attitude-readout">
        <span>ROLL {rollDeg >= 0 ? '+' : ''}{rollDeg.toFixed(0)}°</span>
        <span>PITCH {pitchDeg >= 0 ? '+' : ''}{pitchDeg.toFixed(0)}°</span>
      </div>
    </div>
  );
};
