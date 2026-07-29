import { Vector3 } from 'three';

export const formatNumber = (num: number, decimals: number = 2): string => {
  return num.toFixed(decimals);
};

export const clamp = (val: number, min: number, max: number): number => {
  return Math.max(min, Math.min(max, val));
};

export const lerp = (start: number, end: number, amt: number): number => {
  return (1 - amt) * start + amt * end;
};

export const radToDeg = (rad: number): number => {
  return rad * (180 / Math.PI);
};

export const degToRad = (deg: number): number => {
  return deg * (Math.PI / 180);
};

export const vector3ToString = (vec: Vector3, decimals: number = 2): string => {
  return `${vec.x.toFixed(decimals)}, ${vec.y.toFixed(decimals)}, ${vec.z.toFixed(decimals)}`;
};
