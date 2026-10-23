import type { DeviceInfo, JointInfo } from './protocol';

export const DEG_PER_COUNT = 360 / 16384;

export function exoDeg(info: DeviceInfo | null, counts: number): number {
  return counts * (info?.degrees_per_count ?? DEG_PER_COUNT);
}

export function stepsToDeg(joint: JointInfo, steps: number): number {
  return steps * joint.degrees_per_step;
}

export function fmt(value: number | null | undefined, digits = 0): string {
  return value === null || value === undefined || !Number.isFinite(value) ? '—' : value.toFixed(digits);
}

export function modelShort(model: number): string {
  return model === 2 ? 'HLS' : model === 1 ? 'SCS' : '?';
}
