// Pure helpers for the Teach page.

import type { Marker, PlayerState } from './protocol';

export const REPLAY_SPEEDS = [0.25, 0.5, 0.75, 1];

export function clock(seconds: number): string {
  const s = Math.max(0, seconds);
  const m = Math.floor(s / 60);
  return `${m}:${(s - 60 * m).toFixed(1).padStart(4, '0')}`;
}

// The last marker at or before `t` (recording time).
export function markerAt(markers: Marker[], t: number): Marker | null {
  let found: Marker | null = null;
  for (const m of markers) if (m.t <= t) found = m;
  return found;
}

export const replayBusy = (state: PlayerState | undefined) =>
  state === 'preparing' || state === 'ready' || state === 'playing';
