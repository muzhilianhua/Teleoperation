// The seam of a servo's position reading: SCS0009 1023 -> 0 (beyond it ~60 deg
// without feedback, so a travel across it cannot be controlled: the horn must
// be re-seated), HLS3606 4095 -> 0 (movable with the position offset, `zero <j>`).

import { MODEL_HLS } from './protocol';

export function positionRange(model: number): number {
  return model === MODEL_HLS ? 4096 : 1024;
}

// Reading a joint is re-seated or zeroed to: the middle of the range.
export function positionCenter(model: number): number {
  return positionRange(model) / 2;
}

// Between two consecutive readings a joint moved by hand covers far less than
// half the range; a bigger step is the reading wrapping at the seam.
export function seamJump(prev: number, next: number, model: number): boolean {
  return Math.abs(next - prev) > positionRange(model) / 2;
}

// Steps from the travel's middle to the range's middle (positive: above it).
export function centerError(min: number, max: number, model: number): number {
  return (min + max) / 2 - positionCenter(model);
}

// Steps between each end of the travel and the seam.
export function seamMargins(min: number, max: number, model: number): { low: number; high: number } {
  return { low: min, high: positionRange(model) - 1 - max };
}
