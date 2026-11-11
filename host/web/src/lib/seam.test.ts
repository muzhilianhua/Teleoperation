import { describe, expect, it } from 'vitest';
import { MODEL_HLS, MODEL_SCS } from './protocol';
import { centerError, positionCenter, seamJump, seamMargins } from './seam';

describe('seam', () => {
  it('detects a reading that wraps at the seam', () => {
    expect(seamJump(1020, 3, MODEL_SCS)).toBe(true);
    expect(seamJump(500, 560, MODEL_SCS)).toBe(false);
    expect(seamJump(4090, 10, MODEL_HLS)).toBe(true);
    expect(seamJump(1000, 2000, MODEL_HLS)).toBe(false);
  });

  it('measures how far a travel sits from the centre and the seam', () => {
    expect(positionCenter(MODEL_SCS)).toBe(512);
    expect(centerError(300, 900, MODEL_SCS)).toBe(88);
    expect(centerError(1048, 3048, MODEL_HLS)).toBe(0);
    expect(seamMargins(30, 1000, MODEL_SCS)).toEqual({ low: 30, high: 23 });
    expect(seamMargins(100, 4000, MODEL_HLS)).toEqual({ low: 100, high: 95 });
  });
});
