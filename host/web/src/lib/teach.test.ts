import { describe, expect, it } from 'vitest';
import { clock, markerAt, replayBusy } from './teach';

describe('teach helpers', () => {
  it('formats elapsed time', () => {
    expect(clock(0)).toBe('0:00.0');
    expect(clock(65.25)).toBe('1:05.3');
    expect(clock(-3)).toBe('0:00.0');
  });
  it('finds the current marker', () => {
    const markers = [{ t: 1, label: 'reach' }, { t: 2.5, label: 'grasp' }];
    expect(markerAt(markers, 0.5)).toBeNull();
    expect(markerAt(markers, 2.5)?.label).toBe('grasp');
    expect(markerAt(markers, 1.7)?.label).toBe('reach');
  });
  it('knows when a replay holds the hands', () => {
    expect(replayBusy('ready')).toBe(true);
    expect(replayBusy('done')).toBe(false);
  });
});
