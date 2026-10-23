import { describe, expect, it } from 'vitest';
import { jointRanges } from './limits';
import type { DeviceInfo, JointInfo } from './protocol';
import { exoDeg, fmt, modelShort } from './units';

const hls: JointInfo = {
  servo_id: 2, model: 2, travel_min: 800, travel_max: 1900, vmax: 1333, vsat: 167, amax: 26666, max_speed: 12492,
  encoder_default: 2, enabled_default: 0, step_scale: 300 / 1024 / (360 / 4096), degrees_per_step: 360 / 4096, name: 'j2',
};
const info = { max_current_limit_ma: 13306, max_current_trip_pct: 100, degrees_per_count: 360 / 16384 } as DeviceInfo;

describe('jointRanges', () => {
  it('scales step-based limits like teleop::validateJoint', () => {
    const r = jointRanges(hls, info);
    expect(r.vmax).toEqual({ min: 1, max: 12492 });
    expect(r.trk.max).toBe(3333);
    expect(r.grip.max).toBe(1333);
    expect(r.amax.max).toBe(65000);
  });
});

describe('units', () => {
  it('converts and formats', () => {
    expect(exoDeg(info, 16384)).toBeCloseTo(360);
    expect(fmt(NaN)).toBe('—');
    expect(fmt(1.234, 1)).toBe('1.2');
    expect(modelShort(2)).toBe('HLS');
  });
});
