// Client-side hints mirroring teleop::validateJoint / validateGlobal
// (lib/TeleopCore/src/joint_config.cpp). The firmware stays the authority.

import type { DeviceInfo, JointInfo } from './protocol';

export interface Range {
  min: number;
  max: number;
}

export function jointRanges(j: JointInfo, info: DeviceInfo): Record<string, Range> {
  const s = j.step_scale;
  return {
    vmax: { min: 1, max: j.max_speed },
    vsat: { min: 1, max: j.max_speed },
    amax: { min: 10, max: Math.round(Math.min(20000 * s, 65000)) },
    trk: { min: 5, max: Math.round(1000 * s) },
    grip: { min: 0, max: Math.round(400 * s) },
    ilimit: { min: 0, max: info.max_current_limit_ma },
    itrip: { min: 0, max: info.max_current_trip_pct },
  };
}

export const GLOBAL_PARAMS = [
  { key: 'lead', field: 'lead_ms', label: 'Look-ahead', unit: 'ms', min: 0, max: 300, step: 1 },
  { key: 'mincut', field: 'min_cutoff_hz', label: 'One-Euro min cutoff', unit: 'Hz', min: 0.01, max: 30, step: 0.01 },
  { key: 'beta', field: 'beta', label: 'One-Euro beta', unit: '1/deg', min: 0, max: 10, step: 0.001 },
  { key: 'jump', field: 'jump_deg', label: 'Encoder jump guard', unit: 'deg/cycle', min: 1, max: 180, step: 1 },
  { key: 'gripload', field: 'grip_load_pct', label: 'Grip hold target', unit: '% load / % of HLS cap', min: 20, max: 95, step: 1 },
] as const;
