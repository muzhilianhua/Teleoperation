// Per-joint view combining HELLO (static), telemetry (fast) and slow status.

import { app } from './state.svelte';
import type { JointInfo, SlowJoint, TelemetryJoint } from './protocol';

export interface JointView {
  index: number;
  info: JointInfo;
  tel: TelemetryJoint | null;
  slow: SlowJoint | null;
}

export function jointViews(): JointView[] {
  const info = app.info;
  if (!info) return [];
  return info.joints.map((j, i) => ({
    index: i,
    info: j,
    tel: app.tel?.joints[i] ?? null,
    slow: app.slow?.joints[i] ?? null,
  }));
}

export const STATE_CLASS = ['', 'info', 'ok', 'warn', 'info'];

export function stateName(state: number | undefined): string {
  return state === undefined ? '—' : (app.info?.states[state] ?? String(state));
}

export function faultName(fault: number): string {
  return app.info?.faults[fault] ?? `fault ${fault}`;
}

export function travelDefined(j: JointInfo): boolean {
  return j.travel_min !== j.travel_max;
}
