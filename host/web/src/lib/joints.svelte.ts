// Per-joint view combining HELLO (static), telemetry (fast) and slow status,
// for the selected hand or a given one.

import { app, type HandId } from './state.svelte';
import type { JointInfo, SlowJoint, TelemetryJoint } from './protocol';

export interface JointView {
  hand: HandId;
  index: number;
  info: JointInfo;
  tel: TelemetryJoint | null;
  slow: SlowJoint | null;
}

export function jointViews(hand: HandId = app.current): JointView[] {
  const h = app.hands[hand];
  if (!h.info) return [];
  return h.info.joints.map((j, i) => ({
    hand,
    index: i,
    info: j,
    tel: h.tel?.joints[i] ?? null,
    slow: h.slow?.joints[i] ?? null,
  }));
}

export const STATE_CLASS = ['', 'info', 'ok', 'warn', 'info'];

export function stateName(state: number | undefined, hand: HandId = app.current): string {
  return state === undefined ? '—' : (app.hands[hand].info?.states[state] ?? String(state));
}

export function faultName(fault: number, hand: HandId = app.current): string {
  return app.hands[hand].info?.faults[fault] ?? `fault ${fault}`;
}

export function travelDefined(j: JointInfo): boolean {
  return j.travel_min !== j.travel_max;
}
