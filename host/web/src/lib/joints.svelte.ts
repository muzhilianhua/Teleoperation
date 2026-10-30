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

// The robot's stored travel (`set <j> travel`); config.h's default until the
// first slow status arrives.
export function travelOf(v: JointView): { min: number; max: number } {
  // A server or firmware from before the stored travel sends none: show the default.
  return v.slow && v.slow.travel_min !== undefined
    ? { min: v.slow.travel_min, max: v.slow.travel_max }
    : { min: v.info.travel_min, max: v.info.travel_max };
}

export function travelDefined(v: JointView): boolean {
  const t = travelOf(v);
  return t.min < t.max;
}

// Margin kept inside a mechanical end: ~15 SCS0009 steps, ~50 HLS3606 steps.
export function travelMargin(j: JointInfo): number {
  return Math.round(15 * j.step_scale);
}
