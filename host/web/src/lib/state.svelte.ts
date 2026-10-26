// Global reactive state for both hands, fed by the WebSocket (ws.ts).
// Pages work on the selected hand through the `app.info` / `app.tel` / ...
// shortcuts; `app.hands.right|left` reach a specific one. Telemetry and slow
// status are replaced wholesale ($state.raw) at up to 30 Hz per hand.

import type { DeviceEvent, DeviceInfo, LinkInfo, PluginInfo, PowerWindow, SlowStatus, Telemetry, TripRecord } from './protocol';

export type HandId = 'right' | 'left';
export const HANDS: HandId[] = ['right', 'left'];
export const HAND_LABEL: Record<HandId, string> = { right: 'Right', left: 'Left' };

export interface LogLine {
  id: number;
  source: number; // 0 console, 1 esp log, 2 boot, -1 unframed text
  text: string;
  time: number;
}

export interface Toast {
  id: number;
  kind: 'info' | 'error' | 'success';
  text: string;
}

export interface ActivityEntry {
  id: number;
  time: number;
  hand: HandId;
  text: string;
  ok: boolean;
  detail: string;
}

const MAX_EVENTS = 1000;
const MAX_LOGS = 1000;
const DISCONNECTED: LinkInfo = { state: 'disconnected', port: '', detail: '', firmware: '' };

let nextId = 1;

export class HandState {
  link = $state<LinkInfo | null>(null); // null: no board bound to this hand
  port = $state('');
  info = $state.raw<DeviceInfo | null>(null);
  tel = $state.raw<Telemetry | null>(null);
  slow = $state.raw<SlowStatus | null>(null);
  events = $state.raw<(DeviceEvent & { id: number; time: number })[]>([]);
  logs = $state.raw<LogLine[]>([]);

  get connected(): boolean {
    return this.link?.state === 'connected';
  }

  get armedCount(): number {
    return this.connected ? (this.tel?.armed_count ?? 0) : 0;
  }

  addEvents(events: DeviceEvent[]): void {
    const now = Date.now();
    const added = events.map((e) => ({ ...e, id: nextId++, time: now }));
    this.events = [...this.events, ...added].slice(-MAX_EVENTS);
  }

  addLogs(lines: { source: number; text: string }[]): void {
    const now = Date.now();
    const added = lines.map((l) => ({ ...l, id: nextId++, time: now }));
    this.logs = [...this.logs, ...added].slice(-MAX_LOGS);
  }

  clear(): void {
    this.link = null;
    this.port = '';
    this.info = this.tel = this.slow = null;
  }
}

function loadCurrent(): HandId {
  try {
    return localStorage.getItem('teleop.hand') === 'left' ? 'left' : 'right';
  } catch {
    return 'right';
  }
}

class AppState {
  wsConnected = $state(false);
  hands: Record<HandId, HandState> = { right: new HandState(), left: new HandState() };
  current = $state<HandId>(loadCurrent());
  plugins = $state.raw<PluginInfo[]>([]);
  power = $state.raw<Partial<Record<HandId, PowerWindow>>>({});
  trips = $state.raw<TripRecord[]>([]); // newest first
  toasts = $state<Toast[]>([]);
  activity = $state.raw<ActivityEntry[]>([]);

  select(hand: HandId): void {
    this.current = hand;
    try {
      localStorage.setItem('teleop.hand', hand);
    } catch {
      /* storage unavailable: selection is per page load */
    }
  }

  // ---- the selected hand -------------------------------------------------------
  get hand(): HandState {
    return this.hands[this.current];
  }
  get link(): LinkInfo {
    return this.hand.link ?? DISCONNECTED;
  }
  get info(): DeviceInfo | null {
    return this.hand.info;
  }
  get tel(): Telemetry | null {
    return this.hand.tel;
  }
  get slow(): SlowStatus | null {
    return this.hand.slow;
  }
  get events() {
    return this.hand.events;
  }
  get logs(): LogLine[] {
    return this.hand.logs;
  }
  get connected(): boolean {
    return this.wsConnected && this.hand.connected;
  }
  get armedCount(): number {
    return this.hand.armedCount;
  }
  addEvents(events: DeviceEvent[]): void {
    this.hand.addEvents(events);
  }
  addLogs(lines: { source: number; text: string }[]): void {
    this.hand.addLogs(lines);
  }

  // ---- both hands -----------------------------------------------------------------
  get connectedHands(): HandId[] {
    return this.wsConnected ? HANDS.filter((h) => this.hands[h].connected) : [];
  }
  get armedTotal(): number {
    return HANDS.reduce((n, h) => n + this.hands[h].armedCount, 0);
  }

  addActivity(hand: HandId, text: string, ok: boolean, detail = ''): void {
    this.activity = [{ id: nextId++, time: Date.now(), hand, text, ok, detail }, ...this.activity].slice(0, 200);
  }

  toast(kind: Toast['kind'], text: string, ms = 4000): void {
    const id = nextId++;
    this.toasts.push({ id, kind, text });
    setTimeout(() => {
      this.toasts = this.toasts.filter((t) => t.id !== id);
    }, ms);
  }
}

export const app = new AppState();
