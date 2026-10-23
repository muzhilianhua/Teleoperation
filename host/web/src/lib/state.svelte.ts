// Global reactive state, fed by the WebSocket (ws.ts). Telemetry and slow
// status are replaced wholesale ($state.raw) at up to 30 Hz.

import type {
  DeviceEvent,
  DeviceInfo,
  LinkInfo,
  PluginInfo,
  SlowStatus,
  Telemetry,
} from './protocol';

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
  text: string;
  ok: boolean;
  detail: string;
}

const MAX_EVENTS = 1000;
const MAX_LOGS = 1000;

let nextId = 1;

class AppState {
  wsConnected = $state(false);
  link = $state<LinkInfo>({ state: 'disconnected', port: '', detail: '', firmware: '' });
  info = $state.raw<DeviceInfo | null>(null);
  tel = $state.raw<Telemetry | null>(null);
  slow = $state.raw<SlowStatus | null>(null);
  events = $state.raw<(DeviceEvent & { id: number; time: number })[]>([]);
  logs = $state.raw<LogLine[]>([]);
  plugins = $state.raw<PluginInfo[]>([]);
  toasts = $state<Toast[]>([]);
  activity = $state.raw<ActivityEntry[]>([]);

  get connected(): boolean {
    return this.wsConnected && this.link.state === 'connected';
  }

  get armedCount(): number {
    return this.tel?.armed_count ?? 0;
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

  addActivity(text: string, ok: boolean, detail = ''): void {
    this.activity = [{ id: nextId++, time: Date.now(), text, ok, detail }, ...this.activity].slice(0, 200);
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
