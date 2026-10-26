// One WebSocket to teleop-host: both hands' state in, commands / E-stop out.

import { app, HANDS, type HandId } from './state.svelte';
import type { CommandResult, DeviceEvent } from './protocol';

// t_us per hand; series keys are "<hand>.j<id>.<channel>".
export type PlotBatch = { t_us: Partial<Record<HandId, number[]>>; series: Record<string, number[]> };
type PlotListener = (batch: PlotBatch) => void;

const unreachable = (text: string, error = 'GUI server unreachable'): CommandResult => ({
  text,
  ok: false,
  status: 'not_connected',
  error,
  events: [],
  logs: [],
  config: null,
});

const isHand = (h: unknown): h is HandId => h === 'right' || h === 'left';

class Socket {
  private ws: WebSocket | null = null;
  private nextId = 1;
  private pending = new Map<number, (r: CommandResult) => void>();
  private plotListeners = new Set<PlotListener>();
  private plotKeys: string[] = [];
  private retryMs = 500;

  start(): void {
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${scheme}://${location.host}/ws`);
    this.ws = ws;
    ws.onopen = () => {
      app.wsConnected = true;
      this.retryMs = 500;
      if (this.plotKeys.length) this.send({ type: 'plot_subscribe', keys: this.plotKeys });
    };
    ws.onclose = () => {
      app.wsConnected = false;
      for (const resolve of this.pending.values()) resolve(unreachable(''));
      this.pending.clear();
      setTimeout(() => this.start(), this.retryMs);
      this.retryMs = Math.min(this.retryMs * 2, 5000);
    };
    ws.onmessage = (msg) => this.dispatch(JSON.parse(msg.data));
  }

  private send(message: unknown): boolean {
    if (this.ws?.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify(message));
    return true;
  }

  private dispatch(m: { type: string; hand?: string | null; port?: string; data: any; id?: number }): void {
    // Messages from a board whose HELLO has not arrived yet go to the selected hand's log.
    const hand = isHand(m.hand) ? app.hands[m.hand] : null;
    switch (m.type) {
      case 'snapshot':
        app.plugins = m.data.plugins;
        for (const h of HANDS) {
          const s = m.data.hands[h];
          const state = app.hands[h];
          state.link = s.link;
          state.port = s.port;
          state.info = s.info;
          state.slow = s.slow;
          state.tel = s.tel;
          state.events = [];
          state.logs = [];
          state.addEvents(s.events);
          state.addLogs(s.logs.map((text: string) => ({ source: 0, text })));
        }
        break;
      case 'link':
        if (hand) {
          hand.link = m.data;
          hand.port = m.port ?? hand.port;
          if (m.data.state === 'disconnected' && !m.data.port) hand.clear();
        }
        break;
      case 'info':
        if (hand) hand.info = m.data;
        break;
      case 'tel':
        if (hand) hand.tel = m.data;
        break;
      case 'slow':
        if (hand) hand.slow = m.data;
        break;
      case 'event':
        (hand ?? app.hand).addEvents([m.data as DeviceEvent]);
        break;
      case 'log':
        (hand ?? app.hand).addLogs([m.data]);
        break;
      case 'notice':
        app.toast('error', m.data.text, 10000);
        break;
      case 'plot':
        for (const listener of this.plotListeners) listener(m.data);
        break;
      case 'cmd_result': {
        const resolve = this.pending.get(m.id ?? -1);
        this.pending.delete(m.id ?? -1);
        resolve?.(m.data);
        break;
      }
    }
  }

  command(text: string, hand: HandId = app.current): Promise<CommandResult> {
    const id = this.nextId++;
    return new Promise((resolve) => {
      if (!this.send({ type: 'cmd', id, hand, text })) {
        resolve(unreachable(text));
        return;
      }
      this.pending.set(id, resolve);
    });
  }

  // Both hands: the server forwards it to every connected board.
  estop(): boolean {
    return this.send({ type: 'estop' });
  }

  setTelemetryRate(telemetryDivider: number, slowDivider: number): void {
    this.send({ type: 'telemetry_rate', telemetry_divider: telemetryDivider, slow_divider: slowDivider });
  }

  subscribePlot(keys: string[], listener: PlotListener): () => void {
    this.plotKeys = keys;
    this.plotListeners.add(listener);
    this.send({ type: 'plot_subscribe', keys });
    return () => {
      this.plotListeners.delete(listener);
      if (this.plotListeners.size === 0) {
        this.plotKeys = [];
        this.send({ type: 'plot_subscribe', keys: [] });
      }
    };
  }
}

export const socket = new Socket();
