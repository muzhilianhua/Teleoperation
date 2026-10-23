// One WebSocket to teleop-host: state updates in, commands / E-stop out.

import { app } from './state.svelte';
import type { CommandResult, DeviceEvent } from './protocol';

type PlotBatch = { t_us: number[]; series: Record<string, number[]> };
type PlotListener = (batch: PlotBatch) => void;

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
      for (const resolve of this.pending.values()) {
        resolve({ text: '', ok: false, status: 'not_connected', error: 'GUI server unreachable', events: [], logs: [], config: null });
      }
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

  private dispatch(m: { type: string; data: any; id?: number }): void {
    switch (m.type) {
      case 'snapshot':
        app.link = m.data.link;
        app.info = m.data.info;
        app.slow = m.data.slow;
        app.tel = m.data.tel;
        app.plugins = m.data.plugins;
        app.events = [];
        app.logs = [];
        app.addEvents(m.data.events);
        app.addLogs(m.data.logs.map((text: string) => ({ source: 0, text })));
        break;
      case 'link':
        app.link = m.data;
        break;
      case 'info':
        app.info = m.data;
        break;
      case 'tel':
        app.tel = m.data;
        break;
      case 'slow':
        app.slow = m.data;
        break;
      case 'event':
        app.addEvents([m.data as DeviceEvent]);
        break;
      case 'log':
        app.addLogs([m.data]);
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

  command(text: string): Promise<CommandResult> {
    const id = this.nextId++;
    return new Promise((resolve) => {
      if (!this.send({ type: 'cmd', id, text })) {
        resolve({ text, ok: false, status: 'not_connected', error: 'GUI server unreachable', events: [], logs: [], config: null });
        return;
      }
      this.pending.set(id, resolve);
    });
  }

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
