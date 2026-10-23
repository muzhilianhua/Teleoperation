// Sending console commands from the UI: every command goes through run(),
// which records it in the activity list and reports failures as toasts.

import { app } from './state.svelte';
import { socket } from './ws';
import type { CommandResult } from './protocol';

export interface RunOptions {
  quiet?: boolean; // no success toast
  successText?: string;
}

export async function run(text: string, options: RunOptions = {}): Promise<CommandResult> {
  const result = await socket.command(text);
  const detail = result.error || result.events.map((e) => e.text).join('\n');
  app.addActivity(text, result.ok, detail);
  if (!result.ok) {
    app.toast('error', `${text}: ${result.error || result.status}`, 6000);
  } else if (!options.quiet) {
    app.toast('success', options.successText ?? `${text}: ok`, 2500);
  }
  return result;
}

export async function runAll(lines: string[]): Promise<boolean> {
  for (const line of lines) {
    if (!(await run(line, { quiet: true })).ok) return false;
  }
  return true;
}

export function estop(): void {
  if (!socket.estop()) app.toast('error', 'E-stop not sent: GUI server unreachable. Press BOOT on the board!', 8000);
  app.addActivity('E-STOP', true);
}

export const cmd = {
  arm: (joint?: number) => (joint ? `arm ${joint}` : 'arm'),
  disarm: (joint?: number) => (joint ? `disarm ${joint}` : 'disarm'),
  set: (joint: number, key: string, ...values: (string | number)[]) => `set ${joint} ${key} ${values.join(' ')}`,
  setGlobal: (key: string, value: string | number) => `set ${key} ${value}`,
};
