// Sending console commands from the UI: every command goes through run(),
// which records it in the activity list and reports failures as toasts.
// Commands go to the selected hand unless `hand` says otherwise.

import { app, HAND_LABEL, type HandId } from './state.svelte';
import { socket } from './ws';
import type { CommandResult } from './protocol';

export interface RunOptions {
  quiet?: boolean; // no success toast
  successText?: string;
  hand?: HandId;
}

export async function run(text: string, options: RunOptions = {}): Promise<CommandResult> {
  const hand = options.hand ?? app.current;
  const result = await socket.command(text, hand);
  const detail = result.error || result.events.map((e) => e.text).join('\n');
  app.addActivity(hand, text, result.ok, detail);
  const label = `${HAND_LABEL[hand]}: ${text}`;
  if (!result.ok) {
    app.toast('error', `${label}: ${result.error || result.status}`, 6000);
  } else if (!options.quiet) {
    app.toast('success', options.successText ?? `${label}: ok`, 2500);
  }
  return result;
}

export async function runAll(lines: string[], hand?: HandId): Promise<boolean> {
  for (const line of lines) {
    if (!(await run(line, { quiet: true, hand })).ok) return false;
  }
  return true;
}

// Every connected hand at once (Disarm all).
export async function runOnAllHands(text: string): Promise<void> {
  await Promise.all(app.connectedHands.map((hand) => run(text, { quiet: true, hand })));
}

export function estop(): void {
  if (!socket.estop()) app.toast('error', 'E-stop not sent: GUI server unreachable. Press BOOT on the boards!', 8000);
  for (const hand of app.connectedHands) app.addActivity(hand, 'E-STOP', true);
}

export const cmd = {
  arm: (joint?: number) => (joint ? `arm ${joint}` : 'arm'),
  disarm: (joint?: number) => (joint ? `disarm ${joint}` : 'disarm'),
  set: (joint: number, key: string, ...values: (string | number)[]) => `set ${joint} ${key} ${values.join(' ')}`,
  setGlobal: (key: string, value: string | number) => `set ${key} ${value}`,
};
