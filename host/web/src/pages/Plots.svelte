<script lang="ts">
  import { onDestroy } from 'svelte';
  import Plot from '../lib/components/Plot.svelte';
  import { jointViews } from '../lib/joints.svelte';
  import { app, HAND_LABEL, HANDS, type HandId } from '../lib/state.svelte';
  import { socket, type PlotBatch } from '../lib/ws';

  const PRESETS = [
    { id: 'position', title: 'Servo position', unit: 'steps', channels: ['target', 'goal', 'sent', 'present', 'ref'] },
    { id: 'exo', title: 'Exoskeleton', unit: 'deg from A', channels: ['exo_deg', 'exo_filt_deg'] },
    { id: 'load', title: 'Load', unit: '%', channels: ['load_pct'] },
    { id: 'current', title: 'Current', unit: 'mA', channels: ['current_ma', 'supply_est_ma'] },
    { id: 'grip', title: 'Grip lead', unit: 'steps', channels: ['grip_lead'] },
    { id: 'speed', title: 'Speed command', unit: 'steps/s', channels: ['speed_cmd'] },
    { id: 'temp', title: 'Temperature', unit: '°C', channels: ['temp_c'] },
    { id: 'supply', title: 'Servo supply', unit: 'V', channels: ['voltage_v'] },
  ] as const;

  // Selected joints as "<hand>.<servo id>", e.g. "right.1".
  let selectedJoints = $state<string[]>([`${app.current}.1`]);
  let selectedPresets = $state<string[]>(['position', 'exo', 'load']);
  let windowS = $state(10);
  let paused = $state(false);

  const handsShown = $derived(HANDS.filter((h) => app.hands[h].info));
  const short = (h: HandId) => (h === 'right' ? 'R' : 'L');
  const keyOf = (sel: string, channel: string) => {
    const [hand, id] = sel.split('.');
    return `${hand}.j${id}.${channel}`;
  };
  const labelOf = (sel: string, channel: string) => {
    const [hand, id] = sel.split('.');
    return `${short(hand as HandId)}${id} ${channel}`;
  };
  const keys = $derived(
    PRESETS.filter((p) => selectedPresets.includes(p.id)).flatMap((p) =>
      selectedJoints.flatMap((j) => p.channels.map((c) => keyOf(j, c))),
    ),
  );

  // One row per sample of either hand. Each hand runs on its own clock, so
  // its time starts at 0 with its first sample after subscribing; the two
  // streams start within one flush (~50 ms) of each other.
  let time = $state.raw<number[]>([]);
  let data = $state.raw<Record<string, (number | null)[]>>({});
  const clock: Partial<Record<HandId, { t0: number; last: number; wraps: number }>> = {};
  let version = $state(0);
  let pending = false;

  function deviceSeconds(hand: HandId, us: number): number {
    const c = (clock[hand] ??= { t0: NaN, last: us, wraps: 0 });
    if (us < c.last) c.wraps += 1;
    c.last = us;
    const t = (c.wraps * 2 ** 32 + us) / 1e6;
    if (Number.isNaN(c.t0)) c.t0 = t;
    return t - c.t0;
  }

  function onBatch(batch: PlotBatch) {
    if (paused) return;
    const rows: { t: number; hand: HandId; i: number }[] = [];
    for (const hand of HANDS) {
      (batch.t_us[hand] ?? []).forEach((us, i) => rows.push({ t: deviceSeconds(hand, us), hand, i }));
    }
    rows.sort((a, b) => a.t - b.t);
    // uPlot needs ascending x: a sample a few ms behind the other hand's last one is drawn at that time.
    let latest = time.length ? time[time.length - 1] : 0;
    for (const row of rows) {
      latest = Math.max(row.t, latest);
      time.push(latest);
      for (const key of Object.keys(data)) {
        data[key].push(key.startsWith(`${row.hand}.`) ? (batch.series[key]?.[row.i] ?? null) : null);
      }
    }
    const keep = time.length - Math.round(windowS * 100 * Math.max(1, handsShown.length));
    if (keep > 0) {
      // Reassigned (not mutated) so the charts receive the trimmed arrays.
      time = time.slice(keep);
      data = Object.fromEntries(Object.entries(data).map(([key, values]) => [key, values.slice(-time.length)]));
    }
    if (!pending) {
      pending = true;
      requestAnimationFrame(() => {
        pending = false;
        version += 1;
      });
    }
  }

  let unsubscribe: (() => void) | null = null;
  $effect(() => {
    const k = keys;
    unsubscribe?.();
    time = [];
    data = Object.fromEntries(k.map((key) => [key, []]));  // stable arrays the charts can hold on to
    for (const hand of HANDS) delete clock[hand];
    unsubscribe = socket.subscribePlot(k, onBatch);
  });
  onDestroy(() => unsubscribe?.());

  function toggle<T>(list: T[], item: T): T[] {
    return list.includes(item) ? list.filter((x) => x !== item) : [...list, item];
  }

  function exportCsv() {
    const cols = Object.keys(data);
    const rows = [['t_s', ...cols].join(',')];
    for (let i = 0; i < time.length; i++) rows.push([time[i].toFixed(4), ...cols.map((c) => data[c][i] ?? '')].join(','));
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([rows.join('\n')], { type: 'text/csv' }));
    a.download = `teleop-plot-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
    a.click();
  }
</script>

<div class="page">
  <div class="page-header">
    <h1>Plots</h1>
    <p>Every control cycle (up to 100 Hz) of the selected channels. Each hand is drawn on its own board's clock, both starting at 0 when the selection changes.</p>
  </div>
  {#if !handsShown.length || !app.wsConnected}
    <div class="notice">Connect to the device first.</div>
  {:else}
    <div class="panel controls">
      {#each handsShown as hand (hand)}
        <div class="row">
          <span class="muted hand">{HAND_LABEL[hand]}</span>
          {#each jointViews(hand) as j (j.info.servo_id)}
            {@const sel = `${hand}.${j.info.servo_id}`}
            <button class="small {selectedJoints.includes(sel) ? 'primary' : ''}" onclick={() => (selectedJoints = toggle(selectedJoints, sel))}>
              {j.info.servo_id} {j.info.name}
            </button>
          {/each}
        </div>
      {/each}
      <div class="row">
        <span class="muted">charts</span>
        {#each PRESETS as p}
          <button class="small {selectedPresets.includes(p.id) ? 'primary' : ''}" onclick={() => (selectedPresets = toggle(selectedPresets, p.id))}>{p.title}</button>
        {/each}
        <span class="spacer"></span>
        <select bind:value={windowS}>
          {#each [5, 10, 30, 60] as w}<option value={w}>{w} s</option>{/each}
        </select>
        <button onclick={() => (paused = !paused)}>{paused ? 'Resume' : 'Pause'}</button>
        <button onclick={exportCsv}>Export CSV</button>
      </div>
    </div>
    {#each PRESETS.filter((p) => selectedPresets.includes(p.id)) as p (p.id)}
      <div class="panel">
        <Plot
          title={p.title}
          unit={p.unit}
          {time}
          series={selectedJoints.flatMap((j) => p.channels.map((c) => ({ label: labelOf(j, c), data: data[keyOf(j, c)] ?? [] })))}
          {version}
        />
      </div>
    {/each}
  {/if}
</div>

<style>
  .controls {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .spacer {
    flex: 1;
  }
  .hand {
    width: 42px;
  }
</style>
