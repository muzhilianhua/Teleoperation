<script lang="ts">
  import { onDestroy } from 'svelte';
  import Plot from '../lib/components/Plot.svelte';
  import { jointViews } from '../lib/joints.svelte';
  import { app } from '../lib/state.svelte';
  import { socket } from '../lib/ws';

  const PRESETS = [
    { id: 'position', title: 'Servo position', unit: 'steps', channels: ['target', 'goal', 'sent', 'present', 'ref'] },
    { id: 'exo', title: 'Exoskeleton', unit: 'deg from A', channels: ['exo_deg', 'exo_filt_deg'] },
    { id: 'load', title: 'Load', unit: '%', channels: ['load_pct'] },
    { id: 'current', title: 'Current', unit: 'mA', channels: ['current_ma', 'supply_est_ma'] },
    { id: 'grip', title: 'Grip lead', unit: 'steps', channels: ['grip_lead'] },
    { id: 'speed', title: 'Speed command', unit: 'steps/s', channels: ['speed_cmd'] },
    { id: 'temp', title: 'Temperature', unit: '°C', channels: ['temp_c'] },
  ] as const;

  const joints = $derived(jointViews());
  let selectedJoints = $state<number[]>([1]);
  let selectedPresets = $state<string[]>(['position', 'exo', 'load']);
  let windowS = $state(10);
  let paused = $state(false);

  const keys = $derived(
    PRESETS.filter((p) => selectedPresets.includes(p.id)).flatMap((p) =>
      selectedJoints.flatMap((j) => p.channels.map((c) => `j${j}.${c}`)),
    ),
  );

  // Buffers: device time in seconds (micros unwrapped) and one array per key.
  // Raw state: pushes do not re-render (version does), reassignments pass the new arrays down.
  let time = $state.raw<number[]>([]);
  let data = $state.raw<Record<string, number[]>>({});
  let t0: number | null = null;
  let lastUs = 0;
  let wraps = 0;
  let version = $state(0);
  let pending = false;

  function onBatch(batch: { t_us: number[]; series: Record<string, number[]> }) {
    if (paused) return;
    for (const us of batch.t_us) {
      if (us < lastUs) wraps += 1;
      lastUs = us;
      const t = (wraps * 2 ** 32 + us) / 1e6;
      if (t0 === null) t0 = t;
      time.push(t - t0);
    }
    for (const [key, values] of Object.entries(batch.series)) data[key]?.push(...values);
    const keep = time.length - Math.round(windowS * 100);
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
    t0 = null;
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
    <p>Every control cycle (up to 100 Hz) of the selected channels, on the device's clock.</p>
  </div>
  {#if !app.connected}
    <div class="notice">Connect to the device first.</div>
  {:else}
    <div class="panel controls">
      <div class="row">
        <span class="muted">joints</span>
        {#each joints as j (j.info.servo_id)}
          <button class="small {selectedJoints.includes(j.info.servo_id) ? 'primary' : ''}" onclick={() => (selectedJoints = toggle(selectedJoints, j.info.servo_id))}>
            {j.info.servo_id} {j.info.name}
          </button>
        {/each}
      </div>
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
          series={selectedJoints.flatMap((j) => p.channels.map((c) => ({ label: `j${j} ${c}`, data: data[`j${j}.${c}`] ?? [] })))}
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
</style>
