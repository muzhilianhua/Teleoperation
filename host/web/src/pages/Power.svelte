<script lang="ts">
  import { onDestroy } from 'svelte';
  import Plot from '../lib/components/Plot.svelte';
  import type { PowerWindow } from '../lib/protocol';
  import { app, HAND_LABEL, HANDS, type HandId } from '../lib/state.svelte';
  import { fmt } from '../lib/units';

  // The bench supply's settings (this browser only).
  function stored(key: string, fallback: number): number {
    try {
      const v = Number(localStorage.getItem(key));
      return Number.isFinite(v) && v > 0 ? v : fallback;
    } catch {
      return fallback;
    }
  }
  let limitA = $state(stored('teleop.psu.limit', 5.2));
  let shared = $state(stored('teleop.psu.shared', 1) === 1);
  $effect(() => {
    try {
      localStorage.setItem('teleop.psu.limit', String(limitA));
      localStorage.setItem('teleop.psu.shared', shared ? '1' : '0');
    } catch {
      /* not kept across reloads */
    }
  });

  const SUPPLY_MIN_V = 5.5; // firmware supply window
  const w = (hand: HandId): PowerWindow | undefined => app.power[hand];
  const totalPeak = $derived(HANDS.reduce((a, h) => a + (w(h)?.peak_supply_a ?? 0), 0));
  const budget = (peak: number) => (shared ? totalPeak : peak);
  const level = (peak: number) => (peak > limitA ? 'err' : peak > 0.8 * limitA ? 'warn' : 'ok');
  const voltClass = (v: number | null | undefined) =>
    v == null ? '' : v < SUPPLY_MIN_V ? 'err' : v < SUPPLY_MIN_V + 0.2 ? 'warn' : 'ok';

  // A minute of the 5 Hz power windows, for the charts.
  const HISTORY = 300;
  let time = $state.raw<number[]>([]);
  let series = $state.raw<Record<string, (number | null)[]>>({});
  let version = $state(0);
  const t0 = Date.now();
  const timer = setInterval(() => {
    const t = (Date.now() - t0) / 1000;
    const next: Record<string, (number | null)[]> = {};
    const add = (k: string, v: number | null | undefined) => (next[k] = [...(series[k] ?? []), v ?? null].slice(-HISTORY));
    add('right.median', w('right')?.now_median_v);
    add('right.min', w('right')?.min_v);
    add('left.median', w('left')?.now_median_v);
    add('left.min', w('left')?.min_v);
    add('right.supply', w('right')?.peak_supply_a);
    add('left.supply', w('left')?.peak_supply_a);
    add('total', totalPeak);
    add('limit', limitA);
    time = [...time, t].slice(-HISTORY);
    series = next;
    version += 1;
  }, 200);
  onDestroy(() => clearInterval(timer));

  const voltSeries = $derived([
    { label: 'R median', data: series['right.median'] ?? [] },
    { label: 'R lowest', data: series['right.min'] ?? [] },
    { label: 'L median', data: series['left.median'] ?? [] },
    { label: 'L lowest', data: series['left.min'] ?? [] },
  ]);
  const currentSeries = $derived([
    { label: 'R est.', data: series['right.supply'] ?? [] },
    { label: 'L est.', data: series['left.supply'] ?? [] },
    { label: 'both', data: series['total'] ?? [] },
    { label: 'supply limit', data: series['limit'] ?? [] },
  ]);

  const when = (s: number) => new Date(s * 1000).toLocaleTimeString();
</script>

<div class="page">
  <div class="page-header">
    <h1>Power</h1>
    <p>Servo supply voltage and estimated supply current of both hands, and what they were in the second before each fault.</p>
  </div>

  <div class="panel row settings">
    <span class="muted">Bench supply</span>
    <label class="row">limit <input id="psu-limit" type="number" step="0.1" min="0.1" bind:value={limitA} /> A</label>
    <label class="row"><input id="psu-shared" type="checkbox" bind:checked={shared} /> both hands on this one supply</label>
  </div>

  <div class="cards">
    {#each HANDS as hand (hand)}
      {@const p = w(hand)}
      <div class="panel card">
        <div class="row head"><h2>{HAND_LABEL[hand]} hand</h2><span class="muted">{p ? `${p.armed} armed` : 'no data'}</span></div>
        <div class="metrics">
          <div><span class="k">median now</span><span class="num {voltClass(p?.now_median_v)}">{fmt(p?.now_median_v, 2)} V</span></div>
          <div><span class="k">lowest servo, 1 s</span><span class="num {voltClass(p?.min_v)}">{fmt(p?.min_v, 1)} V{p?.min_v_joint ? ` (j${p.min_v_joint})` : ''}</span></div>
          <div><span class="k">est. supply now</span><span class="num">{fmt(p?.now_supply_a, 2)} A</span></div>
          <div><span class="k">est. supply peak, 1 s</span><span class="num {level(budget(p?.peak_supply_a ?? 0))}">{fmt(p?.peak_supply_a, 2)} A</span></div>
          <div><span class="k">phase current peak</span><span class="num">{fmt(p?.peak_phase_a, 1)} A</span></div>
        </div>
      </div>
    {/each}
    <div class="panel card">
      <div class="row head"><h2>{shared ? 'Shared supply' : 'Per hand'}</h2></div>
      <div class="bar"><div class="fill {level(totalPeak)}" style:width={`${Math.min(100, (100 * totalPeak) / limitA)}%`}></div></div>
      <p class="num big {level(totalPeak)}">{fmt(totalPeak, 2)} / {fmt(limitA, 1)} A</p>
      <p class="hint">Peak of both hands' estimate over the last second{shared ? ', against the one supply' : ''}. When it reaches the limit, a constant-current supply lowers its voltage: <code>supply voltage</code> below {SUPPLY_MIN_V} V, then <code>servo voltage error</code>, then <code>comm lost</code>.</p>
    </div>
  </div>

  <div class="panel"><Plot title="Servo voltage" unit="V" {time} series={voltSeries} {version} /></div>
  <div class="panel"><Plot title="Estimated supply current" unit="A" {time} series={currentSeries} {version} /></div>

  <div class="panel">
    <h2>Faults that switched joints off</h2>
    <table>
      <thead>
        <tr><th>time</th><th>hand</th><th>fault</th><th>joint</th>
          {#each HANDS as h}<th>{HAND_LABEL[h]}: median / lowest V</th><th>{HAND_LABEL[h]}: est. peak A</th>{/each}
        </tr>
      </thead>
      <tbody>
        {#each app.trips as trip (trip.wall_time + trip.hand + trip.text)}
          <tr>
            <td class="num muted">{when(trip.wall_time)}</td>
            <td>{trip.hand}</td>
            <td><span class="pill err">{trip.fault}</span></td>
            <td class="num">{trip.joint || 'all'}</td>
            {#each HANDS as h}
              {@const x = trip.hands[h]}
              <td class="num {voltClass(x?.min_v)}">{x ? `${fmt(x.min_median_v, 2)} / ${fmt(x.min_v, 1)}${x.min_v_joint ? ` j${x.min_v_joint}` : ''}` : '—'}</td>
              <td class="num">{x ? fmt(x.peak_supply_a, 2) : '—'}</td>
            {/each}
          </tr>
        {:else}
          <tr><td colspan="8" class="muted">No faults recorded since teleop-host started.</td></tr>
        {/each}
      </tbody>
    </table>
  </div>

  <div class="panel hint">
    <p><b>Reading the estimate.</b> An HLS3606 draws about phase current × drive duty from the supply. SCS0009 joints report no current and are not included, and the HLS current scale (6.5 mA per unit) is not yet verified on the bench, so treat the numbers as a lower bound with an uncertain scale. The voltages are what the servos measure themselves.</p>
    <p><b>With one {fmt(limitA, 1)} A supply for both hands</b>, a single HLS joint accelerating at its default current cap (about 5.2 A phase) can use the whole supply. Give each hand its own supply with enough current for its peaks, keep power cables short and thick, and lower <code>ilimit</code>, <code>vmax</code> and <code>amax</code> until then.</p>
  </div>
</div>

<style>
  .settings input[type='number'] {
    width: 70px;
  }
  .cards {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 12px;
  }
  .card {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .head {
    justify-content: space-between;
  }
  .metrics > div {
    display: flex;
    justify-content: space-between;
    gap: 8px;
  }
  .k {
    color: var(--muted);
    font-size: 12.5px;
  }
  .num.ok {
    color: var(--ok);
  }
  .num.warn {
    color: var(--warn);
  }
  .num.err {
    color: var(--err);
  }
  .big {
    font-size: 20px;
    margin: 0;
  }
  .bar {
    height: 10px;
    border: 1px solid var(--border);
    border-radius: 5px;
    background: var(--bg);
    overflow: hidden;
  }
  .fill {
    height: 100%;
    background: var(--ok);
  }
  .fill.warn {
    background: var(--warn);
  }
  .fill.err {
    background: var(--err);
  }
  .hint p {
    margin: 0 0 6px;
  }
  td.err {
    color: var(--err);
  }
  td.warn {
    color: var(--warn);
  }
</style>
