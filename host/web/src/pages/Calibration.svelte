<script lang="ts">
  import HandTag from '../lib/components/HandTag.svelte';
  import TravelBar from '../lib/components/TravelBar.svelte';
  import { run } from '../lib/commands';
  import { confirm } from '../lib/confirm.svelte';
  import { jointViews, travelDefined } from '../lib/joints.svelte';
  import { JointFlag, NO_ENCODER, has } from '../lib/protocol';
  import { app } from '../lib/state.svelte';
  import { DEG_PER_COUNT, fmt, modelShort } from '../lib/units';

  const joints = $derived(jointViews());
  let selectedId = $state(1);
  const joint = $derived(joints.find((j) => j.info.servo_id === selectedId) ?? joints[0]);
  const id = $derived(joint?.info.servo_id ?? 0);
  const s = $derived(joint?.slow);
  const t = $derived(joint?.tel);
  const flags = $derived(t?.flags ?? 0);
  const armed = $derived(app.armedCount > 0);
  const monitorOn = $derived(app.slow?.monitor === 1);
  const degPerCount = $derived(app.info?.degrees_per_count ?? DEG_PER_COUNT);

  // Encoder activity over the last 3 s, to spot which one the operator is moving.
  const WINDOW_MS = 3000;
  let history: { t: number; angles: number[] }[] = [];
  let historyHand = app.current;
  let activity = $state<number[]>([]);
  $effect(() => {
    const encoders = app.slow?.encoders;
    if (!encoders) return;
    if (historyHand !== app.current) {
      history = []; // the other hand's encoders
      historyHand = app.current;
    }
    const now = Date.now();
    history.push({ t: now, angles: encoders.map((e) => e.angle) });
    history = history.filter((h) => now - h.t <= WINDOW_MS);
    activity = encoders.map((_, i) => {
      let travel = 0;
      for (let k = 1; k < history.length; k++) {
        let d = history[k].angles[i] - history[k - 1].angles[i];
        if (d > 8192) d -= 16384;
        if (d < -8192) d += 16384;
        travel += Math.abs(d);
      }
      return travel * degPerCount;
    });
  });
  const mostActive = $derived(activity.length && Math.max(...activity) > 5 ? activity.indexOf(Math.max(...activity)) : -1);

  // Live servo position: monitor rows when monitoring, else telemetry.
  const servoPos = $derived(
    monitorOn && app.slow ? app.slow.monitor_position[joint?.index ?? 0] : (t?.position ?? null),
  );
  const spanDeg = $derived(s && s.exo_b ? s.exo_b * degPerCount : null);

  let rawRef = $state(0);
  let rawSpan = $state(0);
  let rawA = $state(0);
  let rawB = $state(0);

  async function directExo() {
    if (await confirm({ title: 'Enter exo endpoints?', body: `set ${id} exospan ${rawRef} ${rawSpan}`, confirmText: 'Apply' }))
      run(`set ${id} exospan ${rawRef} ${rawSpan}`);
  }
  async function directServo() {
    if (await confirm({ title: 'Enter servo endpoints?', body: `set ${id} servo ${rawA} ${rawB}`, confirmText: 'Apply' }))
      run(`set ${id} servo ${rawA} ${rawB}`);
  }
  async function save() {
    if (await confirm({ title: 'Save to NVS?', body: 'Overwrites the stored configuration with the current settings.', confirmText: 'Save' }))
      run('save');
  }

  function readiness(view: (typeof joints)[number]): string {
    const f = view.tel?.flags ?? 0;
    if (has(f, JointFlag.READY)) return 'ready';
    if (!travelDefined(view.info)) return 'no travel';
    if (!view.slow || view.slow.encoder_index === NO_ENCODER) return 'no encoder';
    if (view.slow.servo_a === view.slow.servo_b) return 'servo A/B';
    if (!view.slow.exo_b) return 'exo A/B';
    if (!view.slow.enabled) return 'disabled';
    return '…';
  }
</script>

<div class="page">
  <div class="page-header">
    <h1>Calibration</h1>
    <HandTag />
    <p>Per joint, disarmed: assign the encoder, capture servo and exo endpoints, home multi-turn joints, enable, save.</p>
  </div>

  {#if !app.connected || !joint}
    <div class="notice">Connect to the device first.</div>
  {:else}
    {#if armed}
      <div class="notice">Joints are armed: calibration commands are refused until everything is disarmed.</div>
    {/if}
    <div class="layout">
      <div class="panel list">
        {#each joints as view (view.info.servo_id)}
          {@const r = readiness(view)}
          <button class="jbtn {view.info.servo_id === id ? 'active' : ''}" onclick={() => (selectedId = view.info.servo_id)}>
            <span class="num">{view.info.servo_id}</span>
            <span class="jname">{view.info.name}</span>
            <span class="pill {r === 'ready' ? 'ok' : ''}">{r}</span>
          </button>
        {/each}
      </div>

      <div class="steps">
        <div class="panel step">
          <div class="step-head"><span class="n">1</span><h2>Encoder</h2>
            <span class="muted">assigned: {s && s.encoder_index !== NO_ENCODER ? `#${s.encoder_index}` : 'none'}</span>
          </div>
          <p class="hint">Turn on the monitor, then move only this joint of the glove: the encoder that moves is highlighted.</p>
          <div class="row">
            <button class={monitorOn ? '' : 'primary'} disabled={armed} onclick={() => run(`monitor ${monitorOn ? 'off' : 'on'}`, { quiet: true })}>
              Monitor {monitorOn ? 'off' : 'on'}
            </button>
          </div>
          {#if app.slow}
            <table class="enc">
              <thead><tr><th>#</th><th>addr</th><th>angle</th><th>moved (3 s)</th><th>state</th><th></th></tr></thead>
              <tbody>
                {#each app.slow.encoders as e, i}
                  <tr class:hot={i === mostActive} class:mine={s?.encoder_index === i}>
                    <td class="num">{i}</td>
                    <td class="num">0x{e.address.toString(16).toUpperCase()}</td>
                    <td class="num">{fmt(e.angle * degPerCount, 1)}°</td>
                    <td class="num">{fmt(activity[i] ?? 0, 0)}°</td>
                    <td>{['ok', 'no magnet', 'offline'][e.state] ?? e.state}</td>
                    <td><button class="small" disabled={armed || s?.encoder_index === i} onclick={() => run(`set ${id} enc ${i}`)}>Assign</button></td>
                  </tr>
                {/each}
              </tbody>
            </table>
            <button class="small ghost" disabled={armed} onclick={() => run(`set ${id} enc none`)}>Unassign</button>
          {/if}
        </div>

        <div class="panel step">
          <div class="step-head"><span class="n">2</span><h2>Servo endpoints</h2>
            <span class="muted">torque off: move the finger by hand</span>
          </div>
          {#if travelDefined(joint.info)}
            <TravelBar min={joint.info.travel_min} max={joint.info.travel_max} a={s?.servo_a ?? null} b={s?.servo_b ?? null} present={servoPos} />
            <div class="row">
              <span class="muted">travel {joint.info.travel_min}..{joint.info.travel_max} · now</span>
              <span class="num big">{servoPos ?? '—'}</span>
              <span class="muted">· A</span><span class="num">{s && s.servo_a !== s.servo_b ? s.servo_a : '—'}</span>
              <span class="muted">B</span><span class="num">{s && s.servo_a !== s.servo_b ? s.servo_b : '—'}</span>
            </div>
            <div class="row">
              <button disabled={armed} onclick={() => run(`cal ${id} hand a`)}>Capture A (open)</button>
              <button disabled={armed} onclick={() => run(`cal ${id} hand b`)}>Capture B (closed)</button>
            </div>
          {:else}
            <div class="notice">Travel is not defined in config.h for this joint: measure it with the monitor, enter it, reflash.</div>
          {/if}
        </div>

        <div class="panel step">
          <div class="step-head"><span class="n">3</span><h2>Exo endpoints</h2>
            <span class="muted">glove in the matching poses: A ↔ A, B ↔ B</span>
          </div>
          <div class="row">
            <span class="muted">exo now</span>
            <span class="num big">{fmt((t?.exo_counts ?? NaN) * degPerCount, 1)}°</span>
            <span class="muted">from A · span</span>
            <span class="num">{spanDeg !== null ? `${fmt(spanDeg, 1)}° (${fmt(spanDeg / 360, 2)} turns)` : has(flags, JointFlag.REF_CAPTURED) ? 'A captured, B pending' : '—'}</span>
          </div>
          <p class="hint">Capture A, then move continuously to the other end (any number of turns, up to 3) and capture B. An encoder dropout in between requires redoing A.</p>
          <div class="row">
            <button disabled={armed} onclick={() => run(`cal ${id} a`)}>Capture A</button>
            <button disabled={armed || !(has(flags, JointFlag.REF_CAPTURED) || (s?.exo_b ?? 0) !== 0)} onclick={() => run(`cal ${id} b`)}>Capture B</button>
          </div>
        </div>

        <div class="panel step">
          <div class="step-head"><span class="n">4</span><h2>Home</h2>
            <span class="pill {has(flags, JointFlag.SINGLE_TURN) ? 'ok' : has(flags, JointFlag.HOMED) ? 'ok' : 'err'}">
              {has(flags, JointFlag.SINGLE_TURN) ? 'single-turn: homes itself' : has(flags, JointFlag.HOMED) ? 'homed' : 'HOME! needed'}
            </span>
          </div>
          <p class="hint">Multi-turn joints (span + dead zone ≥ one turn) need this after every power-up, with the glove held at pose A.</p>
          <button disabled={has(flags, JointFlag.SINGLE_TURN) || (t?.state ?? 0) !== 0} onclick={() => run(`home ${id}`)}>Home at pose A</button>
        </div>

        <div class="panel step">
          <div class="step-head"><span class="n">5</span><h2>Enable and save</h2>
            <span class="pill {has(flags, JointFlag.READY) ? 'ok' : ''}">{readiness(joint)}</span>
          </div>
          <div class="row">
            <label class="row"><input type="checkbox" checked={!!s?.enabled} disabled={armed} onchange={(e) => run(`set ${id} ${e.currentTarget.checked ? 'on' : 'off'}`)} /> enabled</label>
            <button class="primary" disabled={armed} onclick={save}>Save to NVS</button>
            <span class="hint">Settings live in RAM until saved. Back them up in Config store.</span>
          </div>
        </div>

        <details class="panel step">
          <summary><h2 class="inline">Direct entry</h2> <span class="muted">(dump format, e.g. from a backup)</span></summary>
          <div class="row">
            <span class="muted mono">set {id} exospan</span>
            <input type="number" bind:value={rawRef} min="0" max="16383" />
            <input type="number" bind:value={rawSpan} />
            <button disabled={armed} onclick={directExo}>Apply</button>
          </div>
          <div class="row">
            <span class="muted mono">set {id} servo</span>
            <input type="number" bind:value={rawA} />
            <input type="number" bind:value={rawB} />
            <button disabled={armed} onclick={directServo}>Apply</button>
          </div>
          <p class="hint">{modelShort(joint.info.model)} · {fmt(joint.info.degrees_per_step, 3)}°/step · encoder {fmt(degPerCount, 4)}°/count</p>
        </details>
      </div>
    </div>
  {/if}
</div>

<style>
  .layout {
    display: grid;
    grid-template-columns: 230px 1fr;
    gap: 14px;
    align-items: start;
  }
  .list {
    display: flex;
    flex-direction: column;
    gap: 4px;
    padding: 8px;
    position: sticky;
    top: 0;
  }
  .jbtn {
    display: grid;
    grid-template-columns: 20px 1fr auto;
    gap: 6px;
    align-items: center;
    text-align: left;
    background: transparent;
    border-color: transparent;
  }
  .jbtn.active {
    background: var(--accent-weak);
    border-color: var(--accent);
  }
  .jname {
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .steps {
    display: flex;
    flex-direction: column;
    gap: 12px;
  }
  .step {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .step-head {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .n {
    width: 22px;
    height: 22px;
    border-radius: 50%;
    background: var(--accent-weak);
    color: var(--accent);
    display: grid;
    place-items: center;
    font-weight: 700;
    font-size: 12px;
  }
  .big {
    font-size: 18px;
  }
  .enc tr.hot td {
    background: var(--warn-weak);
  }
  .enc tr.mine td:first-child {
    box-shadow: inset 3px 0 var(--accent);
  }
  .inline {
    display: inline;
  }
  summary {
    cursor: pointer;
  }
  p {
    margin: 0;
  }
</style>
