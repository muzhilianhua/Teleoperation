<script lang="ts">
  import HandTag from '../lib/components/HandTag.svelte';
  import ParamField from '../lib/components/ParamField.svelte';
  import { run } from '../lib/commands';
  import { confirm } from '../lib/confirm.svelte';
  import { jointViews } from '../lib/joints.svelte';
  import { EventCode, MODEL_HLS, type DeviceEvent } from '../lib/protocol';
  import { app } from '../lib/state.svelte';
  import { fmt, modelShort } from '../lib/units';

  const joints = $derived(jointViews());
  const armed = $derived(app.armedCount > 0);
  const byId = (id: number) => joints.find((j) => j.info.servo_id === id);

  // ---- ping
  let pingRows = $state<DeviceEvent[] | null>(null);
  async function ping() {
    const r = await run('ping', { quiet: true });
    pingRows = r.events.filter((e) => e.code === EventCode.PING_RESULT);
  }

  // ---- limits
  let limitRows = $state<DeviceEvent[] | null>(null);
  let limitSummary = $state('');
  async function checkLimits() {
    const r = await run('limits check', { quiet: true });
    limitRows = r.events.filter((e) => e.code === EventCode.LIMITS_ROW);
    limitSummary = r.events.filter((e) => e.code === EventCode.GENERIC).map((e) => e.text).join(' ');
  }
  async function writeLimits(target: number | 'all') {
    const hls = joints.some((j) => j.info.model === MODEL_HLS);
    const ok = await confirm({
      title: `Write angle limits ${target === 'all' ? 'for every joint' : `for joint ${target}`}?`,
      body: `Writes the travel from config.h into the servo EEPROM (only where it differs).${hls ? '\nPower-cycle the servos afterwards: HLS3606 limits take effect after a restart.' : ''}`,
      confirmText: 'Write',
    });
    if (!ok) return;
    await run(`limits write ${target}`);
    await checkLimits();
  }
  const limitStatus = (m: number) => (m === 1 ? ['ok', 'ok'] : m === 0 ? ['MISMATCH', 'err'] : m === -1 ? ['no reply', 'warn'] : ['travel undefined', '']);

  // ---- sweep
  let sweepId = $state(1);
  let sweepStartEvent = $state(0);
  const sweepEvents = $derived(
    sweepStartEvent ? app.events.filter((e) => e.id > sweepStartEvent && e.joint === sweepId) : [],
  );
  const sweepRuns = $derived(sweepEvents.filter((e) => e.code === EventCode.SWEEP_RUN));
  const sweepDone = $derived(sweepEvents.find((e) => e.code === EventCode.SWEEP_DONE));
  async function startSweep() {
    const ok = await confirm({
      title: `Sweep joint ${sweepId}?`,
      body: 'Arms this joint alone and drives it back and forth across its range at four speed tiers (10–60 s). Every other joint must be off. Keep hands clear; x / Esc / BOOT stops it.',
      confirmText: 'Start sweep',
      danger: true,
    });
    if (!ok) return;
    sweepStartEvent = app.events.at(-1)?.id ?? 0;
    await run(`sweep ${sweepId}`, { quiet: true });
  }
</script>

<div class="page">
  <div class="page-header">
    <h1>Servo setup</h1>
    <HandTag />
    <p>Identify the servos, sync their angle-limit registers with the travel, measure speed limits and set HLS current limits.</p>
  </div>

  {#if !app.connected}
    <div class="notice">Connect to the device first.</div>
  {:else}
    {#if armed}<div class="notice">Ping, limits and sweep need every joint disarmed.</div>{/if}

    <div class="panel">
      <div class="row"><h2>Identify</h2><button disabled={armed} onclick={ping}>Ping all</button></div>
      {#if pingRows}
        <table>
          <thead><tr><th>id</th><th>name</th><th>config.h</th><th>on the bus</th><th>firmware</th><th></th></tr></thead>
          <tbody>
            {#each pingRows as e}
              {@const j = byId(e.joint)}
              <tr>
                <td class="num">{e.joint}</td>
                <td>{j?.info.name}</td>
                <td>{app.info?.models[j?.info.model ?? 0]}</td>
                <td>{e.args[0] ? app.info?.models[e.args[0]] : '—'}</td>
                <td class="num">{e.args[0] ? `${e.args[1]}.${e.args[2]}` : ''}</td>
                <td><span class="pill {e.args[3] ? 'ok' : e.args[0] ? 'err' : 'warn'}">{e.args[3] ? 'ok' : e.args[0] ? 'model mismatch: cannot arm' : 'no reply'}</span></td>
              </tr>
            {/each}
          </tbody>
        </table>
      {/if}
    </div>

    <div class="panel">
      <div class="row">
        <h2>Angle limits</h2>
        <button disabled={armed} onclick={checkLimits}>Check</button>
        <button disabled={armed} onclick={() => writeLimits('all')}>Write all</button>
        <span class="hint">{limitSummary}</span>
      </div>
      {#if limitRows}
        <table>
          <thead><tr><th>id</th><th>name</th><th>servo registers</th><th>travel (config.h)</th><th></th><th></th></tr></thead>
          <tbody>
            {#each limitRows as e}
              {@const j = byId(e.joint)}
              {@const [label, cls] = limitStatus(e.args[2])}
              <tr>
                <td class="num">{e.joint}</td>
                <td>{j?.info.name}</td>
                <td class="num">{e.args[2] === -1 ? '—' : `${e.args[0]}..${e.args[1]}`}</td>
                <td class="num">{j ? `${j.info.travel_min}..${j.info.travel_max}` : ''}</td>
                <td><span class="pill {cls}">{label}{e.args[0] === 0 && e.args[1] === 0 && j?.info.model === MODEL_HLS && e.args[2] === 0 ? ' (HLS factory multi-turn)' : ''}</span></td>
                <td>{#if e.args[2] === 0}<button class="small" disabled={armed} onclick={() => writeLimits(e.joint)}>Write</button>{/if}</td>
              </tr>
            {/each}
          </tbody>
        </table>
      {/if}
    </div>

    <div class="panel">
      <div class="row">
        <h2>Speed sweep</h2>
        <select bind:value={sweepId}>
          {#each joints as j (j.info.servo_id)}<option value={j.info.servo_id}>{j.info.servo_id} {j.info.name}</option>{/each}
        </select>
        <button class="danger" disabled={armed} onclick={startSweep}>Start sweep</button>
      </div>
      <p class="hint">Measures saturation speed in both directions and suggests vmax ≈ 1.2 × faster, vsat ≈ 0.9 × slower. A direction that never arrives from the second tier on is force-limited (friction / preload): no suggestion.</p>
      {#if sweepStartEvent}
        <table>
          <thead><tr><th>command</th><th>measured</th><th>arrived</th><th>max load</th></tr></thead>
          <tbody>
            {#each sweepRuns as e (e.id)}
              <tr>
                <td class="num">{e.args[0]} steps/s</td>
                <td class="num">{e.args[1] < 0 ? 'skipped' : `${fmt(e.args[1] / 10)} steps/s`}</td>
                <td>{e.args[1] < 0 ? '' : e.args[2] ? 'yes' : 'TIMED OUT'}</td>
                <td class="num">{fmt(e.args[3] / 10)} %</td>
              </tr>
            {:else}
              <tr><td colspan="4" class="muted">running…</td></tr>
            {/each}
          </tbody>
        </table>
        {#if sweepDone}
          {#if sweepDone.args[2]}
            <div class="row">
              <span class="pill ok">suggests vmax {sweepDone.args[0]} · vsat {sweepDone.args[1]}</span>
              <button class="primary" onclick={() => run(`sweep ${sweepId} apply`)}>Apply</button>
              <span class="hint">then Save (Parameters / Config store)</span>
            </div>
          {:else}
            <div class="notice">{sweepDone.args[3] ? 'Sweep aborted' : 'No suggestion: force-limited direction'} — {sweepDone.text}</div>
          {/if}
        {/if}
      {/if}
    </div>

    <div class="panel">
      <h2>HLS current</h2>
      <p class="hint">Goal current = min(servo protection current, ilimit) is sent with every goal; it bounds torque (motor phase current, not supply current). over-current trips at itrip × cap for 0.2 s; the grip limiter holds a grasp near gripload × cap.</p>
      <table>
        <thead><tr><th>id</th><th>name</th><th>ilimit (mA)</th><th>itrip (%)</th><th>cap now</th><th>current</th><th>% of cap</th><th>supply est.</th></tr></thead>
        <tbody>
          {#each joints.filter((j) => j.info.model === MODEL_HLS) as j (j.info.servo_id)}
            {@const cap = j.slow?.effective_current_ma ?? 0}
            {@const cur = j.tel?.current_ma ?? 0}
            <tr>
              <td class="num">{j.info.servo_id}</td>
              <td>{j.info.name} <span class="pill">{modelShort(j.info.model)}</span></td>
              <td><ParamField value={j.slow?.current_limit_ma} min={0} max={app.info?.max_current_limit_ma} apply={(v) => run(`set ${j.info.servo_id} ilimit ${v}`, { quiet: true })} /></td>
              <td><ParamField value={j.slow?.current_trip_pct} min={0} max={100} apply={(v) => run(`set ${j.info.servo_id} itrip ${v}`, { quiet: true })} /></td>
              <td class="num">{cap ? `${cap} mA` : 'when armed'}</td>
              <td class="num">{cur} mA</td>
              <td class="num">{cap ? fmt((100 * Math.abs(cur)) / cap) : '—'}</td>
              <td class="num">{fmt((Math.abs(cur) * Math.abs(j.tel?.load_deci_pct ?? 0)) / 1000)} mA</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</div>

<style>
  .panel {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .hint {
    margin: 0;
  }
</style>
