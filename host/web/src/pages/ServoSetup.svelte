<script lang="ts">
  import HandTag from '../lib/components/HandTag.svelte';
  import ParamField from '../lib/components/ParamField.svelte';
  import { run } from '../lib/commands';
  import { confirm } from '../lib/confirm.svelte';
  import { jointViews, offsetOf, travelMargin, travelOf, type JointView } from '../lib/joints.svelte';
  import { EventCode, MODEL_HLS, MODEL_SCS, type DeviceEvent } from '../lib/protocol';
  import { positionCenter, seamJump, seamMargins } from '../lib/seam';
  import { app } from '../lib/state.svelte';
  import { fmt, modelShort, stepsToDeg } from '../lib/units';

  const joints = $derived(jointViews());
  const armed = $derived(app.armedCount > 0);
  const byId = (id: number) => joints.find((j) => j.info.servo_id === id);

  // ---- ping
  let pingRows = $state<DeviceEvent[] | null>(null);
  async function ping() {
    const r = await run('ping', { quiet: true });
    pingRows = r.events.filter((e) => e.code === EventCode.PING_RESULT);
  }

  // ---- travel (this robot's mechanical limits, stored with the calibration)
  const monitorOn = $derived(app.slow?.monitor === 1);
  // Draft min / max per servo id while editing; cleared once applied.
  let drafts = $state<Record<number, { min: number; max: number }>>({});
  const draftOf = (j: JointView) => drafts[j.info.servo_id] ?? travelOf(j);
  const dirty = (j: JointView) => {
    const d = drafts[j.info.servo_id];
    const t = travelOf(j);
    return !!d && (d.min !== t.min || d.max !== t.max);
  };
  function livePosition(j: JointView): number | null {
    const s = app.slow;
    if (s && s.monitor && s.monitor_ok_mask & (1 << j.index)) return s.monitor_position[j.index];
    return j.tel ? j.tel.position : null;
  }
  function setDraft(j: JointView, end: 'min' | 'max', value: number) {
    drafts = { ...drafts, [j.info.servo_id]: { ...draftOf(j), [end]: Math.round(value) } };
  }
  function capture(j: JointView, end: 'min' | 'max') {
    const pos = livePosition(j);
    if (pos === null) return;
    // Keep the margin on the inside of the mechanical end.
    setDraft(j, end, end === 'min' ? pos + travelMargin(j.info) : pos - travelMargin(j.info));
  }
  async function applyTravel(j: JointView) {
    const d = draftOf(j);
    const t = travelOf(j);
    const ok = await confirm({
      title: `Set the travel of joint ${j.info.servo_id} (${j.info.name})?`,
      body: `${t.min}..${t.max} → ${d.min}..${d.max} servo steps. Every goal is clamped to this range and \`limits write\` stores it in the servo. Calibrated servo endpoints outside it must be recaptured first.\nThen: Save to keep it, and Write the angle limits${j.info.model === MODEL_HLS ? ' (power-cycle HLS servos afterwards)' : ''}.`,
      confirmText: 'Set travel',
      danger: true,
    });
    if (!ok) return;
    const r = await run(`set ${j.info.servo_id} travel ${d.min} ${d.max}`);
    if (r.ok) {
      const { [j.info.servo_id]: _, ...rest } = drafts;
      drafts = rest;
      clearSeam(j.info.servo_id);
    }
  }
  function resetToDefault(j: JointView) {
    drafts = { ...drafts, [j.info.servo_id]: { min: j.info.travel_min, max: j.info.travel_max } };
  }

  // ---- zero (HLS position offset)
  async function zero(j: JointView) {
    const id = j.info.servo_id;
    const ok = await confirm({
      title: `Zero joint ${id} (${j.info.name})?`,
      body: `Torque off: hold the joint by hand in the middle of its travel. Writes the HLS position offset so this position reads 2048 (now ${livePosition(j) ?? '?'}), and leaves the servo in multi-turn mode (angle limits 0/0) until the limits are written.\nThe stored travel and servo endpoints move with the offset; if the travel would then cross the 4095/0 seam they are cleared and the joint is switched off.\nThen: check or measure the travel → Set → Save → Write limits → power-cycle the servo → Check → calibrate.`,
      confirmText: 'Zero',
      danger: true,
    });
    if (!ok) return;
    const r = await run(`zero ${id}`);
    if (r.ok) {
      const { [id]: _, ...rest } = drafts;
      drafts = rest;
      limitRows = null;
      lastPos = {};  // the reading jumped with the offset, not across the seam
      clearSeam(id);
    }
  }

  // ---- seam (where the reading wraps: SCS 1023/0, HLS 4095/0)
  // Crossings seen while monitoring: an HLS joint is zeroed, an SCS horn re-seated.
  let lastPos: Record<number, number> = {};
  let seamCrossed = $state<Record<number, boolean>>({});
  $effect(() => {
    if (!monitorOn) {
      lastPos = {};
      return;
    }
    for (const j of joints) {
      const pos = livePosition(j);
      if (pos === null) continue;
      const id = j.info.servo_id;
      const prev = lastPos[id];
      lastPos[id] = pos;
      if (prev !== undefined && seamJump(prev, pos, j.info.model)) seamCrossed = { ...seamCrossed, [id]: true };
    }
  });
  function clearSeam(id: number) {
    const { [id]: _, ...rest } = seamCrossed;
    seamCrossed = rest;
  }
  // Degrees between each end of the travel and the seam; warn below 10 deg.
  function seamDeg(j: JointView): { low: number; high: number } | null {
    const d = draftOf(j);
    if (d.min >= d.max) return null;
    const m = seamMargins(d.min, d.max, j.info.model);
    return { low: stepsToDeg(j.info, m.low), high: stepsToDeg(j.info, m.high) };
  }
  // SCS0009 centring aid: live reading against the middle of the range, in degrees.
  let centering = $state(false);
  async function toggleCentering() {
    centering = !centering;
    if (centering && !monitorOn) await run('monitor on', { quiet: true });
  }
  const scsJoints = $derived(joints.filter((j) => j.info.model !== MODEL_HLS));

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
      body: `Writes each joint's stored travel (and, on HLS joints, the stored position offset) into the servo EEPROM (only where it differs).${hls ? '\nPower-cycle the servos afterwards: HLS3606 limits take effect after a restart.' : ''}`,
      confirmText: 'Write',
    });
    if (!ok) return;
    await run(`limits write ${target}`);
    lastPos = {};  // a written HLS offset moves the reading
    await checkLimits();
  }
  async function storeOffset(id: number, offset: number) {
    const ok = await confirm({
      title: `Store offset ${offset} for joint ${id}?`,
      body: `Records the servo's current position offset as the one this robot's travel was measured with. From then on arming and Check refuse a servo holding another offset, and backups carry it.\nOnly if the stored travel was measured with this offset in place. Then Save.`,
      confirmText: 'Store',
    });
    if (ok && (await run(`set ${id} offset ${offset}`)).ok) await checkLimits();
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
        <h2>Travel</h2>
        <button disabled={armed} onclick={() => run(`monitor ${monitorOn ? 'off' : 'on'}`, { quiet: true })}>Monitor {monitorOn ? 'off' : 'on'}</button>
        <button class="primary" disabled={armed} onclick={() => run('save')}>Save to NVS</button>
        <button class:primary={centering} disabled={armed || scsJoints.length === 0} onclick={toggleCentering} title="SCS0009: re-seat a horn so the joint's middle reads 512">Center {centering ? 'off' : 'on'}</button>
      </div>
      <p class="hint">This robot's mechanical limits, in servo steps. They are stored with the calibration, so a backup carries them to another session or board. To measure: torque off, monitor on, push the finger to each end by hand and press Min / Max (keeps ~15 SCS / ~50 HLS steps inside the end), then Set. An HLS travel across the 4095/0 seam: hold the joint mid-travel and Zero it first. An SCS travel across 1023/0 cannot be fixed in software (no feedback beyond the seam): re-seat the horn with Center on.</p>
      {#if Object.keys(seamCrossed).length}
        <div class="notice">
          Crossed the seam while monitoring:
          {#each joints.filter((j) => seamCrossed[j.info.servo_id]) as j (j.info.servo_id)}
            <span class="pill err">{j.info.servo_id} {j.info.name}: {j.info.model === MODEL_HLS ? 'hold it mid-travel and Zero it' : 're-seat the horn (Center on)'}
              <button class="small ghost" onclick={() => clearSeam(j.info.servo_id)} title="clear">×</button></span>
          {/each}
        </div>
      {/if}
      {#if centering}
        <p class="hint">SCS0009: hold the joint in the middle of its travel, take the horn off and put it back on the spline where the reading is closest to {positionCenter(MODEL_SCS)} (within ±5°), then measure the travel again. The whole travel must fit inside 0..1023 (about 300°).</p>
        <div class="center-grid">
          {#each scsJoints as j (j.info.servo_id)}
            {@const pos = livePosition(j)}
            {@const err = pos === null ? null : stepsToDeg(j.info, pos - positionCenter(j.info.model))}
            <div class="center-tile" class:ok={err !== null && Math.abs(err) <= 5}>
              <span class="muted">{j.info.servo_id} {j.info.name}</span>
              <span class="big">{err === null ? '—' : `${err > 0 ? '+' : ''}${fmt(err, 1)}°`}</span>
              <span class="muted">reads {pos ?? '—'} · centre {positionCenter(j.info.model)}</span>
            </div>
          {/each}
        </div>
      {/if}
      <table>
        <thead><tr><th>id</th><th>name</th><th>live</th><th>min</th><th></th><th>max</th><th></th><th></th><th title="degrees from each end of the travel to the seam">seam</th><th>offset</th></tr></thead>
        <tbody>
          {#each joints as j (j.info.servo_id)}
            {@const d = draftOf(j)}
            {@const pos = livePosition(j)}
            <tr class:dirty={dirty(j)}>
              <td class="num">{j.info.servo_id}</td>
              <td>{j.info.name} <span class="pill">{modelShort(j.info.model)}</span></td>
              <td class="num">{pos ?? '—'}</td>
              <td><input id={`travel-min-${j.info.servo_id}`} type="number" value={d.min} min="0" disabled={armed} oninput={(e) => setDraft(j, 'min', Number(e.currentTarget.value))} /></td>
              <td><button class="small" disabled={armed || pos === null} onclick={() => capture(j, 'min')} title="from the live position">Min</button></td>
              <td><input id={`travel-max-${j.info.servo_id}`} type="number" value={d.max} min="0" disabled={armed} oninput={(e) => setDraft(j, 'max', Number(e.currentTarget.value))} /></td>
              <td><button class="small" disabled={armed || pos === null} onclick={() => capture(j, 'max')} title="from the live position">Max</button></td>
              <td>
                <div class="row nowrap">
                  <button class="small primary" disabled={armed || !dirty(j)} onclick={() => applyTravel(j)}>Set</button>
                  <button class="small ghost" disabled={armed} onclick={() => resetToDefault(j)} title={`config.h: ${j.info.travel_min}..${j.info.travel_max}`}>Default</button>
                </div>
              </td>
              <td>
                {#if seamCrossed[j.info.servo_id]}
                  <span class="pill err">crossed</span>
                {:else}
                  {@const m = seamDeg(j)}
                  {#if m}
                    <span class="num" class:warn-text={Math.min(m.low, m.high) < 10} title={`${fmt(m.low)}° below min, ${fmt(m.high)}° above max to the seam`}>{fmt(m.low)}°·{fmt(m.high)}°</span>
                  {:else}
                    <span class="muted">—</span>
                  {/if}
                {/if}
              </td>
              <td>
                {#if j.info.model === MODEL_HLS}
                  <div class="row nowrap">
                    <span class="num" title="stored HLS position offset">{offsetOf(j) ?? 'none'}</span>
                    <button class="small" disabled={armed || pos === null} onclick={() => zero(j)} title="joint held mid-travel: make it read 2048">Zero</button>
                  </div>
                {:else}
                  <span class="muted">—</span>
                {/if}
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
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
          <thead><tr><th>id</th><th>name</th><th>servo registers</th><th>travel (stored)</th><th>offset servo / stored</th><th></th><th></th></tr></thead>
          <tbody>
            {#each limitRows as e}
              {@const j = byId(e.joint)}
              {@const [label, cls] = limitStatus(e.args[2])}
              <tr>
                <td class="num">{e.joint}</td>
                <td>{j?.info.name}</td>
                <td class="num">{e.args[2] === -1 ? '—' : `${e.args[0]}..${e.args[1]}`}</td>
                <td class="num">{j ? `${travelOf(j).min}..${travelOf(j).max}` : ''}</td>
                <td>
                  {#if j?.info.model === MODEL_HLS && e.args[2] !== -1}
                    {@const stored = offsetOf(j)}
                    <div class="row nowrap">
                      <span class="num" class:err-text={stored !== null && stored !== e.args[3]}>{e.args[3]} / {stored ?? 'none'}</span>
                      {#if stored === null}
                        <button class="small ghost" disabled={armed} onclick={() => storeOffset(e.joint, e.args[3])} title="keep the servo's offset as the one the travel was measured with">Store</button>
                      {/if}
                    </div>
                  {:else}
                    <span class="muted">—</span>
                  {/if}
                </td>
                <td><span class="pill {cls}">{label}{e.args[0] === 0 && e.args[1] === 0 && j?.info.model === MODEL_HLS && e.args[2] === 0 ? ' (HLS multi-turn)' : ''}</span></td>
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
  tr.dirty td {
    background: var(--warn-weak);
  }
  td :global(input[type='number']) {
    width: 72px;
  }
  .nowrap {
    flex-wrap: nowrap;
    white-space: nowrap;
  }
  .panel {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .hint {
    margin: 0;
  }
  .err-text {
    color: var(--err);
  }
  .warn-text {
    color: var(--warn);
  }
  .center-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
    gap: 8px;
  }
  .center-tile {
    display: flex;
    flex-direction: column;
    gap: 2px;
    padding: 8px 10px;
    border: 1px solid var(--border);
    border-radius: 6px;
  }
  .center-tile.ok {
    border-color: var(--ok);
  }
  .center-tile .big {
    font-size: 1.6em;
    font-variant-numeric: tabular-nums;
  }
</style>
