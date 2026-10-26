<script lang="ts">
  import HandTag from '../lib/components/HandTag.svelte';
  import ParamField from '../lib/components/ParamField.svelte';
  import { run } from '../lib/commands';
  import { jointViews } from '../lib/joints.svelte';
  import { GLOBAL_PARAMS, jointRanges } from '../lib/limits';
  import { MODEL_HLS, NO_ENCODER } from '../lib/protocol';
  import { app } from '../lib/state.svelte';
  import { modelShort } from '../lib/units';

  const joints = $derived(jointViews());
  const armed = $derived(app.armedCount > 0);
  const slow = $derived(app.slow);
  const columns = [
    { key: 'vmax', field: 'vmax', label: 'vmax', title: 'rate-limiter ceiling, steps/s (~1.2 x faster saturation)' },
    { key: 'amax', field: 'amax', label: 'amax', title: 'acceleration, steps/s²' },
    { key: 'vsat', field: 'vsat', label: 'vsat', title: 'measured capability for the tracking check, steps/s' },
    { key: 'trk', field: 'track_steps', label: 'trk', title: 'tracking error threshold, steps' },
    { key: 'grip', field: 'grip_steps', label: 'grip', title: 'grip limiter lead, steps (0 = off)' },
  ] as const;

  const set = (id: number, key: string, v: number | string) => run(`set ${id} ${key} ${v}`, { quiet: true });
</script>

<div class="page">
  <div class="page-header">
    <h1>Parameters</h1>
    <HandTag />
    <p>Per-joint motion, tracking, grip and current settings (live, also while armed) and the global filter / grip settings. Values are in each servo's own steps. Enter or leave a field to apply; <b>Save</b> to keep them.</p>
  </div>

  {#if !app.connected || !app.info || !slow}
    <div class="notice">Connect to the device first.</div>
  {:else}
    <div class="panel">
      <div class="row head">
        <h2>Joints</h2>
        <span class="spacer"></span>
        <button class="primary" disabled={armed} onclick={() => run('save')}>Save to NVS</button>
      </div>
      <table>
        <thead>
          <tr>
            <th>id</th><th>name</th><th>model</th><th title="disarmed only">on</th><th title="disarmed only">enc</th>
            {#each columns as c}<th title={c.title}>{c.label}</th>{/each}
            <th title="HLS goal-current cap, mA (0 = servo protection current)">ilimit</th>
            <th title="HLS over-current trip, % of the cap (0 = off; not saved)">itrip</th>
          </tr>
        </thead>
        <tbody>
          {#each joints as j (j.info.servo_id)}
            {@const s = slow.joints[j.index]}
            {@const r = jointRanges(j.info, app.info)}
            {@const id = j.info.servo_id}
            <tr>
              <td class="num">{id}</td>
              <td>{j.info.name}</td>
              <td><span class="pill">{modelShort(j.info.model)}</span></td>
              <td><input type="checkbox" checked={!!s.enabled} disabled={armed} onchange={(e) => set(id, e.currentTarget.checked ? 'on' : 'off', '')} /></td>
              <td>
                <select value={s.encoder_index} disabled={armed} onchange={(e) => set(id, 'enc', e.currentTarget.value === String(NO_ENCODER) ? 'none' : e.currentTarget.value)}>
                  <option value={NO_ENCODER}>none</option>
                  {#each Array(app.info.encoder_count) as _, i}<option value={i}>#{i}</option>{/each}
                </select>
              </td>
              {#each columns as c}
                <td><ParamField value={s[c.field]} min={r[c.key].min} max={r[c.key].max} apply={(v) => set(id, c.key, v)} /></td>
              {/each}
              {#if j.info.model === MODEL_HLS}
                <td><ParamField value={s.current_limit_ma} min={0} max={r.ilimit.max} apply={(v) => set(id, 'ilimit', v)} /></td>
                <td><ParamField value={s.current_trip_pct} min={0} max={100} apply={(v) => set(id, 'itrip', v)} /></td>
              {:else}
                <td class="muted">—</td><td class="muted">—</td>
              {/if}
            </tr>
          {/each}
        </tbody>
      </table>
      <p class="hint">Hover a column for its meaning; red = outside the firmware's range. <code>enc</code> and <code>on</code> need every joint disarmed.</p>
    </div>

    <div class="panel">
      <h2>Global</h2>
      <table class="globals">
        <tbody>
          {#each GLOBAL_PARAMS as g}
            <tr>
              <td>{g.label}</td>
              <td><ParamField value={slow[g.field]} min={g.min} max={g.max} step={g.step} apply={(v) => run(`set ${g.key} ${v}`, { quiet: true })} /></td>
              <td class="muted">{g.unit} · {g.min} .. {g.max}</td>
            </tr>
          {/each}
          <tr>
            <td>Firmware load trips</td>
            <td><input type="checkbox" checked={slow.load_trip === 1} onchange={(e) => run(`set loadtrip ${e.currentTarget.checked ? 'on' : 'off'}`, { quiet: true })} /></td>
            <td class="muted">off: only the servo's own overload protection releases a grasp</td>
          </tr>
        </tbody>
      </table>
    </div>
  {/if}
</div>

<style>
  .head {
    margin-bottom: 8px;
  }
  .spacer {
    flex: 1;
  }
  td :global(input[type='number']) {
    width: 78px;
  }
  .globals td:first-child {
    width: 220px;
  }
  .globals td:nth-child(2) {
    width: 120px;
  }
  .hint {
    margin: 8px 0 0;
  }
</style>
