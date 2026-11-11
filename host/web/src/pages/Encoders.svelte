<script lang="ts">
  import HandTag from '../lib/components/HandTag.svelte';
  import { run } from '../lib/commands';
  import { jointViews } from '../lib/joints.svelte';
  import { app } from '../lib/state.svelte';
  import { DEG_PER_COUNT, fmt } from '../lib/units';

  const joints = $derived(jointViews());
  const slow = $derived(app.slow);
  const k = $derived(app.info?.degrees_per_count ?? DEG_PER_COUNT);
  const drives = (i: number) => joints.filter((j) => j.slow?.encoder_index === i).map((j) => `j${j.info.servo_id}`).join(' ');
</script>

<div class="page">
  <div class="page-header">
    <h1>Encoders</h1>
    <HandTag />
    <p>AS5048B channels on the I²C bus, with the joint each one drives. The monitor also reads every servo's position (disarmed only).</p>
  </div>
  {#if !app.connected || !slow}
    <div class="notice">Connect to the device first.</div>
  {:else}
    <div class="panel">
      <table>
        <thead><tr><th>#</th><th>address</th><th>state</th><th>valid</th><th>angle</th><th>raw</th><th>AGC</th><th>magnet</th><th>errors</th><th>drives</th></tr></thead>
        <tbody>
          {#each slow.encoders as e, i}
            <tr>
              <td class="num">{i}</td>
              <td class="num">0x{e.address.toString(16).toUpperCase()}</td>
              <td><span class="pill {e.state === 0 ? 'ok' : e.state === 1 ? 'warn' : 'err'}">{['online', 'no magnet', 'offline'][e.state] ?? e.state}</span></td>
              <td>{e.valid ? 'yes' : 'no'}</td>
              <td class="num">{fmt(e.angle * k, 2)}°</td>
              <td class="num">{e.angle}</td>
              <td class="num">{e.agc}</td>
              <td>{e.magnet_ok ? 'ok' : 'check'}</td>
              <td class="num">{e.errors}</td>
              <td class="mono">{drives(i)}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
    <div class="panel">
      <div class="row">
        <h2>Servo monitor</h2>
        <button disabled={app.armedCount > 0} onclick={() => run(`monitor ${slow.monitor ? 'off' : 'on'}`, { quiet: true })}>Monitor {slow.monitor ? 'off' : 'on'}</button>
        <span class="hint">{slow.monitor ? 'polling one servo per cycle' : 'off'}</span>
      </div>
      {#if slow.monitor}
        <table>
          <thead><tr>{#each joints as j}<th>{j.info.servo_id} {j.info.name}</th>{/each}</tr></thead>
          <tbody>
            <tr>{#each joints as j}<td class="num">{slow.monitor_ok_mask & (1 << j.index) ? slow.monitor_position[j.index] : '—'}</td>{/each}</tr>
          </tbody>
        </table>
      {/if}
    </div>
  {/if}
</div>
