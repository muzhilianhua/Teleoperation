<script lang="ts">
  import { run } from '../lib/commands';
  import { jointViews } from '../lib/joints.svelte';
  import { app } from '../lib/state.svelte';
  import { socket } from '../lib/ws';

  const s = $derived(app.slow);
  const joints = $derived(jointViews());
  let rate = $state(1);
  const rejects = $derived(joints.filter((j) => (j.slow?.jump_rejects ?? 0) > 0));
</script>

<div class="page">
  <div class="page-header">
    <h1>Diagnostics</h1>
    <p>Control loop, servo bus, encoder bus and host link statistics (the console's <code>stats</code>).</p>
  </div>
  {#if !app.connected || !s}
    <div class="notice">Connect to the device first.</div>
  {:else}
    <div class="row">
      <button onclick={() => run('stats reset')}>Reset statistics</button>
      <span class="muted">telemetry</span>
      <select bind:value={rate} onchange={() => socket.setTelemetryRate(rate, 10)}>
        <option value={1}>100 Hz</option><option value={2}>50 Hz</option><option value={5}>20 Hz</option><option value={10}>10 Hz</option>
      </select>
    </div>
    <div class="cols">
      <div class="panel">
        <h2>Control loop</h2>
        <table><tbody>
          <tr><td>cycles</td><td class="num">{s.cycles}</td></tr>
          <tr><td>period last</td><td class="num">{s.last_period_us} µs</td></tr>
          <tr><td>jitter max / p99</td><td class="num">{s.max_jitter_us} / ≤{s.p99_jitter_us} µs</td></tr>
          <tr><td>exec avg / max</td><td class="num">{s.average_exec_us} / {s.max_exec_us} µs</td></tr>
          <tr><td>overruns</td><td class="num">{s.overruns}</td></tr>
        </tbody></table>
        <p class="hint">Acceptance: jitter max &lt; 500 µs, average exec ≤ 5 ms, no overruns apart from arming.</p>
      </div>
      <div class="panel">
        <h2>Servo bus</h2>
        <table><tbody>
          <tr><td>sent / ok</td><td class="num">{s.bus_sent} / {s.bus_received}</td></tr>
          <tr><td>timeouts</td><td class="num">{s.bus_timeouts}</td></tr>
          <tr><td>parse / echo errors</td><td class="num">{s.bus_parse_errors} / {s.bus_echo_errors}</td></tr>
          <tr><td>reply last / max</td><td class="num">{s.bus_last_reply_us} / {s.bus_max_reply_us} µs</td></tr>
          <tr><td>echo</td><td>{s.bus_echo ? 'detected, filtered' : 'none'}</td></tr>
        </tbody></table>
      </div>
      <div class="panel">
        <h2>Encoder bus</h2>
        <table><tbody>
          <tr><td>pass avg / max</td><td class="num">{s.encoder_average_us} / {s.encoder_max_us} µs</td></tr>
          <tr><td>skew max</td><td class="num">{s.encoder_max_skew_us} µs</td></tr>
          {#each rejects as j}<tr><td>j{j.info.servo_id} jump rejects</td><td class="num">{j.slow?.jump_rejects}</td></tr>{/each}
        </tbody></table>
      </div>
      <div class="panel">
        <h2>Host link</h2>
        <table><tbody>
          <tr><td>dropped telemetry</td><td class="num">{s.dropped_telemetry}</td></tr>
          <tr><td>dropped events</td><td class="num">{s.dropped_events}</td></tr>
          <tr><td>device rx CRC / malformed</td><td class="num">{s.rx_crc_errors} / {s.rx_malformed}</td></tr>
          <tr><td>firmware</td><td class="mono">{app.info?.firmware}</td></tr>
        </tbody></table>
      </div>
    </div>
    <div class="panel">
      <h2>Commands sent from this GUI</h2>
      <table>
        <tbody>
          {#each app.activity.slice(0, 50) as a (a.id)}
            <tr>
              <td class="num muted">{new Date(a.time).toLocaleTimeString()}</td>
              <td class="mono">{a.text}</td>
              <td><span class="pill {a.ok ? 'ok' : 'err'}">{a.ok ? 'ok' : 'failed'}</span></td>
              <td class="hint">{a.detail}</td>
            </tr>
          {:else}
            <tr><td class="muted">none yet</td></tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</div>

<style>
  .cols {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 12px;
  }
  td:first-child {
    color: var(--muted);
  }
</style>
