<script lang="ts">
  import EventList from '../lib/components/EventList.svelte';
  import JointCard from '../lib/components/JointCard.svelte';
  import { run } from '../lib/commands';
  import { confirm } from '../lib/confirm.svelte';
  import { jointViews } from '../lib/joints.svelte';
  import { HandFlag, has } from '../lib/protocol';
  import { app } from '../lib/state.svelte';

  const joints = $derived(jointViews());
  const s = $derived(app.slow);

  async function armAll() {
    const enabled = joints.filter((j) => j.slow?.enabled).map((j) => j.info.servo_id);
    if (
      await confirm({
        title: 'Arm every enabled joint?',
        body: `Joints ${enabled.join(', ') || '(none enabled)'} switch torque on, hold, then align to the glove at the arm speed. Keep the glove still.`,
        confirmText: 'Arm',
      })
    )
      run('arm');
  }
</script>

<div class="page">
  <div class="page-header">
    <h1>Dashboard</h1>
    <p>Per-joint state, live values and arming. E-STOP (Esc) is always in the top bar; BOOT on the board does the same.</p>
  </div>

  {#if !app.connected}
    <div class="notice">Connect to the device (top bar) to see the hand.</div>
  {:else}
    <div class="row">
      <button class="primary" onclick={armAll}>Arm all enabled</button>
      <button onclick={() => run('disarm')} disabled={app.armedCount === 0}>Disarm all</button>
      <span class="spacer"></span>
      {#if s}
        <span class="pill" title="loop jitter max / p99">jitter {s.max_jitter_us} / {s.p99_jitter_us} µs</span>
        <span class="pill" title="control cycle execution average / max">exec {s.average_exec_us} / {s.max_exec_us} µs</span>
        <span class="pill {s.overruns ? 'warn' : ''}">overruns {s.overruns}</span>
        {#if s.dropped_telemetry || s.dropped_events}
          <span class="pill warn">dropped tel {s.dropped_telemetry} · ev {s.dropped_events}</span>
        {/if}
      {/if}
      {#if app.tel && !has(app.tel.hand_flags, HandFlag.TORQUE_OFF_CONFIRMED)}
        <span class="pill err">torque-off NOT confirmed</span>
      {/if}
    </div>

    <div class="cards">
      {#each joints as joint (joint.info.servo_id)}
        <JointCard {joint} />
      {/each}
    </div>

    <div class="panel">
      <h2>Recent events</h2>
      <EventList limit={60} />
    </div>
  {/if}
</div>

<style>
  .cards {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
    gap: 12px;
  }
  .spacer {
    flex: 1;
  }
  .panel h2 {
    margin-bottom: 8px;
  }
</style>
