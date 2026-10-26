<script lang="ts">
  // One hand on the dashboard: its arming controls, loop health and joint cards.
  import JointCard from './JointCard.svelte';
  import { run } from '../commands';
  import { confirm } from '../confirm.svelte';
  import { jointViews } from '../joints.svelte';
  import { HandFlag, has } from '../protocol';
  import { app, HAND_LABEL, type HandId } from '../state.svelte';

  let { hand, compact = false }: { hand: HandId; compact?: boolean } = $props();

  const h = $derived(app.hands[hand]);
  const joints = $derived(jointViews(hand));
  const s = $derived(h.slow);

  async function armAll() {
    const enabled = joints.filter((j) => j.slow?.enabled).map((j) => j.info.servo_id);
    if (
      await confirm({
        title: `Arm every enabled joint of the ${hand} hand?`,
        body: `Joints ${enabled.join(', ') || '(none enabled)'} switch torque on, hold, then align to the glove at the arm speed. Keep the glove still.`,
        confirmText: 'Arm',
      })
    )
      run('arm', { hand });
  }
</script>

<section class="board" class:compact>
  {#if compact}
    <div class="row title">
      <h2>{HAND_LABEL[hand]} hand</h2>
      <span class="muted mono">{h.port}</span>
    </div>
  {/if}
  {#if !h.connected}
    <div class="notice">{HAND_LABEL[hand]} hand not connected{h.link?.detail ? `: ${h.link.detail}` : ''}. Connect its board in the top bar.</div>
  {:else}
    <div class="row">
      <button class="primary" onclick={armAll}>Arm all enabled</button>
      <button onclick={() => run('disarm', { hand })} disabled={h.armedCount === 0}>Disarm</button>
      <span class="spacer"></span>
      {#if s}
        <span class="pill" title="loop jitter max / p99">jitter {s.max_jitter_us} / {s.p99_jitter_us} µs</span>
        <span class="pill" title="control cycle execution average / max">exec {s.average_exec_us} / {s.max_exec_us} µs</span>
        <span class="pill {s.overruns ? 'warn' : ''}">overruns {s.overruns}</span>
        {#if s.dropped_telemetry || s.dropped_events}
          <span class="pill warn">dropped tel {s.dropped_telemetry} · ev {s.dropped_events}</span>
        {/if}
      {/if}
      {#if h.tel && !has(h.tel.hand_flags, HandFlag.TORQUE_OFF_CONFIRMED)}
        <span class="pill err">torque-off NOT confirmed</span>
      {/if}
    </div>
    <div class="cards">
      {#each joints as joint (joint.info.servo_id)}
        <JointCard {joint} />
      {/each}
    </div>
  {/if}
</section>

<style>
  .board {
    display: flex;
    flex-direction: column;
    gap: 12px;
    min-width: 0;
  }
  .title {
    align-items: baseline;
  }
  .cards {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
    gap: 12px;
  }
  .compact .cards {
    grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
  }
  .spacer {
    flex: 1;
  }
</style>
