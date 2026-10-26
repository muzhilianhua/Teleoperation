<script lang="ts">
  import { app, type HandId } from '../state.svelte';

  let { limit = 40, joint = 0, hand }: { limit?: number; joint?: number; hand?: HandId } = $props();
  const h = $derived(app.hands[hand ?? app.current]);

  const events = $derived(
    h.events.filter((e) => !joint || e.joint === joint || e.joint === 0).slice(-limit).reverse(),
  );
  const sevClass = ['info', 'warn', 'err'];
  const time = (t: number) => new Date(t).toLocaleTimeString();
</script>

<div class="list">
  {#each events as e (e.id)}
    <div class="ev {sevClass[e.severity] ?? ''}">
      <span class="t num">{time(e.time)}</span>
      <span class="code">{h.info?.events[e.code] ?? e.code}</span>
      <span class="text">{e.text}</span>
    </div>
  {:else}
    <div class="hint">No events yet.</div>
  {/each}
</div>

<style>
  .list {
    display: flex;
    flex-direction: column;
    gap: 2px;
    max-height: 340px;
    overflow-y: auto;
    font-size: 12.5px;
  }
  .ev {
    display: grid;
    grid-template-columns: 74px 110px 1fr;
    gap: 8px;
    padding: 3px 6px;
    border-left: 3px solid var(--border);
  }
  .ev.warn {
    border-left-color: var(--warn);
  }
  .ev.err {
    border-left-color: var(--err);
    background: var(--err-weak);
  }
  .t {
    color: var(--faint);
    font-size: 11.5px;
  }
  .code {
    color: var(--muted);
    font-family: var(--mono);
    font-size: 11.5px;
  }
  .text {
    white-space: pre-wrap;
    word-break: break-word;
  }
</style>
