<script lang="ts">
  import { app, HAND_LABEL, HANDS } from '../state.svelte';

  // Pages work on the selected hand; each button shows its hand at a glance.
  const dotClass = (state: string | undefined) =>
    state === 'connected' ? 'ok' : state === 'error' ? 'err' : state ? 'warn' : 'off';
</script>

<div class="switch" role="tablist" aria-label="Hand">
  {#each HANDS as hand (hand)}
    {@const h = app.hands[hand]}
    <button
      role="tab"
      aria-selected={app.current === hand}
      class:active={app.current === hand}
      onclick={() => app.select(hand)}
      title={h.link ? `${h.port} · ${h.link.state}${h.link.detail ? ` · ${h.link.detail}` : ''}` : 'no board'}
    >
      <span class="dot {dotClass(h.link?.state)}"></span>
      {HAND_LABEL[hand]}
      {#if h.armedCount > 0}<span class="armed">{h.armedCount} armed</span>{/if}
    </button>
  {/each}
</div>

<style>
  .switch {
    display: inline-flex;
    border: 1px solid var(--border);
    border-radius: 8px;
    overflow: hidden;
  }
  button {
    border: 0;
    border-radius: 0;
    background: transparent;
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 5px 12px;
  }
  button + button {
    border-left: 1px solid var(--border);
  }
  button.active {
    background: var(--accent-weak);
    color: var(--text);
  }
  .dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--faint);
  }
  .dot.ok {
    background: var(--ok);
  }
  .dot.warn {
    background: var(--warn);
  }
  .dot.err {
    background: var(--err);
  }
  .armed {
    font-size: 11px;
    color: var(--warn);
  }
</style>
