<script lang="ts">
  import { confirmState } from '../confirm.svelte';
</script>

{#if confirmState.request}
  {@const r = confirmState.request}
  <div class="backdrop" role="presentation" onclick={() => confirmState.answer(false)}>
    <div class="dialog panel" role="dialog" aria-modal="true" tabindex="-1" onclick={(e) => e.stopPropagation()} onkeydown={() => {}}>
      <h2>{r.title}</h2>
      <p>{r.body}</p>
      <div class="row end">
        <button onclick={() => confirmState.answer(false)}>Cancel</button>
        <button class={r.danger ? 'danger' : 'primary'} onclick={() => confirmState.answer(true)}>
          {r.confirmText ?? 'Confirm'}
        </button>
      </div>
    </div>
  </div>
{/if}

<style>
  .backdrop {
    position: fixed;
    inset: 0;
    background: rgba(0, 0, 0, 0.55);
    display: grid;
    place-items: center;
    z-index: 40;
  }
  .dialog {
    width: min(480px, 92vw);
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  .dialog p {
    margin: 0;
    color: var(--muted);
    white-space: pre-wrap;
  }
  .end {
    justify-content: flex-end;
  }
</style>
