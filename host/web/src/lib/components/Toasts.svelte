<script lang="ts">
  import { app } from '../state.svelte';
</script>

<div class="toasts">
  {#each app.toasts as toast (toast.id)}
    <div class="toast {toast.kind}">{toast.text}</div>
  {/each}
</div>

<style>
  .toasts {
    position: fixed;
    right: 16px;
    bottom: 16px;
    display: flex;
    flex-direction: column;
    gap: 8px;
    z-index: 50;
    max-width: 460px;
  }
  .toast {
    background: var(--panel-2);
    border: 1px solid var(--border);
    border-left: 4px solid var(--accent);
    padding: 8px 12px;
    border-radius: 6px;
    white-space: pre-wrap;
    box-shadow: 0 6px 20px rgba(0, 0, 0, 0.4);
  }
  .toast.error {
    border-left-color: var(--err);
  }
  .toast.success {
    border-left-color: var(--ok);
  }
</style>
