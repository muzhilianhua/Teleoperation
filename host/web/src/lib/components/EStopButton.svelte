<script lang="ts">
  import { estop } from '../commands';

  // Esc anywhere is an E-stop (also when a dialog is open: stopping is never wrong).
  function onKey(e: KeyboardEvent) {
    if (e.key === 'Escape') estop();
  }
</script>

<svelte:window onkeydown={onKey} />

<button class="estop" onclick={estop} title="Torque off on every servo of both hands (Esc)">
  E-STOP <span class="key">Esc</span>
</button>

<style>
  .estop {
    background: var(--estop);
    border: 2px solid #ff7b86;
    color: white;
    font-weight: 700;
    letter-spacing: 0.06em;
    padding: 7px 18px;
    border-radius: 8px;
    box-shadow: 0 0 0 3px rgba(215, 38, 61, 0.25);
  }
  .estop:hover {
    background: #ef2f48;
  }
  .key {
    font-weight: 400;
    font-size: 11px;
    opacity: 0.8;
    margin-left: 6px;
    border: 1px solid rgba(255, 255, 255, 0.5);
    border-radius: 4px;
    padding: 0 4px;
  }
</style>
