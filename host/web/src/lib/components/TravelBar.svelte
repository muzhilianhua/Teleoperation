<script lang="ts">
  // Servo travel with the calibrated A..B span, the goal and the present position.
  let {
    min,
    max,
    a = null,
    b = null,
    goal = null,
    present = null,
  }: { min: number; max: number; a?: number | null; b?: number | null; goal?: number | null; present?: number | null } =
    $props();

  const span = $derived(Math.max(1, max - min));
  const pct = (v: number) => `${Math.min(100, Math.max(0, ((v - min) / span) * 100))}%`;
</script>

<div class="bar" title="travel {min}..{max}">
  {#if a !== null && b !== null && a !== b}
    <div class="cal" style:left={pct(Math.min(a, b))} style:width={`calc(${pct(Math.max(a, b))} - ${pct(Math.min(a, b))})`}></div>
  {/if}
  {#if goal !== null}<div class="goal" style:left={pct(goal)}></div>{/if}
  {#if present !== null}<div class="present" style:left={pct(present)}></div>{/if}
</div>

<style>
  .bar {
    position: relative;
    height: 10px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 5px;
  }
  .cal {
    position: absolute;
    top: 0;
    bottom: 0;
    background: var(--accent-weak);
  }
  .goal,
  .present {
    position: absolute;
    top: -3px;
    width: 2px;
    height: 14px;
    transform: translateX(-1px);
  }
  .goal {
    background: var(--warn);
  }
  .present {
    background: var(--ok);
    width: 3px;
  }
</style>
