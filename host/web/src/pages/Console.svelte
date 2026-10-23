<script lang="ts">
  import { tick } from 'svelte';
  import { run } from '../lib/commands';
  import { app } from '../lib/state.svelte';

  let input = $state('');
  let history: string[] = [];
  let cursor = -1;
  let output: HTMLDivElement | undefined = $state();

  // Logs (printed lines) and events in arrival order.
  const lines = $derived(
    [
      ...app.logs.map((l) => ({ id: l.id, kind: l.source === -1 ? 'raw' : 'log', text: l.text, sev: 0 })),
      ...app.events.map((e) => ({ id: e.id, kind: 'event', text: `[ctl] ${e.text}`, sev: e.severity })),
    ]
      .sort((a, b) => a.id - b.id)
      .slice(-800),
  );

  $effect(() => {
    lines.length;
    tick().then(() => output?.scrollTo({ top: output.scrollHeight }));
  });

  async function send(text = input) {
    const line = text.trim();
    if (!line) return;
    history = [line, ...history.filter((h) => h !== line)].slice(0, 100);
    cursor = -1;
    input = '';
    app.addLogs([{ source: 0, text: `> ${line}` }]);
    const result = await run(line, { quiet: true });
    if (!result.ok) app.addLogs([{ source: 0, text: `  ${result.status}: ${result.error}` }]);
    if (result.config) app.addLogs(result.config.trimEnd().split('\n').map((text) => ({ source: 0, text })));
  }

  function onKey(e: KeyboardEvent) {
    if (e.key === 'Enter') send();
    else if (e.key === 'ArrowUp' && history.length) {
      cursor = Math.min(cursor + 1, history.length - 1);
      input = history[cursor];
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      cursor = Math.max(cursor - 1, -1);
      input = cursor >= 0 ? history[cursor] : '';
      e.preventDefault();
    }
  }
</script>

<div class="page full">
  <div class="page-header">
    <h1>Console</h1>
    <p>Any text console command, as on the serial port. Tables print here; events show as <code>[ctl]</code>.</p>
  </div>
  <div class="row">
    {#each ['help', 'status', 'joints', 'enc', 'stats', 'dump'] as quick}
      <button class="small" onclick={() => send(quick)}>{quick}</button>
    {/each}
  </div>
  <div class="out panel mono" bind:this={output}>
    {#each lines as line (line.kind + line.id)}
      <div class="line {line.kind} sev{line.sev}">{line.text}</div>
    {/each}
  </div>
  <input class="cmd mono" placeholder="command, e.g. set 2 vmax 900   (↑/↓ history)" bind:value={input} onkeydown={onKey} />
</div>

<style>
  .full {
    height: 100%;
    max-width: none;
  }
  .out {
    flex: 1;
    min-height: 300px;
    overflow-y: auto;
    white-space: pre-wrap;
    font-size: 12.5px;
    line-height: 1.5;
  }
  .line.event {
    color: var(--accent);
  }
  .line.sev1 {
    color: var(--warn);
  }
  .line.sev2 {
    color: var(--err);
  }
  .line.raw {
    color: var(--faint);
  }
  .cmd {
    width: 100%;
    padding: 8px 10px;
  }
</style>
