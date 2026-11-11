<script lang="ts">
  import { api } from '../api';
  import { confirm } from '../confirm.svelte';
  import type { PortInfo } from '../protocol';
  import { app, HAND_LABEL } from '../state.svelte';

  // Boards join a hand by their own GPIO4 strap (HELLO), not by the port picked here.
  let ports = $state<PortInfo[]>([]);
  let inUse = $state<string[]>([]);
  let selected = $state('');
  let busy = $state(false);

  const free = $derived(ports.filter((p) => !inUse.includes(p.device)));

  async function refresh() {
    try {
      const r = await api<{ ports: PortInfo[]; in_use: string[] }>('/ports');
      ports = r.ports;
      inUse = r.in_use;
      if (!free.some((p) => p.device === selected)) selected = free.find((p) => p.is_esp32)?.device ?? free[0]?.device ?? '';
    } catch (e) {
      app.toast('error', `ports: ${(e as Error).message}`);
    }
  }

  async function connect() {
    busy = true;
    try {
      await api('/connect', { body: { port: selected } });
      setTimeout(refresh, 300);
    } catch (e) {
      app.toast('error', `connect: ${(e as Error).message}`);
    } finally {
      busy = false;
    }
  }

  async function disconnect() {
    const hand = app.current;
    if (
      app.armedCount > 0 &&
      !(await confirm({
        title: `Disconnect the ${hand} hand while armed?`,
        body: 'The board leaves the host link but keeps its joints armed and following the glove.',
        confirmText: 'Disconnect',
        danger: true,
      }))
    )
      return;
    await api('/disconnect', { body: { hand } });
    setTimeout(refresh, 300);
  }

  $effect(() => {
    if (app.wsConnected) refresh();
  });
  $effect(() => {
    // A board appearing or leaving changes which ports are free.
    app.hands.right.link?.state;
    app.hands.left.link?.state;
    if (app.wsConnected) refresh();
  });
</script>

<div class="row bar">
  {#if !app.wsConnected}
    <span class="pill err">GUI server offline</span>
  {:else}
    {#if app.hand.link}
      <span class="muted mono port" title={app.link.detail || app.link.firmware}>{app.hand.port}{app.link.firmware ? ` · ${app.link.firmware}` : ''}</span>
      <button class="ghost small" onclick={disconnect}>Disconnect {HAND_LABEL[app.current].toLowerCase()}</button>
    {/if}
    {#if free.length && (!app.hands.right.link || !app.hands.left.link)}
      <select bind:value={selected} onfocus={refresh} aria-label="Serial port">
        {#each free as port (port.device)}
          <option value={port.device}>{port.device}{port.is_esp32 ? ' (ESP32)' : ''}</option>
        {/each}
      </select>
      <button class="primary small" onclick={connect} disabled={busy || !selected}>Connect board</button>
    {/if}
  {/if}
</div>

<style>
  .bar {
    flex-wrap: nowrap;
    min-width: 0;
  }
  .port {
    max-width: 260px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
