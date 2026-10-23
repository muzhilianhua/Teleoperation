<script lang="ts">
  import { api } from '../api';
  import { confirm } from '../confirm.svelte';
  import type { PortInfo } from '../protocol';
  import { app } from '../state.svelte';

  let ports = $state<PortInfo[]>([]);
  let selected = $state('');
  let busy = $state(false);

  async function refresh() {
    try {
      const r = await api<{ ports: PortInfo[]; suggested: string | null }>('/ports');
      ports = r.ports;
      if (!selected) selected = app.link.port || r.suggested || ports[0]?.device || '';
    } catch (e) {
      app.toast('error', `ports: ${(e as Error).message}`);
    }
  }

  async function connect() {
    busy = true;
    try {
      await api('/connect', { body: { port: selected || null } });
    } catch (e) {
      app.toast('error', `connect: ${(e as Error).message}`);
    } finally {
      busy = false;
    }
  }

  async function disconnect() {
    if (
      app.armedCount > 0 &&
      !(await confirm({
        title: 'Disconnect while armed?',
        body: 'The device leaves the host link but keeps its joints armed and following the glove.',
        confirmText: 'Disconnect',
        danger: true,
      }))
    )
      return;
    await api('/disconnect', { body: {} });
  }

  $effect(() => {
    if (app.wsConnected) refresh();
  });

  const pillClass = $derived(
    !app.wsConnected ? 'err' : app.link.state === 'connected' ? 'ok' : app.link.state === 'error' ? 'err' : 'warn',
  );
  const label = $derived(
    !app.wsConnected ? 'GUI server offline' : app.link.state === 'connected' ? `connected · ${app.link.port}` : app.link.state,
  );
</script>

<div class="row bar">
  <span class="pill {pillClass}" title={app.link.detail}>● {label}</span>
  {#if app.link.state === 'connected' && app.link.firmware}
    <span class="muted mono" title="firmware build">{app.link.firmware}</span>
  {/if}
  {#if app.link.state !== 'connected'}
    <select bind:value={selected} onfocus={refresh} disabled={!app.wsConnected}>
      {#each ports as port (port.device)}
        <option value={port.device}>{port.device}{port.is_esp32 ? ' (ESP32)' : ''}</option>
      {/each}
    </select>
    <button class="primary" onclick={connect} disabled={busy || !app.wsConnected || app.link.state === 'connecting'}>
      Connect
    </button>
    {#if app.link.detail && app.link.state !== 'connecting'}
      <span class="hint detail" title={app.link.detail}>{app.link.detail}</span>
    {/if}
  {:else}
    <button class="ghost small" onclick={disconnect}>Disconnect</button>
  {/if}
</div>

<style>
  .bar {
    flex-wrap: nowrap;
    min-width: 0;
  }
  .detail {
    max-width: 320px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
