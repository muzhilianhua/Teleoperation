<script lang="ts">
  import type { Component } from 'svelte';
  import ConfirmDialog from './lib/components/ConfirmDialog.svelte';
  import ConnectionBar from './lib/components/ConnectionBar.svelte';
  import EStopButton from './lib/components/EStopButton.svelte';
  import Toasts from './lib/components/Toasts.svelte';
  import { run } from './lib/commands';
  import { HandFlag, has } from './lib/protocol';
  import { app } from './lib/state.svelte';
  import { corePages, placeholderPage, pluginPages, type PageDef } from './plugins/registry';

  const pages = $derived<PageDef[]>([
    ...corePages,
    ...app.plugins.map((p) => ({
      id: p.id,
      title: p.title,
      group: 'Plugins' as const,
      load: pluginPages[p.id] ?? placeholderPage,
    })),
  ]);
  const groups = ['Operate', 'Setup', 'Diagnose', 'Plugins'] as const;

  let route = $state(location.hash.slice(2) || 'dashboard');
  const current = $derived(pages.find((p) => p.id === route) ?? pages[0]);
  let PageComponent = $state<Component<any> | null>(null);

  $effect(() => {
    const page = current;
    if (!page) return;
    page.load().then((m) => {
      if (current === page) PageComponent = m.default;
    });
  });

  function navigate(id: string) {
    location.hash = `/${id}`;
  }

  const handFault = $derived(app.tel && app.tel.hand_fault ? app.info?.faults[app.tel.hand_fault] : null);
  const sweeping = $derived(app.tel ? has(app.tel.hand_flags, HandFlag.SWEEP_ACTIVE) : false);
</script>

<svelte:window onhashchange={() => (route = location.hash.slice(2) || 'dashboard')} />

<div class="shell">
  <header class="top">
    <div class="brand">Teleop<span class="muted">Console</span></div>
    <ConnectionBar />
    <div class="spacer"></div>
    {#if app.connected}
      <span class="pill {app.armedCount > 0 ? 'warn' : ''}">{app.armedCount} armed</span>
      {#if sweeping}<span class="pill info">sweep running</span>{/if}
      {#if handFault}<span class="pill err" title="last hand-level fault">{handFault}</span>{/if}
      <button class="ghost" onclick={() => run('disarm')} disabled={app.armedCount === 0}>Disarm all</button>
    {/if}
    <EStopButton />
  </header>

  <nav class="side">
    {#each groups as group}
      {@const items = pages.filter((p) => p.group === group)}
      {#if items.length}
        <div class="group">{group}</div>
        {#each items as page (page.id)}
          <button class="nav {current?.id === page.id ? 'active' : ''}" onclick={() => navigate(page.id)}>
            {page.title}
          </button>
        {/each}
      {/if}
    {/each}
  </nav>

  <main class="main">
    {#if !app.wsConnected}
      <div class="page"><div class="notice">GUI server not reachable: is <code>teleop-host</code> running?</div></div>
    {:else if PageComponent}
      <PageComponent plugin={app.plugins.find((p) => p.id === current?.id)} />
    {/if}
  </main>
</div>

<ConfirmDialog />
<Toasts />

<style>
  .shell {
    display: grid;
    grid-template-columns: 190px 1fr;
    grid-template-rows: auto 1fr;
    height: 100%;
  }
  .top {
    grid-column: 1 / -1;
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 8px 16px;
    background: var(--panel);
    border-bottom: 1px solid var(--border);
    min-width: 0;
  }
  .brand {
    font-weight: 700;
    font-size: 16px;
    letter-spacing: 0.02em;
    margin-right: 8px;
  }
  .brand .muted {
    font-weight: 400;
    margin-left: 4px;
  }
  .spacer {
    flex: 1;
  }
  .side {
    background: var(--panel);
    border-right: 1px solid var(--border);
    padding: 10px 8px;
    display: flex;
    flex-direction: column;
    gap: 2px;
    overflow-y: auto;
  }
  .group {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--faint);
    padding: 12px 10px 4px;
  }
  .nav {
    text-align: left;
    background: transparent;
    border: 1px solid transparent;
    padding: 6px 10px;
  }
  .nav.active {
    background: var(--accent-weak);
    border-color: var(--accent);
  }
  .main {
    overflow-y: auto;
    min-width: 0;
  }
</style>
