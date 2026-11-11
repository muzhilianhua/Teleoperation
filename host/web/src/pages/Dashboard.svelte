<script lang="ts">
  import EventList from '../lib/components/EventList.svelte';
  import HandBoard from '../lib/components/HandBoard.svelte';
  import { app, HAND_LABEL, HANDS } from '../lib/state.svelte';

  function loadView(): 'selected' | 'both' {
    try {
      return localStorage.getItem('teleop.dashboard') === 'both' ? 'both' : 'selected';
    } catch {
      return 'selected';
    }
  }
  let view = $state<'selected' | 'both'>(loadView());
  function setView(v: 'selected' | 'both') {
    view = v;
    try {
      localStorage.setItem('teleop.dashboard', v);
    } catch {
      /* per page load */
    }
  }
</script>

<div class="page" class:wide={view === 'both'}>
  <div class="page-header">
    <h1>Dashboard</h1>
    <div class="switch row">
      <button class="small {view === 'selected' ? 'primary' : ''}" onclick={() => setView('selected')}>{HAND_LABEL[app.current]} hand</button>
      <button class="small {view === 'both' ? 'primary' : ''}" onclick={() => setView('both')}>Both hands</button>
    </div>
    <p>Per-joint state, live values and arming. E-STOP (Esc) in the top bar stops both hands; BOOT on a board stops that hand.</p>
  </div>

  {#if view === 'both'}
    <div class="both">
      {#each HANDS as hand (hand)}
        <HandBoard {hand} compact />
      {/each}
    </div>
    <div class="both">
      {#each HANDS as hand (hand)}
        <div class="panel">
          <h2>{HAND_LABEL[hand]}: recent events</h2>
          <EventList limit={40} {hand} />
        </div>
      {/each}
    </div>
  {:else}
    <HandBoard hand={app.current} />
    {#if app.connected}
      <div class="panel">
        <h2>Recent events</h2>
        <EventList limit={60} />
      </div>
    {/if}
  {/if}
</div>

<style>
  .page.wide {
    max-width: none;
  }
  .both {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 20px;
    align-items: start;
  }
  @media (max-width: 1100px) {
    .both {
      grid-template-columns: minmax(0, 1fr);
    }
  }
  .panel h2 {
    margin-bottom: 8px;
  }
</style>
