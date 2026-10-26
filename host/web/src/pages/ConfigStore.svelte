<script lang="ts">
  import { api } from '../lib/api';
  import { run } from '../lib/commands';
  import { confirm } from '../lib/confirm.svelte';
  import { app } from '../lib/state.svelte';

  interface BackupMeta {
    name: string;
    saved: string;
    tag: string;
    note: string;
  }
  interface Change {
    scope: string;
    key: string;
    current: string | null;
    wanted: string | null;
  }

  const armed = $derived(app.armedCount > 0);
  let backups = $state<BackupMeta[]>([]);
  let tag = $state('');
  let note = $state('');
  let dumpText = $state('');
  let pasted = $state('');
  let selected = $state<string | null>(null);
  let changes = $state<Change[] | null>(null);
  let restoreSave = $state(true);
  let message = $state('');

  async function refresh() {
    try {
      backups = await api<BackupMeta[]>(`/${app.current}/backups`);
    } catch (e) {
      app.toast('error', (e as Error).message);
    }
  }
  $effect(() => {
    if (app.connected) refresh();
  });

  async function guarded(title: string, body: string, command: string, confirmText: string) {
    if (await confirm({ title, body, confirmText, danger: command !== 'save' })) run(command);
  }

  async function showDump() {
    const r = await run('dump', { quiet: true });
    dumpText = r.config ?? '';
  }

  async function createBackup() {
    try {
      const b = await api<BackupMeta>(`/${app.current}/backups`, { body: { tag, note } });
      app.toast('success', `saved ${b.name}`);
      tag = note = '';
      await refresh();
    } catch (e) {
      app.toast('error', `backup: ${(e as Error).message}`);
    }
  }

  async function importPasted() {
    try {
      const b = await api<BackupMeta>(`/${app.current}/backups/import`, { body: { text: pasted, tag: tag || 'imported', note } });
      app.toast('success', `imported as ${b.name}`);
      pasted = '';
      await refresh();
    } catch (e) {
      app.toast('error', `import: ${(e as Error).message}`);
    }
  }

  async function diff(name: string) {
    selected = name;
    message = '';
    changes = null;
    try {
      changes = (await api<{ changes: Change[] }>(`/${app.current}/backups/${encodeURIComponent(name)}/diff`, { body: {} })).changes;
    } catch (e) {
      message = (e as Error).message;
    }
  }

  async function restore(name: string) {
    await diff(name);
    if (
      !(await confirm({
        title: `Restore ${name}?`,
        body: `${changes?.length ?? '?'} setting(s) change. The backup is applied command by command, verified with a fresh dump${restoreSave ? ', then saved to NVS' : ' (not saved)'}. Any rejected command reverts RAM with \`load\`.`,
        confirmText: 'Restore',
        danger: true,
      }))
    )
      return;
    try {
      const r = await api<{ ok: boolean; message: string }>(`/${app.current}/backups/${encodeURIComponent(name)}/restore`, { body: { save: restoreSave } });
      message = r.message;
      app.toast(r.ok ? 'success' : 'error', r.message, 6000);
      if (r.ok) await diff(name);
    } catch (e) {
      message = (e as Error).message;
    }
  }

  const copy = () => navigator.clipboard.writeText(dumpText).then(() => app.toast('info', 'dump copied'));
</script>

<div class="page">
  <div class="page-header">
    <h1>Config store</h1>
    <p>NVS on the device and calibration backups on this computer (<code>calib/*.txt</code>, the same files as <code>tools/calib.py</code>).</p>
  </div>
  {#if !app.connected}
    <div class="notice">Connect to the device first.</div>
  {:else}
    {#if armed}<div class="notice">Save, load, defaults and restore need every joint disarmed.</div>{/if}

    <div class="panel">
      <h2>Device NVS</h2>
      <div class="row">
        <button class="primary" disabled={armed} onclick={() => guarded('Save to NVS?', 'Overwrites the stored configuration with the current settings.', 'save', 'Save')}>Save</button>
        <button disabled={armed} onclick={() => guarded('Load from NVS?', 'Replaces every setting in RAM with the stored configuration.', 'load', 'Load')}>Load</button>
        <button disabled={armed} onclick={() => guarded('Load defaults?', 'Resets every setting in RAM to the config.h defaults (not saved until you Save).', 'defaults', 'Load defaults')}>Defaults</button>
        <button onclick={showDump}>Show dump</button>
      </div>
      {#if dumpText}
        <div class="row"><button class="small" onclick={copy}>Copy</button><span class="hint">Paste it into the text console (then <code>save</code>) or into Import below.</span></div>
        <pre class="dump">{dumpText}</pre>
      {/if}
    </div>

    <div class="panel">
      <h2>Backups</h2>
      <div class="row">
        <input placeholder="tag (e.g. operator)" bind:value={tag} />
        <input class="wide" placeholder="note" bind:value={note} />
        <button class="primary" onclick={createBackup}>Back up current settings</button>
        <span class="spacer"></span>
        <label class="row"><input type="checkbox" bind:checked={restoreSave} /> save to NVS after restore</label>
      </div>
      <table>
        <thead><tr><th>file</th><th>saved</th><th>tag</th><th>note</th><th></th></tr></thead>
        <tbody>
          {#each backups as b (b.name)}
            <tr class:sel={selected === b.name}>
              <td class="mono">{b.name}</td>
              <td class="muted">{b.saved}</td>
              <td>{b.tag}</td>
              <td class="muted">{b.note}</td>
              <td class="row">
                <button class="small" onclick={() => diff(b.name)}>Diff</button>
                <button class="small danger" disabled={armed} onclick={() => restore(b.name)}>Restore</button>
                <a class="small" href={`/api/${app.current}/backups/${encodeURIComponent(b.name)}/download`} download={b.name}>Download</a>
              </td>
            </tr>
          {:else}
            <tr><td colspan="5" class="muted">No backups yet.</td></tr>
          {/each}
        </tbody>
      </table>
      {#if selected}
        <h3>{selected}: device → backup</h3>
        {#if message}<div class="notice">{message}</div>{/if}
        {#if changes}
          {#if changes.length === 0}
            <span class="pill ok">device matches the backup</span>
          {:else}
            <table class="diff">
              <thead><tr><th>scope</th><th>key</th><th>device now</th><th>backup</th></tr></thead>
              <tbody>
                {#each changes as c}
                  <tr><td>{c.scope === 'global' ? 'global' : `joint ${c.scope}`}</td><td class="mono">{c.key}</td><td class="num">{c.current ?? '—'}</td><td class="num">{c.wanted ?? '—'}</td></tr>
                {/each}
              </tbody>
            </table>
          {/if}
        {/if}
      {/if}
    </div>

    <div class="panel">
      <h2>Import a dump</h2>
      <textarea rows="6" class="mono" placeholder="Paste dump text (defaults ... # end of dump), e.g. from an old console session" bind:value={pasted}></textarea>
      <div class="row"><button disabled={!pasted.trim()} onclick={importPasted}>Import as backup</button><span class="hint">Then Diff / Restore it above.</span></div>
    </div>
  {/if}
</div>

<style>
  .panel {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .dump {
    max-height: 320px;
    overflow: auto;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 8px 10px;
    margin: 0;
    font-family: var(--mono);
    font-size: 12px;
  }
  .wide {
    width: 280px;
  }
  .spacer {
    flex: 1;
  }
  tr.sel td {
    background: var(--accent-weak);
  }
  textarea {
    width: 100%;
    resize: vertical;
  }
  a.small {
    font-size: 12px;
  }
</style>
