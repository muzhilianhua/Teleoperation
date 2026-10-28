<script lang="ts">
  import Plot from '../lib/components/Plot.svelte';
  import { api } from '../lib/api';
  import { confirm } from '../lib/confirm.svelte';
  import type { RecordingMeta, RecordingPreview, TeachStatus } from '../lib/protocol';
  import { app, HAND_LABEL, HANDS, type HandId } from '../lib/state.svelte';
  import { clock, markerAt, REPLAY_SPEEDS, replayBusy } from '../lib/teach';
  import { fmt } from '../lib/units';

  const record = $derived(app.teach?.record);
  const replay = $derived(app.teach?.replay);
  const recording = $derived(record?.recording ?? false);
  const busy = $derived(replayBusy(replay?.state));

  async function call<T>(path: string, init?: { method?: string; body?: unknown }): Promise<T | null> {
    try {
      const result = await api<T>(path, init);
      if (result && typeof result === 'object' && 'record' in (result as object)) app.teach = result as unknown as TeachStatus;
      return result;
    } catch (e) {
      app.toast('error', (e as Error).message, 6000);
      return null;
    }
  }

  // ---- record -------------------------------------------------------------------------

  let recordHands = $state<Record<HandId, boolean>>({ right: true, left: false });
  let name = $state('');
  let markerLabel = $state('');
  const chosenToRecord = $derived(HANDS.filter((h) => recordHands[h] && app.hands[h].connected));

  async function startRecording() {
    await call('/teach/record/start', { body: { hands: chosenToRecord, name: name.trim() } });
  }
  async function stopRecording() {
    const meta = await call<RecordingMeta>('/teach/record/stop', { body: {} });
    if (meta) {
      name = '';
      selectedId = meta.id;
      await refresh();
    }
  }
  async function cancelRecording() {
    const ok = await confirm({ title: 'Discard this recording?', body: 'Nothing is saved.', confirmText: 'Discard', danger: true });
    if (ok) await call('/teach/record/cancel', { body: {} });
  }
  async function addMarker() {
    await call('/teach/record/marker', { body: { label: markerLabel.trim() || `mark ${(record?.markers.length ?? 0) + 1}` } });
    markerLabel = '';
  }

  // ---- recordings -----------------------------------------------------------------------

  let recordings = $state.raw<RecordingMeta[]>([]);
  let selectedId = $state<string | null>(null);
  const selected = $derived(recordings.find((r) => r.id === selectedId) ?? null);

  async function refresh() {
    recordings = (await call<RecordingMeta[]>('/teach/recordings')) ?? recordings;
    if (selectedId && !recordings.some((r) => r.id === selectedId)) selectedId = null;
  }
  $effect(() => {
    app.recordingsVersion;
    if (app.wsConnected) refresh();
  });

  async function rename(r: RecordingMeta) {
    const next = prompt('New name', r.name);
    if (next && next.trim() && next.trim() !== r.name) {
      await call(`/teach/recordings/${encodeURIComponent(r.id)}`, { method: 'PATCH', body: { name: next.trim() } });
      await refresh();
    }
  }
  async function remove(r: RecordingMeta) {
    const ok = await confirm({
      title: `Delete “${r.name}”?`,
      body: `The folder recordings/${r.id} is removed from disk. This cannot be undone.`,
      confirmText: 'Delete',
      danger: true,
    });
    if (ok && (await call(`/teach/recordings/${encodeURIComponent(r.id)}`, { method: 'DELETE' }))) await refresh();
  }

  // ---- preview of the selected recording ------------------------------------------------

  let preview = $state.raw<RecordingPreview | null>(null);
  let previewVersion = $state(0);
  $effect(() => {
    const id = selectedId;
    preview = null;
    if (!id) return;
    call<RecordingPreview>(`/teach/recordings/${encodeURIComponent(id)}/preview`).then((p) => {
      if (selectedId === id) {
        preview = p;
        previewVersion += 1;
      }
    });
  });
  const jointName = (r: RecordingMeta, hand: string, sid: number) =>
    r.hands[hand]?.joints.find((j) => j.servo_id === sid)?.name ?? `j${sid}`;

  // ---- replay -------------------------------------------------------------------------------

  let replayHands = $state<Record<HandId, boolean>>({ right: true, left: true });
  let speed = $state(0.5);
  const recordedHands = $derived(selected ? HANDS.filter((h) => h in selected.hands) : []);
  const chosenToReplay = $derived(recordedHands.filter((h) => replayHands[h]));
  const missing = $derived(chosenToReplay.filter((h) => !app.hands[h].connected));
  // Joints armed for teleoperation (not the REPLAY state a finished replay leaves): the board refuses `replay begin`.
  const REPLAY_STATE = 5;
  const teleopArmed = $derived(
    chosenToReplay.some((h) => app.hands[h].tel?.joints.some((j) => j.state !== 0 && j.state !== REPLAY_STATE)),
  );
  const replayMarkers = $derived(recordings.find((r) => r.id === replay?.recording_id)?.markers ?? []);
  const progress = $derived(replay && replay.duration_s > 0 ? Math.min(1, replay.position_s / replay.duration_s) : 0);

  function jointsOf(r: RecordingMeta, hands: HandId[]): string {
    return hands.map((h) => `${HAND_LABEL[h]}: ${r.hands[h].active.map((s) => jointName(r, h, s)).join(', ') || 'none'}`).join('; ');
  }

  async function prepare() {
    if (!selected) return;
    const ok = await confirm({
      title: 'Prepare replay?',
      body:
        `“${selected.name}” at ${speed}×. These joints are armed and move slowly to the first frame: ${jointsOf(selected, chosenToReplay)}. ` +
        'Keep the hands clear and the E-STOP within reach.',
      confirmText: 'Arm and approach',
      danger: true,
    });
    if (ok) await call('/teach/replay/prepare', { body: { id: selected.id, hands: chosenToReplay, speed } });
  }
  async function go() {
    const ok = await confirm({
      title: 'Start replay?',
      body: `${Object.keys(replay?.hands ?? {}).map((h) => HAND_LABEL[h as HandId]).join(' + ')} play the recorded motion together. The grip limiter and every safety check stay active.`,
      confirmText: 'Play',
      danger: true,
    });
    if (ok) await call('/teach/replay/go', { body: {} });
  }
  async function stopReplay() {
    await call('/teach/replay/stop', { body: {} });
  }

  const stateClass: Record<string, string> = {
    preparing: 'info', ready: 'ok', playing: 'warn', done: 'ok', stopped: '', error: 'err', idle: '',
  };
</script>

<div class="page">
  <div class="page-header">
    <h1>Teach</h1>
    <p>Record the hands while you teleoperate them, then replay the motion on one or both hands.</p>
  </div>

  <!-- Record -->
  <div class="panel">
    <div class="row head">
      <h2>Record</h2>
      {#if recording}
        <span class="pill err">● REC {clock(record?.elapsed_s ?? 0)}</span>
        {#if (record?.waiting_s ?? 0) > 0}<span class="pill info">starts in {fmt(record?.waiting_s, 1)} s</span>{/if}
      {/if}
    </div>
    {#if !recording}
      <div class="row">
        {#each HANDS as h (h)}
          <label class="row" title={app.hands[h].connected ? '' : 'not connected'}>
            <input type="checkbox" bind:checked={recordHands[h]} disabled={!app.hands[h].connected} />
            {HAND_LABEL[h]}
          </label>
        {/each}
        <input id="teach-name" placeholder="name, e.g. pick cup" bind:value={name} maxlength="60" />
        <button class="primary" onclick={startRecording} disabled={!chosenToRecord.length}>Start recording</button>
      </div>
      <p class="hint">
        Arm the joints on the Dashboard and teleoperate as usual. Every control cycle is recorded; the joints that follow the
        glove while recording are the ones a replay moves. Recordings are saved under <code>recordings/</code> (not in git).
      </p>
    {:else}
      <table>
        <thead><tr><th>hand</th><th>samples</th><th>dropped cycles</th><th>armed</th></tr></thead>
        <tbody>
          {#each record?.hands ?? [] as h (h)}
            <tr>
              <td>{HAND_LABEL[h as HandId]}</td>
              <td class="num">{record?.samples[h] ?? 0}</td>
              <td class="num {(record?.dropped[h] ?? 0) > 0 ? 'warn' : ''}">{record?.dropped[h] ?? 0}</td>
              <td class="num">{app.hands[h as HandId].armedCount}</td>
            </tr>
          {/each}
        </tbody>
      </table>
      <div class="row">
        <input id="teach-marker" placeholder="marker label" bind:value={markerLabel} onkeydown={(e) => e.key === 'Enter' && addMarker()} />
        <button onclick={addMarker}>Add marker</button>
        <div class="spacer"></div>
        <button class="ghost" onclick={cancelRecording}>Discard</button>
        <button class="primary" onclick={stopRecording}>Stop and save</button>
      </div>
      {#if record?.markers.length}
        <p class="markers">{#each record.markers as m}<span class="pill">{clock(m.t)} {m.label}</span>{/each}</p>
      {/if}
    {/if}
  </div>

  <!-- Recordings -->
  <div class="panel">
    <div class="row head"><h2>Recordings</h2><button class="ghost small" onclick={refresh}>Refresh</button></div>
    {#if recordings.length === 0}
      <p class="hint">No recordings yet.</p>
    {:else}
      <table>
        <thead><tr><th></th><th>name</th><th>created</th><th>length</th><th>hands: replayed joints</th><th>markers</th><th>dropped</th><th></th></tr></thead>
        <tbody>
          {#each recordings as r (r.id)}
            <tr class:sel={r.id === selectedId} onclick={() => (selectedId = r.id)}>
              <td><input type="radio" name="recording" checked={r.id === selectedId} /></td>
              <td>{r.name}<div class="faint mono">{r.id}</div></td>
              <td>{r.created.replace('T', ' ')}</td>
              <td class="num">{clock(r.duration_s)}</td>
              <td>
                {#each HANDS.filter((h) => h in r.hands) as h}
                  <div><span class="pill">{HAND_LABEL[h]}</span> {r.hands[h].active.map((s) => jointName(r, h, s)).join(', ') || '—'}</div>
                {/each}
              </td>
              <td class="num">{r.markers.length}</td>
              <td class="num">{Object.values(r.hands).reduce((n, h) => n + h.dropped, 0)}</td>
              <td class="actions">
                {#each HANDS.filter((h) => h in r.hands) as h}
                  <a class="small" href={`/api/teach/recordings/${encodeURIComponent(r.id)}/${h}.csv`} download>{h}.csv</a>
                {/each}
                <button class="ghost small" onclick={(e) => (e.stopPropagation(), rename(r))}>Rename</button>
                <button class="ghost small" onclick={(e) => (e.stopPropagation(), remove(r))} disabled={busy && replay?.recording_id === r.id}>Delete</button>
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>

  {#if selected && preview}
    {#each Object.entries(preview.hands) as [hand, p] (hand)}
      {@const sids = p.active.length ? p.active : Object.keys(p.joints).map(Number)}
      <div class="panel">
        <Plot
          title={`${selected.name}: ${HAND_LABEL[hand as HandId]} hand, recorded goal`}
          unit="steps"
          time={p.t}
          series={sids.map((s) => ({ label: jointName(selected, hand, s), data: p.joints[String(s)]?.goal ?? [] }))}
          version={previewVersion}
        />
      </div>
    {/each}
    {#if selected.markers.length}
      <p class="markers">{#each selected.markers as m}<span class="pill">{clock(m.t)} {m.label}</span>{/each}</p>
    {/if}
  {/if}

  <!-- Replay -->
  <div class="panel">
    <div class="row head">
      <h2>Replay</h2>
      {#if replay && replay.state !== 'idle'}<span class="pill {stateClass[replay.state]}">{replay.state}</span>{/if}
    </div>
    {#if busy || (replay && ['done', 'stopped', 'error'].includes(replay.state))}
      <p>
        <strong>{replay?.name}</strong> at {replay?.speed}× — {replay?.message}
      </p>
      <div class="bar"><div class="fill" style:width={`${100 * progress}%`}></div></div>
      <div class="row">
        <span class="num">{clock(replay?.position_s ?? 0)} / {clock(replay?.duration_s ?? 0)}</span>
        {#if markerAt(replayMarkers, replay?.position_s ?? 0)}<span class="pill info">{markerAt(replayMarkers, replay?.position_s ?? 0)?.label}</span>{/if}
        <div class="spacer"></div>
        {#if replay?.state === 'ready'}<button class="primary" onclick={go}>Play</button>{/if}
        {#if busy}<button class="danger" onclick={stopReplay}>Stop (hold pose)</button>{/if}
      </div>
      <table>
        <thead><tr><th>hand</th><th>board phase</th><th>frame</th><th>buffered ahead</th><th>underruns</th><th></th></tr></thead>
        <tbody>
          {#each Object.entries(replay?.hands ?? {}) as [h, s] (h)}
            <tr>
              <td>{HAND_LABEL[h as HandId]}</td>
              <td>{s.phase}</td>
              <td class="num">{s.play_index} / {s.frames}</td>
              <td class="num">{s.sent - s.play_index}</td>
              <td class="num {s.underruns ? 'warn' : ''}">{s.underruns}</td>
              <td class="warn">{s.ended}</td>
            </tr>
          {/each}
        </tbody>
      </table>
      {#if replay && !busy}
        <p class="hint">The replayed joints stay armed and hold their last pose. Use <strong>Disarm all</strong> (or the Dashboard) to release them, then prepare again.</p>
      {/if}
    {/if}
    {#if !busy}
      {#if !selected}
        <p class="hint">Select a recording above.</p>
      {:else}
        <div class="row">
          <span class="muted">“{selected.name}”</span>
          {#each recordedHands as h (h)}
            <label class="row"><input type="checkbox" bind:checked={replayHands[h]} /> {HAND_LABEL[h]}</label>
          {/each}
          <label class="row">speed
            <select bind:value={speed}>
              {#each REPLAY_SPEEDS as s}<option value={s}>{s}×</option>{/each}
            </select>
          </label>
          <button class="primary" onclick={prepare} disabled={!chosenToReplay.length || missing.length > 0 || teleopArmed}>Prepare</button>
        </div>
        {#if missing.length}<p class="hint warn">Not connected: {missing.map((h) => HAND_LABEL[h]).join(', ')}.</p>{/if}
        {#if teleopArmed}<p class="hint warn">Joints are armed for teleoperation: disarm them first.</p>{/if}
        <p class="hint">
          Prepare arms the recorded joints and moves them slowly to the first frame; Play starts all selected hands together.
          A recording only replays on the hand it was recorded on, with the same joint table.
        </p>
      {/if}
    {/if}
  </div>
</div>

<style>
  .head {
    justify-content: space-between;
    margin-bottom: 8px;
  }
  .head h2 {
    margin: 0;
  }
  .spacer {
    flex: 1;
  }
  .markers {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
  }
  tr.sel td {
    background: var(--accent-weak);
  }
  tbody tr {
    cursor: pointer;
  }
  .actions {
    white-space: nowrap;
  }
  .actions a {
    margin-right: 6px;
    color: var(--accent);
  }
  .mono {
    font-family: var(--mono);
    font-size: 11px;
  }
  .faint {
    color: var(--faint);
  }
  .bar {
    height: 8px;
    background: var(--panel-3);
    border-radius: 4px;
    overflow: hidden;
    margin: 8px 0;
  }
  .fill {
    height: 100%;
    background: var(--accent);
  }
  .warn {
    color: var(--warn);
  }
</style>
