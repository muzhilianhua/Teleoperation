<script lang="ts">
  import { run } from '../commands';
  import { faultName, STATE_CLASS, stateName, travelDefined, type JointView } from '../joints.svelte';
  import { JointFlag, has, JointState } from '../protocol';
  import { app } from '../state.svelte';
  import { exoDeg, fmt, modelShort } from '../units';
  import TravelBar from './TravelBar.svelte';

  let { joint }: { joint: JointView } = $props();

  const t = $derived(joint.tel);
  const s = $derived(joint.slow);
  const flags = $derived(t?.flags ?? 0);
  const armed = $derived((t?.state ?? 0) !== JointState.OFF);
  const enabled = $derived(has(flags, JointFlag.ENABLED));
  const multiTurnUnhomed = $derived(!has(flags, JointFlag.SINGLE_TURN) && !has(flags, JointFlag.HOMED));
</script>

<div class="card panel" class:armed class:disabled={!enabled}>
  <div class="head">
    <span class="id num">{joint.info.servo_id}</span>
    <span class="name" title={joint.info.name}>{joint.info.name}</span>
    <span class="pill">{modelShort(joint.info.model)}</span>
    <span class="spacer"></span>
    <span class="state pill {STATE_CLASS[t?.state ?? 0]}">{stateName(t?.state, joint.hand)}</span>
  </div>

  <div class="values">
    <div><span class="k">exo</span><span class="num">{fmt(exoDeg(app.hands[joint.hand].info, t?.exo_filtered_counts ?? NaN), 1)}°</span></div>
    <div><span class="k">target</span><span class="num">{fmt(t?.target)}</span></div>
    <div><span class="k">sent</span><span class="num">{fmt(t?.sent_goal)}</span></div>
    <div><span class="k">present</span><span class="num">{t ? t.position : '—'}</span></div>
    <div><span class="k">load</span><span class="num">{fmt((t?.load_deci_pct ?? NaN) / 10)}%</span></div>
    {#if has(flags, JointFlag.HAS_CURRENT)}
      <div><span class="k">current</span><span class="num">{t?.current_ma ?? '—'} mA</span></div>
    {:else}
      <div><span class="k">temp</span><span class="num">{t?.temperature_c ?? '—'} °C</span></div>
    {/if}
  </div>

  {#if travelDefined(joint.info)}
    <TravelBar
      min={joint.info.travel_min}
      max={joint.info.travel_max}
      a={s?.servo_a ?? null}
      b={s?.servo_b ?? null}
      goal={armed ? (t?.sent_goal ?? null) : null}
      present={t?.position ?? null}
    />
  {:else}
    <div class="hint">travel undefined in config.h</div>
  {/if}

  <div class="flags">
    <span class="pill {has(flags, JointFlag.READY) ? 'ok' : ''}">{has(flags, JointFlag.READY) ? 'ready' : 'not ready'}</span>
    <span class="pill err" class:inactive={!multiTurnUnhomed}>HOME!</span>
    <span class="pill warn" class:inactive={has(flags, JointFlag.ENC_FRESH) || !s || s.encoder_index === 255}>ENC!</span>
    <span class="pill info" class:inactive={!has(flags, JointFlag.CONTACT)}>contact</span>
    <span class="pill err" class:inactive={has(flags, JointFlag.MODEL_OK)}>model?</span>
    <span class="pill warn" class:inactive={!has(flags, JointFlag.TORQUE_ON)}>torque</span>
  </div>

  <div class="fault" role="status">
    {#if t && t.fault}last off: {faultName(t.fault, joint.hand)}{/if}
  </div>

  <div class="actions row">
    {#if armed}
      <button class="small" onclick={() => run(`disarm ${joint.info.servo_id}`, { hand: joint.hand })}>Disarm</button>
    {:else}
      <button class="small primary" disabled={!has(flags, JointFlag.READY)} onclick={() => run(`arm ${joint.info.servo_id}`, { hand: joint.hand })}>
        Arm
      </button>
    {/if}
    {#if multiTurnUnhomed && !armed}
      <button class="small" onclick={() => run(`home ${joint.info.servo_id}`, { hand: joint.hand })} title="glove held at pose A">Home</button>
    {/if}
  </div>
</div>

<style>
  .card {
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 12px;
    min-width: 0;
  }
  .card.armed {
    border-color: var(--ok);
  }
  .card.disabled {
    opacity: 0.75;
  }
  .head {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .id {
    font-weight: 700;
    color: var(--accent);
  }
  .name {
    font-weight: 600;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    min-width: 0;
  }
  .head .pill {
    flex-shrink: 0;
  }
  .state {
    min-width: calc(6ch + 18px);
    justify-content: center;
  }
  .spacer {
    flex: 1;
  }
  .fault {
    color: var(--err);
    font-size: 12px;
    /* Keep two lines even without a fault; longer firmware text stays readable by scrolling. */
    height: 2.9em;
    flex-shrink: 0;
    overflow: auto;
    overflow-wrap: anywhere;
  }
  .values {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 8px 10px;
  }
  .values > div {
    display: flex;
    flex-direction: column;
    min-width: 0;
  }
  .values .num {
    /* Signed int16 current plus " mA" occupies at most nine monospace characters. */
    min-width: 9ch;
    align-self: flex-end;
    text-align: right;
    white-space: nowrap;
  }
  .k {
    color: var(--muted);
    font-size: 12px;
  }
  .flags {
    /* Inactive slots reserve space without exposing absent warnings to assistive technology. */
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 4px;
  }
  .flags .pill {
    justify-content: center;
  }
  .inactive {
    visibility: hidden;
  }
  .actions {
    margin-top: auto;
    flex-wrap: nowrap;
  }
  .actions button {
    min-width: 80px;
  }
</style>
