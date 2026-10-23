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
    <span class="name">{joint.info.name}</span>
    <span class="pill">{modelShort(joint.info.model)}</span>
    <span class="spacer"></span>
    <span class="pill {STATE_CLASS[t?.state ?? 0]}">{stateName(t?.state)}</span>
  </div>

  {#if t && t.fault}
    <div class="fault">last off: {faultName(t.fault)}</div>
  {/if}

  <div class="values">
    <div><span class="k">exo</span><span class="num">{fmt(exoDeg(app.info, t?.exo_filtered_counts ?? NaN), 1)}°</span></div>
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
    {#if multiTurnUnhomed}<span class="pill err">HOME!</span>{/if}
    {#if !has(flags, JointFlag.ENC_FRESH) && s && s.encoder_index !== 255}<span class="pill warn">ENC!</span>{/if}
    {#if has(flags, JointFlag.CONTACT)}<span class="pill info">contact</span>{/if}
    {#if !has(flags, JointFlag.MODEL_OK)}<span class="pill err">model?</span>{/if}
    {#if has(flags, JointFlag.TORQUE_ON)}<span class="pill warn">torque</span>{/if}
  </div>

  <div class="actions row">
    {#if armed}
      <button class="small" onclick={() => run(`disarm ${joint.info.servo_id}`)}>Disarm</button>
    {:else}
      <button class="small primary" disabled={!has(flags, JointFlag.READY)} onclick={() => run(`arm ${joint.info.servo_id}`)}>
        Arm
      </button>
    {/if}
    {#if multiTurnUnhomed && !armed}
      <button class="small" onclick={() => run(`home ${joint.info.servo_id}`)} title="glove held at pose A">Home</button>
    {/if}
  </div>
</div>

<style>
  .card {
    display: flex;
    flex-direction: column;
    gap: 8px;
    padding: 12px;
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
  }
  .spacer {
    flex: 1;
  }
  .fault {
    color: var(--err);
    font-size: 12px;
  }
  .values {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 2px 10px;
  }
  .values div {
    display: flex;
    justify-content: space-between;
    gap: 4px;
  }
  .k {
    color: var(--muted);
    font-size: 12px;
  }
  .flags {
    display: flex;
    gap: 4px;
    flex-wrap: wrap;
  }
</style>
