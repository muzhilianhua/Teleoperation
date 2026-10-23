<script lang="ts">
  // Number input bound to a device value: edits stay local until Enter / blur,
  // then `apply` sends the set command; the field follows the device again.
  let {
    value,
    min,
    max,
    step = 1,
    disabled = false,
    title = '',
    apply,
  }: {
    value: number | null | undefined;
    min?: number;
    max?: number;
    step?: number;
    disabled?: boolean;
    title?: string;
    apply: (v: number) => Promise<unknown> | void;
  } = $props();

  let draft = $state<string | null>(null);
  const shown = $derived(draft ?? (value === null || value === undefined ? '' : String(value)));
  const invalid = $derived.by(() => {
    if (draft === null) return false;
    const v = Number(draft);
    return draft.trim() === '' || !Number.isFinite(v) || (min !== undefined && v < min) || (max !== undefined && v > max);
  });

  async function commit() {
    if (draft === null) return;
    if (invalid || Number(draft) === value) {
      draft = null;
      return;
    }
    const v = Number(draft);
    await apply(v);
    draft = null;
  }
</script>

<input
  type="number"
  class:dirty={draft !== null}
  class:invalid
  value={shown}
  {min}
  {max}
  {step}
  {disabled}
  title={title || (min !== undefined ? `${min} .. ${max}` : '')}
  oninput={(e) => (draft = e.currentTarget.value)}
  onkeydown={(e) => {
    if (e.key === 'Enter') commit();
  }}
  onblur={commit}
/>

<style>
  .dirty {
    border-color: var(--warn);
  }
  .invalid {
    border-color: var(--err);
    background: var(--err-weak);
  }
</style>
