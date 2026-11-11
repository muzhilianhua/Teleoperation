<script lang="ts">
  import type { PluginInfo } from '../lib/protocol';

  let { plugin }: { plugin?: PluginInfo } = $props();
</script>

<div class="page">
  <div class="page-header">
    <h1>{plugin?.title ?? 'Page'}</h1>
    <span class="pill info">{plugin?.status ?? 'planned'}</span>
  </div>
  <div class="panel">
    <p>{plugin?.description ?? 'Not available yet.'}</p>
    <p class="hint">
      This page is reserved. A plugin adds a Python class in <code>host/src/teleop_host/plugins/</code> (it receives every
      telemetry frame and may add REST routes) and a Svelte page registered in <code>web/src/plugins/registry.ts</code>
      under the same id.
    </p>
  </div>
</div>
