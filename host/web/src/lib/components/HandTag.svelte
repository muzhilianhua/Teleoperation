<script lang="ts">
  // Page header tag: which hand this page works on (switch it in the top bar).
  import { app, HAND_LABEL } from '../state.svelte';
</script>

<span class="pill {app.connected ? 'info' : 'warn'}" title="switch hands in the top bar">
  {HAND_LABEL[app.current]} hand{app.connected ? '' : ' · not connected'}
</span>
