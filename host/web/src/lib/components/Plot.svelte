<script lang="ts">
  import uPlot from 'uplot';
  import 'uplot/dist/uPlot.min.css';
  import { onDestroy, onMount } from 'svelte';

  // A strip chart over shared time data; the parent owns the buffers and
  // calls redraw via the `version` prop.
  let {
    title,
    unit = '',
    time,
    series,
    version,
  }: { title: string; unit?: string; time: number[]; series: { label: string; data: (number | null)[] }[]; version: number } =
    $props();

  const COLORS = ['#5b9cf6', '#3fb97f', '#e0a23a', '#ef5d55', '#b78cf2', '#4fc1c9', '#f28cb1', '#c2c86b', '#9aa3b2'];
  let el: HTMLDivElement;
  let plot: uPlot | null = null;
  let width = $state(600);

  function build() {
    plot?.destroy();
    const opts: uPlot.Options = {
      width,
      height: 220,
      title: `${title}${unit ? ` (${unit})` : ''}`,
      scales: { x: { time: false } },
      axes: [
        { stroke: '#959eae', grid: { stroke: '#2b313c' }, label: 's' },
        { stroke: '#959eae', grid: { stroke: '#2b313c' }, size: 60 },
      ],
      legend: { live: true },
      series: [{ label: 't' }, ...series.map((s, i) => ({ label: s.label, stroke: COLORS[i % COLORS.length], width: 1.5, points: { show: false }, spanGaps: true }))],
    };
    plot = new uPlot(opts, [time, ...series.map((s) => s.data)], el);
  }

  onMount(() => {
    const ro = new ResizeObserver(() => {
      width = el.clientWidth;
      plot?.setSize({ width, height: 220 });
    });
    ro.observe(el);
    width = el.clientWidth;
    build();
    return () => ro.disconnect();
  });
  onDestroy(() => plot?.destroy());

  let labels = $derived(series.map((s) => s.label).join('|'));
  let builtFor = '';
  $effect(() => {
    version;
    if (!plot) return;
    if (labels !== builtFor) {
      builtFor = labels;
      build();
    } else {
      plot.setData([time, ...series.map((s) => s.data)]);
    }
  });
</script>

<div class="plot" bind:this={el}></div>

<style>
  .plot :global(.u-title) {
    font-size: 13px;
    font-weight: 600;
    color: var(--text);
  }
  .plot :global(.u-legend) {
    font-size: 12px;
    color: var(--muted);
  }
</style>
