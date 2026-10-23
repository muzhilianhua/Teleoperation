// Navigation: core pages (one per group of console commands) and plugin
// pages. A plugin page whose id matches a server plugin (GET /api/plugins)
// is listed under "Plugins"; unknown server plugins get the placeholder.

import type { Component } from 'svelte';

export interface PageDef {
  id: string;
  title: string;
  group: 'Operate' | 'Setup' | 'Diagnose' | 'Plugins';
  load: () => Promise<{ default: Component<any> }>;
}

export const corePages: PageDef[] = [
  { id: 'dashboard', title: 'Dashboard', group: 'Operate', load: () => import('../pages/Dashboard.svelte') },
  { id: 'plots', title: 'Plots', group: 'Operate', load: () => import('../pages/Plots.svelte') },
  { id: 'calibration', title: 'Calibration', group: 'Setup', load: () => import('../pages/Calibration.svelte') },
  { id: 'servo-setup', title: 'Servo setup', group: 'Setup', load: () => import('../pages/ServoSetup.svelte') },
  { id: 'parameters', title: 'Parameters', group: 'Setup', load: () => import('../pages/Parameters.svelte') },
  { id: 'config-store', title: 'Config store', group: 'Setup', load: () => import('../pages/ConfigStore.svelte') },
  { id: 'encoders', title: 'Encoders', group: 'Diagnose', load: () => import('../pages/Encoders.svelte') },
  { id: 'diagnostics', title: 'Diagnostics', group: 'Diagnose', load: () => import('../pages/Diagnostics.svelte') },
  { id: 'console', title: 'Console', group: 'Diagnose', load: () => import('../pages/Console.svelte') },
];

// Pages for server plugins, keyed by plugin id. Missing ones use PluginPlaceholder.
export const pluginPages: Record<string, PageDef['load']> = {};

export const placeholderPage: PageDef['load'] = () => import('../pages/PluginPlaceholder.svelte');
