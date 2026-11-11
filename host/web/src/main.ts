import { mount } from 'svelte';
import './app.css';
import App from './App.svelte';
import { socket } from './lib/ws';

socket.start();

export default mount(App, { target: document.getElementById('app')! });
