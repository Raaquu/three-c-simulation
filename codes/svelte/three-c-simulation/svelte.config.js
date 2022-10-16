import adapter from '@sveltejs/adapter-auto';

/** @type {import('@sveltejs/kit').Config} */
const config = {
	kit: {
		adapter: adapter()
	},
	// Trying to change the global config of vite, not working
	vite: {
		define: {
			global: {'globalThis.global':'globalThis'}
		},
		optimizeDeps: {
			exclude: ['svelte-knobby']
		}
	}
};

export default config;
