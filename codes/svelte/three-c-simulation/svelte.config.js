import vercel from '@sveltejs/adapter-vercel';

/** @type {import('@sveltejs/kit').Config} */
const config = {
	kit: {
		adapter: vercel()
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
