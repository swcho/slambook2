export interface HelloModule {
  greet(who: string): string;
  add(a: number, b: number): number;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<HelloModule>;

let cached: Promise<HelloModule> | null = null;

export function loadHelloWasm(variant: 'baseline' = 'baseline'): Promise<HelloModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_hello.${variant}`;
  cached = import(/* @vite-ignore */ `${base}.js`).then((mod: { default: EmscriptenFactory }) => {
    return mod.default({
      locateFile: (path: string) => {
        if (path.endsWith('.wasm')) return `${base}.wasm`;
        return `/wasm/${path}`;
      },
    });
  });
  return cached;
}
