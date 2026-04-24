// Loader for the Phase B / Step 2 Camera WASM module.
// Matches the Embind surface declared in wasm-src/myslam/bindings/bind_camera.cpp.

export interface CameraHandle {
  readonly fx: number;
  readonly fy: number;
  readonly cx: number;
  readonly cy: number;
  readonly baseline: number;
  readonly tx: number;
  readonly ty: number;
  readonly tz: number;
  cameraToPixel(xc: number, yc: number, zc: number): [number, number];
  pixelToCamera(u: number, v: number, depth: number): [number, number, number];
  worldToPixel(xw: number, yw: number, zw: number): [number, number];
  pixelToWorld(u: number, v: number, depth: number): [number, number, number];
  /** Emscripten-owned handle — must be released to avoid leaking WASM heap memory. */
  delete(): void;
}

type CameraCtor = new (
  fx: number,
  fy: number,
  cx: number,
  cy: number,
  baseline: number,
  tx: number,
  ty: number,
  tz: number,
) => CameraHandle;

export interface CameraModule {
  Camera: CameraCtor;
  projectBatch(cam: CameraHandle, xyz: Float64Array): Float64Array;
  roundTripMaxError(cam: CameraHandle, xyz: Float64Array): number;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<CameraModule>;

let cached: Promise<CameraModule> | null = null;

export function loadCameraWasm(variant: 'baseline' = 'baseline'): Promise<CameraModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_camera.${variant}`;
  cached = import(/* @vite-ignore */ `${base}.js`).then(
    (mod: { default: EmscriptenFactory }) =>
      mod.default({
        locateFile: (path: string) => {
          if (path.endsWith('.wasm')) return `${base}.wasm`;
          return `/wasm/${path}`;
        },
      }),
  );
  return cached;
}
