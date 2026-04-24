#!/usr/bin/env node
// Deterministic KITTI-format mini fixture generator.
//
// Phase B note: real KITTI odometry data requires a EULA-gated download, so
// this script synthesises a KITTI-layout dataset that the ch13-wasm pipeline
// can parse and render while the repo stays self-contained. The calib.txt
// contains the *real* KITTI 05 P0..P3 projection matrices; the images are
// procedurally rendered grayscale pairs (checkerboard + moving marker) sized
// to KITTI's 1226x370. Left↔right images differ by a horizontal disparity
// so the stereo offset is visually obvious.
//
// No runtime deps: uses Node's zlib for PNG IDAT deflate + a tiny CRC32 fn.
// Run with: node --experimental-strip-types scripts/gen-kitti-mini.ts
// Re-running is idempotent.

import { mkdirSync, writeFileSync } from 'node:fs';
import { deflateSync } from 'node:zlib';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const W = 1226;
const H = 370;
const N_FRAMES = 5;

type CalibRow = readonly [tag: string, ...values: number[]];

// Real KITTI odometry sequence 05 calibration values (P0..P3).
// Baseline (P0→P1) ≈ 0.5371 m from |P1[0,3]| / fx. These are the values
// ch13/src/dataset.cpp expects to parse.
const KITTI05_CALIB: readonly CalibRow[] = [
  ['P0', 7.070912e2, 0, 6.018873e2, 0,          0, 7.070912e2, 1.831104e2, 0,          0, 0, 1, 0],
  ['P1', 7.070912e2, 0, 6.018873e2, -3.798145e2, 0, 7.070912e2, 1.831104e2, 0,          0, 0, 1, 0],
  ['P2', 7.070912e2, 0, 6.018873e2, 4.688783e1,  0, 7.070912e2, 1.831104e2, 1.178601e-1, 0, 0, 1, 6.203223e-3],
  ['P3', 7.070912e2, 0, 6.018873e2, -3.334597e2, 0, 7.070912e2, 1.831104e2, 1.930130e-3, 0, 0, 1, 3.318498e-3],
];

function formatCalibLine(row: CalibRow): string {
  const [tag, ...nums] = row;
  // Match KITTI's %e width: single digit before '.', 12 digits after, two-digit exponent.
  const cells = nums.map((n) =>
    n.toExponential(12).replace(/e([+-])(\d)$/, 'e$10$2'),
  );
  return `${tag}: ${cells.join(' ')}`;
}

function writeCalib(outDir: string): void {
  const text = KITTI05_CALIB.map(formatCalibLine).join('\n') + '\n';
  writeFileSync(resolve(outDir, 'calib.txt'), text, 'utf8');
}

// --- PNG encoding (grayscale 8-bit, minimal IHDR+IDAT+IEND) ----------------

function crc32(bytes: Uint8Array): number {
  let c = 0xffffffff >>> 0;
  for (let i = 0; i < bytes.length; i++) {
    c = (c ^ bytes[i]) >>> 0;
    for (let k = 0; k < 8; k++) {
      c = ((c >>> 1) ^ (0xedb88320 & -(c & 1))) >>> 0;
    }
  }
  return (c ^ 0xffffffff) >>> 0;
}

function encodeChunk(type: string, data: Uint8Array): Uint8Array {
  const len = data.length;
  const typeBytes = new Uint8Array(4);
  for (let i = 0; i < 4; i++) typeBytes[i] = type.charCodeAt(i);
  const crcInput = new Uint8Array(4 + len);
  crcInput.set(typeBytes, 0);
  crcInput.set(data, 4);
  const out = new Uint8Array(12 + len);
  const view = new DataView(out.buffer);
  view.setUint32(0, len);
  out.set(typeBytes, 4);
  out.set(data, 8);
  view.setUint32(8 + len, crc32(crcInput));
  return out;
}

function encodeGrayPng(w: number, h: number, pixels: Uint8Array): Uint8Array {
  const stride = w + 1;
  const raw = new Uint8Array(stride * h);
  for (let y = 0; y < h; y++) {
    raw[y * stride] = 0; // filter None
    raw.set(pixels.subarray(y * w, (y + 1) * w), y * stride + 1);
  }
  const idat = deflateSync(raw, { level: 9 });

  const ihdr = new Uint8Array(13);
  const ihdrView = new DataView(ihdr.buffer);
  ihdrView.setUint32(0, w);
  ihdrView.setUint32(4, h);
  ihdr[8] = 8;  // bit depth
  ihdr[9] = 0;  // color type: grayscale
  ihdr[10] = 0; // compression: deflate
  ihdr[11] = 0; // filter: adaptive
  ihdr[12] = 0; // interlace: none

  const sig = new Uint8Array([137, 80, 78, 71, 13, 10, 26, 10]);
  const ihdrChunk = encodeChunk('IHDR', ihdr);
  const idatChunk = encodeChunk('IDAT', new Uint8Array(idat));
  const iendChunk = encodeChunk('IEND', new Uint8Array(0));

  const total = new Uint8Array(
    sig.length + ihdrChunk.length + idatChunk.length + iendChunk.length,
  );
  let o = 0;
  total.set(sig, o); o += sig.length;
  total.set(ihdrChunk, o); o += ihdrChunk.length;
  total.set(idatChunk, o); o += idatChunk.length;
  total.set(iendChunk, o);
  return total;
}

// --- Scene rendering -------------------------------------------------------

function clamp8(v: number): number {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v | 0;
}

function renderFrame(frameIdx: number, cameraIdx: 0 | 1): Uint8Array {
  const pixels = new Uint8Array(W * H);
  const forwardShift = frameIdx * 14;         // world moves left as frames advance
  const stereoShift = cameraIdx === 0 ? 0 : 22; // right camera sees scene shifted left

  // Anchor markers that remain visible across frames.
  const markerXLeft = Math.floor(W * 0.48) - forwardShift;
  const markerXRight = markerXLeft - stereoShift;
  const markerY = Math.floor(H * 0.46);

  const m2xLeft = Math.floor(W * 0.25) - forwardShift;
  const m2xRight = m2xLeft - 11;
  const m2y = Math.floor(H * 0.72);

  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const sx = x + stereoShift + forwardShift;
      // Background: horizontal gradient + diagonal stripes for stable texture.
      let v = 40 + ((sx >> 1) & 0x3f);
      if ((sx + y) % 48 < 2) v += 35;
      // 32-px checkerboard to give corner features.
      if ((((sx >> 5) ^ (y >> 5)) & 1) === 1) v += 55;
      // Horizon reference line.
      if (y === Math.floor(H * 0.5)) v = 230;
      // Bright primary anchor.
      const mx = cameraIdx === 0 ? markerXLeft : markerXRight;
      if (Math.abs(x - mx) < 5 && Math.abs(y - markerY) < 5) v = 255;
      // Secondary anchor confirming disparity direction.
      const m2x = cameraIdx === 0 ? m2xLeft : m2xRight;
      if (Math.abs(x - m2x) < 4 && Math.abs(y - m2y) < 4) v = 250;
      pixels[y * W + x] = clamp8(v);
    }
  }
  return pixels;
}

// --- Main ------------------------------------------------------------------

function main(): void {
  const here = dirname(fileURLToPath(import.meta.url));
  const outRoot = resolve(here, '..', 'public', 'datasets', 'kitti05-mini');
  mkdirSync(resolve(outRoot, 'image_0'), { recursive: true });
  mkdirSync(resolve(outRoot, 'image_1'), { recursive: true });

  writeCalib(outRoot);
  console.log(`[gen-kitti-mini] wrote ${outRoot}/calib.txt`);

  for (let f = 0; f < N_FRAMES; f++) {
    for (const cam of [0, 1] as const) {
      const pixels = renderFrame(f, cam);
      const png = encodeGrayPng(W, H, pixels);
      const name = `${String(f).padStart(6, '0')}.png`;
      const out = resolve(outRoot, `image_${cam}`, name);
      writeFileSync(out, png);
      console.log(`[gen-kitti-mini] wrote ${out} (${png.length} B)`);
    }
  }
  console.log(`[gen-kitti-mini] done. ${N_FRAMES} stereo pair(s) @ ${W}x${H}.`);
}

main();
