// KITTI odometry dataset parser + image loader for the browser.
//
// Mirrors ch13/src/dataset.cpp: each P_i (3x4) row in calib.txt is split into
// intrinsic K (3x3) and a translation t = K^{-1} * P[:,3]. The book downsamples
// the image by 0.5 at load time and scales K to match; `parseKittiCalib`
// accepts the same factor. Baseline is |t| (P0 has t=0, P1 encodes the
// left↔right offset).

export interface KittiCamera {
  id: number;
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  /** |t| in metres — meaningful for P1 (horizontal stereo baseline). */
  baseline: number;
  /** Extrinsic translation in the rectified camera rig frame. */
  t: [number, number, number];
  /** Intrinsic matrix, row-major 9 values, already scaled by `downsample`. */
  K: [number, number, number, number, number, number, number, number, number];
  /** Raw 3x4 projection matrix as read from calib.txt (pre-downsample). */
  P: number[];
  /** Downsample factor applied to K (1.0 = original resolution). */
  downsample: number;
}

export interface StereoFrame {
  index: number;
  width: number;
  height: number;
  /** RGBA pixels decoded from the left image (R=G=B for grayscale sources). */
  left: ImageData;
  right: ImageData;
}

/**
 * Parse a KITTI-odometry-style calib.txt.
 * Expected per line: `P<k>: f11 f12 f13 f14 ... f33 f34` (12 numbers).
 * `downsample` scales K (and cx/cy/fx/fy) uniformly; baseline stays in metres.
 */
export function parseKittiCalib(text: string, downsample = 0.5): KittiCamera[] {
  const lines = text.split('\n').map((l) => l.trim()).filter(Boolean);
  const cameras: KittiCamera[] = [];
  for (const line of lines) {
    const match = line.match(/^P(\d):\s*(.+)$/);
    if (!match) continue;
    const id = Number.parseInt(match[1], 10);
    const nums = match[2].split(/\s+/).map(Number);
    if (nums.length !== 12 || nums.some((n) => !Number.isFinite(n))) {
      throw new Error(`calib.txt: malformed P${id} row: ${line}`);
    }
    cameras.push(buildCamera(id, nums, downsample));
  }
  if (cameras.length === 0) {
    throw new Error('calib.txt contained no P<k> rows');
  }
  return cameras.sort((a, b) => a.id - b.id);
}

function buildCamera(id: number, p: number[], downsample: number): KittiCamera {
  // p is P in row-major 3x4. Split into K (columns 0..2) and last column.
  // K (pre-downsample):
  //   [ p0 p1 p2 ]
  //   [ p4 p5 p6 ]
  //   [ p8 p9 p10]
  const rawK = [p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]];
  const lastCol: [number, number, number] = [p[3], p[7], p[11]];
  const t = multiplyInverseK(rawK, lastCol);
  const baseline = Math.hypot(t[0], t[1], t[2]);

  // Apply ch13's downsample convention: K *= downsample (rows scaled uniformly).
  const K: KittiCamera['K'] = [
    rawK[0] * downsample, rawK[1] * downsample, rawK[2] * downsample,
    rawK[3] * downsample, rawK[4] * downsample, rawK[5] * downsample,
    rawK[6],              rawK[7],              rawK[8],
  ];
  return {
    id,
    fx: K[0],
    fy: K[4],
    cx: K[2],
    cy: K[5],
    baseline,
    t,
    K,
    P: p.slice(),
    downsample,
  };
}

/** Compute K^{-1} * v for a row-major 3x3 K with [0 0 1] as the last row. */
function multiplyInverseK(
  K: number[],
  v: [number, number, number],
): [number, number, number] {
  // K = [[fx, s, cx], [0, fy, cy], [0, 0, 1]]
  const fx = K[0];
  const s = K[1];
  const cx = K[2];
  const fy = K[4];
  const cy = K[5];
  // Inverse:
  //  [1/fx, -s/(fx*fy), (s*cy - cx*fy)/(fx*fy)]
  //  [0,    1/fy,       -cy/fy]
  //  [0,    0,          1]
  const z = v[2];
  const y = (v[1] - cy * z) / fy;
  const x = (v[0] - s * y - cx * z) / fx;
  return [x, y, z];
}

/**
 * Fetch and decode a single stereo pair from `datasetDir`
 * (e.g. `/datasets/kitti05-mini`). Uses fetch → Blob → createImageBitmap →
 * OffscreenCanvas to obtain RGBA ImageData. Filenames follow KITTI's
 * `image_{0,1}/NNNNNN.png` convention.
 */
export async function loadKittiFrame(
  datasetDir: string,
  index: number,
  downsample = 1,
): Promise<StereoFrame> {
  const name = `${String(index).padStart(6, '0')}.png`;
  const [left, right] = await Promise.all([
    fetchImageData(`${datasetDir}/image_0/${name}`, downsample),
    fetchImageData(`${datasetDir}/image_1/${name}`, downsample),
  ]);
  if (left.width !== right.width || left.height !== right.height) {
    throw new Error(
      `stereo size mismatch: L=${left.width}x${left.height} R=${right.width}x${right.height}`,
    );
  }
  return {
    index,
    width: left.width,
    height: left.height,
    left,
    right,
  };
}

async function fetchImageData(url: string, downsample: number): Promise<ImageData> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`fetch ${url} failed: ${res.status}`);
  const blob = await res.blob();
  const bmp = await createImageBitmap(blob);
  const w = Math.max(1, Math.round(bmp.width * downsample));
  const h = Math.max(1, Math.round(bmp.height * downsample));
  // OffscreenCanvas is available in all Chromium/Firefox/Safari versions we target.
  const canvas = new OffscreenCanvas(w, h);
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('OffscreenCanvas 2D context unavailable');
  ctx.drawImage(bmp, 0, 0, w, h);
  return ctx.getImageData(0, 0, w, h);
}

/** Format a 3x3 row-major matrix as 3 fixed-precision rows for UI display. */
export function formatMatrix3(K: readonly number[], decimals = 3): string[] {
  if (K.length !== 9) throw new Error(`formatMatrix3 expects 9 values, got ${K.length}`);
  const rows: string[] = [];
  for (let r = 0; r < 3; r++) {
    const row = [K[r * 3], K[r * 3 + 1], K[r * 3 + 2]]
      .map((v) => v.toFixed(decimals).padStart(12, ' '))
      .join(' ');
    rows.push(row);
  }
  return rows;
}
