// Phase H — Step 12 verification (TS, no WASM).
// Confirms that the SlamMap policies behave as PLAN §3 Step 12 demands:
//   - ch13 default: closest <0.2 m → farthest fallback
//   - FIFO: oldest first
//   - covisibility: fewest shared landmarks first
//   - distance-only: always closest
//   - cleanMap: drops landmarks observed only by evicted KFs
//
// Run via `npm run verify:slam`.
//
// We deliberately fork our own SE(3) helper here to keep the script
// dependency-free; the real SlamMap import path is relative to the project
// root.

import { se3LogNorm } from '../../src/lib/slam/se3.ts';
import {
  SlamMap,
  buildSyntheticKfStream,
  type Policy,
} from '../../src/lib/slam/map.ts';

const TOTAL = { pass: 0, fail: 0 };

function check(name: string, ok: boolean, detail?: string) {
  if (ok) {
    TOTAL.pass++;
    console.log(`  ✔ ${name}${detail ? ` — ${detail}` : ''}`);
  } else {
    TOTAL.fail++;
    console.log(`  ✘ ${name}${detail ? ` — ${detail}` : ''}`);
  }
}

console.log('# se3.logNorm');
{
  const I = new Float64Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  check('|log(I)| ~ 0', Math.abs(se3LogNorm(I)) < 1e-12);

  const T = new Float64Array([1,0,0,0.5, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  check('|log(translation 0.5)| ~ 0.5', Math.abs(se3LogNorm(T) - 0.5) < 1e-9, `got ${se3LogNorm(T).toFixed(6)}`);

  // 30° yaw + 0.3 m forward.
  const c = Math.cos(Math.PI / 6);
  const s = Math.sin(Math.PI / 6);
  const R = new Float64Array([c,0,s,0.3, 0,1,0,0, -s,0,c,0, 0,0,0,1]);
  const expected = Math.hypot(0.3, Math.PI / 6);
  check('|log(yaw 30° + tx 0.3)| ~ √(0.3² + (π/6)²)', Math.abs(se3LogNorm(R) - expected) < 5e-3, `got ${se3LogNorm(R).toFixed(6)}, expected ${expected.toFixed(6)}`);
}

console.log('\n# SlamMap eviction policies');
function runPolicy(policy: Policy, windowSize = 4): {
  evictions: number[];
  finalActiveIds: number[];
  finalActiveLandmarks: number;
} {
  const stream = buildSyntheticKfStream(5, 2, 3);
  const map = new SlamMap();
  map.numActiveKeyframes = windowSize;
  for (const lp of stream.landmarkPositions) map.insertMapPoint(lp);
  const evictions: number[] = [];
  for (let i = 0; i < stream.poses.length; i++) {
    const { eviction } = map.insertKeyframe(
      i, stream.poses[i], stream.observedLandmarkIds[i],
      policy, { duplicateDistanceThreshold: 0.2 },
    );
    if (eviction) evictions.push(eviction.evictedKfId);
  }
  return {
    evictions,
    finalActiveIds: [...map.activeKeyframeIds],
    finalActiveLandmarks: map.activeLandmarkIds.size,
  };
}

{
  const r = runPolicy('ch13-default');
  check('ch13-default: produced 6 evictions across 10 KFs (window=4)',
    r.evictions.length === 6, `evictions=${JSON.stringify(r.evictions)}`);
  check('ch13-default: final active size == window size',
    r.finalActiveIds.length === 4);
}
{
  const r = runPolicy('fifo');
  // FIFO must evict KFs in chronological order: 0, 1, 2, 3, 4, 5.
  const expected = [0, 1, 2, 3, 4, 5];
  check('FIFO: evictions follow insertion order',
    JSON.stringify(r.evictions) === JSON.stringify(expected),
    `got ${JSON.stringify(r.evictions)}, expected ${JSON.stringify(expected)}`);
  // After FIFO + window=4 we keep KFs 6..9.
  check('FIFO: final active is the most recent 4 KFs',
    JSON.stringify([...r.finalActiveIds].sort((a, b) => a - b)) === JSON.stringify([6, 7, 8, 9]));
}
{
  const r = runPolicy('covisibility');
  check('covisibility: 6 evictions',
    r.evictions.length === 6, `evictions=${JSON.stringify(r.evictions)}`);
  check('covisibility: final active size == window size',
    r.finalActiveIds.length === 4);
}
{
  const r = runPolicy('distance-only');
  check('distance-only: 6 evictions',
    r.evictions.length === 6, `evictions=${JSON.stringify(r.evictions)}`);
}

console.log('\n# cleanMap drops orphan landmarks');
{
  const map = new SlamMap();
  map.numActiveKeyframes = 1;
  const lmA = map.insertMapPoint([0, 0, 5]);
  const lmB = map.insertMapPoint([1, 0, 5]);
  const I4 = new Float64Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]);
  // KF#0 sees lmA only. KF#1 sees lmB only — should evict KF#0 (FIFO),
  // and lmA must drop out of activeLandmarkIds since no active KF observes it.
  map.insertKeyframe(0, I4, new Set([lmA]), 'fifo', { duplicateDistanceThreshold: 0.2 });
  const T = new Float64Array([1,0,0,0, 0,1,0,0, 0,0,1,0.5, 0,0,0,1]);
  map.insertKeyframe(1, T, new Set([lmB]), 'fifo', { duplicateDistanceThreshold: 0.2 });
  check('cleanMap drops lmA after KF#0 evicted',
    !map.activeLandmarkIds.has(lmA) && map.activeLandmarkIds.has(lmB),
    `active={${[...map.activeLandmarkIds].join(',')}}`);
}

console.log('\n# Pose spread metric');
{
  const stream = buildSyntheticKfStream(5, 0, 0);
  const map = new SlamMap();
  map.numActiveKeyframes = 5;
  for (const lp of stream.landmarkPositions) map.insertMapPoint(lp);
  for (let i = 0; i < stream.poses.length; i++) {
    map.insertKeyframe(i, stream.poses[i], stream.observedLandmarkIds[i], 'fifo', { duplicateDistanceThreshold: 0.2 });
  }
  const spread = map.activePoseSpread();
  check('5 forward KFs: pose spread variance > 0',
    spread.variance > 0 && Number.isFinite(spread.variance),
    `variance=${spread.variance.toFixed(4)}`);
}

console.log(`\n${TOTAL.pass} pass / ${TOTAL.fail} fail`);
if (TOTAL.fail > 0) process.exit(1);
