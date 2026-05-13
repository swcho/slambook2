#!/usr/bin/env node
// Rename files whose basenames are zero-padded integers (e.g. KITTI's
// `0000000000.png`) to a fixed 6-digit width (`000000.png`). Accepts one or
// more glob patterns; each match is renamed in place.
//
// Run: node --experimental-strip-types scripts/rename-to-6digits.ts '<glob>...'
// Flags: --dry-run  preview only (no rename)
//
// Quote the glob so the shell doesn't expand it before the script sees it.

import { existsSync, globSync, renameSync, statSync } from 'node:fs';
import { basename, dirname, extname, join } from 'node:path';

const PAD = 6;

function usage(): never {
  console.error(
    "usage: rename-to-6digits.ts [--dry-run] '<glob>' ['<glob>'...]\n" +
      "  e.g. rename-to-6digits.ts 'public/datasets/kitti-sample/image_*/**/*.png'",
  );
  process.exit(2);
}

function main(): void {
  const args = process.argv.slice(2);
  const dryRun = args.includes('--dry-run');
  const patterns = args.filter((a) => a !== '--dry-run');
  if (patterns.length === 0) usage();

  const seen = new Set<string>();
  const planned = new Map<string, string>(); // src -> dst
  let skipped = 0;
  let alreadyOk = 0;

  for (const pattern of patterns) {
    const matches = globSync(pattern);
    for (const src of matches) {
      if (seen.has(src)) continue;
      seen.add(src);

      try {
        if (!statSync(src).isFile()) continue;
      } catch {
        continue;
      }

      const ext = extname(src);
      const stem = basename(src, ext);
      if (!/^\d+$/.test(stem)) {
        console.warn(`skip (non-numeric stem): ${src}`);
        skipped++;
        continue;
      }
      const n = Number(stem);
      if (!Number.isInteger(n) || n < 0 || n >= 10 ** PAD) {
        console.warn(`skip (out of ${PAD}-digit range): ${src}`);
        skipped++;
        continue;
      }
      const newStem = String(n).padStart(PAD, '0');
      if (newStem === stem) {
        alreadyOk++;
        continue;
      }
      const dst = join(dirname(src), newStem + ext);
      planned.set(src, dst);
    }
  }

  // Collision check: a destination already exists (and isn't the source itself),
  // OR two sources map to the same destination.
  const dstCounts = new Map<string, number>();
  for (const dst of planned.values()) {
    dstCounts.set(dst, (dstCounts.get(dst) ?? 0) + 1);
  }
  const collisions: string[] = [];
  for (const [src, dst] of planned) {
    if ((dstCounts.get(dst) ?? 0) > 1) {
      collisions.push(`multiple sources -> ${dst} (one of them: ${src})`);
    } else if (existsSync(dst) && dst !== src) {
      collisions.push(`target exists: ${dst} (from ${src})`);
    }
  }
  if (collisions.length > 0) {
    console.error('aborting due to collisions:');
    for (const c of collisions) console.error('  ' + c);
    process.exit(1);
  }

  let renamed = 0;
  for (const [src, dst] of planned) {
    if (dryRun) {
      console.log(`[dry-run] ${src} -> ${dst}`);
    } else {
      renameSync(src, dst);
      console.log(`${src} -> ${dst}`);
    }
    renamed++;
  }

  const verb = dryRun ? 'would rename' : 'renamed';
  console.log(
    `done. ${verb} ${renamed}, already ${PAD}-digit ${alreadyOk}, skipped ${skipped}.`,
  );
}

main();
