import { test, expect, type Page, type ConsoleMessage } from '@playwright/test';

// Mirror src/steps/index.ts STEPS table. Keep this in lockstep with the
// router; the assertion below verifies each route renders its `<h1>` with
// the expected title text.
const STEPS = [
  { id: 1,  slug: 'dataset',           title: 'Dataset Loader (KITTI)' },
  { id: 2,  slug: 'camera',            title: 'Camera Model' },
  { id: 3,  slug: 'feature-detection', title: 'Feature Detection' },
  { id: 4,  slug: 'stereo-matching',   title: 'Stereo Matching (LK)' },
  { id: 5,  slug: 'triangulation',     title: 'Triangulation (SVD)' },
  { id: 6,  slug: 'initial-map',       title: 'Initial Map Construction' },
  { id: 7,  slug: 'frame-tracking',    title: 'Frame Tracking (LK prev→curr)' },
  { id: 8,  slug: 'pose-estimation',   title: 'Pose Estimation (PnP)' },
  { id: 9,  slug: 'keyframe',          title: 'Keyframe Decision' },
  { id: 10, slug: 'new-mappoints',     title: 'New MapPoints via Keyframe' },
  { id: 11, slug: 'bundle-adjustment', title: 'Bundle Adjustment' },
  { id: 12, slug: 'sliding-window',    title: 'Sliding Window' },
  { id: 13, slug: 'full-pipeline',     title: 'Full Pipeline (End-to-End VO)' },
];

// Console errors we tolerate during smoke. Some Step pages bind to user
// interaction or async pipelines that emit warnings; the smoke gate is
// "no uncaught exception, no React render error".
const TOLERATED_CONSOLE_PATTERNS: RegExp[] = [
  /\[vite\]/i,
  /Download the React DevTools/i,
  /WebGL/i, // r3f canvas init can warn on headless
];

function attachErrorWatcher(page: Page) {
  const errors: string[] = [];
  page.on('pageerror', (err) => errors.push(`pageerror: ${err.message}`));
  page.on('console', (msg: ConsoleMessage) => {
    if (msg.type() !== 'error') return;
    const text = msg.text();
    if (TOLERATED_CONSOLE_PATTERNS.some((re) => re.test(text))) return;
    errors.push(`console.error: ${text}`);
  });
  return errors;
}

test.describe('ch13-wasm smoke', () => {
  test('home renders environment checklist', async ({ page }) => {
    const errors = attachErrorWatcher(page);
    await page.goto('/');
    await expect(page.getByRole('heading', { name: 'ch13-wasm Playground' })).toBeVisible();
    // Sidebar lists all 13 step links.
    for (const step of STEPS) {
      await expect(
        page.getByRole('link', { name: new RegExp(step.title.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')) }),
      ).toBeVisible();
    }
    expect(errors, errors.join('\n')).toEqual([]);
  });

  for (const step of STEPS) {
    test(`step ${String(step.id).padStart(2, '0')} (${step.slug}) renders`, async ({ page }) => {
      const errors = attachErrorWatcher(page);
      await page.goto(`/step/${step.slug}`);
      // h1 of the step appears once StepLayout (or the step's own header)
      // mounts. r3f / lazy chunks / WASM modules can finish loading later;
      // smoke does not wait for them.
      await expect(
        page.getByRole('heading', { level: 1, name: step.title }),
      ).toBeVisible({ timeout: 15_000 });
      expect(errors, errors.join('\n')).toEqual([]);
    });
  }

  test('unknown route redirects to home', async ({ page }) => {
    const errors = attachErrorWatcher(page);
    await page.goto('/step/this-route-does-not-exist');
    await expect(page.getByRole('heading', { name: 'ch13-wasm Playground' })).toBeVisible();
    expect(errors, errors.join('\n')).toEqual([]);
  });
});
