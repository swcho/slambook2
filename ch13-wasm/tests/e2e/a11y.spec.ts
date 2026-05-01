import { test, expect, type Page } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';

// Mirror src/steps/index.ts. Each Step page is checked against axe rules
// for WCAG 2.1 AA. Heading-order and other axe "best-practice" rules are
// scoped *out* — the Step components share an h1 → h3 pattern (legacy from
// the Phase A scaffold) which is well-formed but not strictly hierarchical.
// Promoting all the per-step ParamPanel sub-headings is a separate slice.
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

const WCAG_TAGS = ['wcag2a', 'wcag2aa', 'wcag21a', 'wcag21aa'];

async function runAxe(page: Page) {
  return new AxeBuilder({ page }).withTags(WCAG_TAGS).analyze();
}

test.describe('ch13-wasm a11y (axe-core, WCAG 2.1 AA)', () => {
  test('home: no axe violations', async ({ page }) => {
    await page.goto('/');
    await expect(page.getByRole('heading', { name: 'ch13-wasm Playground' })).toBeVisible();
    const result = await runAxe(page);
    expect(result.violations, JSON.stringify(result.violations, null, 2)).toEqual([]);
  });

  for (const step of STEPS) {
    test(`step ${String(step.id).padStart(2, '0')} (${step.slug}): no axe violations`, async ({ page }) => {
      await page.goto(`/step/${step.slug}`);
      await expect(
        page.getByRole('heading', { level: 1, name: step.title }),
      ).toBeVisible({ timeout: 15_000 });
      // Lazy chunks (Step 6/10/11/12/13) and r3f Canvas mount asynchronously.
      // Wait for the page to settle so axe scans the fully-mounted DOM, not
      // the Suspense fallback.
      await page.waitForLoadState('networkidle');
      const result = await runAxe(page);
      expect(result.violations, JSON.stringify(result.violations, null, 2)).toEqual([]);
    });
  }

  test('skip-to-content link is the first focusable element', async ({ page }) => {
    await page.goto('/');
    await page.keyboard.press('Tab');
    const focused = page.locator(':focus');
    await expect(focused).toHaveText(/Skip to main content/i);
    // Activating it moves focus to <main>.
    await page.keyboard.press('Enter');
    const main = page.locator('main#main-content');
    await expect(main).toBeFocused();
  });
});
