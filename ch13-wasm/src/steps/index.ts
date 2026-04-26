import { lazy, type ComponentType } from 'react';
import { Step01Dataset } from './Step01_Dataset';
import { Step02Camera } from './Step02_Camera';
import { Step03FeatureDetection } from './Step03_FeatureDetection';
import { Step04StereoMatching } from './Step04_StereoMatching';
import { Step05Triangulation } from './Step05_Triangulation';
// Step 6 is r3f / drei / three-heavy → split into its own chunk so the
// initial bundle stays under PLAN §10's 400 KB gzipped budget.
const Step06InitialMap = lazy(() =>
  import('./Step06_InitialMap').then((m) => ({ default: m.Step06InitialMap })),
);

export interface StepMeta {
  id: number;
  slug: string;
  title: string;
  summary: string;
  Component?: ComponentType;
}

export const STEPS: StepMeta[] = [
  { id: 1,  slug: 'dataset',           title: 'Dataset Loader (KITTI)',        summary: '스테레오 쌍 + 캘리브레이션 로딩', Component: Step01Dataset },
  { id: 2,  slug: 'camera',            title: 'Camera Model',                  summary: '핀홀 투영 / 역투영', Component: Step02Camera },
  { id: 3,  slug: 'feature-detection', title: 'Feature Detection',             summary: 'GFTT / FAST / ORB / Harris', Component: Step03FeatureDetection },
  { id: 4,  slug: 'stereo-matching',   title: 'Stereo Matching (LK)',          summary: 'LK 피라미드 광학 흐름', Component: Step04StereoMatching },
  { id: 5,  slug: 'triangulation',     title: 'Triangulation (SVD)',           summary: 'DLT 삼각화 + 조건수', Component: Step05Triangulation },
  { id: 6,  slug: 'initial-map',       title: 'Initial Map Construction',      summary: '첫 키프레임 + 초기 지도', Component: Step06InitialMap },
  { id: 7,  slug: 'frame-tracking',    title: 'Frame Tracking (LK prev→curr)', summary: '투영 초기치 + LK 추적' },
  { id: 8,  slug: 'pose-estimation',   title: 'Pose Estimation (PnP)',         summary: 'g2o / solvePnPRansac / EPnP' },
  { id: 9,  slug: 'keyframe',          title: 'Keyframe Decision',             summary: '키프레임 삽입 정책' },
  { id: 10, slug: 'new-mappoints',     title: 'New MapPoints via Keyframe',    summary: '재검출 + 재삼각화' },
  { id: 11, slug: 'bundle-adjustment', title: 'Bundle Adjustment',             summary: '창 단위 BA + 적응적 chi²' },
  { id: 12, slug: 'sliding-window',    title: 'Sliding Window',                summary: '중복 제거 vs 공간 다양성' },
  { id: 13, slug: 'full-pipeline',     title: 'Full Pipeline (End-to-End VO)', summary: '실시간 VO 궤적 + 지도' },
];
