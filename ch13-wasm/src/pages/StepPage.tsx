import { StepLayout } from '../components/StepLayout';
import { STEPS } from '../steps';

export function StepPage({ slug }: { slug: string }) {
  const step = STEPS.find((s) => s.slug === slug);
  if (!step) return <div>unknown step</div>;
  return (
    <StepLayout step={step}>
      <div style={{ color: '#888' }}>
        (Phase A 스캐폴드 — 실제 단계 구현은 이후 Phase에서 채워집니다.)
      </div>
    </StepLayout>
  );
}
