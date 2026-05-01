import { useBenchStore } from '../state/benchStore';

export function PerfMeter({ stepId }: { stepId: number }) {
  const allSamples = useBenchStore((s) => s.samples);
  const samples = allSamples.filter((x) => x.step === stepId);
  const last = samples.at(-1);
  return (
    <section
      aria-label="Performance meter"
      style={{
        border: '1px solid var(--border-subtle)',
        borderRadius: 6,
        padding: 10,
        display: 'flex',
        gap: 16,
        fontSize: 13,
        color: 'var(--color-fg-default)',
        background: 'var(--surface-panel-deep)',
      }}
    >
      <span>
        last: <strong>{last ? `${last.ms.toFixed(2)} ms` : '—'}</strong>
      </span>
      <span>
        samples: <strong>{samples.length}</strong>
      </span>
      <span style={{ color: 'var(--color-fg-faint)' }}>
        (PerfMeter skeleton — uPlot 차트는 Phase C+에서 연결)
      </span>
    </section>
  );
}
