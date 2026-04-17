import { useBenchStore } from '../state/benchStore';

export function PerfMeter({ stepId }: { stepId: number }) {
  const samples = useBenchStore((s) =>
    s.samples.filter((x) => x.step === stepId),
  );
  const last = samples.at(-1);
  return (
    <section
      style={{
        border: '1px solid #333',
        borderRadius: 6,
        padding: 10,
        display: 'flex',
        gap: 16,
        fontSize: 13,
        color: '#ccc',
        background: '#141414',
      }}
    >
      <span>
        last: <strong>{last ? `${last.ms.toFixed(2)} ms` : '—'}</strong>
      </span>
      <span>
        samples: <strong>{samples.length}</strong>
      </span>
      <span style={{ color: '#666' }}>
        (PerfMeter skeleton — uPlot 차트는 Phase C+에서 연결)
      </span>
    </section>
  );
}
