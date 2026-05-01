export function ParamPanel({ stepId: _stepId }: { stepId: number }) {
  return (
    <section
      aria-label="Parameters"
      style={{
        border: '1px solid var(--border-subtle)',
        borderRadius: 6,
        padding: 12,
        background: 'var(--surface-panel)',
      }}
    >
      <h3
        style={{
          margin: '0 0 8px',
          fontSize: 13,
          color: 'var(--color-fg-muted)',
          textTransform: 'uppercase',
          letterSpacing: 0.5,
        }}
      >
        Parameters
      </h3>
      <div style={{ color: 'var(--color-fg-faint)', fontSize: 13 }}>
        (ParamPanel: 스키마 기반 자동 생성 — Phase B+에서 구현)
      </div>
    </section>
  );
}
