export function ParamPanel({ stepId: _stepId }: { stepId: number }) {
  return (
    <section
      style={{
        border: '1px solid #333',
        borderRadius: 6,
        padding: 12,
        background: '#181818',
      }}
    >
      <h3
        style={{
          margin: '0 0 8px',
          fontSize: 13,
          color: '#aaa',
          textTransform: 'uppercase',
          letterSpacing: 0.5,
        }}
      >
        Parameters
      </h3>
      <div style={{ color: '#666', fontSize: 13 }}>
        (ParamPanel: 스키마 기반 자동 생성 — Phase B+에서 구현)
      </div>
    </section>
  );
}
