import { usePipelineStore } from '../state/pipelineStore';

export function VerifyGate({ stepId }: { stepId: number }) {
  const gate = usePipelineStore((s) => s.gates[stepId]);
  const setGate = usePipelineStore((s) => s.setGate);
  const verified = gate?.verified ?? false;

  return (
    <section
      style={{
        border: '1px solid',
        borderColor: verified ? '#3a6' : '#555',
        borderRadius: 6,
        padding: 12,
        background: verified ? '#113a22' : '#161616',
        display: 'flex',
        alignItems: 'center',
        gap: 12,
      }}
    >
      <strong>{verified ? '✅ Verified' : '⬜ Not verified yet'}</strong>
      <span style={{ color: '#888', fontSize: 13 }}>
        (게이트 체크리스트는 각 Step 구현 시 연결)
      </span>
      <button
        type="button"
        onClick={() =>
          setGate(stepId, { verified: !verified, checklist: {} })
        }
        style={{
          marginLeft: 'auto',
          padding: '4px 10px',
          background: '#2a3d5c',
          color: '#fff',
          border: 0,
          borderRadius: 4,
          cursor: 'pointer',
        }}
      >
        {verified ? 'Reset gate' : 'Mark verified (dev)'}
      </button>
    </section>
  );
}
