import { useEffect } from 'react';
import type { ReactNode } from 'react';
import { usePipelineStore } from '../state/pipelineStore';

export interface VerifyItem {
  id: string;
  label: string;
  pass: boolean;
  detail?: string;
}

interface Props {
  stepId: number;
  /** Auto checklist. Gate flips to verified when *every* item has pass=true. */
  items?: VerifyItem[];
  children?: ReactNode;
}

export function VerifyGate({ stepId, items, children }: Props) {
  const gate = usePipelineStore((s) => s.gates[stepId]);
  const setGate = usePipelineStore((s) => s.setGate);

  const autoAll = items !== undefined && items.length > 0 && items.every((i) => i.pass);
  const verified = items !== undefined ? autoAll : (gate?.verified ?? false);

  useEffect(() => {
    if (items === undefined) return;
    const checklist: Record<string, boolean> = {};
    for (const it of items) checklist[it.id] = it.pass;
    const current = gate ?? { verified: false, checklist: {} };
    const checklistChanged =
      Object.keys(current.checklist).length !== items.length ||
      items.some((it) => current.checklist[it.id] !== it.pass);
    if (current.verified === autoAll && !checklistChanged) return;
    setGate(stepId, { verified: autoAll, checklist });
  }, [items, autoAll, gate, setGate, stepId]);

  return (
    <section
      aria-label="Verification gate"
      style={{
        border: '1px solid',
        borderColor: verified ? 'var(--border-success)' : 'var(--border-muted)',
        borderRadius: 6,
        padding: 12,
        background: verified ? 'var(--surface-panel-success)' : 'var(--surface-panel-faint)',
        display: 'flex',
        flexDirection: 'column',
        gap: 8,
      }}
    >
      <div
        role="status"
        aria-live="polite"
        style={{ display: 'flex', alignItems: 'center', gap: 12 }}
      >
        <strong>
          <span aria-hidden="true">{verified ? '✅ ' : '⬜ '}</span>
          {verified ? 'Verified' : 'Not verified yet'}
        </strong>
        {items === undefined && (
          <>
            <span style={{ color: 'var(--color-fg-faint)', fontSize: 13 }}>
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
                background: 'var(--surface-panel-active)',
                color: 'var(--color-fg-strong)',
                border: 0,
                borderRadius: 4,
                cursor: 'pointer',
              }}
            >
              {verified ? 'Reset gate' : 'Mark verified (dev)'}
            </button>
          </>
        )}
      </div>

      {items !== undefined && (
        <ul style={{ listStyle: 'none', margin: 0, padding: 0, display: 'flex', flexDirection: 'column', gap: 4 }}>
          {items.map((it) => (
            <li key={it.id} style={{ fontSize: 13, color: 'var(--color-fg-default)' }}>
              <span
                aria-hidden="true"
                style={{ color: it.pass ? 'var(--color-pass)' : 'var(--color-fail)', marginRight: 8 }}
              >
                {it.pass ? '✔' : '✘'}
              </span>
              <span className="sr-only">{it.pass ? 'Pass: ' : 'Fail: '}</span>
              {it.label}
              {it.detail ? (
                <span style={{ color: 'var(--color-fg-faint)', marginLeft: 8 }}>— {it.detail}</span>
              ) : null}
            </li>
          ))}
        </ul>
      )}

      {children}
    </section>
  );
}
