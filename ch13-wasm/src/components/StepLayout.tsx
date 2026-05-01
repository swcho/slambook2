import type { ReactNode } from 'react';
import type { StepMeta } from '../steps';
import { ParamPanel } from './ParamPanel';
import { PerfMeter } from './PerfMeter';
import { VerifyGate, type VerifyItem } from './VerifyGate';

export interface StepLayoutProps {
  step: StepMeta;
  paramPanel?: ReactNode;
  input?: ReactNode;
  output?: ReactNode;
  children?: ReactNode;
  verifyItems?: VerifyItem[];
  verifyChildren?: ReactNode;
}

export function StepLayout({
  step,
  paramPanel,
  input,
  output,
  children,
  verifyItems,
  verifyChildren,
}: StepLayoutProps) {
  const inputHeadingId = `step-${step.id}-input-heading`;
  const outputHeadingId = `step-${step.id}-output-heading`;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
      <header style={{ borderBottom: '1px solid var(--border-subtle)', paddingBottom: 8 }}>
        <div style={{ color: 'var(--color-fg-faint)', fontSize: 12 }}>
          Step {String(step.id).padStart(2, '0')}
        </div>
        <h1 style={{ margin: '4px 0' }}>{step.title}</h1>
        <div style={{ color: 'var(--color-fg-muted)' }}>{step.summary}</div>
      </header>

      <div
        style={{
          display: 'grid',
          gridTemplateColumns: '260px 1fr 1fr',
          gap: 16,
          alignItems: 'start',
        }}
      >
        {paramPanel ?? <ParamPanel stepId={step.id} />}
        <section style={panelStyle} aria-labelledby={inputHeadingId}>
          <h3 id={inputHeadingId} style={h3Style}>Input</h3>
          {input ?? <div style={placeholderStyle}>(placeholder)</div>}
        </section>
        <section style={panelStyle} aria-labelledby={outputHeadingId}>
          <h3 id={outputHeadingId} style={h3Style}>Output</h3>
          {output ?? <div style={placeholderStyle}>(placeholder)</div>}
        </section>
      </div>

      <PerfMeter stepId={step.id} />

      {children}

      <VerifyGate stepId={step.id} items={verifyItems}>
        {verifyChildren}
      </VerifyGate>
    </div>
  );
}

const panelStyle: React.CSSProperties = {
  border: '1px solid var(--border-subtle)',
  borderRadius: 6,
  padding: 12,
  background: 'var(--surface-panel)',
  minHeight: 200,
};

const h3Style: React.CSSProperties = {
  margin: '0 0 8px',
  fontSize: 13,
  color: 'var(--color-fg-muted)',
  textTransform: 'uppercase',
  letterSpacing: 0.5,
};

const placeholderStyle: React.CSSProperties = { color: 'var(--color-fg-faint)' };
