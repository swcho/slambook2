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
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 16 }}>
      <header style={{ borderBottom: '1px solid #333', paddingBottom: 8 }}>
        <div style={{ color: '#888', fontSize: 12 }}>
          Step {String(step.id).padStart(2, '0')}
        </div>
        <h1 style={{ margin: '4px 0' }}>{step.title}</h1>
        <div style={{ color: '#aaa' }}>{step.summary}</div>
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
        <section style={panelStyle}>
          <h3 style={h3Style}>Input</h3>
          {input ?? <div style={placeholderStyle}>(placeholder)</div>}
        </section>
        <section style={panelStyle}>
          <h3 style={h3Style}>Output</h3>
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
  border: '1px solid #333',
  borderRadius: 6,
  padding: 12,
  background: '#181818',
  minHeight: 200,
};

const h3Style: React.CSSProperties = {
  margin: '0 0 8px',
  fontSize: 13,
  color: '#aaa',
  textTransform: 'uppercase',
  letterSpacing: 0.5,
};

const placeholderStyle: React.CSSProperties = { color: '#666' };
