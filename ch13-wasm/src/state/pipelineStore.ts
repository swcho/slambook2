import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface StepGateState {
  verified: boolean;
  checklist: Record<string, boolean>;
}

interface PipelineState {
  gates: Record<number, StepGateState>;
  setGate: (stepId: number, state: StepGateState) => void;
  resetAll: () => void;
}

export const usePipelineStore = create<PipelineState>()(
  persist(
    (set) => ({
      gates: {},
      setGate: (stepId, state) =>
        set((s) => ({ gates: { ...s.gates, [stepId]: state } })),
      resetAll: () => set({ gates: {} }),
    }),
    { name: 'ch13-wasm:pipeline' },
  ),
);
