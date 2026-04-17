import { create } from 'zustand';

type ParamValue = number | string | boolean;

interface ParamState {
  byStep: Record<number, Record<string, ParamValue>>;
  set: (stepId: number, key: string, value: ParamValue) => void;
  reset: (stepId: number) => void;
}

export const useParamStore = create<ParamState>((set) => ({
  byStep: {},
  set: (stepId, key, value) =>
    set((s) => ({
      byStep: {
        ...s.byStep,
        [stepId]: { ...(s.byStep[stepId] ?? {}), [key]: value },
      },
    })),
  reset: (stepId) =>
    set((s) => {
      const next = { ...s.byStep };
      delete next[stepId];
      return { byStep: next };
    }),
}));
