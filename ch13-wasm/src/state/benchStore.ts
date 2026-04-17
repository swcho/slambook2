import { create } from 'zustand';

export interface BenchSample {
  step: number;
  algo: string;
  accel: string;
  ms: number;
  ts: number;
}

interface BenchState {
  samples: BenchSample[];
  push: (s: BenchSample) => void;
  clear: () => void;
}

export const useBenchStore = create<BenchState>((set) => ({
  samples: [],
  push: (sample) => set((s) => ({ samples: [...s.samples, sample] })),
  clear: () => set({ samples: [] }),
}));
