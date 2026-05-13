// URL-driven dataset selection.
//
// Each step reads the active dataset from the `?dataset=<id>` query param.
// Invalid/missing values fall back to DEFAULT_DATASET_ID (kitti05-mini).
// `setDataset` clears the param when switching back to the default, so links
// without `?dataset=` stay clean.

import { useCallback, useMemo } from 'react';
import { useSearchParams } from 'react-router-dom';
import {
  ALL_DATASETS,
  DEFAULT_DATASET_ID,
  getDataset,
  isDatasetId,
  type DatasetDef,
  type DatasetId,
} from './datasets';

export interface UseDatasetResult {
  dataset: DatasetDef;
  setDataset: (id: DatasetId) => void;
  all: readonly DatasetDef[];
}

export function useDataset(): UseDatasetResult {
  const [searchParams, setSearchParams] = useSearchParams();
  const requested = searchParams.get('dataset');
  const dataset = useMemo(() => getDataset(requested), [requested]);

  const setDataset = useCallback(
    (id: DatasetId) => {
      const next = new URLSearchParams(searchParams);
      if (!isDatasetId(id)) return;
      if (id === DEFAULT_DATASET_ID) next.delete('dataset');
      else next.set('dataset', id);
      setSearchParams(next, { replace: false });
    },
    [searchParams, setSearchParams],
  );

  return { dataset, setDataset, all: ALL_DATASETS };
}
