import { useEffect, useState } from 'react';
import { loadHelloWasm } from '../wasm/loader';

export function Home() {
  const [greeting, setGreeting] = useState<string>('loading…');
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    loadHelloWasm()
      .then((mod) => {
        if (cancelled) return;
        setGreeting(mod.greet('ch13-wasm'));
      })
      .catch((e: unknown) => {
        if (cancelled) return;
        setError(String(e));
      });
    return () => {
      cancelled = true;
    };
  }, []);

  return (
    <div>
      <h1>ch13-wasm Playground</h1>
      <p>
        slambook2 ch13 (Stereo Visual Odometry)을 브라우저에서 단계별로 실험합니다.
        왼쪽 사이드바에서 각 Step으로 이동하세요.
      </p>

      <section style={{ marginTop: 24 }}>
        <h2 style={{ fontSize: 16 }}>환경 점검</h2>
        <ul style={{ lineHeight: 1.8 }}>
          <li>
            <code>crossOriginIsolated</code>:{' '}
            <strong>{String(globalThis.crossOriginIsolated ?? 'unknown')}</strong>
          </li>
          <li>
            <code>SharedArrayBuffer</code>:{' '}
            <strong>{typeof SharedArrayBuffer !== 'undefined' ? 'yes' : 'no'}</strong>
          </li>
          <li>
            <code>hardwareConcurrency</code>:{' '}
            <strong>{navigator.hardwareConcurrency ?? 'n/a'}</strong>
          </li>
          <li>
            <code>navigator.gpu</code>:{' '}
            <strong>
              {'gpu' in navigator && navigator.gpu !== undefined ? 'available' : 'n/a'}
            </strong>
          </li>
          <li>
            WASM hello_world: <strong>{error ? `error: ${error}` : greeting}</strong>
          </li>
        </ul>
      </section>
    </div>
  );
}
