import { useEffect, useRef, useState } from 'react';
import { Link, NavLink, Outlet, useLocation } from 'react-router-dom';
import { STEPS } from './steps';

type Theme = 'dark' | 'light';

const THEME_STORAGE_KEY = 'ch13wasm.theme';

function readInitialTheme(): Theme | null {
  if (typeof window === 'undefined') return null;
  const stored = window.localStorage.getItem(THEME_STORAGE_KEY);
  if (stored === 'dark' || stored === 'light') return stored;
  return null;
}

export function App() {
  const [navOpen, setNavOpen] = useState(false);
  const [theme, setTheme] = useState<Theme | null>(() => readInitialTheme());
  const location = useLocation();
  const closeBtnRef = useRef<HTMLButtonElement | null>(null);

  // Apply / remove the data-theme attribute on <html>. When user has no
  // explicit pick the attribute stays absent → CSS falls back to
  // prefers-color-scheme.
  useEffect(() => {
    const root = document.documentElement;
    if (theme === null) {
      root.removeAttribute('data-theme');
    } else {
      root.setAttribute('data-theme', theme);
      window.localStorage.setItem(THEME_STORAGE_KEY, theme);
    }
  }, [theme]);

  // Close the mobile drawer on every route change so a step link tap
  // doesn't leave the sidebar covering the page.
  useEffect(() => {
    setNavOpen(false);
  }, [location.pathname]);

  // Close on Esc when the drawer is open.
  useEffect(() => {
    if (!navOpen) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setNavOpen(false);
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [navOpen]);

  const toggleTheme = () => {
    const current: Theme =
      theme ??
      (window.matchMedia?.('(prefers-color-scheme: light)').matches ? 'light' : 'dark');
    setTheme(current === 'dark' ? 'light' : 'dark');
  };
  const themeLabel: Theme =
    theme ??
    (typeof window !== 'undefined' &&
    window.matchMedia?.('(prefers-color-scheme: light)').matches
      ? 'light'
      : 'dark');

  return (
    <div className="app-shell">
      <a href="#main-content" className="skip-link">
        Skip to main content
      </a>

      <div className="app-mobile-bar">
        <button
          type="button"
          aria-label={navOpen ? 'Close steps menu' : 'Open steps menu'}
          aria-expanded={navOpen}
          aria-controls="app-sidebar"
          onClick={() => setNavOpen((v) => !v)}
        >
          <span aria-hidden="true">☰</span>
        </button>
        <Link
          to="/"
          style={{
            textDecoration: 'none',
            color: 'var(--color-fg-strong)',
            fontWeight: 600,
          }}
        >
          ch13-wasm
        </Link>
      </div>

      <div
        className="app-sidebar-backdrop"
        data-open={navOpen}
        onClick={() => setNavOpen(false)}
        aria-hidden="true"
      />

      <aside
        id="app-sidebar"
        className="app-sidebar"
        data-open={navOpen}
        aria-labelledby="sidebar-heading"
      >
        <Link to="/" style={{ textDecoration: 'none' }}>
          <h2 id="sidebar-heading" style={{ margin: 0, color: 'var(--color-fg-strong)' }}>
            ch13-wasm
          </h2>
        </Link>
        <nav
          aria-label="Steps"
          style={{ display: 'flex', flexDirection: 'column', gap: 4 }}
        >
          {STEPS.map((step) => (
            <NavLink
              key={step.id}
              to={`/step/${step.slug}`}
              style={({ isActive }) => ({
                display: 'block',
                padding: '6px 8px',
                borderRadius: 4,
                textDecoration: 'none',
                color: isActive ? 'var(--color-fg-strong)' : 'var(--color-fg-muted)',
                background: isActive ? 'var(--surface-panel-active)' : 'transparent',
                fontSize: 13,
              })}
            >
              <span
                aria-hidden="true"
                style={{ color: 'var(--color-fg-faint)', marginRight: 6 }}
              >
                {String(step.id).padStart(2, '0')}
              </span>
              <span className="sr-only">Step {step.id}: </span>
              {step.title}
            </NavLink>
          ))}
        </nav>
        <button
          ref={closeBtnRef}
          type="button"
          className="theme-toggle"
          aria-label={`Switch to ${themeLabel === 'dark' ? 'light' : 'dark'} theme`}
          onClick={toggleTheme}
        >
          <span aria-hidden="true">{themeLabel === 'dark' ? '☀ Light' : '🌙 Dark'}</span>
        </button>
      </aside>

      <main
        id="main-content"
        className="app-main"
        tabIndex={-1}
      >
        <Outlet />
      </main>
    </div>
  );
}
