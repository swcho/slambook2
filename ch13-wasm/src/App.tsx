import { Link, NavLink, Outlet } from 'react-router-dom';
import { STEPS } from './steps';

export function App() {
  return (
    <div style={{ display: 'grid', gridTemplateColumns: '260px 1fr', minHeight: '100vh' }}>
      <a href="#main-content" className="skip-link">
        Skip to main content
      </a>
      <aside
        aria-labelledby="sidebar-heading"
        style={{
          borderRight: '1px solid var(--border-subtle)',
          padding: '16px 12px',
          overflowY: 'auto',
          background: 'var(--surface-sidebar)',
        }}
      >
        <Link to="/" style={{ textDecoration: 'none' }}>
          <h2
            id="sidebar-heading"
            style={{ margin: '0 0 16px', color: 'var(--color-fg-strong)' }}
          >
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
      </aside>
      <main
        id="main-content"
        tabIndex={-1}
        style={{ padding: 24, overflow: 'auto' }}
      >
        <Outlet />
      </main>
    </div>
  );
}
