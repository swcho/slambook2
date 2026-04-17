import { Link, NavLink, Outlet } from 'react-router-dom';
import { STEPS } from './steps';

export function App() {
  return (
    <div style={{ display: 'grid', gridTemplateColumns: '260px 1fr', minHeight: '100vh' }}>
      <aside
        style={{
          borderRight: '1px solid #333',
          padding: '16px 12px',
          overflowY: 'auto',
          background: '#1a1a1a',
        }}
      >
        <Link to="/" style={{ textDecoration: 'none' }}>
          <h2 style={{ margin: '0 0 16px', color: '#fff' }}>ch13-wasm</h2>
        </Link>
        <nav style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
          {STEPS.map((step) => (
            <NavLink
              key={step.id}
              to={`/step/${step.slug}`}
              style={({ isActive }) => ({
                display: 'block',
                padding: '6px 8px',
                borderRadius: 4,
                textDecoration: 'none',
                color: isActive ? '#fff' : '#bbb',
                background: isActive ? '#2a3d5c' : 'transparent',
                fontSize: 13,
              })}
            >
              <span style={{ color: '#666', marginRight: 6 }}>
                {String(step.id).padStart(2, '0')}
              </span>
              {step.title}
            </NavLink>
          ))}
        </nav>
      </aside>
      <main style={{ padding: 24, overflow: 'auto' }}>
        <Outlet />
      </main>
    </div>
  );
}
