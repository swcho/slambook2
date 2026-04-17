import { createBrowserRouter, Navigate } from 'react-router-dom';
import { App } from './App';
import { Home } from './pages/Home';
import { StepPage } from './pages/StepPage';
import { STEPS } from './steps';

export const router = createBrowserRouter([
  {
    path: '/',
    element: <App />,
    children: [
      { index: true, element: <Home /> },
      ...STEPS.map((step) => ({
        path: `step/${step.slug}`,
        element: <StepPage slug={step.slug} />,
      })),
      { path: '*', element: <Navigate to="/" replace /> },
    ],
  },
]);
