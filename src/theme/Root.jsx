/**
 * Root Theme Component - Wraps entire application
 * Provides global providers (AuthProvider, PreferencesProvider)
 * and mounts NavbarItems into the navbar placeholder
 */

import React, { useEffect } from 'react';
import { createRoot } from 'react-dom/client';
import { AuthProvider } from '../contexts/AuthContext';
import { PreferencesProvider } from '../contexts/PreferencesContext';
import NavbarItems from '../components/NavbarItems';

export default function Root({ children }) {
  useEffect(() => {
    // Mount NavbarItems into the navbar placeholder
    const container = document.getElementById('navbar-auth-buttons');
    if (container && !container.hasChildNodes()) {
      const root = createRoot(container);
      root.render(<NavbarItems />);
    }
  }, []);

  return (
    <AuthProvider>
      <PreferencesProvider>
        {children}
      </PreferencesProvider>
    </AuthProvider>
  );
}
