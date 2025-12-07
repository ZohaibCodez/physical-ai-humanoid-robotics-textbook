import React from 'react';
import { PreferencesProvider } from '../contexts/PreferencesContext';

/**
 * Root component wrapper - Docusaurus swizzled component
 * Provides global context providers for the entire application
 */
export default function Root({ children }) {
  return (
    <PreferencesProvider>
      {children}
    </PreferencesProvider>
  );
}
