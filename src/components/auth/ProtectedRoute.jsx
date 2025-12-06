/**
 * ProtectedRoute Component - Wrapper for authenticated-only routes
 * 
 * Redirects unauthenticated users to login page with return URL.
 * Shows loading state while checking authentication status.
 */

import React from 'react';
import { Redirect } from '@docusaurus/router';
import { useAuth } from '../hooks/useAuth';

export default function ProtectedRoute({ children }) {
  const { isAuthenticated, isLoading } = useAuth();

  // Show loading state while checking authentication
  if (isLoading) {
    return (
      <div style={{ 
        display: 'flex', 
        justifyContent: 'center', 
        alignItems: 'center', 
        minHeight: '400px' 
      }}>
        <div>Loading...</div>
      </div>
    );
  }

  // Redirect to login if not authenticated
  if (!isAuthenticated) {
    const currentPath = typeof window !== 'undefined' ? window.location.pathname : '/';
    return <Redirect to={`/login?redirect=${encodeURIComponent(currentPath)}`} />;
  }

  // Render protected content
  return children;
}
