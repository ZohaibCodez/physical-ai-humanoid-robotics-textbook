/**
 * ProtectedRoute Component - Wrapper for authenticated-only routes
 * 
 * Shows friendly signup prompt to unauthenticated users instead of hard redirect.
 * Shows loading state while checking authentication status.
 */

import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import SignupPrompt from './SignupPrompt';

export default function ProtectedRoute({ children, promptContext = 'protected-page', customMessage }) {
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

  // Show friendly signup prompt if not authenticated
  if (!isAuthenticated) {
    return (
      <div style={{ padding: '2rem 0' }}>
        <SignupPrompt context={promptContext} message={customMessage} />
      </div>
    );
  }

  // Render protected content
  return children;
}
