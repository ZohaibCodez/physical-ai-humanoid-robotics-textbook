/**
 * AuthContext - React Context for global authentication state management
 * 
 * Provides authentication state (user, isAuthenticated, isLoading) and
 * authentication functions (login, logout, signup) to all components.
 */

import React, { createContext, useState, useEffect, useCallback } from 'react';

// API base URL - use window.location for production compatibility
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'zohaibcodez.github.io'
  ? 'https://your-backend-url.railway.app' // Replace with actual production backend URL
  : 'http://localhost:8000';

/**
 * Authentication context type definition
 */
export const AuthContext = createContext({
  user: null,
  preferences: null,
  isAuthenticated: false,
  isLoading: true,
  login: async (email, password) => {},
  logout: async () => {},
  signup: async (userData) => {},
  updateProfile: async (profileData) => {},
  restoreSession: async () => {},
});

/**
 * AuthProvider component - wraps app to provide auth state
 */
export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [preferences, setPreferences] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  /**
   * Restore user session on app mount (check for valid access token)
   */
  const restoreSession = useCallback(async () => {
    setIsLoading(true);
    try {
      const response = await fetch(`${API_BASE_URL}/v1/auth/session`, {
        method: 'GET',
        credentials: 'include', // Send HTTP-only cookies
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
        setPreferences(data.preferences);
        setIsAuthenticated(true);
      } else {
        // No valid session - user is guest
        setUser(null);
        setPreferences(null);
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error('Session restore failed:', error);
      setUser(null);
      setPreferences(null);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Login function - authenticate user with email/password
   */
  const login = useCallback(async (email, password) => {
    try {
      const response = await fetch(`${API_BASE_URL}/v1/auth/login`, {
        method: 'POST',
        credentials: 'include', // Send/receive HTTP-only cookies
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail?.message || 'Login failed');
      }

      const data = await response.json();
      setUser(data.user);
      setPreferences(data.preferences);
      setIsAuthenticated(true);
      
      return { success: true, data };
    } catch (error) {
      console.error('Login error:', error);
      return { success: false, error: error.message };
    }
  }, []);

  /**
   * Signup function - create new user account with profiling
   */
  const signup = useCallback(async (userData) => {
    try {
      const response = await fetch(`${API_BASE_URL}/v1/auth/signup`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail?.message || 'Signup failed');
      }

      const data = await response.json();
      setUser(data.user);
      setPreferences(data.preferences);
      setIsAuthenticated(true);
      
      return { success: true, data };
    } catch (error) {
      console.error('Signup error:', error);
      return { success: false, error: error.message };
    }
  }, []);

  /**
   * Logout function - clear session and remove cookies
   */
  const logout = useCallback(async () => {
    try {
      await fetch(`${API_BASE_URL}/v1/auth/logout`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      // Always clear local state, even if API call fails
      setUser(null);
      setPreferences(null);
      setIsAuthenticated(false);
    }
  }, []);

  /**
   * Update user profile and preferences
   */
  const updateProfile = useCallback(async (profileData) => {
    try {
      const response = await fetch(`${API_BASE_URL}/v1/user/profile`, {
        method: 'PUT',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(profileData),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail?.message || 'Profile update failed');
      }

      const data = await response.json();
      setUser(data.user);
      setPreferences(data.preferences);
      
      return { success: true, data };
    } catch (error) {
      console.error('Profile update error:', error);
      return { success: false, error: error.message };
    }
  }, []);

  // Restore session on mount
  useEffect(() => {
    restoreSession();
  }, [restoreSession]);

  const value = {
    user,
    preferences,
    isAuthenticated,
    isLoading,
    login,
    logout,
    signup,
    updateProfile,
    restoreSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}
