/**
 * Custom Navbar Items Component
 * Feature: 005-ui-ux-redesign
 * 
 * Enhanced navbar with profile dropdown for authenticated users
 * and styled login/signup buttons for guests
 */

import React from 'react';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import ProfileDropdown from '@site/src/components/ProfileDropdown';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './NavbarItems.module.css';

export default function NavbarItems() {
  const { user, logout, isLoading } = useAuth();
  const location = useLocation();

  // Don't show auth buttons on auth pages - check pathname includes these routes
  const isAuthPage = location.pathname.includes('/login') || 
                     location.pathname.includes('/signup') ||
                     location.pathname.includes('/profile');

  // Create redirect URL with current page
  const getAuthUrl = (path) => {
    if (isAuthPage) return path;
    return `${path}?redirect=${encodeURIComponent(location.pathname)}`;
  };

  if (isLoading) {
    return (
      <div className={styles.authButtons}>
        <div className={styles.skeleton} />
      </div>
    );
  }

  if (user) {
    return (
      <div className={styles.userSection}>
        <ProfileDropdown user={user} onLogout={logout} />
      </div>
    );
  }

  if (isAuthPage) {
    return null;
  }

  return (
    <div className={styles.authButtons}>
      <Link to={getAuthUrl('/login')} className={styles.loginButton}>
        Login
      </Link>
      <Link to={getAuthUrl('/signup')} className={styles.signupButton}>
        Sign Up
      </Link>
    </div>
  );
}
