/**
 * ProfileDropdown Component
 * Feature: 005-ui-ux-redesign
 * 
 * Displays user avatar/name with dropdown menu for logged-in users
 */

import React, { useState, useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import styles from './ProfileDropdown.module.css';

export default function ProfileDropdown({ user, onLogout }) {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef(null);
  const history = useHistory();

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    }

    // Close on Escape key
    function handleEscape(event) {
      if (event.key === 'Escape') {
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      document.addEventListener('keydown', handleEscape);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleEscape);
    };
  }, [isOpen]);

  // Get user initials for avatar
  const getInitials = (name) => {
    if (!name) return 'U';
    const parts = name.trim().split(' ');
    if (parts.length >= 2) {
      return (parts[0][0] + parts[parts.length - 1][0]).toUpperCase();
    }
    return name.slice(0, 2).toUpperCase();
  };

  const handleLogout = async () => {
    setIsOpen(false);
    if (onLogout) {
      await onLogout();
      // Redirect to home page after logout
      history.push('/');
    }
  };

  const initials = getInitials(user?.name || user?.email);

  return (
    <div className={styles.profileDropdown} ref={dropdownRef}>
      <button
        className={styles.trigger}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
        aria-label="User menu"
      >
        <div className={styles.avatar}>
          {user?.avatar ? (
            <img src={user.avatar} alt={user.name || 'User'} />
          ) : (
            <span className={styles.initials}>{initials}</span>
          )}
        </div>
        <svg
          className={styles.chevron}
          width="16"
          height="16"
          viewBox="0 0 16 16"
          fill="currentColor"
        >
          <path d="M4.427 5.927l3.396 3.396a.25.25 0 00.354 0l3.396-3.396A.25.25 0 0011.396 5.5H4.604a.25.25 0 00-.177.427z" />
        </svg>
      </button>

      {isOpen && (
        <div className={styles.dropdown} role="menu">
          <div className={styles.header}>
            <div className={styles.userInfo}>
              <div className={styles.name}>{user?.name || 'User'}</div>
              <div className={styles.email}>{user?.email}</div>
            </div>
          </div>

          <div className={styles.divider} />

          <Link
            to="/profile"
            className={styles.item}
            role="menuitem"
            onClick={() => setIsOpen(false)}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 8a3 3 0 100-6 3 3 0 000 6zm2 1H6a4 4 0 00-4 4v1h12v-1a4 4 0 00-4-4z" />
            </svg>
            <span>Profile</span>
          </Link>

          <Link
            to="/settings"
            className={styles.item}
            role="menuitem"
            onClick={() => setIsOpen(false)}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 4.754a3.246 3.246 0 100 6.492 3.246 3.246 0 000-6.492zM5.754 8a2.246 2.246 0 114.492 0 2.246 2.246 0 01-4.492 0z" />
              <path d="M9.796 1.343c-.527-1.79-3.065-1.79-3.592 0l-.094.319a.873.873 0 01-1.255.52l-.292-.16c-1.64-.892-3.433.902-2.54 2.541l.159.292a.873.873 0 01-.52 1.255l-.319.094c-1.79.527-1.79 3.065 0 3.592l.319.094a.873.873 0 01.52 1.255l-.16.292c-.892 1.64.901 3.434 2.541 2.54l.292-.159a.873.873 0 011.255.52l.094.319c.527 1.79 3.065 1.79 3.592 0l.094-.319a.873.873 0 011.255-.52l.292.16c1.64.893 3.434-.902 2.54-2.541l-.159-.292a.873.873 0 01.52-1.255l.319-.094c1.79-.527 1.79-3.065 0-3.592l-.319-.094a.873.873 0 01-.52-1.255l.16-.292c.893-1.64-.902-3.433-2.541-2.54l-.292.159a.873.873 0 01-1.255-.52l-.094-.319zm-2.633.283c.246-.835 1.428-.835 1.674 0l.094.319a1.873 1.873 0 002.693 1.115l.291-.16c.764-.415 1.6.42 1.184 1.185l-.159.292a1.873 1.873 0 001.116 2.692l.318.094c.835.246.835 1.428 0 1.674l-.319.094a1.873 1.873 0 00-1.115 2.693l.16.291c.415.764-.42 1.6-1.185 1.184l-.291-.159a1.873 1.873 0 00-2.693 1.116l-.094.318c-.246.835-1.428.835-1.674 0l-.094-.319a1.873 1.873 0 00-2.692-1.115l-.292.16c-.764.415-1.6-.42-1.184-1.185l.159-.291A1.873 1.873 0 001.945 8.93l-.319-.094c-.835-.246-.835-1.428 0-1.674l.319-.094A1.873 1.873 0 003.06 4.377l-.16-.292c-.415-.764.42-1.6 1.185-1.184l.292.159a1.873 1.873 0 002.692-1.115l.094-.319z" />
            </svg>
            <span>Settings</span>
          </Link>

          <div className={styles.divider} />

          <button
            className={styles.item}
            role="menuitem"
            onClick={handleLogout}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M2 2a2 2 0 012-2h4.5a.5.5 0 010 1H4a1 1 0 00-1 1v10a1 1 0 001 1h4.5a.5.5 0 010 1H4a2 2 0 01-2-2V2zm9.854 1.646a.5.5 0 010 .708L9.207 7H12.5a.5.5 0 010 1H9.207l2.647 2.646a.5.5 0 01-.708.708l-3.5-3.5a.5.5 0 010-.708l3.5-3.5a.5.5 0 01.708 0z" />
            </svg>
            <span>Logout</span>
          </button>
        </div>
      )}
    </div>
  );
}
