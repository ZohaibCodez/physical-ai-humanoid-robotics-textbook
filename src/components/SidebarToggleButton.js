import React from 'react';
import styles from './SidebarToggleButton.module.css';

export default function SidebarToggleButton({ onClick, isCollapsed }) {
  return (
    <button
      className={`${styles.toggleButton} ${isCollapsed ? styles.collapsed : ''}`}
      onClick={onClick}
      aria-label="Toggle sidebar"
    >
      <svg width="24" height="24" viewBox="0 0 24 24">
        <path
          fill="currentColor"
          d="M3,6H21V8H3V6M3,11H21V13H3V11M3,16H21V18H3V16Z"
        />
      </svg>
    </button>
  );
}
