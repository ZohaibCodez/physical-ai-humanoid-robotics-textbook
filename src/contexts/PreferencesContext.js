import React, { createContext, useState, useEffect, useMemo } from 'react';

/**
 * PreferencesContext - Manages user preferences with localStorage persistence
 * Stores: theme, sidebar state, chatbot visibility, language preference
 */
export const PreferencesContext = createContext({
  theme: 'light',
  setTheme: () => {},
  sidebarCollapsed: false,
  setSidebarCollapsed: () => {},
  chatbotSeen: false,
  setChatbotSeen: () => {},
  language: 'en',
  setLanguage: () => {},
});

const STORAGE_KEYS = {
  THEME: 'textbook-theme',
  SIDEBAR: 'textbook-sidebar-collapsed',
  CHATBOT: 'textbook-chatbot-seen',
  LANGUAGE: 'textbook-language',
};

export function PreferencesProvider({ children }) {
  // Initialize state from localStorage or defaults
  const [theme, setThemeState] = useState(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem(STORAGE_KEYS.THEME) || 'light';
    }
    return 'light';
  });

  const [sidebarCollapsed, setSidebarCollapsedState] = useState(() => {
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem(STORAGE_KEYS.SIDEBAR);
      // Default to collapsed on mobile
      if (stored === null) {
        return window.innerWidth < 768;
      }
      return stored === 'true';
    }
    return false;
  });

  const [chatbotSeen, setChatbotSeenState] = useState(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem(STORAGE_KEYS.CHATBOT) === 'true';
    }
    return false;
  });

  const [language, setLanguageState] = useState(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem(STORAGE_KEYS.LANGUAGE) || 'en';
    }
    return 'en';
  });

  // Persist theme changes
  const setTheme = (newTheme) => {
    setThemeState(newTheme);
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEYS.THEME, newTheme);
    }
  };

  // Persist sidebar state
  const setSidebarCollapsed = (collapsed) => {
    setSidebarCollapsedState(collapsed);
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEYS.SIDEBAR, String(collapsed));
    }
  };

  // Persist chatbot seen state
  const setChatbotSeen = (seen) => {
    setChatbotSeenState(seen);
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEYS.CHATBOT, String(seen));
    }
  };

  // Persist language preference
  const setLanguage = (lang) => {
    setLanguageState(lang);
    if (typeof window !== 'undefined') {
      localStorage.setItem(STORAGE_KEYS.LANGUAGE, lang);
    }
  };

  // Handle window resize for mobile sidebar auto-collapse
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleResize = () => {
      const isMobile = window.innerWidth < 768;
      if (isMobile && !sidebarCollapsed) {
        setSidebarCollapsed(true);
      }
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [sidebarCollapsed]);

  const value = useMemo(
    () => ({
      theme,
      setTheme,
      sidebarCollapsed,
      setSidebarCollapsed,
      chatbotSeen,
      setChatbotSeen,
      language,
      setLanguage,
    }),
    [theme, sidebarCollapsed, chatbotSeen, language]
  );

  return (
    <PreferencesContext.Provider value={value}>
      {children}
    </PreferencesContext.Provider>
  );
}
