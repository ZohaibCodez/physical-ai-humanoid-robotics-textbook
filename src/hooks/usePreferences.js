import { useContext } from 'react';
import { PreferencesContext } from '../contexts/PreferencesContext';

/**
 * usePreferences hook - Access user preferences from context
 * @returns {Object} Preferences context value
 */
export function usePreferences() {
  const context = useContext(PreferencesContext);
  
  if (!context) {
    throw new Error('usePreferences must be used within a PreferencesProvider');
  }
  
  return context;
}
