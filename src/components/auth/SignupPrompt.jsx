/**
 * SignupPrompt - Friendly signup invitation for guest users
 * 
 * Shown to unauthenticated users on protected pages or as a banner.
 * Emphasizes benefits of creating an account (progress tracking, personalization).
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './AuthForms.module.css';

/**
 * SignupPrompt component
 * 
 * @param {Object} props
 * @param {string} props.context - Where the prompt is shown (e.g., 'profile', 'protected-page', 'banner')
 * @param {string} props.message - Optional custom message
 * @returns {JSX.Element} Signup prompt UI
 */
export default function SignupPrompt({ context = 'general', message }) {
  const defaultMessages = {
    'profile': 'Sign up to save your learning progress and personalize your experience.',
    'protected-page': 'This feature requires an account. Create one to continue.',
    'banner': 'Get personalized learning recommendations and track your progress.',
    'general': 'Create an account to unlock personalized features and track your progress.',
  };

  const displayMessage = message || defaultMessages[context] || defaultMessages['general'];

  return (
    <div className={styles.signupPrompt}>
      <div className={styles.signupPromptContent}>
        <h3>📚 Unlock Personalized Learning</h3>
        <p>{displayMessage}</p>
        <div className={styles.signupPromptActions}>
          <Link 
            to="/signup" 
            className="button button--primary button--lg"
          >
            Sign Up Free
          </Link>
          <Link 
            to="/login" 
            className="button button--secondary button--lg"
          >
            Log In
          </Link>
        </div>
      </div>
    </div>
  );
}
