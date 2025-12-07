/**
 * LoginForm Component - Email/Password authentication form
 * 
 * Provides login interface for returning users with email and password.
 * Integrates with AuthContext for global authentication state management.
 */

import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { useHistory } from '@docusaurus/router';
import styles from './AuthForms.module.css';

export default function LoginForm() {
  const { login } = useAuth();
  const history = useHistory();
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });
  const [errors, setErrors] = useState({});
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [apiError, setApiError] = useState('');

  /**
   * Handle form field changes
   */
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
    // Clear field-specific error when user types
    if (errors[name]) {
      setErrors((prev) => ({
        ...prev,
        [name]: '',
      }));
    }
    // Clear API error when user modifies form
    if (apiError) {
      setApiError('');
    }
  };

  /**
   * Validate form fields client-side
   */
  const validateForm = () => {
    const newErrors = {};

    // Email validation
    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Invalid email format';
    }

    // Password validation
    if (!formData.password) {
      newErrors.password = 'Password is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e) => {
    e.preventDefault();

    // Client-side validation
    if (!validateForm()) {
      return;
    }

    setIsSubmitting(true);
    setApiError('');

    try {
      // Call login function from AuthContext
      const result = await login(formData.email, formData.password);

      if (result.success) {
        // Redirect to intended destination or home
        const redirectPath = new URLSearchParams(window.location.search).get('redirect') || '/';
        history.push(redirectPath);
      } else {
        // Display generic error message (don't reveal which field is wrong for security)
        setApiError(result.error || 'Invalid email or password');
      }
    } catch (error) {
      console.error('Login error:', error);
      setApiError('An unexpected error occurred. Please try again.');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <form className={styles.authForm} onSubmit={handleSubmit} noValidate>
      <h2 className={styles.formTitle}>Log In</h2>

      {/* API Error Message */}
      {apiError && (
        <div className={styles.errorMessage} role="alert" aria-live="polite">
          {apiError}
        </div>
      )}

      {/* Email Field */}
      <div className={styles.formGroup}>
        <label htmlFor="email" className={styles.formLabel}>
          Email Address <span className={styles.required}>*</span>
        </label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          className={`${styles.formInput} ${errors.email ? styles.inputError : ''}`}
          aria-invalid={!!errors.email}
          aria-describedby={errors.email ? 'email-error' : undefined}
          disabled={isSubmitting}
          autoComplete="email"
          required
        />
        {errors.email && (
          <span id="email-error" className={styles.fieldError} role="alert">
            {errors.email}
          </span>
        )}
      </div>

      {/* Password Field */}
      <div className={styles.formGroup}>
        <label htmlFor="password" className={styles.formLabel}>
          Password <span className={styles.required}>*</span>
        </label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          className={`${styles.formInput} ${errors.password ? styles.inputError : ''}`}
          aria-invalid={!!errors.password}
          aria-describedby={errors.password ? 'password-error' : undefined}
          disabled={isSubmitting}
          autoComplete="current-password"
          required
        />
        {errors.password && (
          <span id="password-error" className={styles.fieldError} role="alert">
            {errors.password}
          </span>
        )}
      </div>

      {/* Submit Button */}
      <button
        type="submit"
        className={styles.submitButton}
        disabled={isSubmitting}
        aria-busy={isSubmitting}
      >
        {isSubmitting ? (
          <>
            <span className={styles.spinner} aria-hidden="true"></span>
            Logging in...
          </>
        ) : (
          'Log In'
        )}
      </button>

      {/* Additional Links */}
      <div className={styles.formFooter}>
        <p className={styles.footerText}>
          Don't have an account?{' '}
          <Link to="/signup" className={styles.footerLink}>
            Sign up
          </Link>
        </p>
      </div>
    </form>
  );
}
