/**
 * SignupForm - User registration form with background profiling
 */

import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '../../hooks/useAuth';
import styles from './AuthForms.module.css';

export default function SignupForm({ onSuccess }) {
  const { signup } = useAuth();
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    software_level: 'beginner',
    hardware_access: 'cloud_only',
    preferred_language: 'en',
  });
  const [errors, setErrors] = useState({});
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Handle input changes
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    // Clear error for this field
    if (errors[name]) {
      setErrors((prev) => ({ ...prev, [name]: '' }));
    }
  };

  // Client-side validation
  const validate = () => {
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
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    // Name validation
    if (!formData.name) {
      newErrors.name = 'Name is required';
    }

    // Profiling fields validation
    if (!formData.software_level) {
      newErrors.software_level = 'Software level is required';
    }
    if (!formData.hardware_access) {
      newErrors.hardware_access = 'Hardware access is required';
    }
    if (!formData.preferred_language) {
      newErrors.preferred_language = 'Preferred language is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validate form
    if (!validate()) {
      return;
    }

    setIsSubmitting(true);

    try {
      const result = await signup(formData);

      if (result.success) {
        // Redirect to home page or call onSuccess callback
        if (onSuccess) {
          onSuccess();
        } else {
          window.location.href = '/';
        }
      } else {
        // Display error message
        setErrors({ submit: result.error || 'Signup failed. Please try again.' });
      }
    } catch (error) {
      setErrors({ submit: 'An unexpected error occurred. Please try again.' });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2>Create Your Account</h2>
      <p className={styles.subtitle}>
        Join to track your progress and get personalized content
      </p>

      {/* Email Field */}
      <div className={styles.formGroup}>
        <label htmlFor="email">Email Address *</label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          className={errors.email ? styles.inputError : ''}
          aria-invalid={!!errors.email}
          aria-describedby={errors.email ? 'email-error' : undefined}
          required
        />
        {errors.email && (
          <span id="email-error" className={styles.errorMessage} role="alert">
            {errors.email}
          </span>
        )}
      </div>

      {/* Password Field */}
      <div className={styles.formGroup}>
        <label htmlFor="password">Password *</label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          className={errors.password ? styles.inputError : ''}
          aria-invalid={!!errors.password}
          aria-describedby={errors.password ? 'password-error' : undefined}
          minLength={8}
          required
        />
        <small className={styles.helpText}>Minimum 8 characters</small>
        {errors.password && (
          <span id="password-error" className={styles.errorMessage} role="alert">
            {errors.password}
          </span>
        )}
      </div>

      {/* Name Field */}
      <div className={styles.formGroup}>
        <label htmlFor="name">Full Name *</label>
        <input
          type="text"
          id="name"
          name="name"
          value={formData.name}
          onChange={handleChange}
          className={errors.name ? styles.inputError : ''}
          aria-invalid={!!errors.name}
          aria-describedby={errors.name ? 'name-error' : undefined}
          maxLength={255}
          required
        />
        {errors.name && (
          <span id="name-error" className={styles.errorMessage} role="alert">
            {errors.name}
          </span>
        )}
      </div>

      {/* Background Profiling Section */}
      <h3 className={styles.sectionTitle}>Tell us about yourself</h3>

      {/* Software Level */}
      <div className={styles.formGroup}>
        <label htmlFor="software_level">Programming Experience *</label>
        <select
          id="software_level"
          name="software_level"
          value={formData.software_level}
          onChange={handleChange}
          className={errors.software_level ? styles.inputError : ''}
          required
        >
          <option value="beginner">Beginner - New to programming</option>
          <option value="intermediate">Intermediate - Some experience</option>
          <option value="advanced">Advanced - Proficient developer</option>
        </select>
        {errors.software_level && (
          <span className={styles.errorMessage} role="alert">
            {errors.software_level}
          </span>
        )}
      </div>

      {/* Hardware Access */}
      <div className={styles.formGroup}>
        <label htmlFor="hardware_access">Hardware Access *</label>
        <select
          id="hardware_access"
          name="hardware_access"
          value={formData.hardware_access}
          onChange={handleChange}
          className={errors.hardware_access ? styles.inputError : ''}
          required
        >
          <option value="cloud_only">Cloud Only - No physical hardware</option>
          <option value="basic">Basic - Microcontrollers/sensors</option>
          <option value="full_lab">Full Lab - Complete robotics setup</option>
        </select>
        {errors.hardware_access && (
          <span className={styles.errorMessage} role="alert">
            {errors.hardware_access}
          </span>
        )}
      </div>

      {/* Preferred Language */}
      <div className={styles.formGroup}>
        <label htmlFor="preferred_language">Preferred Language *</label>
        <select
          id="preferred_language"
          name="preferred_language"
          value={formData.preferred_language}
          onChange={handleChange}
          className={errors.preferred_language ? styles.inputError : ''}
          required
        >
          <option value="en">English</option>
          <option value="ur">Urdu</option>
          <option value="both">Both (English & Urdu)</option>
        </select>
        {errors.preferred_language && (
          <span className={styles.errorMessage} role="alert">
            {errors.preferred_language}
          </span>
        )}
      </div>

      {/* Submit Error */}
      {errors.submit && (
        <div className={styles.submitError} role="alert">
          {errors.submit}
        </div>
      )}

      {/* Submit Button */}
      <button
        type="submit"
        className={styles.submitButton}
        disabled={isSubmitting}
        aria-busy={isSubmitting}
      >
        {isSubmitting ? 'Creating Account...' : 'Create Account'}
      </button>

      {/* Login Link */}
      <p className={styles.switchForm}>
        Already have an account? <Link to="/login">Log in</Link>
      </p>
    </form>
  );
}
