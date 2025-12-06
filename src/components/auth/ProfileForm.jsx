/**
 * ProfileForm - User profile and preferences update form
 * 
 * Allows authenticated users to update their name and learning preferences.
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './AuthForms.module.css';

/**
 * ProfileForm component
 * 
 * @param {Object} props
 * @param {Function} props.onUpdate - Callback function when profile is updated
 * @returns {JSX.Element} Profile form UI
 */
export default function ProfileForm({ onUpdate }) {
  const { user, preferences, updateProfile } = useAuth();
  
  const [formData, setFormData] = useState({
    name: '',
    software_level: 'beginner',
    hardware_access: 'cloud_only',
    preferred_language: 'en',
  });
  
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [submitError, setSubmitError] = useState('');
  const [submitSuccess, setSubmitSuccess] = useState(false);

  // Populate form with current user data
  useEffect(() => {
    if (user && preferences) {
      setFormData({
        name: user.name || '',
        software_level: preferences.software_level || 'beginner',
        hardware_access: preferences.hardware_access || 'cloud_only',
        preferred_language: preferences.preferred_language || 'en',
      });
    }
  }, [user, preferences]);

  /**
   * Handle form field changes
   */
  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    // Clear errors when user types
    setSubmitError('');
    setSubmitSuccess(false);
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsSubmitting(true);
    setSubmitError('');
    setSubmitSuccess(false);

    try {
      const result = await updateProfile(formData);
      
      if (result.success) {
        setSubmitSuccess(true);
        if (onUpdate) {
          onUpdate(result.data);
        }
        
        // Clear success message after 3 seconds
        setTimeout(() => setSubmitSuccess(false), 3000);
      } else {
        setSubmitError(result.error || 'Failed to update profile');
      }
    } catch (error) {
      setSubmitError('An unexpected error occurred');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <form className={styles.authForm} onSubmit={handleSubmit}>
      <h2>Update Profile</h2>
      <p className={styles.subtitle}>
        Manage your account settings and learning preferences
      </p>

      {submitError && (
        <div className={styles.submitError} role="alert">
          {submitError}
        </div>
      )}

      {submitSuccess && (
        <div className={styles.submitSuccess} role="alert">
          Profile updated successfully!
        </div>
      )}

      {/* Basic Information */}
      <h3 className={styles.sectionTitle}>Basic Information</h3>
      
      <div className={styles.formGroup}>
        <label htmlFor="name">Display Name</label>
        <input
          id="name"
          name="name"
          type="text"
          value={formData.name}
          onChange={handleChange}
          disabled={isSubmitting}
          maxLength={255}
          required
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="email">Email Address</label>
        <input
          id="email"
          type="email"
          value={user?.email || ''}
          disabled
          className={styles.inputDisabled}
        />
        <span className={styles.helpText}>Email cannot be changed</span>
      </div>

      {/* Learning Preferences */}
      <h3 className={styles.sectionTitle}>Learning Preferences</h3>

      <div className={styles.formGroup}>
        <label htmlFor="software_level">Software Experience Level</label>
        <select
          id="software_level"
          name="software_level"
          value={formData.software_level}
          onChange={handleChange}
          disabled={isSubmitting}
          required
        >
          <option value="beginner">Beginner - New to programming and AI</option>
          <option value="intermediate">Intermediate - Comfortable with Python and basic AI</option>
          <option value="advanced">Advanced - Experienced with ML frameworks and robotics</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="hardware_access">Hardware Access Level</label>
        <select
          id="hardware_access"
          name="hardware_access"
          value={formData.hardware_access}
          onChange={handleChange}
          disabled={isSubmitting}
          required
        >
          <option value="cloud_only">Cloud Only - Using simulations and cloud resources</option>
          <option value="basic">Basic Hardware - Arduino, Raspberry Pi, sensors</option>
          <option value="full_lab">Full Lab - Complete robotics lab with actuators</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="preferred_language">Preferred Language</label>
        <select
          id="preferred_language"
          name="preferred_language"
          value={formData.preferred_language}
          onChange={handleChange}
          disabled={isSubmitting}
          required
        >
          <option value="en">English</option>
          <option value="ur">Urdu</option>
          <option value="both">Both (English & Urdu)</option>
        </select>
      </div>

      <button
        type="submit"
        className={styles.submitButton}
        disabled={isSubmitting}
        aria-busy={isSubmitting}
      >
        {isSubmitting ? 'Updating...' : 'Update Profile'}
      </button>
    </form>
  );
}
