/**
 * Modern Profile Component
 * Feature: 005-ui-ux-redesign
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './ModernProfile.module.css';

export default function ModernProfile() {
  const { user, preferences, updateProfile } = useAuth();
  const [isEditing, setIsEditing] = useState(false);
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

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    setSubmitError('');
    setSubmitSuccess(false);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsSubmitting(true);
    setSubmitError('');
    setSubmitSuccess(false);

    try {
      const result = await updateProfile(formData);
      
      if (result.success) {
        setSubmitSuccess(true);
        setIsEditing(false);
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

  const getInitials = (name) => {
    if (!name) return 'U';
    return name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2);
  };

  const getLevelLabel = (level) => {
    const labels = {
      beginner: 'Beginner',
      intermediate: 'Intermediate',
      advanced: 'Advanced'
    };
    return labels[level] || level;
  };

  const getHardwareLabel = (hardware) => {
    const labels = {
      cloud_only: 'Cloud Only',
      basic: 'Basic Hardware',
      full_lab: 'Full Lab',
      windows: 'Windows PC',
      mac: 'Mac',
      linux: 'Linux'
    };
    return labels[hardware] || hardware;
  };

  const getLanguageLabel = (lang) => {
    const labels = {
      en: 'English',
      ur: 'Urdu',
      both: 'Both (EN & UR)'
    };
    return labels[lang] || lang;
  };

  return (
    <div className={styles.profileContainer}>
      <div className={styles.profileWrapper}>
        {/* Profile Header */}
        <div className={styles.profileHeader}>
          <div className={styles.avatar}>
            {getInitials(user?.name || user?.email)}
          </div>
          <div className={styles.headerInfo}>
            <h1>{user?.name || 'User'}</h1>
            <div className={styles.email}>{user?.email}</div>
            <div className={styles.badges}>
              <span className={styles.badge}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="currentColor">
                  <path d="M10.394 2.08a1 1 0 00-.788 0l-7 3a1 1 0 000 1.84L5.25 8.051a.999.999 0 01.356-.257l4-1.714a1 1 0 11.788 1.838L7.667 9.088l1.94.831a1 1 0 00.787 0l7-3a1 1 0 000-1.838l-7-3zM3.31 9.397L5 10.12v4.102a8.969 8.969 0 00-1.05-.174 1 1 0 01-.89-.89 11.115 11.115 0 01.25-3.762zM9.3 16.573A9.026 9.026 0 007 14.935v-3.957l1.818.78a3 3 0 002.364 0l5.508-2.361a11.026 11.026 0 01.25 3.762 1 1 0 01-.89.89 8.968 8.968 0 00-5.35 2.524 1 1 0 01-1.4 0zM6 18a1 1 0 001-1v-2.065a8.935 8.935 0 00-2-.712V17a1 1 0 001 1z" />
                </svg>
                {getLevelLabel(preferences?.software_level)}
              </span>
              <span className={styles.badge}>
                <svg width="16" height="16" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M3 5a2 2 0 012-2h10a2 2 0 012 2v8a2 2 0 01-2 2h-2.22l.123.489.804.804A1 1 0 0113 18H7a1 1 0 01-.707-1.707l.804-.804L7.22 15H5a2 2 0 01-2-2V5zm5.771 7H5V5h10v7H8.771z" clipRule="evenodd" />
                </svg>
                {getHardwareLabel(preferences?.hardware_access)}
              </span>
            </div>
          </div>
        </div>

        {/* Content Grid */}
        <div className={styles.contentGrid}>
          <div className={styles.card}>
            <div className={styles.cardHeader}>
              <div className={styles.cardIcon}>
                <svg width="24" height="24" viewBox="0 0 20 20" fill="currentColor">
                  <path d="M9 6a3 3 0 11-6 0 3 3 0 016 0zM17 6a3 3 0 11-6 0 3 3 0 016 0zM12.93 17c.046-.327.07-.66.07-1a6.97 6.97 0 00-1.5-4.33A5 5 0 0119 16v1h-6.07zM6 11a5 5 0 015 5v1H1v-1a5 5 0 015-5z" />
                </svg>
              </div>
              <h2>Account Info</h2>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Display Name</span>
              <span className={styles.infoValue}>{user?.name || 'Not set'}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Email</span>
              <span className={styles.infoValue}>{user?.email}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Member Since</span>
              <span className={styles.infoValue}>
                {user?.created_at ? new Date(user.created_at).toLocaleDateString() : 'N/A'}
              </span>
            </div>
          </div>

          <div className={styles.card}>
            <div className={styles.cardHeader}>
              <div className={styles.cardIcon}>
                <svg width="24" height="24" viewBox="0 0 20 20" fill="currentColor">
                  <path d="M5 3a2 2 0 00-2 2v2a2 2 0 002 2h2a2 2 0 002-2V5a2 2 0 00-2-2H5zM5 11a2 2 0 00-2 2v2a2 2 0 002 2h2a2 2 0 002-2v-2a2 2 0 00-2-2H5zM11 5a2 2 0 012-2h2a2 2 0 012 2v2a2 2 0 01-2 2h-2a2 2 0 01-2-2V5zM14 11a1 1 0 011 1v1h1a1 1 0 110 2h-1v1a1 1 0 11-2 0v-1h-1a1 1 0 110-2h1v-1a1 1 0 011-1z" />
                </svg>
              </div>
              <h2>Learning Preferences</h2>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Experience Level</span>
              <span className={styles.infoValue}>{getLevelLabel(preferences?.software_level)}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Hardware Access</span>
              <span className={styles.infoValue}>{getHardwareLabel(preferences?.hardware_access)}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.infoLabel}>Preferred Language</span>
              <span className={styles.infoValue}>{getLanguageLabel(preferences?.preferred_language)}</span>
            </div>
          </div>
        </div>

        {/* Edit Form */}
        <div className={styles.editFormCard}>
          <div className={styles.formHeader}>
            <h2>Edit Profile</h2>
            <button
              className={`${styles.editToggle} ${isEditing ? styles.cancel : ''}`}
              onClick={() => {
                setIsEditing(!isEditing);
                setSubmitError('');
                setSubmitSuccess(false);
              }}
            >
              {isEditing ? (
                <>
                  <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                  </svg>
                  Cancel
                </>
              ) : (
                <>
                  <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path d="M13.586 3.586a2 2 0 112.828 2.828l-.793.793-2.828-2.828.793-.793zM11.379 5.793L3 14.172V17h2.828l8.38-8.379-2.83-2.828z" />
                  </svg>
                  Edit Profile
                </>
              )}
            </button>
          </div>

          {submitSuccess && (
            <div className={styles.successAlert}>
              <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
              </svg>
              Profile updated successfully!
            </div>
          )}

          {submitError && (
            <div className={styles.errorAlert}>
              <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
              </svg>
              {submitError}
            </div>
          )}

          {isEditing ? (
            <form onSubmit={handleSubmit} className={styles.form}>
              <div className={styles.formSection}>
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
                  />
                  <span className={styles.helpText}>Email cannot be changed</span>
                </div>
              </div>

              <div className={styles.formSection}>
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
                  <label htmlFor="hardware_access">Hardware & OS</label>
                  <select
                    id="hardware_access"
                    name="hardware_access"
                    value={formData.hardware_access}
                    onChange={handleChange}
                    disabled={isSubmitting}
                    required
                  >
                    <option value="windows">Windows PC - Desktop or laptop</option>
                    <option value="mac">Mac - MacBook or iMac</option>
                    <option value="linux">Linux - Ubuntu, Fedora, etc.</option>
                    <option value="cloud_only">Cloud Only - Using simulations</option>
                    <option value="basic">Basic Hardware - Arduino, Raspberry Pi</option>
                    <option value="full_lab">Full Lab - Complete robotics lab</option>
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
              </div>

              <div className={styles.formActions}>
                <button
                  type="submit"
                  className={styles.btnPrimary}
                  disabled={isSubmitting}
                >
                  {isSubmitting ? 'Updating...' : 'Save Changes'}
                </button>
              </div>
            </form>
          ) : (
            <p style={{ color: 'var(--ifm-color-emphasis-600)', textAlign: 'center', margin: 0 }}>
              Click "Edit Profile" to update your information and preferences
            </p>
          )}
        </div>
      </div>
    </div>
  );
}
