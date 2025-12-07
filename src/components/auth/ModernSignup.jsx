/**
 * Modern Signup Component with Multi-Step Form
 * Feature: 005-ui-ux-redesign
 */

import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { useHistory, useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import styles from './ModernSignup.module.css';

export default function ModernSignup() {
  const [currentStep, setCurrentStep] = useState(1);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
    software_level: '',
    hardware_access: '',
    preferred_language: 'en'
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  
  const { signup } = useAuth();
  const history = useHistory();
  const location = useLocation();

  // Get redirect URL from query params or use intro as default
  const getRedirectUrl = () => {
    const params = new URLSearchParams(location.search);
    return params.get('redirect') || '/docs/intro';
  };

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    setError('');
  };

  const handleStepOne = async (e) => {
    e.preventDefault();
    
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }
    
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }
    
    setCurrentStep(2);
  };

  const handleStepTwo = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      const result = await signup({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        software_level: formData.software_level,
        hardware_access: formData.hardware_access,
        preferred_language: formData.preferred_language
      });
      
      if (result.success) {
        history.push(getRedirectUrl());
      } else {
        setError(result.error || 'Signup failed. Please try again.');
      }
    } catch (err) {
      setError(err.message || 'Signup failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const goBack = () => {
    setCurrentStep(1);
    setError('');
  };

  return (
    <div className={styles.signupContainer}>
      <div className={styles.signupCard}>
        {/* Progress Stepper */}
        <div className={styles.stepper}>
          <div className={`${styles.step} ${currentStep >= 1 ? styles.stepActive : ''}`}>
            <div className={styles.stepNumber}>1</div>
            <div className={styles.stepLabel}>Account</div>
          </div>
          <div className={styles.stepLine} />
          <div className={`${styles.step} ${currentStep >= 2 ? styles.stepActive : ''}`}>
            <div className={styles.stepNumber}>2</div>
            <div className={styles.stepLabel}>Learning Profile</div>
          </div>
        </div>

        {error && (
          <div className={styles.errorBanner}>
            <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
              <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z" clipRule="evenodd" />
            </svg>
            {error}
          </div>
        )}

        {/* Step 1: Account Creation */}
        {currentStep === 1 && (
          <>
            <div className={styles.header}>
              <h1>Create your account</h1>
              <p>Start your Physical AI learning journey</p>
            </div>

            <form onSubmit={handleStepOne} className={styles.form}>
              <div className={styles.formGroup}>
                <label htmlFor="name">Full Name</label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  placeholder="Enter your full name"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email">Email Address</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  placeholder="you@example.com"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  placeholder="At least 8 characters"
                  minLength={8}
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword">Confirm Password</label>
                <input
                  type="password"
                  id="confirmPassword"
                  name="confirmPassword"
                  value={formData.confirmPassword}
                  onChange={handleInputChange}
                  placeholder="Re-enter your password"
                  required
                />
              </div>

              <button type="submit" className={styles.btnPrimary}>
                Continue
                <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M10.293 3.293a1 1 0 011.414 0l6 6a1 1 0 010 1.414l-6 6a1 1 0 01-1.414-1.414L14.586 11H3a1 1 0 110-2h11.586l-4.293-4.293a1 1 0 010-1.414z" clipRule="evenodd" />
                </svg>
              </button>
            </form>

            <div className={styles.footer}>
              Already have an account? <Link to="/login">Sign In</Link>
            </div>
          </>
        )}

        {/* Step 2: Learning Profile */}
        {currentStep === 2 && (
          <>
            <div className={styles.header}>
              <button onClick={goBack} className={styles.backButton}>
                <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M9.707 16.707a1 1 0 01-1.414 0l-6-6a1 1 0 010-1.414l6-6a1 1 0 011.414 1.414L5.414 9H17a1 1 0 110 2H5.414l4.293 4.293a1 1 0 010 1.414z" clipRule="evenodd" />
                </svg>
                Back
              </button>
              <h1>Tell us about yourself</h1>
              <p>Help us personalize your learning experience</p>
            </div>

            <form onSubmit={handleStepTwo} className={styles.form}>
              <div className={styles.formGroup}>
                <label>What's your software background?</label>
                <div className={styles.optionCards}>
                  <label className={`${styles.optionCard} ${formData.software_level === 'beginner' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="software_level"
                      value="beginner"
                      checked={formData.software_level === 'beginner'}
                      onChange={handleInputChange}
                      required
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Beginner</div>
                      <div className={styles.optionDesc}>New to programming or robotics</div>
                    </div>
                    {formData.software_level === 'beginner' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>

                  <label className={`${styles.optionCard} ${formData.software_level === 'intermediate' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="software_level"
                      value="intermediate"
                      checked={formData.software_level === 'intermediate'}
                      onChange={handleInputChange}
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Intermediate</div>
                      <div className={styles.optionDesc}>Some experience with Python or similar languages</div>
                    </div>
                    {formData.software_level === 'intermediate' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>

                  <label className={`${styles.optionCard} ${formData.software_level === 'advanced' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="software_level"
                      value="advanced"
                      checked={formData.software_level === 'advanced'}
                      onChange={handleInputChange}
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Advanced</div>
                      <div className={styles.optionDesc}>Professional developer or robotics engineer</div>
                    </div>
                    {formData.software_level === 'advanced' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>
                </div>
              </div>

              <div className={styles.formGroup}>
                <label>What hardware and OS do you use?</label>
                <div className={styles.optionCards}>
                  <label className={`${styles.optionCard} ${formData.hardware_access === 'windows' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="hardware_access"
                      value="windows"
                      checked={formData.hardware_access === 'windows'}
                      onChange={handleInputChange}
                      required
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Windows PC</div>
                      <div className={styles.optionDesc}>Desktop or laptop</div>
                    </div>
                    {formData.hardware_access === 'windows' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>

                  <label className={`${styles.optionCard} ${formData.hardware_access === 'mac' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="hardware_access"
                      value="mac"
                      checked={formData.hardware_access === 'mac'}
                      onChange={handleInputChange}
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Mac</div>
                      <div className={styles.optionDesc}>MacBook or iMac</div>
                    </div>
                    {formData.hardware_access === 'mac' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>

                  <label className={`${styles.optionCard} ${formData.hardware_access === 'linux' ? styles.optionCardSelected : ''}`}>
                    <input
                      type="radio"
                      name="hardware_access"
                      value="linux"
                      checked={formData.hardware_access === 'linux'}
                      onChange={handleInputChange}
                    />
                    <div className={styles.optionContent}>
                      <div className={styles.optionTitle}>Linux</div>
                      <div className={styles.optionDesc}>Ubuntu, Fedora, etc.</div>
                    </div>
                    {formData.hardware_access === 'linux' && (
                      <svg className={styles.checkIcon} width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z"/>
                      </svg>
                    )}
                  </label>
                </div>
              </div>

              <button type="submit" className={styles.btnPrimary} disabled={isLoading}>
                {isLoading ? 'Creating account...' : 'Complete Sign Up'}
                {!isLoading && (
                  <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                  </svg>
                )}
              </button>
            </form>
          </>
        )}
      </div>
    </div>
  );
}
