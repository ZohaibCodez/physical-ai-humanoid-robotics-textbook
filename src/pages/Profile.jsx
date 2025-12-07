/**
 * Profile Page - Modern user profile management
 * 
 * Protected page where authenticated users can view and update their profile.
 */

import React from 'react';
import Layout from '@theme/Layout';
import ProtectedRoute from '../components/auth/ProtectedRoute';
import ModernProfile from '../components/auth/ModernProfile';

export default function Profile() {
  return (
    <Layout
      title="My Profile"
      description="Manage your account settings and learning preferences"
      wrapperClassName="profile-page-wrapper"
      noFooter
    >
      <ProtectedRoute promptContext="profile" customMessage="Sign up to save your learning preferences and track your progress.">
        <ModernProfile />
      </ProtectedRoute>
    </Layout>
  );
}
