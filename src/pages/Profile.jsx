/**
 * Profile Page - User profile management
 * 
 * Protected page where authenticated users can view and update their profile.
 */

import React from 'react';
import Layout from '@theme/Layout';
import ProtectedRoute from '../components/auth/ProtectedRoute';
import ProfileForm from '../components/auth/ProfileForm';

export default function Profile() {
  const handleProfileUpdate = (updatedData) => {
    console.log('Profile updated:', updatedData);
  };

  return (
    <Layout
      title="My Profile"
      description="Manage your account settings and learning preferences"
    >
      <ProtectedRoute promptContext="profile" customMessage="Sign up to save your learning preferences and track your progress.">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <ProfileForm onUpdate={handleProfileUpdate} />
            </div>
          </div>
        </div>
      </ProtectedRoute>
    </Layout>
  );
}
