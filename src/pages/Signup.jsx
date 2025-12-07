/**
 * Signup Page - Modern multi-step registration
 */

import React from 'react';
import Layout from '@theme/Layout';
import ModernSignup from '../components/auth/ModernSignup';

export default function Signup() {
  return (
    <Layout
      title="Sign Up"
      description="Create your account to access personalized robotics content"
      wrapperClassName="auth-page-wrapper"
      noFooter
    >
      <ModernSignup />
    </Layout>
  );
}
