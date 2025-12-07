/**
 * Login Page - Modern signin experience
 * 
 * Wraps ModernSignin component with Docusaurus Layout for consistent theming.
 */

import React from 'react';
import Layout from '@theme/Layout';
import ModernSignin from '../components/auth/ModernSignin';

export default function Login() {
  return (
    <Layout
      title="Sign In"
      description="Sign in to access your personalized robotics learning experience"
      wrapperClassName="auth-page-wrapper"
      noFooter
    >
      <ModernSignin />
    </Layout>
  );
}
