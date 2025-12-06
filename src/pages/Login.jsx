/**
 * Login Page - Authentication page for returning users
 * 
 * Wraps LoginForm component with Docusaurus Layout for consistent theming.
 */

import React from 'react';
import Layout from '@theme/Layout';
import LoginForm from '../components/auth/LoginForm';

export default function Login() {
  return (
    <Layout
      title="Log In"
      description="Log in to access your personalized robotics learning experience"
    >
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <LoginForm />
          </div>
        </div>
      </div>
    </Layout>
  );
}
