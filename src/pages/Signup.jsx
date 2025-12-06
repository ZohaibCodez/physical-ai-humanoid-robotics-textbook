/**
 * Signup Page - User registration with Docusaurus layout
 */

import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/auth/SignupForm';

export default function Signup() {
  return (
    <Layout
      title="Sign Up"
      description="Create your account to access personalized robotics content"
    >
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <SignupForm />
          </div>
        </div>
      </div>
    </Layout>
  );
}
