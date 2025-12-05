import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props) {
  // Backend API URL - update this with your deployed backend URL
  // For local development: http://localhost:8000
  // For Vercel: https://your-backend.vercel.app
  // For Render: https://your-backend.onrender.com
  const backendUrl = process.env.REACT_APP_BACKEND_URL || 
                     (typeof window !== 'undefined' && window.location.hostname === 'localhost' 
                       ? 'http://localhost:8000' 
                       : 'https://your-backend-url-here.vercel.app'); // UPDATE THIS!
  
  return (
    <>
      <Layout {...props} />
      <ChatWidget apiUrl={backendUrl} />
    </>
  );
}
