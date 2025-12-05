import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props) {
  // Backend API URL - Connected to Vercel deployment
  const backendUrl = process.env.REACT_APP_BACKEND_URL || 
                     (typeof window !== 'undefined' && window.location.hostname === 'localhost' 
                       ? 'http://localhost:8000' 
                       : 'https://physical-ai-humanoid-robotics-textb-two.vercel.app');
  
  return (
    <>
      <Layout {...props} />
      <ChatWidget apiUrl={backendUrl} />
    </>
  );
}
