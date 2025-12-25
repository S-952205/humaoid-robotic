/**
 * Custom Layout wrapper
 * This wraps the default Docusaurus Layout and adds the ChatWidget
 */
import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatWidget
        backendUrl="http://localhost:8000"
      />
    </>
  );
};