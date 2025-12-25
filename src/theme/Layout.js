/**
 * Custom Layout wrapper
 * This wraps the default Docusaurus Layout and adds the ChatWidget
 */
import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function LayoutWrapper(props) {
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields?.backendUrl || 'http://localhost:8000';

  return (
    <>
      <Layout {...props} />
      <ChatWidget
        backendUrl={backendUrl}
      />
    </>
  );
};