/**
 * Root theme wrapper for Docusaurus.
 * Provides global context and renders the ChatBot on all pages.
 */

import React, { ReactNode } from 'react';
import { ChatProvider } from '../context/ChatContext';
import ChatBot from '../components/ChatBot';

interface RootProps {
  children: ReactNode;
}

// Get API URL based on environment
const getApiUrl = (): string => {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }
  // Use localhost for development, otherwise use Hugging Face Spaces API
  return window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://aliraza4278-rag-chatbot-api.hf.space';
};

export default function Root({ children }: RootProps) {
  const apiUrl = getApiUrl();

  return (
    <ChatProvider apiUrl={apiUrl}>
      {children}
      <ChatBot />
    </ChatProvider>
  );
}
