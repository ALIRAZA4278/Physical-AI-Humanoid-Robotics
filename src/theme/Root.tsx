/**
 * Root theme wrapper for Docusaurus.
 * Provides global context and renders the ChatBot on all pages.
 */

import React, { ReactNode, useEffect } from 'react';
import { createPortal } from 'react-dom';
import { ChatProvider } from '../context/ChatContext';
import { AuthProvider } from '../context/AuthContext';
import ChatBot from '../components/ChatBot';
import UserMenu from '../components/UserMenu';

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

// Component to portal UserMenu into navbar
function NavbarUserMenu() {
  const [container, setContainer] = React.useState<HTMLElement | null>(null);

  useEffect(() => {
    // Find the navbar right side and inject our menu
    const findNavbar = () => {
      const navbar = document.querySelector('.navbar__items--right');
      if (navbar) {
        setContainer(navbar as HTMLElement);
      }
    };

    findNavbar();
    // Re-check on route changes
    const observer = new MutationObserver(findNavbar);
    observer.observe(document.body, { childList: true, subtree: true });

    return () => observer.disconnect();
  }, []);

  if (!container) return null;

  return createPortal(<UserMenu />, container);
}

export default function Root({ children }: RootProps) {
  const apiUrl = getApiUrl();

  return (
    <AuthProvider apiUrl={apiUrl}>
      <ChatProvider apiUrl={apiUrl}>
        {children}
        <ChatBot />
        <NavbarUserMenu />
      </ChatProvider>
    </AuthProvider>
  );
}
