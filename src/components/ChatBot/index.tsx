/**
 * Main ChatBot component - floating button and chat window.
 */

import React, { useState, useEffect } from 'react';
import { ChatWindow } from './ChatWindow';
import { TextSelectionHandler } from './TextSelectionHandler';
import { useChatContext } from '../../context/ChatContext';
import styles from './ChatBot.module.css';

export default function ChatBot() {
  const { isOpen, toggleChat, closeChat } = useChatContext();
  const [isVisible, setIsVisible] = useState(false);

  // Animate in on mount
  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), 500);
    return () => clearTimeout(timer);
  }, []);

  // Handle escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        closeChat();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen, closeChat]);

  return (
    <>
      {/* Text selection handler */}
      <TextSelectionHandler />

      {/* Floating button */}
      <button
        className={`${styles.floatingButton} ${isVisible ? styles.visible : ''} ${isOpen ? styles.hidden : ''}`}
        onClick={toggleChat}
        aria-label="Open chat assistant"
        aria-expanded={isOpen}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
          className={styles.chatIcon}
        >
          <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
          <path d="M7 9h10v2H7zm0-3h10v2H7z" />
        </svg>
        <span className={styles.buttonLabel}>Ask AI</span>
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.backdrop} onClick={closeChat} />
          <ChatWindow onClose={closeChat} />
        </div>
      )}
    </>
  );
}

// Re-export components for external use
export { ChatWindow } from './ChatWindow';
export { ChatInput } from './ChatInput';
export { Message } from './Message';
export { TextSelectionHandler } from './TextSelectionHandler';
export { useChat } from './hooks/useChat';
export { useTextSelection } from './hooks/useTextSelection';
