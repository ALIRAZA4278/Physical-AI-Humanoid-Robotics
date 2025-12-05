/**
 * Main chat window component containing messages and input.
 */

import React, { useRef, useEffect } from 'react';
import { Message } from './Message';
import { ChatInput } from './ChatInput';
import { useChat, Message as MessageType } from './hooks/useChat';
import { useChatContext } from '../../context/ChatContext';
import styles from './ChatBot.module.css';

interface ChatWindowProps {
  onClose: () => void;
}

const WELCOME_MESSAGE: MessageType = {
  id: 'welcome',
  role: 'assistant',
  content: `Hello! I'm your AI assistant for the **Physical AI & Humanoid Robotics** book.

I can help you with:
- Questions about **ROS 2** fundamentals
- **Gazebo** and **Unity** simulation
- **NVIDIA Isaac** AI integration
- **VLA** voice-controlled robotics

You can also **select text** on any page and ask me to explain it!`,
  timestamp: new Date(),
};

export function ChatWindow({ onClose }: ChatWindowProps) {
  const { selectedText, setSelectedText, sessionId, apiUrl } = useChatContext();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const {
    messages,
    isLoading,
    error,
    sendMessage,
    clearChat,
  } = useChat({
    apiUrl,
    sessionId,
  });

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = (message: string, selected?: string) => {
    sendMessage(message, selected);
  };

  const handleClearSelection = () => {
    setSelectedText(null);
  };

  const handleNewChat = () => {
    clearChat();
  };

  const displayMessages =
    messages.length === 0 ? [WELCOME_MESSAGE] : messages;

  return (
    <div className={styles.chatWindow} data-chatbot>
      {/* Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerInfo}>
          <span className={styles.headerTitle}>AI Assistant</span>
          <span className={styles.headerSubtitle}>
            Physical AI & Humanoid Robotics
          </span>
        </div>
        <div className={styles.headerActions}>
          <button
            onClick={handleNewChat}
            className={styles.headerButton}
            title="New conversation"
            aria-label="Start new conversation"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              width="18"
              height="18"
            >
              <path d="M19 13h-6v6h-2v-6H5v-2h6V5h2v6h6v2z" />
            </svg>
          </button>
          <button
            onClick={onClose}
            className={styles.headerButton}
            title="Close chat"
            aria-label="Close chat"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              width="18"
              height="18"
            >
              <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
            </svg>
          </button>
        </div>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {displayMessages.map((message) => (
          <Message key={message.id} message={message} />
        ))}

        {isLoading && (
          <div className={styles.typingIndicator}>
            <span className={styles.typingDot} />
            <span className={styles.typingDot} />
            <span className={styles.typingDot} />
          </div>
        )}

        {error && (
          <div className={styles.errorMessage}>
            <span>⚠️ {error}</span>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <ChatInput
        onSend={handleSend}
        isLoading={isLoading}
        selectedText={selectedText}
        onClearSelection={handleClearSelection}
      />
    </div>
  );
}
