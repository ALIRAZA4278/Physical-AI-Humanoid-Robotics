/**
 * Global chat context for managing chatbot state across the application.
 */

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useEffect,
  ReactNode,
} from 'react';

interface ChatContextValue {
  isOpen: boolean;
  openChat: () => void;
  closeChat: () => void;
  toggleChat: () => void;
  selectedText: string | null;
  setSelectedText: (text: string | null) => void;
  askAboutSelection: (text: string) => void;
  sessionId: string;
  apiUrl: string;
}

const ChatContext = createContext<ChatContextValue | null>(null);

// Generate a unique session ID
function generateSessionId(): string {
  const stored = localStorage.getItem('chat-session-id');
  if (stored) return stored;

  const newId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  localStorage.setItem('chat-session-id', newId);
  return newId;
}

interface ChatProviderProps {
  children: ReactNode;
  apiUrl?: string;
}

export function ChatProvider({
  children,
  apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000',
}: ChatProviderProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sessionId, setSessionId] = useState('');

  useEffect(() => {
    // Initialize session ID on client side only
    setSessionId(generateSessionId());
  }, []);

  const openChat = useCallback(() => {
    setIsOpen(true);
  }, []);

  const closeChat = useCallback(() => {
    setIsOpen(false);
  }, []);

  const toggleChat = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  const askAboutSelection = useCallback((text: string) => {
    setSelectedText(text);
    setIsOpen(true);
  }, []);

  const value: ChatContextValue = {
    isOpen,
    openChat,
    closeChat,
    toggleChat,
    selectedText,
    setSelectedText,
    askAboutSelection,
    sessionId,
    apiUrl,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}

export function useChatContext(): ChatContextValue {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }
  return context;
}
