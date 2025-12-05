/**
 * Custom hook for managing chat API interactions.
 */

import { useState, useCallback } from 'react';

export interface Source {
  path: string;
  title: string;
  module: string;
  relevance: number;
  snippet?: string;
}

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

interface ChatResponse {
  response: string;
  sources: Source[];
  conversation_id: string;
}

interface UseChatOptions {
  apiUrl: string;
  sessionId: string;
}

export function useChat({ apiUrl, sessionId }: UseChatOptions) {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);

  const sendMessage = useCallback(
    async (content: string, selectedText?: string) => {
      if (!content.trim()) return;

      setIsLoading(true);
      setError(null);

      // Add user message immediately
      const userMessage: Message = {
        id: `user-${Date.now()}`,
        role: 'user',
        content,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, userMessage]);

      try {
        const response = await fetch(`${apiUrl}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: content,
            session_id: sessionId,
            conversation_id: conversationId,
            selected_text: selectedText || null,
          }),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || 'Failed to send message');
        }

        const data: ChatResponse = await response.json();

        // Update conversation ID
        if (data.conversation_id) {
          setConversationId(data.conversation_id);
        }

        // Add assistant message
        const assistantMessage: Message = {
          id: `assistant-${Date.now()}`,
          role: 'assistant',
          content: data.response,
          sources: data.sources,
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, assistantMessage]);
      } catch (err) {
        const errorMessage =
          err instanceof Error ? err.message : 'An error occurred';
        setError(errorMessage);

        // Add error message to chat
        const errorMsg: Message = {
          id: `error-${Date.now()}`,
          role: 'assistant',
          content: `Sorry, I encountered an error: ${errorMessage}. Please try again.`,
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, errorMsg]);
      } finally {
        setIsLoading(false);
      }
    },
    [apiUrl, sessionId, conversationId]
  );

  const clearChat = useCallback(() => {
    setMessages([]);
    setConversationId(null);
    setError(null);
  }, []);

  const loadConversation = useCallback(
    async (convId: string) => {
      try {
        const response = await fetch(
          `${apiUrl}/api/conversations/${convId}/messages`
        );

        if (!response.ok) {
          throw new Error('Failed to load conversation');
        }

        const data = await response.json();

        const loadedMessages: Message[] = data.messages.map(
          (msg: {
            id: string;
            role: 'user' | 'assistant';
            content: string;
            context?: { sources?: Source[] };
            created_at: string;
          }) => ({
            id: msg.id,
            role: msg.role,
            content: msg.content,
            sources: msg.context?.sources,
            timestamp: new Date(msg.created_at),
          })
        );

        setMessages(loadedMessages);
        setConversationId(convId);
      } catch (err) {
        console.error('Failed to load conversation:', err);
      }
    },
    [apiUrl]
  );

  return {
    messages,
    isLoading,
    error,
    conversationId,
    sendMessage,
    clearChat,
    loadConversation,
  };
}
