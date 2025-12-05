/**
 * Chat input component with selected text indicator.
 */

import React, { useState, useRef, useEffect, FormEvent, KeyboardEvent } from 'react';
import styles from './ChatBot.module.css';

interface ChatInputProps {
  onSend: (message: string, selectedText?: string) => void;
  isLoading: boolean;
  selectedText?: string | null;
  onClearSelection?: () => void;
  placeholder?: string;
}

export function ChatInput({
  onSend,
  isLoading,
  selectedText,
  onClearSelection,
  placeholder = 'Ask about the book...',
}: ChatInputProps) {
  const [input, setInput] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea
  useEffect(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 120)}px`;
    }
  }, [input]);

  // Focus when selected text changes
  useEffect(() => {
    if (selectedText && textareaRef.current) {
      textareaRef.current.focus();
    }
  }, [selectedText]);

  const handleSubmit = (e?: FormEvent) => {
    e?.preventDefault();
    if (!input.trim() || isLoading) return;

    onSend(input.trim(), selectedText || undefined);
    setInput('');

    // Clear selection after sending
    if (selectedText && onClearSelection) {
      onClearSelection();
    }

    // Reset textarea height
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <form className={styles.inputForm} onSubmit={handleSubmit}>
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <span className={styles.selectedTextLabel}>Asking about:</span>
          <span className={styles.selectedTextPreview}>
            "{selectedText.slice(0, 100)}
            {selectedText.length > 100 ? '...' : ''}"
          </span>
          <button
            type="button"
            className={styles.clearSelectionBtn}
            onClick={onClearSelection}
            aria-label="Clear selection"
          >
            ×
          </button>
        </div>
      )}

      <div className={styles.inputWrapper}>
        <textarea
          ref={textareaRef}
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={
            selectedText
              ? 'Ask a question about the selected text...'
              : placeholder
          }
          className={styles.input}
          disabled={isLoading}
          rows={1}
        />

        <button
          type="submit"
          className={styles.sendButton}
          disabled={!input.trim() || isLoading}
          aria-label="Send message"
        >
          {isLoading ? (
            <span className={styles.loadingSpinner} />
          ) : (
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="currentColor"
              className={styles.sendIcon}
            >
              <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
            </svg>
          )}
        </button>
      </div>
    </form>
  );
}
