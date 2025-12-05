/**
 * Component that shows a tooltip when text is selected in documentation.
 */

import React from 'react';
import { useTextSelection } from './hooks/useTextSelection';
import { useChatContext } from '../../context/ChatContext';
import styles from './ChatBot.module.css';

export function TextSelectionHandler() {
  const { selectedText, selectionPosition, clearSelection, isSelectionInContent } =
    useTextSelection();
  const { askAboutSelection, isOpen } = useChatContext();

  // Don't show if chat is already open or no valid selection
  if (!isSelectionInContent || !selectedText || !selectionPosition || isOpen) {
    return null;
  }

  const handleAskAbout = () => {
    askAboutSelection(selectedText);
    clearSelection();
  };

  return (
    <div
      className={styles.selectionTooltip}
      style={{
        top: `${selectionPosition.top}px`,
        left: `${selectionPosition.left}px`,
      }}
    >
      <button
        onClick={handleAskAbout}
        className={styles.selectionButton}
        aria-label="Ask about selected text"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
          width="16"
          height="16"
        >
          <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm-7 12h-2v-2h2v2zm0-4h-2V6h2v4z" />
        </svg>
        <span>Ask about this</span>
      </button>
    </div>
  );
}
