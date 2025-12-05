/**
 * Custom hook for detecting and managing text selection.
 */

import { useState, useEffect, useCallback, useRef } from 'react';

interface SelectionPosition {
  top: number;
  left: number;
}

interface UseTextSelectionResult {
  selectedText: string;
  selectionPosition: SelectionPosition | null;
  clearSelection: () => void;
  isSelectionInContent: boolean;
}

export function useTextSelection(): UseTextSelectionResult {
  const [selectedText, setSelectedText] = useState('');
  const [selectionPosition, setSelectionPosition] =
    useState<SelectionPosition | null>(null);
  const [isSelectionInContent, setIsSelectionInContent] = useState(false);
  const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const handleSelectionChange = useCallback(() => {
    // Clear previous timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    // Debounce selection handling
    timeoutRef.current = setTimeout(() => {
      const selection = window.getSelection();

      if (!selection || selection.isCollapsed || selection.rangeCount === 0) {
        setSelectedText('');
        setSelectionPosition(null);
        setIsSelectionInContent(false);
        return;
      }

      const text = selection.toString().trim();

      // Ignore very short selections (reduced to 5 characters for better UX)
      if (text.length < 5) {
        setSelectedText('');
        setSelectionPosition(null);
        setIsSelectionInContent(false);
        return;
      }

      // Check if selection is within documentation content
      const range = selection.getRangeAt(0);
      const container = range.commonAncestorContainer;
      const element =
        container.nodeType === Node.ELEMENT_NODE
          ? (container as Element)
          : container.parentElement;

      if (!element) {
        setSelectedText('');
        setSelectionPosition(null);
        setIsSelectionInContent(false);
        return;
      }

      // Don't show tooltip for selections in chatbot
      const isInChatbot = element.closest('[data-chatbot]') !== null;
      if (isInChatbot) {
        setSelectedText('');
        setSelectionPosition(null);
        setIsSelectionInContent(false);
        return;
      }

      // Don't show for selections in navbar, sidebar, or footer
      const isInExcludedArea =
        element.closest('nav') !== null ||
        element.closest('[class*="sidebar"]') !== null ||
        element.closest('[class*="navbar"]') !== null ||
        element.closest('[class*="footer"]') !== null ||
        element.closest('[class*="menu"]') !== null ||
        element.closest('[class*="tocCollapsible"]') !== null;

      if (isInExcludedArea) {
        setSelectedText('');
        setSelectionPosition(null);
        setIsSelectionInContent(false);
        return;
      }

      // If not in any excluded area, allow the selection
      // This is more permissive and works better with various Docusaurus layouts

      // Get position for tooltip
      const rect = range.getBoundingClientRect();
      const position: SelectionPosition = {
        top: rect.top + window.scrollY - 45, // Position above selection
        left: Math.min(
          Math.max(rect.left + rect.width / 2 + window.scrollX, 80),
          window.innerWidth - 80
        ),
      };

      setSelectedText(text);
      setSelectionPosition(position);
      setIsSelectionInContent(true);
    }, 150);
  }, []);

  const clearSelection = useCallback(() => {
    setSelectedText('');
    setSelectionPosition(null);
    setIsSelectionInContent(false);
    window.getSelection()?.removeAllRanges();
  }, []);

  useEffect(() => {
    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [handleSelectionChange]);

  return {
    selectedText,
    selectionPosition,
    clearSelection,
    isSelectionInContent,
  };
}
