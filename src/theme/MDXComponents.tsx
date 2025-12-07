/**
 * Custom MDX Components for Docusaurus.
 * Wraps standard components and adds chapter controls for personalization/translation.
 */

import React from 'react';
import MDXComponents from '@theme-original/MDXComponents';
import ChapterControls from '../components/ChapterControls';

// Custom wrapper for doc pages that adds chapter controls
function DocWrapper({ children }: { children: React.ReactNode }) {
  // Extract text content from children for personalization/translation
  const extractTextContent = (node: React.ReactNode): string => {
    if (typeof node === 'string') return node;
    if (typeof node === 'number') return String(node);
    if (!node) return '';

    if (Array.isArray(node)) {
      return node.map(extractTextContent).join(' ');
    }

    if (React.isValidElement(node)) {
      const props = node.props as { children?: React.ReactNode };
      if (props.children) {
        return extractTextContent(props.children);
      }
    }

    return '';
  };

  const textContent = extractTextContent(children);

  return (
    <div className="doc-wrapper">
      <ChapterControls chapterContent={textContent} />
      {children}
    </div>
  );
}

export default {
  ...MDXComponents,
  // Add wrapper component that can be used in MDX
  DocWrapper,
  ChapterControls,
};
