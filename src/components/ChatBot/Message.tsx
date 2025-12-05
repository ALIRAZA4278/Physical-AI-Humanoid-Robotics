/**
 * Individual chat message component with markdown support.
 */

import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import type { Message as MessageType, Source } from './hooks/useChat';
import styles from './ChatBot.module.css';

interface MessageProps {
  message: MessageType;
}

// Simple markdown-like formatting
function formatContent(content: string): React.ReactNode {
  // Split by code blocks first
  const parts = content.split(/(```[\s\S]*?```)/);

  return parts.map((part, index) => {
    if (part.startsWith('```') && part.endsWith('```')) {
      // Code block
      const code = part.slice(3, -3);
      const lines = code.split('\n');
      const language = lines[0].trim();
      const codeContent = language ? lines.slice(1).join('\n') : code;

      return (
        <pre key={index} className={styles.codeBlock}>
          <code>{codeContent.trim()}</code>
        </pre>
      );
    }

    // Process inline formatting
    return (
      <span key={index}>
        {part.split('\n').map((line, lineIndex) => {
          // Bold
          let formatted: React.ReactNode = line.replace(
            /\*\*(.+?)\*\*/g,
            '<strong>$1</strong>'
          );

          // Inline code
          formatted = (formatted as string).replace(
            /`([^`]+)`/g,
            '<code class="' + styles.inlineCode + '">$1</code>'
          );

          // Citations [Module X: Section]
          formatted = (formatted as string).replace(
            /\[([^\]]+)\]/g,
            '<span class="' + styles.citation + '">[$1]</span>'
          );

          return (
            <React.Fragment key={lineIndex}>
              <span dangerouslySetInnerHTML={{ __html: formatted as string }} />
              {lineIndex < line.length - 1 && <br />}
            </React.Fragment>
          );
        })}
      </span>
    );
  });
}

function formatModuleName(module: string): string {
  // Format module name for display
  if (module === 'general' || !module) {
    return '';
  }
  if (!module.startsWith('module-')) {
    return module;
  }
  // Convert module-1-ros2 to "Module 1"
  const parts = module.replace('module-', '').split('-', 1);
  if (parts.length >= 1) {
    return `Module ${parts[0]}`;
  }
  return module;
}

function SourceLink({
  source,
}: {
  source: Source;
}) {
  // Use Docusaurus useBaseUrl to get correct path with baseUrl
  const href = useBaseUrl(`/docs/${source.path}`);
  const moduleName = formatModuleName(source.module);

  return (
    <a
      href={href}
      className={styles.sourceLink}
      title={source.snippet}
    >
      {moduleName && <span className={styles.sourceModule}>{moduleName}</span>}
      <span className={styles.sourceTitle}>{source.title}</span>
      <span className={styles.sourceRelevance}>
        {Math.round(source.relevance * 100)}%
      </span>
    </a>
  );
}

export function Message({ message }: MessageProps) {
  const isUser = message.role === 'user';

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
    >
      <div className={styles.messageAvatar}>
        {isUser ? (
          <span className={styles.userAvatar}>You</span>
        ) : (
          <span className={styles.botAvatar}>AI</span>
        )}
      </div>

      <div className={styles.messageContent}>
        <div className={styles.messageText}>
          {formatContent(message.content)}
        </div>

        {message.sources && message.sources.length > 0 && (
          <div className={styles.sources}>
            <span className={styles.sourcesLabel}>Sources:</span>
            <div className={styles.sourcesList}>
              {message.sources.slice(0, 3).map((source, index) => (
                <SourceLink key={index} source={source} />
              ))}
            </div>
          </div>
        )}

        <span className={styles.messageTime}>
          {message.timestamp.toLocaleTimeString([], {
            hour: '2-digit',
            minute: '2-digit',
          })}
        </span>
      </div>
    </div>
  );
}
