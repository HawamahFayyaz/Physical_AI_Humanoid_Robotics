/**
 * ChatMessage component - Displays a single message with markdown and citations.
 */

import React from 'react';
import { Message, Citation } from './types';
import styles from './styles.module.css';

interface ChatMessageProps {
  message: Message;
  onCitationClick?: (citation: Citation) => void;
}

/**
 * Simple markdown renderer for chat messages.
 * Handles bold, italic, code, and code blocks.
 */
function renderMarkdown(text: string): React.ReactNode {
  // Split by code blocks first
  const parts = text.split(/(```[\s\S]*?```)/g);

  return parts.map((part, index) => {
    // Code block
    if (part.startsWith('```')) {
      const match = part.match(/```(\w+)?\n?([\s\S]*?)```/);
      if (match) {
        const [, language, code] = match;
        return (
          <pre key={index} className={styles.codeBlock}>
            {language && <span className={styles.codeLanguage}>{language}</span>}
            <code>{code.trim()}</code>
          </pre>
        );
      }
    }

    // Process inline formatting
    return (
      <span key={index}>
        {part.split(/(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`)/g).map((segment, segIndex) => {
          // Bold
          if (segment.startsWith('**') && segment.endsWith('**')) {
            return <strong key={segIndex}>{segment.slice(2, -2)}</strong>;
          }
          // Italic
          if (segment.startsWith('*') && segment.endsWith('*')) {
            return <em key={segIndex}>{segment.slice(1, -1)}</em>;
          }
          // Inline code
          if (segment.startsWith('`') && segment.endsWith('`')) {
            return <code key={segIndex} className={styles.inlineCode}>{segment.slice(1, -1)}</code>;
          }
          // Plain text - handle line breaks
          return segment.split('\n').map((line, lineIndex, arr) => (
            <React.Fragment key={`${segIndex}-${lineIndex}`}>
              {line}
              {lineIndex < arr.length - 1 && <br />}
            </React.Fragment>
          ));
        })}
      </span>
    );
  });
}

/**
 * Citation link component.
 */
function CitationLink({
  citation,
  index,
  onClick,
}: {
  citation: Citation;
  index: number;
  onClick?: (citation: Citation) => void;
}) {
  const handleClick = (e: React.MouseEvent) => {
    e.preventDefault();
    if (onClick) {
      onClick(citation);
    } else {
      // Navigate to the citation URL
      window.location.href = citation.pageUrl;
    }
  };

  return (
    <a
      href={citation.pageUrl}
      onClick={handleClick}
      className={styles.citationLink}
      title={`${citation.chapter} - ${citation.section}`}
    >
      <span className={styles.citationNumber}>[{index + 1}]</span>
      <span className={styles.citationText}>
        {citation.section || citation.chapter}
      </span>
      <span className={styles.citationScore}>
        {Math.round(citation.relevanceScore * 100)}%
      </span>
    </a>
  );
}

/**
 * Chat message component.
 */
export function ChatMessage({ message, onCitationClick }: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';
  const isLoading = message.isLoading;
  const hasError = !!message.error;

  return (
    <div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
      role="article"
      aria-label={`${isUser ? 'Your' : 'Assistant'} message`}
    >
      <div className={styles.messageAvatar}>
        {isUser ? (
          <span className={styles.userAvatar}>You</span>
        ) : (
          <span className={styles.assistantAvatar}>AI</span>
        )}
      </div>

      <div className={styles.messageContent}>
        {isLoading ? (
          <div className={styles.loadingDots} aria-label="Loading response">
            <span></span>
            <span></span>
            <span></span>
          </div>
        ) : (
          <>
            <div className={styles.messageText}>
              {renderMarkdown(message.content)}
            </div>

            {hasError && (
              <div className={styles.messageError} role="alert">
                {message.error}
              </div>
            )}

            {message.citations && message.citations.length > 0 && (
              <div className={styles.citations}>
                <span className={styles.citationsLabel}>Sources:</span>
                <div className={styles.citationsList}>
                  {message.citations.slice(0, 3).map((citation, index) => (
                    <CitationLink
                      key={`${citation.pageUrl}-${index}`}
                      citation={citation}
                      index={index}
                      onClick={onCitationClick}
                    />
                  ))}
                </div>
              </div>
            )}
          </>
        )}
      </div>

      <time
        className={styles.messageTimestamp}
        dateTime={message.timestamp}
        title={new Date(message.timestamp).toLocaleString()}
      >
        {new Date(message.timestamp).toLocaleTimeString([], {
          hour: '2-digit',
          minute: '2-digit',
        })}
      </time>
    </div>
  );
}

export default ChatMessage;
