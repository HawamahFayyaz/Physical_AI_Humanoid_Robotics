/**
 * ChatPanel component - Main chat interface with message list, input, and header.
 */

import React, { useRef, useEffect, useState, useCallback } from 'react';
import { Message, Citation, SelectionContext } from './types';
import { ChatMessage } from './ChatMessage';
import { LoadingIndicator } from './LoadingIndicator';
import { ErrorState } from './ErrorBoundary';
import { INPUT_PLACEHOLDER, MAX_QUERY_LENGTH, MIN_QUERY_LENGTH } from './constants';
import styles from './styles.module.css';

interface ChatPanelProps {
  /** Array of messages to display */
  messages: Message[];
  /** Whether currently loading a response */
  isLoading: boolean;
  /** Current error message */
  error: string | null;
  /** Selection context to display */
  selectionContext: SelectionContext | null;
  /** Callback when user sends a message */
  onSendMessage: (content: string) => Promise<void>;
  /** Callback when user clears conversation */
  onClearConversation: () => void;
  /** Callback when user clears selection context */
  onClearSelection: () => void;
  /** Callback when user clicks a citation */
  onCitationClick?: (citation: Citation) => void;
  /** Callback to close the panel */
  onClose: () => void;
}

/**
 * Main chat panel component.
 */
export function ChatPanel({
  messages,
  isLoading,
  error,
  selectionContext,
  onSendMessage,
  onClearConversation,
  onClearSelection,
  onCitationClick,
  onClose,
}: ChatPanelProps): JSX.Element {
  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Scroll to bottom when messages change
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages, scrollToBottom]);

  // Focus input when panel opens
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  // Handle send message
  const handleSend = async () => {
    const trimmedInput = input.trim();
    if (!trimmedInput || isLoading) return;
    if (trimmedInput.length < MIN_QUERY_LENGTH) return;

    setInput('');
    await onSendMessage(trimmedInput);
  };

  // Handle keyboard events
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
    if (e.key === 'Escape') {
      onClose();
    }
  };

  // Handle input change with max length
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const value = e.target.value;
    if (value.length <= MAX_QUERY_LENGTH) {
      setInput(value);
    }
  };

  const canSend = input.trim().length >= MIN_QUERY_LENGTH && !isLoading;

  return (
    <div className={styles.chatPanel} role="dialog" aria-label="Chat with AI assistant">
      {/* Header */}
      <div className={styles.panelHeader}>
        <h2 className={styles.panelTitle}>Ask about Physical AI</h2>
        <div className={styles.headerActions}>
          <button
            onClick={onClearConversation}
            className={styles.headerButton}
            title="Clear conversation"
            aria-label="Clear conversation"
          >
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M3 6h18M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2" />
            </svg>
          </button>
          <button
            onClick={onClose}
            className={styles.closeButton}
            aria-label="Close chat"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      </div>

      {/* Selection Context Banner */}
      {selectionContext && (
        <div className={styles.selectionBanner}>
          <div className={styles.selectionContent}>
            <span className={styles.selectionLabel}>Asking about:</span>
            <span className={styles.selectionText}>
              "{selectionContext.selectedText.slice(0, 100)}
              {selectionContext.selectedText.length > 100 ? '...' : ''}"
            </span>
          </div>
          <button
            onClick={onClearSelection}
            className={styles.selectionClear}
            aria-label="Clear selection"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      )}

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.map((message) => (
          <ChatMessage
            key={message.id}
            message={message}
            onCitationClick={onCitationClick}
          />
        ))}

        {error && !isLoading && (
          <ErrorState
            message={error}
            onRetry={() => {
              if (messages.length > 1) {
                const lastUserMessage = [...messages].reverse().find((m) => m.role === 'user');
                if (lastUserMessage) {
                  onSendMessage(lastUserMessage.content);
                }
              }
            }}
          />
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className={styles.inputContainer}>
        <textarea
          ref={inputRef}
          value={input}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          placeholder={INPUT_PLACEHOLDER}
          className={styles.chatInput}
          rows={1}
          disabled={isLoading}
          aria-label="Chat message input"
        />
        <button
          onClick={handleSend}
          disabled={!canSend}
          className={styles.sendButton}
          aria-label="Send message"
        >
          {isLoading ? (
            <LoadingIndicator size="small" />
          ) : (
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13" />
              <polygon points="22 2 15 22 11 13 2 9 22 2" />
            </svg>
          )}
        </button>
      </div>

      {/* Character count */}
      {input.length > MAX_QUERY_LENGTH * 0.8 && (
        <div className={styles.charCount}>
          {input.length}/{MAX_QUERY_LENGTH}
        </div>
      )}
    </div>
  );
}

export default ChatPanel;
