/**
 * ChatWidget component - Floating button with collapsible chat panel.
 */

import React, { useState, useCallback } from 'react';
import { ChatPanel } from './ChatPanel';
import { ErrorBoundary } from './ErrorBoundary';
import { useChat } from './hooks/useChat';
import { Citation, SelectionContext } from './types';
import styles from './styles.module.css';

interface ChatWidgetProps {
  /** Initial open state */
  defaultOpen?: boolean;
  /** Callback when chat is opened */
  onOpen?: () => void;
  /** Callback when chat is closed */
  onClose?: () => void;
  /** Initial selection context (e.g., from text selection) */
  initialContext?: SelectionContext;
}

/**
 * Main chat widget with floating button and expandable panel.
 */
export function ChatWidget({
  defaultOpen = false,
  onOpen,
  onClose,
  initialContext,
}: ChatWidgetProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const {
    messages,
    isLoading,
    error,
    sendMessage,
    clearConversation,
    setSelectionContext,
    selectionContext,
  } = useChat();

  // Set initial context when provided
  React.useEffect(() => {
    if (initialContext) {
      setSelectionContext(initialContext);
      setIsOpen(true);
    }
  }, [initialContext, setSelectionContext]);

  // Handle toggle
  const handleToggle = useCallback(() => {
    setIsOpen((prev) => {
      const newState = !prev;
      if (newState) {
        onOpen?.();
      } else {
        onClose?.();
      }
      return newState;
    });
  }, [onOpen, onClose]);

  // Handle close
  const handleClose = useCallback(() => {
    setIsOpen(false);
    onClose?.();
  }, [onClose]);

  // Handle citation click
  const handleCitationClick = useCallback((citation: Citation) => {
    // Close chat and navigate to citation
    handleClose();
    // Use Docusaurus navigation if available
    if (typeof window !== 'undefined') {
      window.location.href = citation.pageUrl;
    }
  }, [handleClose]);

  // Handle clear selection
  const handleClearSelection = useCallback(() => {
    setSelectionContext(null);
  }, [setSelectionContext]);

  return (
    <ErrorBoundary>
      <div className={styles.chatWidget}>
        {/* Floating Action Button */}
        <button
          className={`${styles.fab} ${isOpen ? styles.fabOpen : ''}`}
          onClick={handleToggle}
          aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
          aria-expanded={isOpen}
        >
          {isOpen ? (
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          ) : (
            <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
            </svg>
          )}
        </button>

        {/* Chat Panel */}
        {isOpen && (
          <ChatPanel
            messages={messages}
            isLoading={isLoading}
            error={error}
            selectionContext={selectionContext}
            onSendMessage={sendMessage}
            onClearConversation={clearConversation}
            onClearSelection={handleClearSelection}
            onCitationClick={handleCitationClick}
            onClose={handleClose}
          />
        )}
      </div>
    </ErrorBoundary>
  );
}

export default ChatWidget;
