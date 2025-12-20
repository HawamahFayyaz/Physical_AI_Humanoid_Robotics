/**
 * Custom hook for managing chat state and interactions.
 */

import { useState, useCallback, useEffect, useRef } from 'react';
import {
  Message,
  Conversation,
  SelectionContext,
  UseChatReturn,
} from '../types';
import { chatQuery, ChatAPIError } from '../api';
import {
  MAX_MESSAGES,
  SESSION_TIMEOUT_MS,
  STORAGE_KEY,
  WELCOME_MESSAGE,
} from '../constants';

/**
 * Generate a unique ID for messages.
 * Uses crypto.randomUUID when available, falls back to timestamp + random.
 */
function generateId(): string {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  // Fallback for older browsers
  return `${Date.now()}-${Math.random().toString(36).slice(2, 11)}`;
}

/**
 * Get stored session from localStorage.
 */
function getStoredSession(): Conversation | null {
  if (typeof window === 'undefined') return null;

  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (!stored) return null;

    const conversation: Conversation = JSON.parse(stored);

    // Check if session has expired
    const lastActivity = new Date(conversation.lastActivityAt).getTime();
    if (Date.now() - lastActivity > SESSION_TIMEOUT_MS) {
      localStorage.removeItem(STORAGE_KEY);
      return null;
    }

    return conversation;
  } catch {
    localStorage.removeItem(STORAGE_KEY);
    return null;
  }
}

/**
 * Save session to localStorage.
 */
function saveSession(conversation: Conversation): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(conversation));
  } catch (error) {
    console.warn('Failed to save chat session:', error);
  }
}

/**
 * Create a new conversation with welcome message.
 */
function createNewConversation(): Conversation {
  const now = new Date().toISOString();
  const sessionId = generateId();

  return {
    sessionId,
    messages: [
      {
        id: generateId(),
        role: 'assistant',
        content: WELCOME_MESSAGE,
        timestamp: now,
      },
    ],
    startedAt: now,
    lastActivityAt: now,
  };
}

/**
 * Custom hook for chat functionality.
 *
 * Manages conversation state, message sending, and session persistence.
 */
export function useChat(): UseChatReturn {
  const [conversation, setConversation] = useState<Conversation | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectionContext, setSelectionContext] = useState<SelectionContext | null>(null);

  // Track if we've initialized
  const initialized = useRef(false);

  // Initialize conversation on mount
  useEffect(() => {
    if (initialized.current) return;
    initialized.current = true;

    const stored = getStoredSession();
    if (stored) {
      setConversation(stored);
    } else {
      setConversation(createNewConversation());
    }
  }, []);

  // Save conversation on changes
  useEffect(() => {
    if (conversation) {
      saveSession(conversation);
    }
  }, [conversation]);

  /**
   * Send a message and get a response.
   */
  const sendMessage = useCallback(
    async (content: string): Promise<void> => {
      if (!content.trim() || isLoading || !conversation) return;

      setError(null);

      // Create user message
      const userMessage: Message = {
        id: generateId(),
        role: 'user',
        content: content.trim(),
        timestamp: new Date().toISOString(),
      };

      // Create placeholder for assistant response
      const assistantPlaceholder: Message = {
        id: generateId(),
        role: 'assistant',
        content: '',
        timestamp: new Date().toISOString(),
        isLoading: true,
      };

      // Update conversation with user message and placeholder
      setConversation((prev) => {
        if (!prev) return prev;

        const messages = [...prev.messages, userMessage, assistantPlaceholder];

        // Trim to max messages
        const trimmedMessages = messages.slice(-MAX_MESSAGES);

        return {
          ...prev,
          messages: trimmedMessages,
          lastActivityAt: new Date().toISOString(),
        };
      });

      setIsLoading(true);

      try {
        // Build conversation history for context
        const historyMessages = conversation.messages
          .filter((m) => !m.isLoading && m.content !== WELCOME_MESSAGE)
          .map((m) => ({
            role: m.role,
            content: m.content,
          }));

        // Make API call
        const response = await chatQuery(content.trim(), {
          context: selectionContext || undefined,
          conversationHistory: historyMessages,
          sessionId: conversation.sessionId,
        });

        // Update assistant message with response
        setConversation((prev) => {
          if (!prev) return prev;

          const messages = prev.messages.map((msg) => {
            if (msg.id === assistantPlaceholder.id) {
              return {
                ...msg,
                content: response.answer,
                citations: response.citations,
                isLoading: false,
              };
            }
            return msg;
          });

          return {
            ...prev,
            messages,
            lastActivityAt: new Date().toISOString(),
          };
        });

        // Clear selection context after use
        setSelectionContext(null);
      } catch (err) {
        const errorMessage =
          err instanceof ChatAPIError
            ? err.message
            : 'An unexpected error occurred. Please try again.';

        // Update assistant message with error
        setConversation((prev) => {
          if (!prev) return prev;

          const messages = prev.messages.map((msg) => {
            if (msg.id === assistantPlaceholder.id) {
              return {
                ...msg,
                content: "I'm sorry, I couldn't process your request.",
                error: errorMessage,
                isLoading: false,
              };
            }
            return msg;
          });

          return {
            ...prev,
            messages,
            lastActivityAt: new Date().toISOString(),
          };
        });

        setError(errorMessage);
      } finally {
        setIsLoading(false);
      }
    },
    [conversation, isLoading, selectionContext]
  );

  /**
   * Clear conversation and start fresh.
   */
  const clearConversation = useCallback((): void => {
    localStorage.removeItem(STORAGE_KEY);
    setConversation(createNewConversation());
    setError(null);
    setSelectionContext(null);
  }, []);

  return {
    messages: conversation?.messages ?? [],
    isLoading,
    error,
    sendMessage,
    clearConversation,
    setSelectionContext,
    selectionContext,
    sessionId: conversation?.sessionId ?? '',
  };
}

export default useChat;
