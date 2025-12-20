/**
 * ChatBot component exports.
 *
 * Main entry point for the RAG chatbot widget.
 */

// Main widget - default export
export { ChatWidget as default } from './ChatWidget';

// Named exports for individual components
export { ChatWidget } from './ChatWidget';
export { ChatPanel } from './ChatPanel';
export { ChatMessage } from './ChatMessage';
export { LoadingIndicator } from './LoadingIndicator';
export { ErrorBoundary, ErrorState } from './ErrorBoundary';

// Hooks
export { useChat } from './hooks/useChat';

// API
export { chatQuery, checkHealth, ChatAPIError } from './api';

// Types
export type {
  Message,
  Citation,
  Conversation,
  SelectionContext,
  ChatQueryRequest,
  ChatQueryResponse,
  ErrorResponse,
  ChatState,
  UseChatReturn,
} from './types';

// Constants
export {
  API_URL,
  MAX_MESSAGES,
  SESSION_TIMEOUT_MS,
  STORAGE_KEY,
  MAX_QUERY_LENGTH,
  INPUT_PLACEHOLDER,
  WELCOME_MESSAGE,
} from './constants';
