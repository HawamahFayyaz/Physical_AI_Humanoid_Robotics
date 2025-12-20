/**
 * Configuration constants for the RAG Chatbot component.
 */

/**
 * Backend API URL - uses environment variable in production.
 * Falls back to localhost for development.
 */
export const API_URL =
  typeof window !== "undefined" &&
  (window as unknown as { CHATBOT_API_URL?: string }).CHATBOT_API_URL
    ? (window as unknown as { CHATBOT_API_URL: string }).CHATBOT_API_URL
    : process.env.CHATBOT_API_URL || "http://localhost:8000";

/**
 * Maximum number of messages to keep in conversation history.
 * Older messages are trimmed to stay within context limits.
 */
export const MAX_MESSAGES = 50;

/**
 * Number of recent messages to send as conversation context.
 * Sent with each query for continuity.
 */
export const CONVERSATION_CONTEXT_SIZE = 5;

/**
 * Session timeout in milliseconds (30 minutes).
 * After this period of inactivity, a new session starts.
 */
export const SESSION_TIMEOUT_MS = 30 * 60 * 1000;

/**
 * LocalStorage key for persisting conversation.
 */
export const STORAGE_KEY = "physical-ai-chatbot-session";

/**
 * Maximum query length in characters.
 */
export const MAX_QUERY_LENGTH = 2000;

/**
 * Minimum query length in characters.
 */
export const MIN_QUERY_LENGTH = 3;

/**
 * Maximum selected text length to include as context.
 */
export const MAX_SELECTION_LENGTH = 1000;

/**
 * Chat panel dimensions.
 */
export const PANEL_WIDTH = 400;
export const PANEL_HEIGHT = 600;
export const PANEL_MOBILE_BREAKPOINT = 768;

/**
 * FAB (Floating Action Button) dimensions.
 */
export const FAB_SIZE = 56;
export const FAB_MARGIN = 16;

/**
 * Animation durations in milliseconds.
 */
export const ANIMATION_DURATION = 200;

/**
 * API request timeout in milliseconds.
 */
export const API_TIMEOUT_MS = 30000;

/**
 * Retry configuration for API calls.
 */
export const API_RETRY_COUNT = 2;
export const API_RETRY_DELAY_MS = 1000;

/**
 * Default error messages.
 */
export const ERROR_MESSAGES = {
  NETWORK_ERROR: "Unable to connect to the server. Please check your internet connection.",
  TIMEOUT_ERROR: "The request took too long. Please try again.",
  RATE_LIMIT_ERROR: "Too many requests. Please wait a moment before trying again.",
  SERVER_ERROR: "Something went wrong on our end. Please try again later.",
  VALIDATION_ERROR: "Invalid request. Please check your input.",
  UNKNOWN_ERROR: "An unexpected error occurred. Please try again.",
} as const;

/**
 * Placeholder text for input field.
 */
export const INPUT_PLACEHOLDER = "Ask about the Physical AI book...";

/**
 * Welcome message shown when chat is first opened.
 */
export const WELCOME_MESSAGE = `Hi! I'm your AI assistant for the Physical AI and Humanoid Robotics book.

I can help you:
- Answer questions about robotics concepts
- Explain code examples
- Find relevant chapters and sections

What would you like to know?`;
