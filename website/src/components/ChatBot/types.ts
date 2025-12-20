/**
 * TypeScript interfaces for the RAG Chatbot component.
 * Based on data-model.md entity definitions.
 */

/**
 * Source citation pointing to book content.
 */
export interface Citation {
  /** Chapter name (e.g., "Introduction to Robotics") */
  chapter: string;
  /** Section within chapter (e.g., "Robot Kinematics") */
  section: string;
  /** Docusaurus page URL path (e.g., "/docs/03-kinematics/inverse-kinematics") */
  pageUrl: string;
  /** Relevance score from vector search (0-1) */
  relevanceScore: number;
  /** Preview text snippet from the source */
  snippet?: string;
}

/**
 * A single message in the conversation.
 */
export interface Message {
  /** Unique message identifier */
  id: string;
  /** Message sender: user or assistant */
  role: "user" | "assistant";
  /** Message text content (may contain markdown) */
  content: string;
  /** Citations for assistant messages */
  citations?: Citation[];
  /** ISO timestamp when message was created */
  timestamp: string;
  /** Whether message is still being generated */
  isLoading?: boolean;
  /** Error message if generation failed */
  error?: string;
}

/**
 * Conversation session containing multiple messages.
 */
export interface Conversation {
  /** Unique session identifier */
  sessionId: string;
  /** All messages in the conversation */
  messages: Message[];
  /** ISO timestamp when session started */
  startedAt: string;
  /** ISO timestamp of last activity */
  lastActivityAt: string;
}

/**
 * Text selection context from the current page.
 */
export interface SelectionContext {
  /** The selected text content */
  selectedText: string;
  /** Current chapter/page title */
  sourceChapter?: string;
  /** Current page URL path */
  sourceUrl?: string;
}

/**
 * Request payload for chat query API.
 */
export interface ChatQueryRequest {
  /** User's question */
  query: string;
  /** Optional text selection context */
  context?: SelectionContext;
  /** Previous messages for conversation continuity (last 5) */
  conversationHistory?: Array<{
    role: "user" | "assistant";
    content: string;
  }>;
  /** Session ID for logging */
  sessionId?: string;
}

/**
 * Response payload from chat query API.
 */
export interface ChatQueryResponse {
  /** Generated answer text */
  answer: string;
  /** Source citations */
  citations: Citation[];
  /** Query processing time in milliseconds */
  processingTimeMs: number;
  /** Session ID (generated if not provided) */
  sessionId: string;
}

/**
 * Error response from API.
 */
export interface ErrorResponse {
  /** Error type identifier */
  error: string;
  /** Human-readable error message */
  message: string;
  /** Additional error details */
  details?: Record<string, unknown>;
}

/**
 * Chat panel state.
 */
export interface ChatState {
  /** Current conversation */
  conversation: Conversation | null;
  /** Whether chat is loading a response */
  isLoading: boolean;
  /** Current error state */
  error: string | null;
  /** Text selection context */
  selectionContext: SelectionContext | null;
}

/**
 * Chat hook return type.
 */
export interface UseChatReturn {
  /** Current messages */
  messages: Message[];
  /** Whether currently loading */
  isLoading: boolean;
  /** Current error */
  error: string | null;
  /** Send a message */
  sendMessage: (content: string) => Promise<void>;
  /** Clear conversation */
  clearConversation: () => void;
  /** Set text selection context */
  setSelectionContext: (context: SelectionContext | null) => void;
  /** Current selection context */
  selectionContext: SelectionContext | null;
  /** Session ID */
  sessionId: string;
}
