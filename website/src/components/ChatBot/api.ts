/**
 * API client for the RAG Chatbot backend.
 */

import {
  ChatQueryRequest,
  ChatQueryResponse,
  ErrorResponse,
  SelectionContext,
} from './types';
import {
  API_URL,
  API_TIMEOUT_MS,
  API_RETRY_COUNT,
  API_RETRY_DELAY_MS,
  ERROR_MESSAGES,
  CONVERSATION_CONTEXT_SIZE,
} from './constants';

/**
 * Custom error class for API errors.
 */
export class ChatAPIError extends Error {
  constructor(
    public readonly type: string,
    message: string,
    public readonly details?: Record<string, unknown>,
    public readonly retryAfter?: number
  ) {
    super(message);
    this.name = 'ChatAPIError';
  }
}

/**
 * Sleep for specified milliseconds.
 */
const sleep = (ms: number): Promise<void> =>
  new Promise((resolve) => setTimeout(resolve, ms));

/**
 * Perform a fetch request with timeout.
 */
async function fetchWithTimeout(
  url: string,
  options: RequestInit,
  timeoutMs: number = API_TIMEOUT_MS
): Promise<Response> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeoutMs);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    return response;
  } finally {
    clearTimeout(timeoutId);
  }
}

/**
 * Send a chat query to the backend API.
 *
 * @param query - The user's question
 * @param options - Optional configuration
 * @returns Promise resolving to ChatQueryResponse
 * @throws ChatAPIError on failure
 */
export async function chatQuery(
  query: string,
  options: {
    context?: SelectionContext;
    conversationHistory?: Array<{ role: 'user' | 'assistant'; content: string }>;
    sessionId?: string;
  } = {}
): Promise<ChatQueryResponse> {
  const { context, conversationHistory, sessionId } = options;

  // Build request payload
  const payload: ChatQueryRequest = {
    query,
    sessionId,
  };

  if (context) {
    payload.context = {
      selectedText: context.selectedText,
      sourceChapter: context.sourceChapter,
      sourceUrl: context.sourceUrl,
    };
  }

  if (conversationHistory && conversationHistory.length > 0) {
    // Only send the last N messages for context
    payload.conversationHistory = conversationHistory.slice(
      -CONVERSATION_CONTEXT_SIZE
    );
  }

  // Retry logic
  let lastError: Error | null = null;

  for (let attempt = 0; attempt <= API_RETRY_COUNT; attempt++) {
    try {
      if (attempt > 0) {
        await sleep(API_RETRY_DELAY_MS * attempt);
      }

      const response = await fetchWithTimeout(
        `${API_URL}/api/chat/query`,
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(payload),
        },
        API_TIMEOUT_MS
      );

      if (!response.ok) {
        const errorData = (await response.json().catch(() => ({}))) as ErrorResponse;

        switch (response.status) {
          case 400:
            throw new ChatAPIError(
              'validation_error',
              errorData.message || ERROR_MESSAGES.VALIDATION_ERROR,
              errorData.details
            );
          case 429:
            throw new ChatAPIError(
              'rate_limit',
              errorData.message || ERROR_MESSAGES.RATE_LIMIT_ERROR,
              errorData.details,
              (errorData as { retryAfter?: number }).retryAfter
            );
          case 503:
            throw new ChatAPIError(
              'service_unavailable',
              errorData.message || ERROR_MESSAGES.SERVER_ERROR,
              errorData.details
            );
          default:
            throw new ChatAPIError(
              'internal_error',
              errorData.message || ERROR_MESSAGES.SERVER_ERROR,
              errorData.details
            );
        }
      }

      const data = await response.json();

      // Map snake_case response to camelCase
      return {
        answer: data.answer,
        citations: data.citations.map((c: Record<string, unknown>) => ({
          chapter: c.chapter,
          section: c.section,
          pageUrl: c.page_url,
          relevanceScore: c.relevance_score,
          snippet: c.snippet,
        })),
        processingTimeMs: data.processing_time_ms,
        sessionId: data.session_id,
      };
    } catch (error) {
      lastError = error as Error;

      // Don't retry on validation errors or rate limits
      if (error instanceof ChatAPIError) {
        if (error.type === 'validation_error') {
          throw error;
        }
        if (error.type === 'rate_limit') {
          throw error;
        }
      }

      // Don't retry on abort (timeout)
      if (error instanceof Error && error.name === 'AbortError') {
        throw new ChatAPIError(
          'timeout',
          ERROR_MESSAGES.TIMEOUT_ERROR
        );
      }

      // Retry on network errors
      if (attempt < API_RETRY_COUNT) {
        console.warn(`Chat API request failed (attempt ${attempt + 1}), retrying...`, error);
        continue;
      }
    }
  }

  // All retries exhausted
  if (lastError instanceof ChatAPIError) {
    throw lastError;
  }

  throw new ChatAPIError(
    'network_error',
    ERROR_MESSAGES.NETWORK_ERROR
  );
}

/**
 * Check if the API is healthy.
 *
 * @returns Promise resolving to health status
 */
export async function checkHealth(): Promise<{
  status: 'healthy' | 'degraded' | 'unhealthy';
  services: Record<string, unknown>;
}> {
  try {
    const response = await fetchWithTimeout(
      `${API_URL}/health`,
      { method: 'GET' },
      5000
    );

    if (!response.ok) {
      return { status: 'unhealthy', services: {} };
    }

    return await response.json();
  } catch {
    return { status: 'unhealthy', services: {} };
  }
}
