"""LLM service using Groq llama-3.3-70b-versatile (ADR-005).

Free tier with fast inference via Groq's LPU hardware.
"""

import logging
from typing import List, Optional

from groq import AsyncGroq
from tenacity import retry, stop_after_attempt, wait_exponential

from config import get_settings

logger = logging.getLogger(__name__)

# Global Groq client
_client: AsyncGroq | None = None


def get_groq_client() -> AsyncGroq:
    """Get or create the Groq async client."""
    global _client
    if _client is None:
        settings = get_settings()
        _client = AsyncGroq(api_key=settings.groq_api_key)
        logger.info("Groq client initialized")
    return _client


# System prompt for the RAG chatbot
SYSTEM_PROMPT = """You are an expert AI assistant for the "Physical AI and Humanoid Robotics" educational book.

Your role:
- Answer questions accurately based ONLY on the provided context from the book
- Use clear, educational language appropriate for students learning robotics
- Include specific references to chapters and sections when citing information
- If the context doesn't contain enough information to answer, say so clearly
- Format responses with markdown for readability (headers, lists, code blocks)

Guidelines:
- Be concise but thorough
- Use examples from the provided context when helpful
- Cite sources by referring to the chapter and section
- If asked about topics not in the context, politely redirect to book-related topics
- For code examples, include complete, runnable snippets when possible

Response format:
- Start with a direct answer to the question
- Provide supporting details from the context
- End with relevant citations in format: [Chapter: Section]"""


def build_context_prompt(
    query: str,
    context_chunks: List[dict],
    selected_text: Optional[str] = None,
    conversation_history: Optional[List[dict]] = None,
) -> List[dict]:
    """Build the messages array for the LLM request.

    Args:
        query: User's question
        context_chunks: Retrieved chunks from vector search
        selected_text: Optional text user selected on page
        conversation_history: Optional previous messages

    Returns:
        List of message dicts for Groq API
    """
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Add conversation history if present
    if conversation_history:
        for msg in conversation_history[-5:]:  # Last 5 messages
            messages.append({
                "role": msg.get("role", "user"),
                "content": msg.get("content", ""),
            })

    # Build context from retrieved chunks
    context_parts = []
    for i, chunk in enumerate(context_chunks, 1):
        context_parts.append(
            f"[Source {i}]\n"
            f"Chapter: {chunk.get('chapter', 'Unknown')}\n"
            f"Section: {chunk.get('section', 'Unknown')}\n"
            f"Content:\n{chunk.get('content', '')}\n"
        )

    context_text = "\n---\n".join(context_parts)

    # Build user message
    user_content = f"""Context from the book:

{context_text}

---

"""

    if selected_text:
        user_content += f"""The user selected this text on the page:
"{selected_text}"

Based on this selection, answer: """

    user_content += query

    messages.append({"role": "user", "content": user_content})

    return messages


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10),
)
async def generate_response_groq(messages: List[dict]) -> str:
    """Generate response using Groq llama-3.3-70b-versatile.

    Args:
        messages: List of message dicts

    Returns:
        Generated response text
    """
    settings = get_settings()
    client = get_groq_client()

    response = await client.chat.completions.create(
        model=settings.groq_llm_model,
        messages=messages,
        max_tokens=1000,
        temperature=0.3,  # Lower for more factual responses
    )

    return response.choices[0].message.content or ""


async def generate_response(
    query: str,
    context_chunks: List[dict],
    selected_text: Optional[str] = None,
    conversation_history: Optional[List[dict]] = None,
) -> str:
    """Generate a response using Groq LLM.

    Args:
        query: User's question
        context_chunks: Retrieved chunks from vector search
        selected_text: Optional text user selected on page
        conversation_history: Optional previous messages

    Returns:
        Generated response text
    """
    messages = build_context_prompt(
        query=query,
        context_chunks=context_chunks,
        selected_text=selected_text,
        conversation_history=conversation_history,
    )

    try:
        logger.debug(f"Generating response with Groq {get_settings().groq_llm_model}")
        return await generate_response_groq(messages)
    except Exception as e:
        logger.error(f"Groq LLM generation failed: {e}")
        raise


async def check_connection() -> dict:
    """Check Groq LLM API connectivity.

    Returns:
        Dict with status and details
    """
    settings = get_settings()

    try:
        client = get_groq_client()
        await client.chat.completions.create(
            model=settings.groq_llm_model,
            messages=[{"role": "user", "content": "test"}],
            max_tokens=5,
        )
        return {
            "status": "healthy",
            "provider": "groq",
            "model": settings.groq_llm_model,
        }
    except Exception as e:
        logger.error(f"Groq health check failed: {e}")
        return {
            "status": "unhealthy",
            "provider": "groq",
            "error": str(e),
        }
