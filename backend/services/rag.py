"""RAG (Retrieval-Augmented Generation) service for the chatbot.

Implements the core RAG pipeline:
1. Embed query using OpenAI text-embedding-3-small
2. Search Qdrant for relevant chunks
3. Build context and generate response with citations
"""

import logging
import re
import time
from typing import Any, Dict, List, Optional

from config import get_settings
from services.embedding import embed_text
from services.vector_store import search_similar_chunks
from services.llm import generate_response

logger = logging.getLogger(__name__)


def convert_file_path_to_url(file_path: str) -> str:
    """Convert a file path to a Docusaurus URL.

    Args:
        file_path: Path like 'website/docs/03-kinematics/inverse.mdx'

    Returns:
        URL path like '/docs/03-kinematics/inverse'
    """
    # Normalize path separators
    path = file_path.replace("\\", "/")

    # Remove website/docs prefix
    if "website/docs/" in path:
        path = path.split("website/docs/")[1]
    elif "docs/" in path:
        path = path.split("docs/")[1]

    # Remove extension
    if path.endswith(".mdx"):
        path = path[:-4]
    elif path.endswith(".md"):
        path = path[:-3]

    # Handle index files
    if path.endswith("/index") or path == "index":
        path = path.replace("/index", "").replace("index", "")

    return f"/docs/{path}" if path else "/docs/"


def build_citations(chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Build citation objects from retrieved chunks.

    Args:
        chunks: List of chunk dicts from vector search

    Returns:
        List of citation dicts for API response
    """
    citations = []
    seen_urls = set()

    for chunk in chunks:
        url = convert_file_path_to_url(chunk.get("file_path", ""))

        # Deduplicate citations by URL
        if url in seen_urls:
            continue
        seen_urls.add(url)

        citations.append({
            "chunk_id": chunk.get("chunk_id", ""),
            "chapter": chunk.get("chapter", "Unknown"),
            "section": chunk.get("section", "Unknown"),
            "page_url": url,
            "url": url,  # Also include as 'url' for API compatibility
            "relevance_score": chunk.get("score", 0.0),
            "snippet": chunk.get("content", "")[:200] if chunk.get("content") else None,
        })

    return citations


async def process_query(
    query: str,
    session_id: Optional[str] = None,
    selected_text: Optional[str] = None,
    source_chapter: Optional[str] = None,
    conversation_history: Optional[List[Dict[str, str]]] = None,
) -> Dict[str, Any]:
    """Process a RAG query and return response with citations.

    This is the main entry point for the RAG pipeline.

    Args:
        query: User's question
        session_id: Session identifier for logging
        selected_text: Optional text selection context
        source_chapter: Optional chapter to prioritize in search
        conversation_history: Optional previous conversation messages

    Returns:
        Dict containing:
        - answer: Generated response text
        - citations: List of source citations
        - processing_time_ms: Total processing time
        - session_id: Echoed session ID
    """
    start_time = time.time()
    settings = get_settings()

    logger.info(f"Processing query: {query[:100]}...")

    # Step 1: Embed the query
    try:
        query_embedding = await embed_text(query)
        logger.debug("Query embedded successfully")
    except Exception as e:
        logger.error(f"Failed to embed query: {e}")
        raise

    # Step 2: Search for relevant chunks
    try:
        # First search with chapter filter if provided
        chunks = []
        if source_chapter:
            chunks = await search_similar_chunks(
                query_embedding=query_embedding,
                top_k=settings.rag_top_k,
                score_threshold=settings.rag_similarity_threshold,
                chapter_filter=source_chapter,
            )
            logger.debug(f"Found {len(chunks)} chunks with chapter filter")

        # If no results or no filter, search without filter
        if not chunks:
            chunks = await search_similar_chunks(
                query_embedding=query_embedding,
                top_k=settings.rag_top_k,
                score_threshold=settings.rag_similarity_threshold,
            )
            logger.debug(f"Found {len(chunks)} chunks without filter")

    except Exception as e:
        logger.error(f"Failed to search vector store: {e}")
        raise

    # Step 3: Handle no results case
    if not chunks:
        logger.warning("No relevant chunks found for query")
        processing_time_ms = int((time.time() - start_time) * 1000)
        return {
            "answer": "I couldn't find relevant information in the book to answer your question. "
                     "Please try rephrasing your question or ask about a topic covered in the Physical AI book.",
            "citations": [],
            "processing_time_ms": processing_time_ms,
            "session_id": session_id or "",
        }

    # Step 4: Generate response using LLM
    try:
        answer = await generate_response(
            query=query,
            context_chunks=chunks,
            selected_text=selected_text,
            conversation_history=conversation_history,
        )
        logger.debug("Response generated successfully")
    except Exception as e:
        logger.error(f"Failed to generate response: {e}")
        raise

    # Step 5: Build citations
    citations = build_citations(chunks)

    # Calculate processing time
    processing_time_ms = int((time.time() - start_time) * 1000)

    logger.info(f"Query processed in {processing_time_ms}ms with {len(citations)} citations")

    return {
        "answer": answer,
        "citations": citations,
        "processing_time_ms": processing_time_ms,
        "session_id": session_id or "",
    }


async def process_query_stream(
    query: str,
    session_id: Optional[str] = None,
    selected_text: Optional[str] = None,
    source_chapter: Optional[str] = None,
    conversation_history: Optional[List[Dict[str, str]]] = None,
):
    """Process a RAG query with streaming response.

    Yields SSE events for real-time response display.

    Args:
        Same as process_query

    Yields:
        SSE event strings
    """
    start_time = time.time()
    settings = get_settings()

    # Step 1: Embed the query
    query_embedding = await embed_text(query)

    # Step 2: Search for relevant chunks
    chunks = await search_similar_chunks(
        query_embedding=query_embedding,
        top_k=settings.rag_top_k,
        score_threshold=settings.rag_similarity_threshold,
        chapter_filter=source_chapter,
    )

    if not chunks:
        chunks = await search_similar_chunks(
            query_embedding=query_embedding,
            top_k=settings.rag_top_k,
            score_threshold=settings.rag_similarity_threshold,
        )

    # Build citations early for streaming
    citations = build_citations(chunks)

    # For now, use non-streaming response since we need to implement
    # streaming separately. This is a placeholder that mimics streaming.
    if not chunks:
        yield 'event: chunk\ndata: {"content": "I couldn\'t find relevant information"}\n\n'
        yield f'event: citations\ndata: {{"citations": []}}\n\n'
        yield f'event: done\ndata: {{"response_time_ms": {int((time.time() - start_time) * 1000)}}}\n\n'
        return

    # Generate full response
    answer = await generate_response(
        query=query,
        context_chunks=chunks,
        selected_text=selected_text,
        conversation_history=conversation_history,
    )

    # Stream response in chunks (simulate streaming)
    import json
    chunk_size = 50
    for i in range(0, len(answer), chunk_size):
        chunk = answer[i:i + chunk_size]
        yield f'event: chunk\ndata: {json.dumps({"content": chunk})}\n\n'

    # Send citations
    yield f'event: citations\ndata: {json.dumps({"citations": citations})}\n\n'

    # Send done event
    processing_time_ms = int((time.time() - start_time) * 1000)
    yield f'event: done\ndata: {json.dumps({"response_time_ms": processing_time_ms})}\n\n'
