"""Chat router for RAG chatbot API.

Implements:
- POST /api/chat/query - Submit a chat query
- POST /api/chat/stream - Submit a streaming chat query
"""

import logging
import time
import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from models.schemas import (
    ChatQueryRequest,
    ChatQueryResponse,
    Citation,
    ErrorResponse,
)
from services.rag import process_query, process_query_stream
from services.database import log_query

logger = logging.getLogger(__name__)

router = APIRouter(tags=["chat"])


@router.post(
    "/query",
    response_model=ChatQueryResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        429: {"model": ErrorResponse, "description": "Rate limit exceeded"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
        503: {"model": ErrorResponse, "description": "Service unavailable"},
    },
    summary="Submit a chat query",
    description="Process a natural language question about the book content.",
)
async def chat_query(request: ChatQueryRequest) -> ChatQueryResponse:
    """Process a chat query using the RAG pipeline.

    Args:
        request: ChatQueryRequest with query and optional context

    Returns:
        ChatQueryResponse with answer and citations
    """
    start_time = time.time()

    # Generate session ID if not provided
    session_id = request.session_id or str(uuid.uuid4())

    try:
        # Extract context if provided
        selected_text = None
        source_chapter = None
        if request.context:
            selected_text = request.context.selected_text
            source_chapter = request.context.source_chapter

        # Convert conversation history to dict format
        conversation_history = None
        if request.conversation_history:
            conversation_history = [
                {"role": msg.role, "content": msg.content}
                for msg in request.conversation_history
            ]

        # Process query through RAG pipeline
        result = await process_query(
            query=request.query,
            session_id=session_id,
            selected_text=selected_text,
            source_chapter=source_chapter,
            conversation_history=conversation_history,
        )

        # Convert citations to Pydantic models
        citations = [
            Citation(
                chapter=c["chapter"],
                section=c["section"],
                page_url=c["page_url"],
                relevance_score=c["relevance_score"],
                snippet=c.get("snippet"),
            )
            for c in result["citations"]
        ]

        # Log query for analytics
        response_time_ms = int((time.time() - start_time) * 1000)
        try:
            await log_query(
                session_id=session_id,
                query_text=request.query,
                response_summary=result["answer"][:500] if result["answer"] else None,
                sources_used=[c["chunk_id"] for c in result["citations"]],
                response_time_ms=response_time_ms,
                context_type="selection" if selected_text else "full",
                selected_text=selected_text,
            )
        except Exception as e:
            # Don't fail the request if logging fails
            logger.warning(f"Failed to log query: {e}")

        return ChatQueryResponse(
            answer=result["answer"],
            citations=citations,
            processing_time_ms=response_time_ms,
            session_id=session_id,
        )

    except ValueError as e:
        logger.warning(f"Validation error: {e}")
        raise HTTPException(
            status_code=400,
            detail={
                "error": "validation_error",
                "message": str(e),
            },
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "internal_error",
                "message": "An unexpected error occurred. Please try again.",
            },
        )


@router.post(
    "/stream",
    summary="Submit a streaming chat query",
    description="Same as /query but returns Server-Sent Events for real-time response.",
)
async def chat_query_stream(request: ChatQueryRequest):
    """Process a chat query with streaming response.

    Args:
        request: ChatQueryRequest with query and optional context

    Returns:
        StreamingResponse with SSE events
    """
    session_id = request.session_id or str(uuid.uuid4())

    # Extract context if provided
    selected_text = None
    source_chapter = None
    if request.context:
        selected_text = request.context.selected_text
        source_chapter = request.context.source_chapter

    # Convert conversation history to dict format
    conversation_history = None
    if request.conversation_history:
        conversation_history = [
            {"role": msg.role, "content": msg.content}
            for msg in request.conversation_history
        ]

    async def generate():
        try:
            async for event in process_query_stream(
                query=request.query,
                session_id=session_id,
                selected_text=selected_text,
                source_chapter=source_chapter,
                conversation_history=conversation_history,
            ):
                yield event
        except Exception as e:
            logger.error(f"Streaming error: {e}")
            yield f'event: error\ndata: {{"error": "internal_error", "message": "Stream interrupted"}}\n\n'

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )
