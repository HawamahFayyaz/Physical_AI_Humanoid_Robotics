"""Pydantic models for API request/response schemas.

Based on specs/002-rag-chatbot/contracts/openapi.yaml
"""

from typing import List, Optional
from pydantic import BaseModel, Field


class SelectionContext(BaseModel):
    """Context from user text selection on page."""

    selected_text: str = Field(
        ...,
        description="The text content that the user selected",
        min_length=1,
        max_length=1000,
    )
    source_chapter: Optional[str] = Field(
        None,
        description="Chapter where text was selected",
    )
    source_url: Optional[str] = Field(
        None,
        description="Page URL where text was selected",
    )


class ConversationMessage(BaseModel):
    """A message in conversation history."""

    role: str = Field(
        ...,
        description="Message sender: 'user' or 'assistant'",
        pattern="^(user|assistant)$",
    )
    content: str = Field(
        ...,
        description="Message content",
        min_length=1,
    )


class ChatQueryRequest(BaseModel):
    """Request payload for POST /api/chat/query."""

    query: str = Field(
        ...,
        description="User's question about the book content",
        min_length=3,
        max_length=2000,
    )
    context: Optional[SelectionContext] = Field(
        None,
        description="Optional text selection context",
    )
    conversation_history: Optional[List[ConversationMessage]] = Field(
        None,
        description="Previous messages for conversation continuity (max 10)",
        max_length=10,
    )
    session_id: Optional[str] = Field(
        None,
        description="Session ID for logging and continuity",
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "query": "What is inverse kinematics?",
                    "session_id": "abc123",
                },
                {
                    "query": "Explain this concept in simpler terms",
                    "context": {
                        "selected_text": "The Jacobian matrix represents the relationship...",
                        "source_chapter": "Robot Kinematics",
                        "source_url": "/docs/03-kinematics/jacobian",
                    },
                    "conversation_history": [
                        {"role": "user", "content": "What is the Jacobian?"},
                        {"role": "assistant", "content": "The Jacobian is a matrix..."},
                    ],
                },
            ]
        }
    }


class Citation(BaseModel):
    """A source citation from the book."""

    chapter: str = Field(..., description="Chapter name")
    section: str = Field(..., description="Section within chapter")
    page_url: str = Field(..., description="Docusaurus page URL path")
    relevance_score: float = Field(
        ...,
        description="Similarity score (0-1)",
        ge=0,
        le=1,
    )
    snippet: Optional[str] = Field(
        None,
        description="Preview text snippet from source",
        max_length=200,
    )


class ChatQueryResponse(BaseModel):
    """Response payload from POST /api/chat/query."""

    answer: str = Field(..., description="Generated answer text (markdown)")
    citations: List[Citation] = Field(
        ...,
        description="Source citations for the answer",
    )
    processing_time_ms: int = Field(
        ...,
        description="Total query processing time in milliseconds",
        ge=0,
    )
    session_id: str = Field(..., description="Session ID for continuity")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "answer": "**Inverse kinematics (IK)** is the process of calculating...",
                    "citations": [
                        {
                            "chapter": "Robot Kinematics",
                            "section": "Inverse Kinematics",
                            "page_url": "/docs/03-kinematics/inverse-kinematics",
                            "relevance_score": 0.92,
                            "snippet": "Inverse kinematics computes the joint angles...",
                        }
                    ],
                    "processing_time_ms": 1850,
                    "session_id": "abc123",
                }
            ]
        }
    }


class ErrorResponse(BaseModel):
    """Error response payload."""

    error: str = Field(..., description="Error type identifier")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(None, description="Additional error details")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "validation_error",
                    "message": "Query must be at least 3 characters",
                    "details": {"field": "query", "min_length": 3},
                }
            ]
        }
    }


class ServiceHealth(BaseModel):
    """Health status of a single service."""

    status: str = Field(..., description="'healthy', 'unhealthy', or 'degraded'")
    error: Optional[str] = Field(None, description="Error message if unhealthy")
    details: Optional[dict] = Field(None, description="Additional status details")


class HealthResponse(BaseModel):
    """Response payload from GET /health."""

    status: str = Field(
        ...,
        description="Overall health: 'healthy', 'degraded', or 'unhealthy'",
    )
    version: str = Field(..., description="API version")
    services: dict = Field(..., description="Individual service health statuses")

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "status": "healthy",
                    "version": "1.0.0",
                    "services": {
                        "database": {"status": "healthy"},
                        "vector_store": {"status": "healthy", "vectors_count": 1500},
                        "openai": {"status": "healthy"},
                    },
                }
            ]
        }
    }
