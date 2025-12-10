"""
RAG Chat Router - Qdrant + Sentence Transformers

Provides conversational Q&A over the Physical AI book content using:
- Qdrant Cloud for vector storage and retrieval
- Sentence Transformers for embedding generation
- Context-aware response generation
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, List
import os
import httpx
from qdrant_client import QdrantClient
from qdrant_client.http import models as qmodels
from sentence_transformers import SentenceTransformer

router = APIRouter()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
COLLECTION_NAME = "physical_ai_docs"
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "all-MiniLM-L6-v2")
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")

# Initialize clients lazily
_qdrant_client: Optional[QdrantClient] = None
_embedding_model: Optional[SentenceTransformer] = None


def get_qdrant_client() -> QdrantClient:
    """Get or create Qdrant client."""
    global _qdrant_client
    if _qdrant_client is None:
        if not QDRANT_URL or not QDRANT_API_KEY:
            raise HTTPException(
                status_code=503,
                detail="Qdrant not configured. Set QDRANT_URL and QDRANT_API_KEY."
            )
        _qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    return _qdrant_client


def get_embedding_model() -> SentenceTransformer:
    """Get or create embedding model."""
    global _embedding_model
    if _embedding_model is None:
        _embedding_model = SentenceTransformer(EMBEDDING_MODEL)
    return _embedding_model


class ChatRequest(BaseModel):
    """Chat request model."""
    message: str = Field(..., min_length=1, max_length=2000)
    conversation_id: Optional[str] = None
    top_k: int = Field(default=5, ge=1, le=20)


class ChatContext(BaseModel):
    """Retrieved context chunk."""
    content: str
    source: str
    score: float


class ChatResponse(BaseModel):
    """Chat response model."""
    response: str
    contexts: List[ChatContext]
    conversation_id: Optional[str] = None


async def generate_response_with_groq(
    query: str,
    contexts: List[ChatContext]
) -> str:
    """Generate response using Groq Llama 3."""
    if not GROQ_API_KEY:
        # Fallback: return context summary if no API key
        context_text = "\n\n".join([c.content for c in contexts[:3]])
        return f"Based on the documentation:\n\n{context_text}"

    context_text = "\n\n---\n\n".join([
        f"Source: {c.source}\n{c.content}" for c in contexts
    ])

    system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics book.
Answer questions based on the provided context from the book.
Be accurate, helpful, and cite specific chapters when relevant.
If the context doesn't contain enough information, say so clearly."""

    user_prompt = f"""Context from the book:
{context_text}

Question: {query}

Provide a helpful, accurate answer based on the context above."""

    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.groq.com/openai/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {GROQ_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": "llama-3.1-8b-instant",
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                "max_tokens": 1024,
                "temperature": 0.7
            },
            timeout=30.0
        )

        if response.status_code != 200:
            raise HTTPException(
                status_code=502,
                detail=f"LLM service error: {response.text}"
            )

        result = response.json()
        return result["choices"][0]["message"]["content"]


@router.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Query the RAG system with a question about Physical AI.

    Returns relevant context chunks and an AI-generated response.
    """
    try:
        # Get embedding for the query
        model = get_embedding_model()
        query_embedding = model.encode(request.message).tolist()

        # Search Qdrant for relevant chunks
        client = get_qdrant_client()
        search_results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=request.top_k,
            with_payload=True
        )

        # Extract contexts
        contexts = [
            ChatContext(
                content=hit.payload.get("content", ""),
                source=hit.payload.get("source", "unknown"),
                score=hit.score
            )
            for hit in search_results
        ]

        # Generate response
        response_text = await generate_response_with_groq(
            request.message,
            contexts
        )

        return ChatResponse(
            response=response_text,
            contexts=contexts,
            conversation_id=request.conversation_id
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def chat_health():
    """Check chat service health."""
    health = {
        "status": "healthy",
        "qdrant_configured": bool(QDRANT_URL and QDRANT_API_KEY),
        "groq_configured": bool(GROQ_API_KEY),
        "embedding_model": EMBEDDING_MODEL
    }
    return health
