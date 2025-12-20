"""Services package for Physical AI RAG Chatbot API."""

from services.database import get_db_pool, close_db_pool, log_query
from services.vector_store import get_qdrant_client, search_similar_chunks
from services.embedding import embed_text, embed_texts
from services.llm import generate_response
from services.rag import process_query, process_query_stream

__all__ = [
    # Database
    "get_db_pool",
    "close_db_pool",
    "log_query",
    # Vector Store
    "get_qdrant_client",
    "search_similar_chunks",
    # Embedding
    "embed_text",
    "embed_texts",
    # LLM
    "generate_response",
    # RAG
    "process_query",
    "process_query_stream",
]
