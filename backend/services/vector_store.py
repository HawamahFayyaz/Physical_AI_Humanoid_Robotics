"""Vector store service for Qdrant operations."""

import logging
from typing import Any, Dict, List, Optional

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct

from config import get_settings

logger = logging.getLogger(__name__)

# Global Qdrant client
_client: Optional[QdrantClient] = None


def get_qdrant_client() -> QdrantClient:
    """Get or create the Qdrant client."""
    global _client
    if _client is None:
        settings = get_settings()
        logger.info(f"Initializing Qdrant client for {settings.qdrant_url}...")
        _client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=30,
        )
        logger.info("Qdrant client initialized")
    return _client


async def init_collection() -> None:
    """Initialize the Qdrant collection if it doesn't exist."""
    settings = get_settings()
    client = get_qdrant_client()
    collection_name = settings.qdrant_collection_name

    # Check if collection exists
    collections = client.get_collections()
    existing_names = [c.name for c in collections.collections]

    if collection_name not in existing_names:
        logger.info(f"Creating Qdrant collection: {collection_name}")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=settings.voyage_embedding_dimensions,  # 512 for voyage-3-lite
                distance=Distance.COSINE,
            ),
        )
        logger.info(f"Collection {collection_name} created")
    else:
        logger.info(f"Collection {collection_name} already exists")


async def upsert_chunk(
    chunk_id: str,
    embedding: List[float],
    chapter: str,
    section: str,
    file_path: str,
    content: str,
    word_count: int,
) -> None:
    """Insert or update a chunk in the vector store.

    Args:
        chunk_id: Unique chunk identifier
        embedding: Vector embedding (512 dimensions for voyage-3-lite)
        chapter: Chapter name
        section: Section name
        file_path: Path to source file
        content: Text content of the chunk
        word_count: Number of words
    """
    settings = get_settings()
    client = get_qdrant_client()

    point = PointStruct(
        id=hash(chunk_id) % (2**63),  # Convert string ID to int
        vector=embedding,
        payload={
            "chunk_id": chunk_id,
            "chapter": chapter,
            "section": section,
            "file_path": file_path,
            "content": content,
            "word_count": word_count,
        },
    )

    client.upsert(
        collection_name=settings.qdrant_collection_name,
        points=[point],
    )


async def search_similar_chunks(
    query_embedding: List[float],
    top_k: int = 5,
    score_threshold: float = 0.7,
    chapter_filter: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """Search for chunks similar to the query embedding.

    Args:
        query_embedding: Query vector (512 dimensions for voyage-3-lite)
        top_k: Number of results to return
        score_threshold: Minimum similarity score (0-1)
        chapter_filter: Optional chapter name to filter results

    Returns:
        List of matching chunks with scores
    """
    settings = get_settings()
    client = get_qdrant_client()

    # Build filter if chapter specified
    search_filter = None
    if chapter_filter:
        search_filter = models.Filter(
            must=[
                models.FieldCondition(
                    key="chapter",
                    match=models.MatchValue(value=chapter_filter),
                )
            ]
        )

    # Use query_points (qdrant-client >= 1.7.0)
    results = client.query_points(
        collection_name=settings.qdrant_collection_name,
        query=query_embedding,
        limit=top_k,
        score_threshold=score_threshold,
        query_filter=search_filter,
    )

    return [
        {
            "chunk_id": hit.payload.get("chunk_id"),
            "chapter": hit.payload.get("chapter"),
            "section": hit.payload.get("section"),
            "file_path": hit.payload.get("file_path"),
            "content": hit.payload.get("content"),
            "word_count": hit.payload.get("word_count"),
            "score": hit.score,
        }
        for hit in results.points
    ]


async def check_connection() -> Dict[str, Any]:
    """Check Qdrant connectivity for health endpoint.

    Returns:
        Dict with status and details
    """
    try:
        settings = get_settings()
        client = get_qdrant_client()

        # Get collection info
        collection_info = client.get_collection(settings.qdrant_collection_name)
        # Handle different qdrant-client versions
        points_count = getattr(collection_info, 'points_count', 0)
        if hasattr(collection_info, 'vectors_count'):
            vectors_count = collection_info.vectors_count
        else:
            vectors_count = points_count  # fallback for newer API
        return {
            "status": "healthy",
            "collection": settings.qdrant_collection_name,
            "vectors_count": vectors_count,
            "points_count": points_count,
        }
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        return {
            "status": "unhealthy",
            "error": str(e),
        }


async def delete_collection() -> None:
    """Delete the collection (for testing/reset)."""
    settings = get_settings()
    client = get_qdrant_client()
    client.delete_collection(collection_name=settings.qdrant_collection_name)
    logger.info(f"Collection {settings.qdrant_collection_name} deleted")
