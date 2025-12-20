"""Embedding service using Voyage AI voyage-3-lite (ADR-005).

Free tier: 50M tokens - sufficient for book content and queries.
"""

import logging
from typing import List

import voyageai
from tenacity import retry, stop_after_attempt, wait_exponential

from config import get_settings

logger = logging.getLogger(__name__)

# Global Voyage AI client
_client: voyageai.AsyncClient | None = None


def get_voyage_client() -> voyageai.AsyncClient:
    """Get or create the Voyage AI async client."""
    global _client
    if _client is None:
        settings = get_settings()
        _client = voyageai.AsyncClient(api_key=settings.voyage_api_key)
        logger.info("Voyage AI client initialized")
    return _client


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10),
)
async def embed_text(text: str, input_type: str = "query") -> List[float]:
    """Generate embedding for a single text using Voyage AI.

    Args:
        text: Text to embed
        input_type: "query" for user queries, "document" for book content

    Returns:
        Embedding vector (512 dimensions with voyage-3-lite)
    """
    settings = get_settings()
    client = get_voyage_client()

    result = await client.embed(
        texts=[text],
        model=settings.voyage_embedding_model,
        input_type=input_type,
    )

    return result.embeddings[0]


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10),
)
async def embed_texts(
    texts: List[str],
    input_type: str = "document",
    batch_size: int = 32,
    delay_between_batches: float = 0.5,
) -> List[List[float]]:
    """Generate embeddings for multiple texts in batches.

    Args:
        texts: List of texts to embed
        input_type: "query" for user queries, "document" for book content
        batch_size: Number of texts per API call (default: 32 for stability)
        delay_between_batches: Seconds to wait between API calls (rate limiting)

    Returns:
        List of embedding vectors
    """
    import asyncio
    settings = get_settings()
    client = get_voyage_client()

    all_embeddings: List[List[float]] = []
    total_batches = (len(texts) + batch_size - 1) // batch_size

    logger.info(f"Starting embedding generation: {len(texts)} texts in {total_batches} batches (batch_size={batch_size})")

    for i in range(0, len(texts), batch_size):
        batch_num = i // batch_size + 1
        batch = texts[i : i + batch_size]

        logger.info(f"Processing batch {batch_num}/{total_batches} ({len(batch)} texts)")
        logger.debug(f"Batch {batch_num} first text preview: {batch[0][:100]}...")

        try:
            result = await client.embed(
                texts=batch,
                model=settings.voyage_embedding_model,
                input_type=input_type,
            )
            all_embeddings.extend(result.embeddings)
            logger.info(f"Batch {batch_num}/{total_batches} completed successfully")

        except Exception as e:
            logger.error(f"Batch {batch_num} failed: {type(e).__name__}: {e}")
            raise

        # Rate limiting delay between batches
        if i + batch_size < len(texts):
            logger.debug(f"Waiting {delay_between_batches}s before next batch...")
            await asyncio.sleep(delay_between_batches)

    logger.info(f"Embedding generation complete: {len(all_embeddings)} embeddings created")
    return all_embeddings


async def check_connection() -> dict:
    """Check Voyage AI API connectivity.

    Returns:
        Dict with status and details
    """
    try:
        settings = get_settings()
        # Test with a minimal embedding
        embedding = await embed_text("test", input_type="query")
        return {
            "status": "healthy",
            "provider": "voyage-ai",
            "model": settings.voyage_embedding_model,
            "dimensions": len(embedding),
        }
    except Exception as e:
        logger.error(f"Voyage AI health check failed: {e}")
        return {
            "status": "unhealthy",
            "provider": "voyage-ai",
            "error": str(e),
        }
