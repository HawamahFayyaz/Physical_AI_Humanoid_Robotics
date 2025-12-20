"""Health check endpoint for monitoring service status."""

import logging
from typing import Any, Dict

from fastapi import APIRouter

from models.schemas import HealthResponse
from services.database import check_connection as check_db
from services.vector_store import check_connection as check_qdrant
from services.embedding import check_connection as check_embedding

logger = logging.getLogger(__name__)

router = APIRouter(tags=["Health"])


@router.get(
    "/health",
    response_model=HealthResponse,
    summary="Health Check",
    description="Check the health status of all backend services",
)
async def health_check() -> HealthResponse:
    """Check connectivity to all dependent services.

    Returns:
        HealthResponse with overall status and individual service statuses
    """
    services: Dict[str, Any] = {}
    overall_status = "healthy"

    # Check database (Neon Postgres)
    db_status = await check_db()
    services["database"] = db_status
    if db_status.get("status") != "healthy":
        overall_status = "degraded"

    # Check vector store (Qdrant)
    qdrant_status = await check_qdrant()
    services["vector_store"] = qdrant_status
    if qdrant_status.get("status") != "healthy":
        overall_status = "degraded"

    # Check embedding service (OpenAI)
    embedding_status = await check_embedding()
    services["embedding"] = embedding_status
    if embedding_status.get("status") != "healthy":
        overall_status = "degraded"

    # If all services are unhealthy, mark overall as unhealthy
    unhealthy_count = sum(
        1 for s in services.values() if s.get("status") == "unhealthy"
    )
    if unhealthy_count == len(services):
        overall_status = "unhealthy"

    return HealthResponse(
        status=overall_status,
        version="1.0.0",
        services=services,
    )


@router.get(
    "/health/live",
    summary="Liveness Check",
    description="Simple liveness probe for Kubernetes/container orchestration",
)
async def liveness() -> dict:
    """Simple liveness probe - always returns OK if the service is running."""
    return {"status": "alive"}


@router.get(
    "/health/ready",
    summary="Readiness Check",
    description="Readiness probe checking if the service can accept traffic",
)
async def readiness() -> dict:
    """Readiness probe - checks if core services are available."""
    try:
        # Quick check of vector store (most critical for serving requests)
        qdrant_status = await check_qdrant()
        if qdrant_status.get("status") == "healthy":
            return {"status": "ready", "vectors_count": qdrant_status.get("vectors_count", 0)}
        return {"status": "not_ready", "reason": "vector_store_unavailable"}
    except Exception as e:
        logger.error(f"Readiness check failed: {e}")
        return {"status": "not_ready", "reason": str(e)}
