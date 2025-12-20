"""Database service for Neon Postgres connection and query logging."""

import json
import logging
from typing import Any, Dict, List, Optional

import asyncpg

from config import get_settings

logger = logging.getLogger(__name__)

# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def get_db_pool() -> asyncpg.Pool:
    """Get or create the database connection pool."""
    global _pool
    if _pool is None:
        settings = get_settings()
        logger.info("Initializing database connection pool...")
        _pool = await asyncpg.create_pool(
            dsn=settings.database_url,
            min_size=2,
            max_size=10,
            command_timeout=30,
        )
        logger.info("Database connection pool created")
    return _pool


async def close_db_pool() -> None:
    """Close the database connection pool."""
    global _pool
    if _pool is not None:
        logger.info("Closing database connection pool...")
        await _pool.close()
        _pool = None
        logger.info("Database connection pool closed")


async def init_schema() -> None:
    """Initialize database schema if not exists."""
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        # Read and execute migration
        schema_sql = """
        -- Enable UUID extension
        CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

        -- Book chunks metadata (vectors stored in Qdrant)
        CREATE TABLE IF NOT EXISTS book_chunks (
            id SERIAL PRIMARY KEY,
            chunk_id VARCHAR(500) UNIQUE NOT NULL,
            chapter VARCHAR(255) NOT NULL,
            section VARCHAR(255) NOT NULL,
            file_path VARCHAR(500) NOT NULL,
            word_count INTEGER NOT NULL CHECK (word_count > 0),
            created_at TIMESTAMPTZ DEFAULT NOW()
        );

        CREATE INDEX IF NOT EXISTS idx_book_chunks_chapter ON book_chunks(chapter);
        CREATE INDEX IF NOT EXISTS idx_book_chunks_file_path ON book_chunks(file_path);

        -- Query logs for analytics
        CREATE TABLE IF NOT EXISTS query_logs (
            id SERIAL PRIMARY KEY,
            session_id VARCHAR(255),
            query_text TEXT NOT NULL,
            response_summary TEXT,
            sources_used JSONB DEFAULT '[]',
            response_time_ms INTEGER NOT NULL CHECK (response_time_ms >= 0),
            context_type VARCHAR(50) DEFAULT 'full',
            selected_text TEXT,
            created_at TIMESTAMPTZ DEFAULT NOW()
        );

        CREATE INDEX IF NOT EXISTS idx_query_logs_session ON query_logs(session_id);
        CREATE INDEX IF NOT EXISTS idx_query_logs_created ON query_logs(created_at DESC);
        """
        await conn.execute(schema_sql)
        logger.info("Database schema initialized")


async def log_query(
    session_id: Optional[str],
    query_text: str,
    response_summary: Optional[str],
    sources_used: List[str],
    response_time_ms: int,
    context_type: str = "full",
    selected_text: Optional[str] = None,
) -> int:
    """Log a query to the database for analytics.

    Args:
        session_id: Browser session identifier
        query_text: User's question
        response_summary: First 500 chars of response
        sources_used: List of chunk_ids used
        response_time_ms: Total response time
        context_type: "full" or "selection"
        selected_text: Text user selected (if applicable)

    Returns:
        The ID of the inserted query log
    """
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        result = await conn.fetchval(
            """
            INSERT INTO query_logs (
                session_id, query_text, response_summary, sources_used,
                response_time_ms, context_type, selected_text
            )
            VALUES ($1, $2, $3, $4, $5, $6, $7)
            RETURNING id
            """,
            session_id,
            query_text,
            response_summary[:500] if response_summary else None,
            json.dumps(sources_used),
            response_time_ms,
            context_type,
            selected_text,
        )
        logger.debug(f"Query logged with ID: {result}")
        return result


async def upsert_chunk_metadata(
    chunk_id: str,
    chapter: str,
    section: str,
    file_path: str,
    word_count: int,
) -> None:
    """Insert or update chunk metadata in the database.

    Args:
        chunk_id: Unique chunk identifier
        chapter: Chapter name
        section: Section name
        file_path: Path to source file
        word_count: Number of words in chunk
    """
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        await conn.execute(
            """
            INSERT INTO book_chunks (chunk_id, chapter, section, file_path, word_count)
            VALUES ($1, $2, $3, $4, $5)
            ON CONFLICT (chunk_id) DO UPDATE SET
                chapter = EXCLUDED.chapter,
                section = EXCLUDED.section,
                file_path = EXCLUDED.file_path,
                word_count = EXCLUDED.word_count
            """,
            chunk_id,
            chapter,
            section,
            file_path,
            word_count,
        )


async def check_connection() -> Dict[str, Any]:
    """Check database connectivity for health endpoint.

    Returns:
        Dict with status and details
    """
    try:
        pool = await get_db_pool()
        async with pool.acquire() as conn:
            version = await conn.fetchval("SELECT version()")
            count = await conn.fetchval("SELECT COUNT(*) FROM book_chunks")
            return {
                "status": "healthy",
                "version": version.split(",")[0] if version else "unknown",
                "chunks_count": count,
            }
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        return {
            "status": "unhealthy",
            "error": str(e),
        }
