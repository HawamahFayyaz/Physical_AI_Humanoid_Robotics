"""Pytest configuration and fixtures for RAG Chatbot tests."""

import asyncio
import os
from typing import AsyncGenerator, Generator
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi.testclient import TestClient


# Set test environment variables before importing app
os.environ.setdefault("OPENAI_API_KEY", "test-openai-key")
os.environ.setdefault("QDRANT_URL", "http://localhost:6333")
os.environ.setdefault("QDRANT_API_KEY", "test-qdrant-key")
os.environ.setdefault("DATABASE_URL", "postgresql://test:test@localhost/test")
os.environ.setdefault("CORS_ORIGINS", "http://localhost:3000")


@pytest.fixture(scope="session")
def event_loop() -> Generator[asyncio.AbstractEventLoop, None, None]:
    """Create event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def mock_openai_client() -> MagicMock:
    """Mock OpenAI client for embedding tests."""
    mock = MagicMock()
    mock.embeddings = MagicMock()
    mock.embeddings.create = AsyncMock(
        return_value=MagicMock(
            data=[MagicMock(embedding=[0.1] * 1536)]
        )
    )
    mock.chat = MagicMock()
    mock.chat.completions = MagicMock()
    mock.chat.completions.create = AsyncMock(
        return_value=MagicMock(
            choices=[MagicMock(message=MagicMock(content="Test response"))]
        )
    )
    return mock


@pytest.fixture
def mock_qdrant_client() -> MagicMock:
    """Mock Qdrant client for vector store tests."""
    mock = MagicMock()
    mock.search = MagicMock(
        return_value=[
            MagicMock(
                payload={
                    "chunk_id": "test-chunk-1",
                    "chapter": "Test Chapter",
                    "section": "Test Section",
                    "file_path": "website/docs/test.mdx",
                    "content": "Test content for retrieval.",
                    "word_count": 5,
                },
                score=0.95,
            )
        ]
    )
    mock.get_collections = MagicMock(
        return_value=MagicMock(collections=[])
    )
    mock.create_collection = MagicMock()
    mock.get_collection = MagicMock(
        return_value=MagicMock(vectors_count=100, points_count=100)
    )
    mock.upsert = MagicMock()
    return mock


@pytest.fixture
def mock_db_pool() -> MagicMock:
    """Mock asyncpg connection pool for database tests."""
    mock_conn = MagicMock()
    mock_conn.fetchval = AsyncMock(return_value=1)
    mock_conn.execute = AsyncMock()

    mock_pool = MagicMock()
    mock_pool.acquire = MagicMock(
        return_value=AsyncMock(
            __aenter__=AsyncMock(return_value=mock_conn),
            __aexit__=AsyncMock(return_value=None),
        )
    )

    return mock_pool


@pytest.fixture
def sample_chunks() -> list:
    """Sample chunk data for tests."""
    return [
        {
            "chunk_id": "chunk_abc123",
            "chapter": "Robot Kinematics",
            "section": "Inverse Kinematics",
            "file_path": "website/docs/03-kinematics/inverse.mdx",
            "content": "Inverse kinematics (IK) calculates joint angles from end-effector position.",
            "word_count": 10,
            "score": 0.95,
        },
        {
            "chunk_id": "chunk_def456",
            "chapter": "Robot Kinematics",
            "section": "Forward Kinematics",
            "file_path": "website/docs/03-kinematics/forward.mdx",
            "content": "Forward kinematics computes end-effector position from joint angles.",
            "word_count": 9,
            "score": 0.88,
        },
    ]


@pytest.fixture
def sample_query() -> str:
    """Sample query for tests."""
    return "What is inverse kinematics?"


@pytest.fixture
def sample_embedding() -> list:
    """Sample embedding vector for tests."""
    return [0.1] * 1536
