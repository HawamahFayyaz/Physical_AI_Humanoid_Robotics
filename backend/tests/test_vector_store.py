"""Tests for vector store service (T023).

Test cases:
- test_search_returns_top_k: Verify correct number of results returned
- test_search_filters_by_chapter: Verify chapter filtering works
"""

import pytest
from unittest.mock import MagicMock, patch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestSearchSimilarChunks:
    """Tests for search_similar_chunks function."""

    @pytest.mark.asyncio
    async def test_search_returns_top_k(self, mock_qdrant_client, sample_embedding):
        """Verify that search returns the requested number of results."""
        # Setup mock to return 5 results
        mock_qdrant_client.search = MagicMock(
            return_value=[
                MagicMock(
                    payload={
                        "chunk_id": f"chunk_{i}",
                        "chapter": "Test Chapter",
                        "section": "Test Section",
                        "file_path": f"test_{i}.mdx",
                        "content": f"Content {i}",
                        "word_count": 10,
                    },
                    score=0.9 - (i * 0.05),
                )
                for i in range(5)
            ]
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import search_similar_chunks

            results = await search_similar_chunks(
                query_embedding=sample_embedding,
                top_k=5,
            )

            assert len(results) == 5
            # Results should be ordered by score
            assert results[0]["score"] >= results[1]["score"]
            # Each result should have required fields
            for result in results:
                assert "chunk_id" in result
                assert "chapter" in result
                assert "section" in result
                assert "content" in result
                assert "score" in result

    @pytest.mark.asyncio
    async def test_search_filters_by_chapter(self, mock_qdrant_client, sample_embedding):
        """Verify that chapter filter is applied correctly."""
        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import search_similar_chunks
            from qdrant_client.http import models

            await search_similar_chunks(
                query_embedding=sample_embedding,
                top_k=5,
                chapter_filter="Robot Kinematics",
            )

            # Verify search was called with a filter
            mock_qdrant_client.search.assert_called_once()
            call_kwargs = mock_qdrant_client.search.call_args[1]
            assert call_kwargs.get("query_filter") is not None

    @pytest.mark.asyncio
    async def test_search_respects_score_threshold(self, mock_qdrant_client, sample_embedding):
        """Verify that score threshold is passed to Qdrant."""
        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import search_similar_chunks

            await search_similar_chunks(
                query_embedding=sample_embedding,
                top_k=5,
                score_threshold=0.8,
            )

            call_kwargs = mock_qdrant_client.search.call_args[1]
            assert call_kwargs.get("score_threshold") == 0.8


class TestUpsertChunk:
    """Tests for upsert_chunk function."""

    @pytest.mark.asyncio
    async def test_upsert_chunk_stores_payload(self, mock_qdrant_client, sample_embedding):
        """Verify that chunk data is correctly stored."""
        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import upsert_chunk

            await upsert_chunk(
                chunk_id="test_chunk_1",
                embedding=sample_embedding,
                chapter="Test Chapter",
                section="Test Section",
                file_path="test.mdx",
                content="Test content",
                word_count=2,
            )

            mock_qdrant_client.upsert.assert_called_once()


class TestCheckConnection:
    """Tests for health check function."""

    @pytest.mark.asyncio
    async def test_check_connection_healthy(self, mock_qdrant_client):
        """Verify health check returns healthy status when connected."""
        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import check_connection

            result = await check_connection()

            assert result["status"] == "healthy"
            assert "vectors_count" in result

    @pytest.mark.asyncio
    async def test_check_connection_unhealthy(self, mock_qdrant_client):
        """Verify health check returns unhealthy status on error."""
        mock_qdrant_client.get_collection = MagicMock(
            side_effect=Exception("Connection failed")
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client):
            import services.vector_store
            services.vector_store._client = mock_qdrant_client

            from services.vector_store import check_connection

            result = await check_connection()

            assert result["status"] == "unhealthy"
            assert "error" in result
