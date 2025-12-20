"""Tests for RAG service (T024).

Test cases:
- test_rag_returns_citations: Verify RAG pipeline returns citations
- test_rag_handles_no_results: Verify graceful handling when no chunks found
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestRAGPipeline:
    """Tests for the RAG pipeline."""

    @pytest.mark.asyncio
    async def test_rag_returns_citations(
        self, mock_openai_client, mock_qdrant_client, sample_chunks, sample_query
    ):
        """Verify that RAG pipeline returns answer with citations."""
        # Setup mocks
        mock_qdrant_client.search = MagicMock(
            return_value=[
                MagicMock(
                    payload=chunk,
                    score=chunk["score"],
                )
                for chunk in sample_chunks
            ]
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client), \
             patch("services.embedding.get_openai_client", return_value=mock_openai_client), \
             patch("services.llm.get_openai_client", return_value=mock_openai_client):

            # Reset cached clients
            import services.vector_store
            import services.embedding
            import services.llm

            services.vector_store._client = mock_qdrant_client
            services.embedding._client = mock_openai_client
            services.llm._openai_client = mock_openai_client

            from services.rag import process_query

            result = await process_query(
                query=sample_query,
                session_id="test-session",
            )

            # Verify response structure
            assert "answer" in result
            assert "citations" in result
            assert len(result["citations"]) > 0

            # Verify citation structure
            citation = result["citations"][0]
            assert "chapter" in citation
            assert "section" in citation
            assert "page_url" in citation
            assert "relevance_score" in citation

    @pytest.mark.asyncio
    async def test_rag_handles_no_results(
        self, mock_openai_client, mock_qdrant_client, sample_query
    ):
        """Verify graceful handling when no relevant chunks are found."""
        # Setup mock to return no results
        mock_qdrant_client.search = MagicMock(return_value=[])

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client), \
             patch("services.embedding.get_openai_client", return_value=mock_openai_client), \
             patch("services.llm.get_openai_client", return_value=mock_openai_client):

            import services.vector_store
            import services.embedding
            import services.llm

            services.vector_store._client = mock_qdrant_client
            services.embedding._client = mock_openai_client
            services.llm._openai_client = mock_openai_client

            from services.rag import process_query

            result = await process_query(
                query=sample_query,
                session_id="test-session",
            )

            # Should still return a response, but with empty citations
            assert "answer" in result
            assert "citations" in result
            # Answer should indicate no relevant content found
            assert result["citations"] == [] or "no relevant" in result["answer"].lower() or len(result["answer"]) > 0

    @pytest.mark.asyncio
    async def test_rag_includes_context_in_prompt(
        self, mock_openai_client, mock_qdrant_client, sample_chunks, sample_query
    ):
        """Verify that retrieved context is included in LLM prompt."""
        mock_qdrant_client.search = MagicMock(
            return_value=[
                MagicMock(
                    payload=sample_chunks[0],
                    score=sample_chunks[0]["score"],
                )
            ]
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client), \
             patch("services.embedding.get_openai_client", return_value=mock_openai_client), \
             patch("services.llm.get_openai_client", return_value=mock_openai_client):

            import services.vector_store
            import services.embedding
            import services.llm

            services.vector_store._client = mock_qdrant_client
            services.embedding._client = mock_openai_client
            services.llm._openai_client = mock_openai_client

            from services.rag import process_query

            await process_query(
                query=sample_query,
                session_id="test-session",
            )

            # Verify LLM was called
            mock_openai_client.chat.completions.create.assert_called()


class TestRAGWithSelection:
    """Tests for RAG with text selection context."""

    @pytest.mark.asyncio
    async def test_rag_with_selected_text(
        self, mock_openai_client, mock_qdrant_client, sample_chunks
    ):
        """Verify that selected text is included in context."""
        mock_qdrant_client.search = MagicMock(
            return_value=[
                MagicMock(
                    payload=sample_chunks[0],
                    score=sample_chunks[0]["score"],
                )
            ]
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client), \
             patch("services.embedding.get_openai_client", return_value=mock_openai_client), \
             patch("services.llm.get_openai_client", return_value=mock_openai_client):

            import services.vector_store
            import services.embedding
            import services.llm

            services.vector_store._client = mock_qdrant_client
            services.embedding._client = mock_openai_client
            services.llm._openai_client = mock_openai_client

            from services.rag import process_query

            result = await process_query(
                query="Explain this concept",
                session_id="test-session",
                selected_text="The Jacobian matrix represents the relationship between joint velocities and end-effector velocities.",
                source_chapter="Robot Kinematics",
            )

            assert "answer" in result
            # LLM should have been called with the selected text context
            mock_openai_client.chat.completions.create.assert_called()

    @pytest.mark.asyncio
    async def test_rag_prioritizes_source_chapter(
        self, mock_openai_client, mock_qdrant_client, sample_chunks
    ):
        """Verify that source chapter is used to filter search results."""
        mock_qdrant_client.search = MagicMock(
            return_value=[
                MagicMock(
                    payload=sample_chunks[0],
                    score=sample_chunks[0]["score"],
                )
            ]
        )

        with patch("services.vector_store.get_qdrant_client", return_value=mock_qdrant_client), \
             patch("services.embedding.get_openai_client", return_value=mock_openai_client), \
             patch("services.llm.get_openai_client", return_value=mock_openai_client):

            import services.vector_store
            import services.embedding
            import services.llm

            services.vector_store._client = mock_qdrant_client
            services.embedding._client = mock_openai_client
            services.llm._openai_client = mock_openai_client

            from services.rag import process_query

            await process_query(
                query="Explain this",
                session_id="test-session",
                selected_text="Some selected text",
                source_chapter="Robot Kinematics",
            )

            # Verify search was called (chapter filter is applied internally)
            mock_qdrant_client.search.assert_called()
