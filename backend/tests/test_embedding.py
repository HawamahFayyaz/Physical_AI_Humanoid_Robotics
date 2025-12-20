"""Tests for embedding service (T022).

Test cases:
- test_embed_query_returns_vector: Verify embedding has correct dimensions
- test_embed_handles_empty: Verify empty input handling
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TestEmbedText:
    """Tests for embed_text function."""

    @pytest.mark.asyncio
    async def test_embed_query_returns_vector(self, mock_openai_client):
        """Verify that embed_text returns a vector with correct dimensions."""
        with patch("services.embedding.get_openai_client", return_value=mock_openai_client):
            from services.embedding import embed_text

            # Clear the cached client
            import services.embedding
            services.embedding._client = mock_openai_client

            result = await embed_text("What is inverse kinematics?")

            # Verify embedding dimensions
            assert isinstance(result, list)
            assert len(result) == 1536

            # Verify OpenAI API was called correctly
            mock_openai_client.embeddings.create.assert_called_once()

    @pytest.mark.asyncio
    async def test_embed_handles_empty(self, mock_openai_client):
        """Verify that embed_text raises for empty input."""
        with patch("services.embedding.get_openai_client", return_value=mock_openai_client):
            import services.embedding
            services.embedding._client = mock_openai_client

            # Mock to raise for empty input
            mock_openai_client.embeddings.create = AsyncMock(
                side_effect=ValueError("Input cannot be empty")
            )

            from services.embedding import embed_text

            with pytest.raises(ValueError):
                await embed_text("")


class TestEmbedTexts:
    """Tests for embed_texts batch function."""

    @pytest.mark.asyncio
    async def test_embed_texts_returns_multiple_vectors(self, mock_openai_client):
        """Verify that embed_texts returns correct number of vectors."""
        # Setup mock to return multiple embeddings
        mock_openai_client.embeddings.create = AsyncMock(
            return_value=MagicMock(
                data=[
                    MagicMock(embedding=[0.1] * 1536),
                    MagicMock(embedding=[0.2] * 1536),
                    MagicMock(embedding=[0.3] * 1536),
                ]
            )
        )

        with patch("services.embedding.get_openai_client", return_value=mock_openai_client):
            import services.embedding
            services.embedding._client = mock_openai_client

            from services.embedding import embed_texts

            texts = ["Text 1", "Text 2", "Text 3"]
            result = await embed_texts(texts)

            assert len(result) == 3
            assert all(len(emb) == 1536 for emb in result)

    @pytest.mark.asyncio
    async def test_embed_texts_batches_large_inputs(self, mock_openai_client):
        """Verify that large inputs are processed in batches."""
        mock_openai_client.embeddings.create = AsyncMock(
            return_value=MagicMock(
                data=[MagicMock(embedding=[0.1] * 1536) for _ in range(50)]
            )
        )

        with patch("services.embedding.get_openai_client", return_value=mock_openai_client):
            import services.embedding
            services.embedding._client = mock_openai_client

            from services.embedding import embed_texts

            # Create 150 texts - should require 2 batches with batch_size=100
            texts = [f"Text {i}" for i in range(150)]
            result = await embed_texts(texts, batch_size=100)

            # Should have called API twice (100 + 50)
            assert mock_openai_client.embeddings.create.call_count >= 2
