# ADR-001: Embedding Model Selection

**Status**: Accepted
**Date**: 2025-12-17
**Decision Makers**: Development Team
**Feature**: 002-rag-chatbot

## Context

The RAG chatbot requires an embedding model to convert book content chunks and user queries into vector representations for semantic search. The model must:
- Support high-quality semantic similarity for technical robotics content
- Meet performance requirements (<500ms per embedding)
- Operate within free-tier or low-cost constraints
- Process approximately 10-15K text chunks during ingestion
- Handle varied content: prose, code blocks, mathematical concepts

## Decision

**Use OpenAI text-embedding-3-small** as the primary embedding model.

## Options Considered

### Option 1: OpenAI text-embedding-3-small (CHOSEN)
- **Dimensions**: 1536
- **Cost**: $0.02 per 1M tokens
- **Quality**: MTEB benchmark leader, excellent for technical content
- **Latency**: ~100ms per request

### Option 2: OpenAI text-embedding-3-large
- **Dimensions**: 3072
- **Cost**: $0.13 per 1M tokens
- **Quality**: Highest accuracy but overkill for this use case
- **Latency**: ~150ms per request

### Option 3: Voyage AI voyage-2
- **Dimensions**: 1024
- **Cost**: Free tier: 50M tokens
- **Quality**: Competitive with OpenAI
- **Latency**: ~80ms per request

### Option 4: Cohere embed-english-v3
- **Dimensions**: 1024
- **Cost**: Free tier: 100 requests/minute
- **Quality**: Good for English content
- **Latency**: ~60ms per request

### Option 5: Self-hosted sentence-transformers
- **Dimensions**: 384-768
- **Cost**: Free (compute costs only)
- **Quality**: Medium, varies by model
- **Latency**: ~50ms (depends on hardware)

## Rationale

1. **Quality**: text-embedding-3-small leads MTEB benchmarks for retrieval tasks and handles technical/code content well.

2. **Consistency**: Using the same provider (OpenAI) for both embeddings and LLM reduces integration complexity and API key management.

3. **Cost-Effectiveness**: At $0.02 per 1M tokens, embedding the entire book (~50K tokens) costs <$0.01. Even with development iterations, total cost stays under $1.

4. **Documentation**: OpenAI has the most comprehensive documentation and examples for RAG implementations.

5. **Reliability**: OpenAI has high uptime (>99.9%) and established error handling patterns.

6. **Future-Proofing**: Easy to upgrade to text-embedding-3-large if quality issues arise without changing infrastructure.

## Consequences

### Positive
- High-quality embeddings improve retrieval accuracy
- Simple integration with single OpenAI SDK
- Well-documented API with consistent behavior
- 1536 dimensions provide good balance of quality vs storage

### Negative
- Vendor lock-in with OpenAI
- No offline/local development option (requires API calls)
- API key security must be managed
- Rate limits could affect bulk ingestion (need batching)

### Mitigations
- Abstract embedding service behind interface for potential model swap
- Use environment variables for API key management
- Implement batch processing for ingestion (50 chunks per request)
- Cache embeddings during development to reduce API calls

## Implementation Notes

```python
# Example usage
from openai import AsyncOpenAI

client = AsyncOpenAI()

async def embed_text(text: str) -> list[float]:
    response = await client.embeddings.create(
        model="text-embedding-3-small",
        input=text,
        encoding_format="float"
    )
    return response.data[0].embedding
```

## References

- [OpenAI Embeddings Documentation](https://platform.openai.com/docs/guides/embeddings)
- [MTEB Leaderboard](https://huggingface.co/spaces/mteb/leaderboard)
- [text-embedding-3 Announcement](https://openai.com/blog/new-embedding-models-and-api-updates)
