# ADR-005: Free-Tier API Providers (Voyage AI + Groq)

**Status**: Accepted
**Date**: 2025-12-20
**Decision Makers**: Development Team
**Feature**: 002-rag-chatbot
**Supersedes**: Modifies ADR-001 (embeddings) and ADR-002 (LLM)

## Context

The original ADRs (001 and 002) selected OpenAI as the primary provider for both embeddings and LLM. However, due to project constraints requiring free-tier services only, we need to adapt the implementation to use alternative providers that offer generous free tiers suitable for hackathon development and initial deployment.

Available free-tier resources:
- **Voyage AI**: 50M free tokens for embeddings
- **Groq**: Free tier with generous rate limits for LLM inference
- **Qdrant Cloud**: Free tier for vector storage (already planned)
- **Neon Postgres**: Free tier for logging (already planned)

## Decision

**Replace OpenAI with Voyage AI for embeddings and Groq as the sole LLM provider.**

### Embedding: Voyage AI voyage-3-lite
- **Dimensions**: 512 (configurable up to 1024)
- **Cost**: Free tier: 50M tokens
- **Quality**: Competitive with OpenAI, optimized for retrieval
- **Latency**: ~80ms per request

### LLM: Groq llama-3.3-70b-versatile
- **Speed**: ~0.3s (fastest inference available)
- **Cost**: Free tier with generous limits
- **Context**: 128K tokens
- **Quality**: Competitive with GPT-4 for RAG applications

## Rationale

### Voyage AI for Embeddings

1. **Free Tier**: 50M tokens is more than sufficient for the ~15K chunks in the book plus query embeddings during development and production use.

2. **Quality**: Voyage AI models are specifically optimized for retrieval tasks and rank competitively on MTEB benchmarks.

3. **Simplicity**: The API is OpenAI-compatible, making migration straightforward.

4. **Dimension Flexibility**: voyage-3-lite supports 512-1024 dimensions, allowing optimization for storage vs quality tradeoff.

### Groq for LLM

1. **Zero Cost**: Free tier eliminates all LLM costs during development and initial production.

2. **Speed**: Groq's custom LPU hardware provides the fastest inference available, easily meeting <3s latency targets.

3. **Quality**: Llama 3.3 70B is highly capable for RAG applications with good instruction following.

4. **Reliability**: High uptime and consistent performance.

5. **Rate Limits**: Sufficient for educational book chatbot usage patterns.

## Implementation Changes

### config.py Updates

```python
# Remove OpenAI configuration
# openai_api_key: str
# openai_embedding_model: str = "text-embedding-3-small"

# Add Voyage AI configuration
voyage_api_key: str
voyage_embedding_model: str = "voyage-3-lite"
voyage_embedding_dimensions: int = 512

# Groq becomes primary (not fallback)
groq_api_key: str
groq_llm_model: str = "llama-3.3-70b-versatile"
```

### embedding.py Updates

```python
import voyageai

client = voyageai.AsyncClient(api_key=settings.voyage_api_key)

async def embed_text(text: str) -> List[float]:
    result = await client.embed(
        texts=[text],
        model=settings.voyage_embedding_model,
        input_type="query"  # or "document" for ingestion
    )
    return result.embeddings[0]
```

### llm.py Updates

```python
# Remove OpenAI, use Groq as primary
from groq import AsyncGroq

client = AsyncGroq(api_key=settings.groq_api_key)

async def generate_response(...) -> str:
    response = await client.chat.completions.create(
        model=settings.groq_llm_model,
        messages=messages,
        max_tokens=1000,
        temperature=0.3,
    )
    return response.choices[0].message.content
```

## Consequences

### Positive
- Zero API costs during development and initial production
- Faster LLM inference with Groq
- Simpler architecture (single provider per function)
- No OpenAI API key required

### Negative
- Smaller embedding dimensions (512 vs 1536) may slightly reduce retrieval quality
- Groq rate limits could affect high-traffic scenarios (unlikely for educational book)
- Less established providers compared to OpenAI
- Two different SDKs to manage (voyageai, groq)

### Mitigations
- Monitor retrieval quality; can upgrade to voyage-3 (1024 dims) if needed
- Implement request queuing if rate limits become an issue
- Abstract services behind interfaces for easy provider swaps
- Pin SDK versions for stability

## Qdrant Collection Update

The Qdrant collection must be configured for the new embedding dimensions:

```python
# Update from 1536 to 512 dimensions
vectors_config=VectorParams(
    size=512,  # Changed from 1536
    distance=Distance.COSINE,
)
```

**Important**: Any existing vectors must be re-indexed after this change.

## References

- [Voyage AI Documentation](https://docs.voyageai.com/)
- [Voyage AI Embedding Models](https://docs.voyageai.com/docs/embeddings)
- [Groq API Documentation](https://console.groq.com/docs/quickstart)
- [Llama 3.3 Model Card](https://huggingface.co/meta-llama/Llama-3.3-70B-Instruct)
