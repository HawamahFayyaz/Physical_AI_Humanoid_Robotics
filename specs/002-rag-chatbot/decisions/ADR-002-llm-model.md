# ADR-002: LLM Model Selection

**Status**: Accepted
**Date**: 2025-12-17
**Decision Makers**: Development Team
**Feature**: 002-rag-chatbot

## Context

The RAG chatbot requires a Large Language Model to generate responses based on retrieved context from the book. The model must:
- Generate accurate, well-structured answers about robotics topics
- Include proper citations to source material
- Respond within the <3 second total latency target
- Support structured output (JSON) for citation formatting
- Operate within budget constraints (free tier preferred)

## Decision

**Use gpt-4o-mini as the primary LLM with Groq llama-3.1-70b as fallback**.

## Options Considered

### Option 1: gpt-4o-mini (CHOSEN - Primary)
- **Speed**: ~1s for typical responses
- **Cost**: $0.15/1M input, $0.60/1M output tokens
- **Context**: 128K tokens
- **Quality**: Excellent instruction following, JSON mode

### Option 2: gpt-4o
- **Speed**: ~2s for typical responses
- **Cost**: $2.50/1M input, $10/1M output tokens
- **Context**: 128K tokens
- **Quality**: Best overall, overkill for this use case

### Option 3: gpt-3.5-turbo
- **Speed**: ~0.5s for typical responses
- **Cost**: $0.50/1M input, $1.50/1M output tokens
- **Context**: 16K tokens
- **Quality**: Good but weaker instruction following

### Option 4: Groq llama-3.1-70b (CHOSEN - Fallback)
- **Speed**: ~0.3s (fastest inference)
- **Cost**: Free tier: 30 requests/minute
- **Context**: 32K tokens
- **Quality**: Good, open-source model

### Option 5: Claude 3 Haiku
- **Speed**: ~1s for typical responses
- **Cost**: $0.25/1M input, $1.25/1M output tokens
- **Context**: 200K tokens
- **Quality**: Very good, excellent reasoning

## Rationale

### Primary: gpt-4o-mini

1. **Best Balance**: Provides optimal speed-quality-cost balance for RAG applications.

2. **JSON Mode**: Native structured output support ensures consistent citation formatting without parsing errors.

3. **Context Window**: 128K tokens allows including extensive retrieved context (~10 chunks of 500 words each).

4. **Instruction Following**: Excellent at following citation format instructions and staying on-topic.

5. **Cost**: At $0.15/1M input tokens, even 1000 queries per day costs <$0.50.

### Fallback: Groq llama-3.1-70b

1. **Free Tier**: 30 requests/minute provides backup without additional cost.

2. **Speed**: Fastest inference available, ensures latency targets even under load.

3. **Availability**: Alternative provider avoids complete outage if OpenAI has issues.

4. **Quality**: Llama 3.1 70B is competitive with GPT-4 for many tasks.

## Consequences

### Positive
- Fast response times (typically <2s for LLM portion)
- Consistent citation formatting with JSON mode
- Cost-effective at scale
- Fallback ensures availability

### Negative
- Two APIs to manage (OpenAI + Groq)
- Groq free tier has rate limits
- Need to handle slightly different response formats between models
- OpenAI dependency for primary path

### Mitigations
- Abstract LLM service behind interface for easy switching
- Implement circuit breaker pattern for automatic fallback
- Standardize prompt format to work with both models
- Monitor rate limits and implement queuing if needed

## Implementation Notes

```python
# Primary: OpenAI gpt-4o-mini
async def generate_response_openai(context: str, query: str) -> dict:
    response = await openai_client.chat.completions.create(
        model="gpt-4o-mini",
        response_format={"type": "json_object"},
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
        ],
        temperature=0.7,
        max_tokens=1000
    )
    return json.loads(response.choices[0].message.content)

# Fallback: Groq llama-3.1-70b
async def generate_response_groq(context: str, query: str) -> dict:
    response = await groq_client.chat.completions.create(
        model="llama-3.1-70b-versatile",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
        ],
        temperature=0.7,
        max_tokens=1000
    )
    return parse_response(response.choices[0].message.content)
```

## System Prompt Design

```
You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
Answer questions based ONLY on the provided context from the book.
Always cite your sources using the format [Chapter: Section].
If the answer is not in the context, say so politely.
Format your response as JSON with "answer" and "citations" fields.
```

## References

- [OpenAI gpt-4o-mini](https://platform.openai.com/docs/models/gpt-4o-mini)
- [Groq API Documentation](https://console.groq.com/docs/quickstart)
- [Llama 3.1 Release](https://ai.meta.com/blog/meta-llama-3-1/)
