# Research: RAG Chatbot Technical Decisions

**Feature**: 002-rag-chatbot
**Date**: 2025-12-17
**Status**: Complete

## Overview

This document captures the research findings and decisions for implementing the RAG chatbot for the Physical AI & Humanoid Robotics book. Each decision includes options evaluated, tradeoffs, and final recommendation.

---

## Decision 1: Embedding Model Selection

### Context
Need an embedding model to convert book content and user queries into vector representations for semantic search. Must support ~10-15K chunks and operate within free tier limits.

### Options Evaluated

| Option | Dimensions | Cost | Latency | Quality |
|--------|------------|------|---------|---------|
| OpenAI text-embedding-3-small | 1536 | $0.02/1M tokens | ~100ms | High |
| OpenAI text-embedding-3-large | 3072 | $0.13/1M tokens | ~150ms | Very High |
| Voyage AI voyage-2 | 1024 | Free tier: 50M tokens | ~80ms | High |
| Cohere embed-english-v3 | 1024 | Free tier: 100 req/min | ~60ms | High |
| HuggingFace sentence-transformers | 384-768 | Free (self-hosted) | ~50ms | Medium |

### Decision: **OpenAI text-embedding-3-small**

### Rationale
1. **Quality**: State-of-the-art performance for semantic search
2. **Consistency**: Same provider as LLM reduces integration complexity
3. **Cost**: Free tier covers initial development; production costs minimal (~$0.02 for entire book)
4. **Documentation**: Best-documented API, extensive examples
5. **Latency**: Meets <500ms requirement easily

### Tradeoffs Accepted
- Vendor lock-in with OpenAI
- No local/offline option
- Requires API key management

---

## Decision 2: LLM Model Selection

### Context
Need an LLM to generate responses based on retrieved context. Must be fast (<3s total response), accurate, and cost-effective within free tier.

### Options Evaluated

| Option | Speed | Cost | Context Window | Quality |
|--------|-------|------|----------------|---------|
| gpt-4o-mini | Fast (~1s) | $0.15/1M input | 128K | Very Good |
| gpt-4o | Medium (~2s) | $2.50/1M input | 128K | Excellent |
| gpt-3.5-turbo | Fast (~0.5s) | $0.50/1M input | 16K | Good |
| Groq llama-3.1-70b | Very Fast (~0.3s) | Free tier: 30 req/min | 32K | Good |
| Groq mixtral-8x7b | Very Fast (~0.2s) | Free tier: 30 req/min | 32K | Good |
| Claude 3 Haiku | Fast (~1s) | $0.25/1M input | 200K | Very Good |

### Decision: **gpt-4o-mini (primary), Groq llama-3.1-70b (fallback)**

### Rationale
1. **gpt-4o-mini**: Best balance of speed, quality, and cost for RAG
2. **128K context**: Can include extensive retrieved content
3. **Structured output**: Native JSON mode for citations
4. **Groq fallback**: Free tier provides backup during rate limits
5. **Response quality**: Superior instruction following for citation formatting

### Tradeoffs Accepted
- Not the absolute cheapest option (Groq is free)
- OpenAI dependency
- Need to implement fallback logic

---

## Decision 3: Chunking Strategy

### Context
Need to split book content into chunks for embedding and retrieval. Must balance context preservation vs. retrieval precision.

### Options Evaluated

| Strategy | Pros | Cons | Use Case |
|----------|------|------|----------|
| Fixed-size (500 words) | Simple, predictable | May split mid-sentence | Large uniform content |
| Semantic (paragraph-based) | Preserves meaning | Variable sizes | Narrative content |
| Sliding window (overlap) | Context continuity | Storage overhead | Technical content |
| Hybrid (headers + fixed) | Best of both | Complex implementation | Documentation |
| RecursiveCharacterTextSplitter | LangChain standard | Less control | General purpose |

### Decision: **Hybrid (Markdown-aware + Fixed-size with overlap)**

### Rationale
1. **Markdown-aware**: Split on headers (##, ###) first to preserve section boundaries
2. **Fixed-size fallback**: Large sections split at 600 words with 100-word overlap
3. **Metadata preservation**: Each chunk retains chapter, section, file path
4. **Code block handling**: Never split within code blocks
5. **Spec compliance**: Meets FR-014 (500-800 words per chunk)

### Implementation Approach
```
1. Parse MDX file, extract frontmatter
2. Split by H2 headers first (chapter sections)
3. If section > 800 words: split by H3 headers
4. If still > 800 words: fixed split with 100-word overlap
5. Add metadata: {chapter, section, file_path, chunk_index}
```

### Tradeoffs Accepted
- More complex than simple fixed-size
- Requires MDX parsing logic
- Slightly higher storage due to overlap

---

## Decision 4: Frontend Framework

### Context
Need a chat UI component for Docusaurus (React-based). Must support text selection detection, mobile responsiveness, and theme integration.

### Options Evaluated

| Option | Pros | Cons | Effort |
|--------|------|------|--------|
| OpenAI ChatKit SDK | Pre-built UI, streaming | Limited customization, may not match theme | Low |
| Vercel AI SDK | Streaming, React hooks | More setup, less UI | Medium |
| Custom React Component | Full control, theme match | Build from scratch | High |
| react-chat-widget | Ready-made widget | Outdated, hard to customize | Low |
| @chatscope/chat-ui-kit | Professional components | Heavy bundle, learning curve | Medium |

### Decision: **Custom React Component (TypeScript)**

### Rationale
1. **Theme integration**: Can exactly match Docusaurus light/dark modes
2. **Text selection**: Full control over Selection API integration
3. **Bundle size**: No unnecessary dependencies
4. **Customization**: Citation links, mobile layout, animations
5. **Maintainability**: No third-party update risks
6. **Spec compliance**: Meets FR-010 (theme matching), FR-009 (mobile)

### Component Structure
```
website/src/components/ChatBot/
├── index.tsx           # Main export, wrapper
├── ChatWidget.tsx      # Floating button + panel
├── ChatPanel.tsx       # Message list + input
├── ChatMessage.tsx     # Individual message with citations
├── TextSelection.tsx   # Selection detection + tooltip
├── hooks/
│   ├── useChat.ts      # Chat state management
│   └── useTextSelection.ts  # Selection API hook
└── styles.module.css   # CSS modules for theme
```

### Tradeoffs Accepted
- Higher initial development effort
- Must implement streaming manually
- Responsible for accessibility compliance

---

## Decision 5: Backend Deployment Platform

### Context
Need to deploy FastAPI backend that handles chat queries. Must be accessible from GitHub Pages frontend (CORS), support free tier, and have reasonable cold start times.

### Options Evaluated

| Platform | Free Tier | Cold Start | CORS | Persistence |
|----------|-----------|------------|------|-------------|
| Railway.app | $5 credit/month | ~2s | Yes | Yes |
| Render.com | 750 hrs/month | ~30s | Yes | Yes |
| Hugging Face Spaces | Unlimited (CPU) | ~10s | Yes | Limited |
| Vercel (serverless) | 100GB-hrs | ~1s | Yes | No |
| Fly.io | 3 shared VMs | ~2s | Yes | Yes |
| Deta Space | Unlimited | ~5s | Yes | Yes |

### Decision: **Railway.app (primary), Render.com (backup)**

### Rationale
1. **Railway**: Best developer experience, instant deploys, good free tier
2. **Cold start**: 2s acceptable, maintains <3s response for warm requests
3. **Persistence**: Can run background ingestion jobs
4. **Monitoring**: Built-in logs and metrics
5. **Render backup**: If Railway credits exhausted, Render free tier available

### Configuration
```
Backend URL: https://physical-ai-rag.railway.app
Health endpoint: /health
Main endpoint: /api/chat/query
CORS origins: ["https://hawamafahyaz.github.io", "http://localhost:3000"]
```

### Tradeoffs Accepted
- Railway free tier is credit-based ($5/month)
- Cold starts require keep-alive strategy
- Render has 30s cold start if used as fallback

---

## Decision 6: Text Selection Implementation

### Context
Need to detect when users select text on chapter pages and offer contextual query option. Must work on both desktop and mobile.

### Options Evaluated

| Approach | User Experience | Implementation | Mobile Support |
|----------|-----------------|----------------|----------------|
| Selection API + Floating Button | Natural, discoverable | Medium | Good |
| Context Menu (right-click) | Familiar | Easy | Poor (no right-click) |
| Permanent "Ask" button per paragraph | Always visible | Easy | Good |
| Selection API + Tooltip | Minimal UI | Medium | Medium |
| Long-press trigger (mobile) | Touch-native | Complex | Excellent |

### Decision: **Selection API + Floating Button (adaptive for mobile)**

### Rationale
1. **Desktop**: Floating button appears near selection, click to query
2. **Mobile**: Same behavior with touch-friendly button size
3. **Non-intrusive**: Button only appears when text is selected
4. **Discoverable**: Clear visual cue without permanent UI clutter
5. **Spec compliance**: Meets FR-004 (detect selection, offer query option)

### Implementation Approach
```typescript
// useTextSelection hook
1. Listen to 'selectionchange' event on document
2. On selection, get Selection API range
3. Calculate position near selection end
4. Show floating button at position
5. On click: send selected text to chat as context
6. On click outside or new selection: hide button
```

### Mobile Adaptations
- Larger touch target (44x44px minimum)
- Position above selection (not below) to avoid keyboard
- Delay appearance to avoid accidental triggers during scroll

### Tradeoffs Accepted
- Selection API has quirks across browsers
- Position calculation requires careful handling
- May conflict with native browser selection UI

---

## Decision 7: Vector Database

### Context
Need vector storage for semantic search of book chunks. Must support free tier with ~15K vectors at 1536 dimensions.

### Options Evaluated

| Option | Free Tier | Dimensions | Query Speed | Filtering |
|--------|-----------|------------|-------------|-----------|
| Qdrant Cloud | 1GB (free forever) | Any | Fast | Rich |
| Pinecone | 100K vectors | 1536 max | Fast | Basic |
| Weaviate Cloud | 14 days trial | Any | Fast | Rich |
| Supabase pgvector | 500MB | Any | Medium | SQL |
| Chroma (self-hosted) | Unlimited | Any | Fast | Basic |

### Decision: **Qdrant Cloud**

### Rationale
1. **Free tier**: 1GB forever, enough for ~50K vectors at 1536d
2. **Filtering**: Payload filtering for chapter/section metadata
3. **Performance**: Sub-200ms queries
4. **Python client**: Well-maintained, async support
5. **Constitution compliance**: Specified in constitution

### Configuration
```
Collection: physical-ai-book
Vector size: 1536
Distance metric: Cosine
Payload schema: {chapter, section, file_path, content}
```

### Tradeoffs Accepted
- External service dependency
- No local development option (use Qdrant Docker locally)
- Limited to 1GB storage

---

## Decision 8: Database for Logging

### Context
Need persistent storage for query logs (analytics) and optionally conversation state. Must support free tier.

### Options Evaluated

| Option | Free Tier | Async Support | Serverless |
|--------|-----------|---------------|------------|
| Neon Postgres | 512MB, 1 project | Yes (asyncpg) | Yes |
| Supabase Postgres | 500MB, 2 projects | Yes | Yes |
| PlanetScale MySQL | 5GB, 1 DB | Yes | Yes |
| Railway Postgres | Included in credit | Yes | No |
| SQLite (local) | Unlimited | No | No |

### Decision: **Neon Serverless Postgres**

### Rationale
1. **Free tier**: 512MB, sufficient for logs
2. **Serverless**: Scales to zero, no idle charges
3. **asyncpg**: Native async support for FastAPI
4. **Constitution compliance**: Specified in constitution
5. **Branching**: Can create dev branches for testing

### Schema Design
```sql
-- Query logs table
CREATE TABLE query_logs (
    id SERIAL PRIMARY KEY,
    query_text TEXT NOT NULL,
    response_summary TEXT,
    sources_used JSONB,
    response_time_ms INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

-- Book chunks table (metadata only, vectors in Qdrant)
CREATE TABLE book_chunks (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(255) UNIQUE NOT NULL,
    chapter VARCHAR(255),
    section VARCHAR(255),
    file_path VARCHAR(500),
    word_count INTEGER,
    created_at TIMESTAMPTZ DEFAULT NOW()
);
```

### Tradeoffs Accepted
- 512MB limit requires log rotation strategy
- Cold starts on first connection
- External service dependency

---

## Summary of Decisions

| Decision | Choice | Key Reason |
|----------|--------|------------|
| Embedding Model | OpenAI text-embedding-3-small | Quality + consistency |
| LLM Model | gpt-4o-mini + Groq fallback | Speed + quality + backup |
| Chunking | Hybrid (MD-aware + fixed + overlap) | Context preservation |
| Frontend | Custom React/TypeScript | Theme + customization |
| Deployment | Railway.app | Best DX + cold start |
| Text Selection | Selection API + Floating Button | Natural UX + mobile |
| Vector DB | Qdrant Cloud | Free tier + filtering |
| Database | Neon Postgres | Serverless + async |

---

## Risks and Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| OpenAI rate limits | Medium | High | Groq fallback, request queuing |
| Railway credit exhaustion | Low | Medium | Monitor usage, Render backup |
| Qdrant downtime | Low | High | Local cache for recent queries |
| Cold start exceeds 3s | Medium | Medium | Keep-alive cron, warm endpoints |
| Text selection browser quirks | Medium | Low | Feature detection, graceful degradation |

---

## Next Steps

1. Create ADR documents for significant decisions (ADR-001 through ADR-004)
2. Proceed to Phase 1: Design data model and API contracts
3. Generate tasks.md for implementation
