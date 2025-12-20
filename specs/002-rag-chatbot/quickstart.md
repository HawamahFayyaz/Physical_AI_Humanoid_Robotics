# Quickstart Guide: RAG Chatbot

**Feature**: 002-rag-chatbot
**Date**: 2025-12-17

## Prerequisites

Before starting development, ensure you have:

- Python 3.11+
- Node.js 18+
- Git
- VS Code (recommended)

## Required Accounts (Free Tier)

1. **OpenAI**: [platform.openai.com](https://platform.openai.com)
   - Create API key for embeddings and chat completion
   - Free trial credits available

2. **Qdrant Cloud**: [cloud.qdrant.io](https://cloud.qdrant.io)
   - Create free cluster (1GB)
   - Note cluster URL and API key

3. **Neon**: [neon.tech](https://neon.tech)
   - Create free Postgres database (512MB)
   - Note connection string

4. **Railway**: [railway.app](https://railway.app)
   - Create account for backend deployment
   - $5/month free credit

## Local Development Setup

### 1. Clone and Navigate

```bash
cd Physical_AI_Humanoid_Robotics
git checkout 002-rag-chatbot
```

### 2. Backend Setup

```bash
# Create backend directory
mkdir -p backend
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn openai qdrant-client asyncpg python-dotenv pydantic

# Create .env file
cat > .env << EOF
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
DATABASE_URL=postgresql://user:pass@host/db
CORS_ORIGINS=http://localhost:3000,https://hawamafahyaz.github.io
EOF

# Run development server
uvicorn main:app --reload --port 8000
```

### 3. Frontend Setup

```bash
# From repo root
cd website

# Install dependencies
npm install

# Add chatbot component dependencies (if needed)
npm install uuid

# Start Docusaurus dev server
npm start
```

### 4. Test the Integration

```bash
# Test backend health
curl http://localhost:8000/health

# Test chat endpoint
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is inverse kinematics?"}'
```

## Project Structure

```
Physical_AI_Humanoid_Robotics/
├── backend/
│   ├── main.py              # FastAPI app entry
│   ├── routers/
│   │   └── chat.py          # Chat endpoints
│   ├── services/
│   │   ├── rag.py           # RAG pipeline
│   │   ├── embedding.py     # OpenAI embeddings
│   │   └── vector_store.py  # Qdrant client
│   ├── models/
│   │   └── schemas.py       # Pydantic models
│   ├── scripts/
│   │   └── ingest_content.py  # Content ingestion
│   ├── tests/
│   ├── .env.example
│   └── requirements.txt
│
├── website/
│   ├── src/
│   │   └── components/
│   │       └── ChatBot/
│   │           ├── index.tsx
│   │           ├── ChatWidget.tsx
│   │           ├── ChatPanel.tsx
│   │           ├── ChatMessage.tsx
│   │           ├── TextSelection.tsx
│   │           ├── hooks/
│   │           │   ├── useChat.ts
│   │           │   └── useTextSelection.ts
│   │           └── styles.module.css
│   │
│   ├── docusaurus.config.js
│   └── package.json
│
└── specs/002-rag-chatbot/
    ├── spec.md
    ├── plan.md
    ├── research.md
    ├── data-model.md
    ├── quickstart.md
    └── contracts/
        └── openapi.yaml
```

## Key Implementation Steps

### Step 1: Backend Foundation

1. Create FastAPI app with CORS middleware
2. Implement `/health` endpoint
3. Connect to Qdrant and Neon
4. Add basic error handling

### Step 2: RAG Pipeline

1. Create embedding service (OpenAI)
2. Create vector store service (Qdrant)
3. Implement context retrieval (top-k search)
4. Create response generation (OpenAI chat)

### Step 3: Content Ingestion

1. Parse MDX files from `website/docs/`
2. Extract frontmatter and headers
3. Chunk content (hybrid strategy)
4. Generate and store embeddings

### Step 4: Chat Endpoint

1. Implement `/api/chat/query`
2. Add request validation
3. Implement citation extraction
4. Add query logging

### Step 5: Frontend Component

1. Create ChatWidget (button + panel)
2. Implement message list and input
3. Add citation rendering with links
4. Style for Docusaurus theme

### Step 6: Text Selection

1. Create useTextSelection hook
2. Add floating button on selection
3. Pass selected text to chat context
4. Style for desktop and mobile

### Step 7: Integration

1. Add ChatBot to Docusaurus Root
2. Test end-to-end flow
3. Deploy backend to Railway
4. Update frontend API URL

## Environment Variables

### Backend (.env)

```
# Required
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...

# Optional
CORS_ORIGINS=http://localhost:3000,https://...github.io
LOG_LEVEL=INFO
RATE_LIMIT_REQUESTS=60
RATE_LIMIT_WINDOW=60
```

### Frontend (docusaurus.config.js)

```javascript
customFields: {
  chatbotApiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000',
}
```

## Testing Checklist

- [ ] Backend `/health` returns 200
- [ ] Qdrant collection is created
- [ ] Postgres tables are created
- [ ] Content ingestion completes
- [ ] Query returns response with citations
- [ ] Frontend chat widget opens/closes
- [ ] Messages display correctly
- [ ] Citations are clickable links
- [ ] Text selection shows floating button
- [ ] Mobile layout is responsive
- [ ] Light/dark theme works

## Common Issues

### CORS Errors
- Add frontend URL to `CORS_ORIGINS` in backend
- Ensure no trailing slashes

### Cold Start Timeouts
- Railway free tier has ~2s cold start
- First request may timeout, retry works

### Missing Citations
- Check that ingestion completed
- Verify Qdrant collection has vectors
- Check embedding dimension matches

### Selection Not Working
- Check Selection API browser support
- Ensure event listeners are on document
- Verify component is mounted on chapter pages

## Deployment

### Backend (Railway)

```bash
cd backend
railway init
railway up
railway domain  # Get public URL
```

### Frontend (GitHub Pages)

Update `docusaurus.config.js`:
```javascript
customFields: {
  chatbotApiUrl: 'https://your-app.railway.app',
}
```

Push to main branch - GitHub Actions deploys automatically.

## Next Steps

After completing quickstart:

1. Run `/sp.tasks` to generate detailed implementation tasks
2. Follow Red-Green-Refactor cycle per task
3. Create PR when all tests pass
