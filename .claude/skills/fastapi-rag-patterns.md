name: FastAPI RAG Patterns
description: Production patterns for RAG chatbot
FastAPI RAG Patterns
Main App
Python

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="RAG API")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"])

@app.post("/api/chat/query")
async def query(request: ChatRequest):
    chunks = await qdrant.search(request.query, top_k=5)
    answer = await llm.generate(request.query, chunks)
    return {"answer": answer, "sources": chunks}

@app.get("/api/health")
async def health():
    return {"status": "healthy"}
Qdrant Service
Python

from qdrant_client import AsyncQdrantClient

class QdrantService:
    async def search(self, query: str, top_k: int = 5):
        vector = await self.embed(query)
        return await self.client.search(
            collection_name="book",
            query_vector=vector,
            limit=top_k
        )
