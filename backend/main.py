"""
Physical AI & Humanoid Robotics - FastAPI Backend

This backend provides:
- RAG-powered chatbot with Qdrant vector storage
- User authentication with Better-Auth + Neon Postgres
- Content personalization with Groq Llama 3
- Urdu translation with caching
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import os
from dotenv import load_dotenv

load_dotenv()

# Import routers
from routers import chat, auth, personalize, translate


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup/shutdown."""
    # Startup
    print("Starting Physical AI Backend...")
    yield
    # Shutdown
    print("Shutting down Physical AI Backend...")


app = FastAPI(
    title="Physical AI & Humanoid Robotics API",
    description="Backend API for the Physical AI book with RAG chatbot, authentication, personalization, and translation",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS middleware configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://*.github.io",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register routers
app.include_router(chat.router, prefix="/api/chat", tags=["Chat"])
app.include_router(auth.router, prefix="/api/auth", tags=["Authentication"])
app.include_router(personalize.router, prefix="/api/personalize", tags=["Personalization"])
app.include_router(translate.router, prefix="/api/translate", tags=["Translation"])


@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring."""
    return {
        "status": "healthy",
        "service": "physical-ai-backend",
        "version": "1.0.0",
    }


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "Physical AI & Humanoid Robotics API",
        "docs": "/docs",
        "health": "/health",
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True,
    )
