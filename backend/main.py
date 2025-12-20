"""FastAPI application for the RAG Chatbot backend."""

import logging
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from config import get_settings
from services.database import get_db_pool, close_db_pool, init_schema
from services.vector_store import init_collection

# Configure logging
logging.basicConfig(
    level=logging.DEBUG if get_settings().debug else logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan manager for startup/shutdown events."""
    settings = get_settings()
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"CORS origins: {settings.cors_origins_list}")

    # Startup: Initialize services
    try:
        logger.info("Initializing database connection pool...")
        await get_db_pool()
        await init_schema()
        logger.info("Database initialized")

        logger.info("Initializing vector store collection...")
        await init_collection()
        logger.info("Vector store initialized")
    except Exception as e:
        logger.error(f"Failed to initialize services: {e}")
        # Continue anyway - health endpoint will report degraded status

    yield

    # Shutdown: Cleanup
    logger.info("Shutting down RAG Chatbot API...")
    await close_db_pool()
    logger.info("Database connection pool closed")


# Create FastAPI application
app = FastAPI(
    title="Physical AI Book RAG Chatbot",
    description="RAG-powered chatbot for the Physical AI and Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS middleware
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["*"],
    max_age=86400,  # Cache preflight for 24 hours
)


# Root endpoint
@app.get("/")
async def root() -> dict:
    """Root endpoint with API information."""
    return {
        "name": "Physical AI Book RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
    }


# Import and include routers
from routers import health, chat
app.include_router(health.router)
app.include_router(chat.router, prefix="/api/chat")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,
    )
