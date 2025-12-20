"""Application configuration using Pydantic Settings."""

from functools import lru_cache
from typing import List

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # Voyage AI Configuration (Embeddings) - ADR-005
    voyage_api_key: str
    voyage_embedding_model: str = "voyage-3-lite"
    voyage_embedding_dimensions: int = 512

    # Groq Configuration (LLM) - ADR-005
    groq_api_key: str
    groq_llm_model: str = "llama-3.3-70b-versatile"

    # Qdrant Vector Store
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "physical-ai-book"

    # Neon Postgres Database
    database_url: str

    # CORS Configuration
    cors_origins: str = "http://localhost:3000"

    # Server Configuration
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = False

    # Rate Limiting
    rate_limit_requests: int = 60
    rate_limit_window_seconds: int = 60

    # RAG Configuration
    rag_top_k: int = 5
    rag_similarity_threshold: float = 0.7
    rag_max_context_tokens: int = 4000

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse comma-separated CORS origins into a list."""
        return [origin.strip() for origin in self.cors_origins.split(",") if origin.strip()]


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
