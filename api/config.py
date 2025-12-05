"""
Application configuration using pydantic-settings.
Loads environment variables from .env file.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Google Gemini
    gemini_api_key: str
    embedding_model: str = "models/text-embedding-004"
    chat_model: str = "gemini-1.5-flash"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "book_content"

    # Database
    database_url: str

    # Application
    environment: str = "development"
    cors_origins: str = "http://localhost:3000"

    # RAG Configuration
    chunk_size: int = 400  # tokens
    chunk_overlap: int = 50  # tokens
    retrieval_top_k: int = 5

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
