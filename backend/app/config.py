"""
Configuration Management

Loads environment variables and provides application settings.
"""

import os
from typing import List
from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""
    
    # API Keys
    google_api_key: str = Field(..., alias="GOOGLE_API_KEY")
    openai_api_key: str = Field(default="", alias="OPENAI_API_KEY")
    
    # Qdrant Vector Database
    qdrant_url: str = Field(..., alias="QDRANT_URL")
    qdrant_api_key: str = Field(..., alias="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="textbook_chunks", alias="QDRANT_COLLECTION_NAME")
    
    # Neon Postgres Database
    neon_database_url: str = Field(..., alias="NEON_DATABASE_URL")
    
    # Application Settings
    environment: str = Field(default="development", alias="ENVIRONMENT")
    log_level: str = Field(default="INFO", alias="LOG_LEVEL")
    max_workers: int = Field(default=4, alias="MAX_WORKERS")
    
    # Rate Limiting
    rate_limit_per_minute: int = Field(default=10, alias="RATE_LIMIT_PER_MINUTE")
    rate_limit_per_hour: int = Field(default=50, alias="RATE_LIMIT_PER_HOUR")
    
    # CORS
    cors_origins: str = Field(
        default="http://localhost:3000,http://localhost:8000",
        alias="CORS_ORIGINS"
    )
    
    # OpenAI Agents SDK / LiteLLM
    litellm_model: str = Field(default="gemini/gemini-1.5-flash", alias="LITELLM_MODEL")
    
    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]
    
    model_config = {
        "env_file": ".env",
        "case_sensitive": False,
        "extra": "ignore"  # Ignore extra fields in .env
    }


# Global settings instance
# Pydantic BaseSettings will automatically load from environment variables
settings = Settings(_env_file='.env', _env_file_encoding='utf-8')  # type: ignore
