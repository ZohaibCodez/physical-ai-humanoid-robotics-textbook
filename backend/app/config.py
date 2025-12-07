"""
Configuration Management

Loads environment variables and provides application settings.
"""

import os
from typing import List
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field, field_validator, ValidationError


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
    litellm_model: str = Field(default="gemini-1.5-flash", alias="LITELLM_MODEL")
    
    # JWT Authentication
    jwt_secret_key: str = Field(..., alias="JWT_SECRET_KEY")
    jwt_algorithm: str = Field(default="HS256", alias="JWT_ALGORITHM")
    access_token_expire_minutes: int = Field(default=30, alias="ACCESS_TOKEN_EXPIRE_MINUTES")
    refresh_token_expire_days: int = Field(default=30, alias="REFRESH_TOKEN_EXPIRE_DAYS")
    
    @field_validator('jwt_secret_key')
    @classmethod
    def validate_jwt_secret_key(cls, v: str) -> str:
        """Validate JWT secret key strength (minimum 32 characters)."""
        if len(v) < 32:
            raise ValueError('JWT_SECRET_KEY must be at least 32 characters long for security')
        return v
    
    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]
    
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )


# Global settings instance
# Pydantic BaseSettings will automatically load from environment variables
try:
    settings = Settings()
except ValidationError as e:
    print("❌ Configuration validation failed:")
    for error in e.errors():
        field = error['loc'][0]
        message = error['msg']
        print(f"  - {field}: {message}")
    raise SystemExit(1)
