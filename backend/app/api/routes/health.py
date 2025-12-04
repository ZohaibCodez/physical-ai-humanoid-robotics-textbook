"""
Health Check Endpoint

Provides system health status for monitoring.
"""

from fastapi import APIRouter
from datetime import datetime
from app.models.chat import HealthResponse

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    
    Returns the health status of the application and its dependencies.
    """
    # TODO: Add actual health checks for Gemini, Qdrant, Postgres
    return HealthResponse(
        status="healthy",
        services={
            "gemini": "not_implemented",
            "qdrant": "not_implemented",
            "postgres": "not_implemented"
        },
        timestamp=datetime.utcnow().isoformat()
    )
