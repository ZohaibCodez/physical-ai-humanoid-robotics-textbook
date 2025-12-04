"""
Health Check Endpoint

Provides system health status for monitoring.
"""

from fastapi import APIRouter
from datetime import datetime
from app.models.chat import HealthResponse
from app.services.agent_service import agent_service
from app.services.vector_store import vector_store_service
from app.services.postgres_service import postgres_service
from app.utils.logger import app_logger as logger

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    
    Returns the health status of the application and its dependencies.
    Checks:
    - Gemini API (via agent service)
    - Qdrant vector database
    - Neon Postgres database
    """
    services = {}
    overall_status = "healthy"
    
    # Check Agent Service (Gemini)
    try:
        gemini_healthy = await agent_service.health_check()
        services["gemini"] = "healthy" if gemini_healthy else "unhealthy"
        if not gemini_healthy:
            overall_status = "degraded"
    except Exception as e:
        logger.error(f"Gemini health check failed: {str(e)}")
        services["gemini"] = "unhealthy"
        overall_status = "degraded"
    
    # Check Qdrant
    try:
        collection_info = await vector_store_service.get_collection_info()
        services["qdrant"] = "healthy" if collection_info.get("points_count", 0) > 0 else "degraded"
        if services["qdrant"] == "degraded":
            overall_status = "degraded"
    except Exception as e:
        logger.error(f"Qdrant health check failed: {str(e)}")
        services["qdrant"] = "unhealthy"
        overall_status = "unhealthy"
    
    # Check Postgres
    try:
        if postgres_service.pool:
            services["postgres"] = "healthy"
        else:
            services["postgres"] = "disabled"
            # Don't mark overall status as unhealthy - postgres is optional
            logger.info("Postgres is not connected - running without conversation history")
    except Exception as e:
        logger.error(f"Postgres health check failed: {str(e)}")
        services["postgres"] = "disabled"
    
    return HealthResponse(
        status=overall_status,
        services=services,
        timestamp=datetime.utcnow().isoformat()
    )
