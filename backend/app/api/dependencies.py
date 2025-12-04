"""
FastAPI Dependency Injection

Provides shared dependencies for API routes.
"""

from typing import Optional
from fastapi import Header, HTTPException, status
from app.utils.exceptions import RateLimitError


async def get_session_id(x_session_id: Optional[str] = Header(None)) -> str:
    """
    Extract session ID from headers.
    
    Args:
        x_session_id: Session ID from X-Session-Id header
    
    Returns:
        Session ID string
    
    Raises:
        HTTPException: If session ID is missing
    """
    if not x_session_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="X-Session-Id header is required"
        )
    return x_session_id


async def check_rate_limit(session_id: str, user_ip: str):
    """
    Check if request is within rate limits.
    
    Args:
        session_id: User session identifier
        user_ip: User IP address
    
    Raises:
        HTTPException: If rate limit is exceeded
    """
    # TODO: Implement actual rate limiting using rate_limiter service
    pass


# TODO: Add more dependencies as needed:
# - get_agent_service()
# - get_vector_store()
# - get_postgres_connection()
# - get_embeddings_service()
