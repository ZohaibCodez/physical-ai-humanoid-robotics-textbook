"""
FastAPI dependencies for authentication and authorization.
"""

from typing import Optional
from fastapi import Depends, HTTPException, status, Cookie
from fastapi.security import OAuth2PasswordBearer

from app.config import Settings
from app.utils.security import verify_token
from app.models.user import TokenData


# OAuth2 password bearer scheme (optional for cookie-based auth)
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/v1/auth/login", auto_error=False)


def get_settings() -> Settings:
    """
    Dependency to get application settings.
    
    Returns:
        Settings instance
    """
    return Settings()


async def get_current_user(
    access_token: Optional[str] = Cookie(None),
    authorization: Optional[str] = Depends(oauth2_scheme),
    settings: Settings = Depends(get_settings)
) -> TokenData:
    """
    Dependency to get current authenticated user from JWT token.
    
    Checks for token in:
    1. HTTP-only cookie (access_token)
    2. Authorization header (Bearer token)
    
    Args:
        access_token: JWT token from HTTP-only cookie
        authorization: JWT token from Authorization header
        settings: Application settings
        
    Returns:
        TokenData with user information
        
    Raises:
        HTTPException: 401 if token is missing, invalid, or expired
    """
    # Try cookie first, then Authorization header
    token = access_token or authorization
    
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Verify token
    token_data = verify_token(token, settings, expected_type="access")
    
    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    return token_data


async def get_current_user_optional(
    access_token: Optional[str] = Cookie(None),
    authorization: Optional[str] = Depends(oauth2_scheme),
    settings: Settings = Depends(get_settings)
) -> Optional[TokenData]:
    """
    Dependency to get current user (if authenticated), or None if guest.
    
    Use this for routes that support both authenticated and guest access.
    
    Args:
        access_token: JWT token from HTTP-only cookie
        authorization: JWT token from Authorization header
        settings: Application settings
        
    Returns:
        TokenData if authenticated, None if guest (no exception raised)
    """
    token = access_token or authorization
    
    if not token:
        return None
    
    # Verify token (return None if invalid, don't raise exception)
    return verify_token(token, settings, expected_type="access")
