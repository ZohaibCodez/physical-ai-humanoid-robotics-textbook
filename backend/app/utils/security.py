"""
Security utilities for password hashing and JWT token management.
"""

from datetime import datetime, timedelta
from typing import Optional
from uuid import UUID
import bcrypt
from jose import JWTError, jwt

from app.config import Settings
from app.models.user import TokenData


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.
    
    Args:
        plain_password: User-provided password
        hashed_password: Stored bcrypt hash
        
    Returns:
        True if password matches, False otherwise
    """
    return bcrypt.checkpw(
        plain_password.encode('utf-8'),
        hashed_password.encode('utf-8')
    )


def get_password_hash(password: str) -> str:
    """
    Hash a plain password using bcrypt.
    
    Args:
        password: Plain text password
        
    Returns:
        Bcrypt hashed password (includes salt)
        
    Raises:
        ValueError: If password exceeds bcrypt's 72-byte limit
    """
    # Bcrypt has a 72-byte limit
    password_bytes = password.encode('utf-8')
    
    if len(password_bytes) > 72:
        raise ValueError(f"Password is {len(password_bytes)} bytes, exceeds bcrypt 72-byte limit")
    
    # Generate salt with 12 rounds (~300ms, good security/performance balance)
    salt = bcrypt.gensalt(rounds=12)
    hashed = bcrypt.hashpw(password_bytes, salt)
    
    return hashed.decode('utf-8')


def create_access_token(user_id: UUID, email: str, settings: Settings) -> str:
    """
    Create a JWT access token.
    
    Args:
        user_id: User's UUID
        email: User's email address
        settings: Application settings (for JWT secret and expiry)
        
    Returns:
        Encoded JWT access token
    """
    now = datetime.utcnow()
    expire = now + timedelta(minutes=settings.access_token_expire_minutes)
    
    payload = {
        "user_id": str(user_id),
        "email": email,
        "exp": expire,
        "iat": now,
        "token_type": "access"
    }
    
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)


def create_refresh_token(user_id: UUID, email: str, settings: Settings) -> str:
    """
    Create a JWT refresh token (long-lived).
    
    Args:
        user_id: User's UUID
        email: User's email address
        settings: Application settings (for JWT secret and expiry)
        
    Returns:
        Encoded JWT refresh token
    """
    now = datetime.utcnow()
    expire = now + timedelta(days=settings.refresh_token_expire_days)
    
    payload = {
        "user_id": str(user_id),
        "email": email,
        "exp": expire,
        "iat": now,
        "token_type": "refresh"
    }
    
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)


def verify_token(token: str, settings: Settings, expected_type: str = "access") -> Optional[TokenData]:
    """
    Verify and decode a JWT token.
    
    Args:
        token: JWT token string
        settings: Application settings (for JWT secret)
        expected_type: Expected token type ("access" or "refresh")
        
    Returns:
        TokenData if valid, None if invalid/expired
    """
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        
        # Verify token type matches expected
        if payload.get("token_type") != expected_type:
            return None
        
        # Extract token data
        token_data = TokenData(
            user_id=UUID(payload.get("user_id")),
            email=payload.get("email"),
            exp=datetime.fromtimestamp(payload.get("exp")),
            iat=datetime.fromtimestamp(payload.get("iat")),
            token_type=payload.get("token_type")
        )
        
        return token_data
        
    except (JWTError, ValueError, KeyError):
        # Invalid token format, signature, or expired
        return None
