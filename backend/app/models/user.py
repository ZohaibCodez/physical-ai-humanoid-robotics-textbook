"""
User and UserPreferences Pydantic models for request/response validation.
"""

from datetime import datetime
from typing import Optional, Literal
from uuid import UUID
from pydantic import BaseModel, EmailStr, Field, field_validator


# ==========================================
# Database Models (ORM-like, for internal use)
# ==========================================

class UserDB(BaseModel):
    """User model matching database schema."""
    id: UUID
    email: EmailStr
    hashed_password: str
    name: str
    is_active: bool = True
    is_verified: bool = False
    created_at: datetime
    updated_at: datetime
    last_login: Optional[datetime] = None

    class Config:
        from_attributes = True  # Pydantic v2 (was orm_mode in v1)


class UserPreferencesDB(BaseModel):
    """User preferences model matching database schema."""
    user_id: UUID
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


# ==========================================
# API Request Models
# ==========================================

class SignupRequest(BaseModel):
    """Signup request payload."""
    email: EmailStr
    password: str = Field(min_length=8, max_length=72, description="8-72 characters (bcrypt limit)")
    name: str = Field(max_length=255)
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]
    
    @field_validator('password')
    @classmethod
    def validate_password_bytes(cls, v: str) -> str:
        """Ensure password doesn't exceed bcrypt's 72-byte limit."""
        if len(v.encode('utf-8')) > 72:
            raise ValueError('Password exceeds 72 bytes (bcrypt limit)')
        return v


class LoginRequest(BaseModel):
    """Login request payload (OAuth2 password flow compatible)."""
    email: EmailStr
    password: str


class ProfileUpdateRequest(BaseModel):
    """Profile update request (all fields optional)."""
    name: Optional[str] = Field(None, max_length=255)
    software_level: Optional[Literal["beginner", "intermediate", "advanced"]] = None
    hardware_access: Optional[Literal["cloud_only", "basic", "full_lab"]] = None
    preferred_language: Optional[Literal["en", "ur", "both"]] = None


class PasswordChangeRequest(BaseModel):
    """Password change request."""
    current_password: str
    new_password: str = Field(min_length=8, max_length=72, description="8-72 characters (bcrypt limit)")
    
    @field_validator('new_password')
    @classmethod
    def validate_new_password_bytes(cls, v: str) -> str:
        """Ensure new password doesn't exceed bcrypt's 72-byte limit."""
        if len(v.encode('utf-8')) > 72:
            raise ValueError('Password exceeds 72 bytes (bcrypt limit)')
        return v


class PasswordResetRequest(BaseModel):
    """Password reset request (stub for future implementation)."""
    email: EmailStr


# ==========================================
# API Response Models
# ==========================================

class UserResponse(BaseModel):
    """User response (excludes sensitive data)."""
    id: UUID
    email: EmailStr
    name: str
    is_active: bool
    is_verified: bool
    created_at: datetime
    last_login: Optional[datetime] = None


class UserPreferencesResponse(BaseModel):
    """User preferences response."""
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]
    updated_at: datetime


class UserProfileResponse(BaseModel):
    """Combined user profile response (user + preferences)."""
    user: UserResponse
    preferences: UserPreferencesResponse


class AuthTokenResponse(BaseModel):
    """Authentication token response."""
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int  # Seconds until access token expires


class AuthResponse(BaseModel):
    """Full authentication response (after signup/login)."""
    user: UserResponse
    preferences: UserPreferencesResponse
    tokens: AuthTokenResponse


# ==========================================
# JWT Payload Models (internal)
# ==========================================

class TokenData(BaseModel):
    """JWT token payload."""
    user_id: UUID
    email: EmailStr
    exp: datetime  # Expiration timestamp
    iat: datetime  # Issued at timestamp
    token_type: Literal["access", "refresh"]
