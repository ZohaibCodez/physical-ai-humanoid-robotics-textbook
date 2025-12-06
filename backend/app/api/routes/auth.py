"""
Authentication API routes - Signup, Login, Logout, Session, Refresh.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Response
from fastapi.responses import JSONResponse
import asyncpg
import traceback

from app.models.user import SignupRequest, LoginRequest, AuthResponse, AuthTokenResponse, UserProfileResponse, ProfileUpdateRequest
from app.services.auth_service import signup_user, login_user, get_user_profile, update_user_preferences
from app.utils.security import create_access_token, create_refresh_token, verify_token
from app.utils.dependencies import get_settings, get_current_user
from app.config import Settings
from app.models.user import TokenData


router = APIRouter(prefix="/v1/auth", tags=["Authentication"])


async def get_db_connection(settings: Settings = Depends(get_settings)):
    """
    Dependency to get database connection from .env NEON_DATABASE_URL.
    
    Yields:
        Database connection
    """
    conn = await asyncpg.connect(settings.neon_database_url)
    try:
        yield conn
    finally:
        await conn.close()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    signup_data: SignupRequest,
    response: Response,
    db: asyncpg.Connection = Depends(get_db_connection),
    settings: Settings = Depends(get_settings)
):
    """
    Create new user account with background profiling.
    
    Request body:
    - email: User's email address
    - password: Plain text password (min 8 chars)
    - name: User's display name
    - software_level: beginner | intermediate | advanced
    - hardware_access: cloud_only | basic | full_lab
    - preferred_language: en | ur | both
    
    Returns:
    - 201: Account created successfully with tokens
    - 400: Validation error
    - 409: Email already exists
    - 500: Server error
    """
    try:
        # Create user account
        profile, error = await signup_user(signup_data, db)
        
        if error:
            # Determine status code based on error type
            if "already exists" in error:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail={"message": error, "code": "EMAIL_EXISTS"}
                )
            elif "Invalid" in error or "must" in error:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail={"message": error, "code": "VALIDATION_ERROR"}
                )
            else:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail={"message": error, "code": "SERVER_ERROR"}
                )
        
        # Generate JWT tokens
        access_token = create_access_token(profile.user.id, profile.user.email, settings)
        refresh_token = create_refresh_token(profile.user.id, profile.user.email, settings)
        
        # Set HTTP-only cookies
        response.set_cookie(
            key="access_token",
            value=access_token,
            httponly=True,
            secure=True,  # HTTPS only in production
            samesite="strict",
            max_age=settings.access_token_expire_minutes * 60
        )
        
        response.set_cookie(
            key="refresh_token",
            value=refresh_token,
            httponly=True,
            secure=True,
            samesite="strict",
            max_age=settings.refresh_token_expire_days * 24 * 60 * 60
        )
        
        # Build response
        tokens = AuthTokenResponse(
            access_token=access_token,
            refresh_token=refresh_token,
            token_type="bearer",
            expires_in=settings.access_token_expire_minutes * 60
        )
        
        return AuthResponse(
            user=profile.user,
            preferences=profile.preferences,
            tokens=tokens
        )
    
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log unexpected errors
        print(f"Unexpected error in signup endpoint: {e}")
        print(f"Error type: {type(e).__name__}")
        traceback.print_exc()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"message": "Internal server error", "code": "SERVER_ERROR"}
        )


@router.post("/login", response_model=AuthResponse)
async def login(
    login_data: LoginRequest,
    response: Response,
    db: asyncpg.Connection = Depends(get_db_connection),
    settings: Settings = Depends(get_settings)
):
    """
    Authenticate user with email and password.
    
    Request body:
    - email: User's email address
    - password: Plain text password
    
    Returns:
    - 200: Login successful with tokens
    - 401: Invalid credentials
    - 403: Account inactive
    - 500: Server error
    """
    # Authenticate user
    profile, error = await login_user(login_data.email, login_data.password, db)
    
    if error:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"message": error, "code": "INVALID_CREDENTIALS"}
        )
    
    # Generate JWT tokens
    access_token = create_access_token(profile.user.id, profile.user.email, settings)
    refresh_token = create_refresh_token(profile.user.id, profile.user.email, settings)
    
    # Set HTTP-only cookies
    response.set_cookie(
        key="access_token",
        value=access_token,
        httponly=True,
        secure=True,
        samesite="strict",
        max_age=settings.access_token_expire_minutes * 60
    )
    
    response.set_cookie(
        key="refresh_token",
        value=refresh_token,
        httponly=True,
        secure=True,
        samesite="strict",
        max_age=settings.refresh_token_expire_days * 24 * 60 * 60
    )
    
    # Build response
    tokens = AuthTokenResponse(
        access_token=access_token,
        refresh_token=refresh_token,
        token_type="bearer",
        expires_in=settings.access_token_expire_minutes * 60
    )
    
    return AuthResponse(
        user=profile.user,
        preferences=profile.preferences,
        tokens=tokens
    )


@router.post("/logout")
async def logout(response: Response):
    """
    Logout user by clearing HTTP-only cookies.
    
    Returns:
    - 200: Logout successful
    """
    response.delete_cookie(key="access_token")
    response.delete_cookie(key="refresh_token")
    
    return {"message": "Logged out successfully"}


@router.get("/session", response_model=UserProfileResponse)
async def get_session(
    current_user: TokenData = Depends(get_current_user),
    db: asyncpg.Connection = Depends(get_db_connection)
):
    """
    Get current user session (verify token and return user data).
    
    Requires valid access token in cookie or Authorization header.
    
    Returns:
    - 200: Session valid with user profile
    - 401: Session invalid or expired
    """
    # Fetch fresh user data from database
    profile = await get_user_profile(current_user.user_id, db)
    
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"message": "User not found or inactive", "code": "USER_NOT_FOUND"}
        )
    
    return profile


@router.get("/refresh", response_model=AuthTokenResponse)
async def refresh_token(
    refresh_token: str,
    response: Response,
    settings: Settings = Depends(get_settings)
):
    """
    Refresh access token using refresh token.
    
    Request: Requires refresh_token in cookie
    
    Returns:
    - 200: New access token generated
    - 401: Refresh token invalid or expired
    """
    # Verify refresh token
    token_data = verify_token(refresh_token, settings, expected_type="refresh")
    
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"message": "Invalid or expired refresh token", "code": "INVALID_TOKEN"}
        )
    
    # Generate new access token
    new_access_token = create_access_token(token_data.user_id, token_data.email, settings)
    
    # Update access token cookie
    response.set_cookie(
        key="access_token",
        value=new_access_token,
        httponly=True,
        secure=True,
        samesite="strict",
        max_age=settings.access_token_expire_minutes * 60
    )
    
    return AuthTokenResponse(
        access_token=new_access_token,
        refresh_token=refresh_token,  # Keep same refresh token
        token_type="bearer",
        expires_in=settings.access_token_expire_minutes * 60
    )


@router.get("/user/profile", response_model=UserProfileResponse)
async def get_profile(
    current_user: TokenData = Depends(get_current_user),
    db: asyncpg.Connection = Depends(get_db_connection)
):
    """
    Get current user's profile and preferences.
    
    Requires valid access token in cookie or Authorization header.
    
    Returns:
    - 200: User profile with preferences
    - 401: Not authenticated
    - 404: User not found
    """
    profile = await get_user_profile(current_user.user_id, db)
    
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={"message": "User not found", "code": "USER_NOT_FOUND"}
        )
    
    return profile


@router.put("/user/profile", response_model=UserProfileResponse)
async def update_profile(
    update_data: ProfileUpdateRequest,
    current_user: TokenData = Depends(get_current_user),
    db: asyncpg.Connection = Depends(get_db_connection)
):
    """
    Update current user's profile and preferences.
    
    Request body (all fields optional):
    - name: User's display name
    - software_level: beginner | intermediate | advanced
    - hardware_access: cloud_only | basic | full_lab
    - preferred_language: en | ur | both
    
    Requires valid access token in cookie or Authorization header.
    
    Returns:
    - 200: Profile updated successfully
    - 400: Validation error
    - 401: Not authenticated
    - 404: User not found
    """
    profile, error = await update_user_preferences(current_user.user_id, update_data, db)
    
    if error:
        if "not found" in error:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail={"message": error, "code": "USER_NOT_FOUND"}
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail={"message": error, "code": "SERVER_ERROR"}
            )
    
    return profile

