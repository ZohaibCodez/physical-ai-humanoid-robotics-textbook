"""
Authentication service - Business logic for user authentication and management.
"""

from typing import Optional, Tuple
from uuid import UUID
from datetime import datetime
import asyncpg
import traceback
import logging

from app.models.user import (
    SignupRequest,
    UserDB,
    UserPreferencesDB,
    UserResponse,
    UserPreferencesResponse,
    UserProfileResponse,
    ProfileUpdateRequest
)
from app.utils.security import get_password_hash, verify_password
from app.utils.validators import validate_email, validate_password

# Security event logger
security_logger = logging.getLogger("security")


async def signup_user(
    signup_data: SignupRequest,
    db_conn: asyncpg.Connection
) -> Tuple[Optional[UserProfileResponse], Optional[str]]:
    """
    Create a new user account with preferences.
    
    Args:
        signup_data: Signup request data
        db_conn: Database connection
        
    Returns:
        Tuple of (UserProfileResponse, error_message)
        Returns (profile, None) on success
        Returns (None, error_message) on failure
    """
    # Validate email format
    email_valid, email_error = validate_email(signup_data.email)
    if not email_valid:
        return None, email_error
    
    # Validate password
    password_valid, password_error = validate_password(signup_data.password)
    if not password_valid:
        return None, password_error
    
    try:
        # Check if email already exists
        existing_user = await db_conn.fetchrow(
            "SELECT id FROM users WHERE email = $1",
            signup_data.email
        )
        
        if existing_user:
            return None, "Account with this email already exists"
        
        # Hash password
        hashed_password = get_password_hash(signup_data.password)
        
        # Begin transaction (atomic user + preferences creation)
        async with db_conn.transaction():
            # Create user record
            user_row = await db_conn.fetchrow(
                """
                INSERT INTO users (email, hashed_password, name)
                VALUES ($1, $2, $3)
                RETURNING id, email, hashed_password, name, is_active, is_verified, 
                          created_at, updated_at, last_login
                """,
                signup_data.email,
                hashed_password,
                signup_data.name
            )
            
            # Create preferences record
            prefs_row = await db_conn.fetchrow(
                """
                INSERT INTO user_preferences (
                    user_id, software_level, hardware_access, preferred_language
                )
                VALUES ($1, $2, $3, $4)
                RETURNING user_id, software_level, hardware_access, preferred_language,
                          created_at, updated_at
                """,
                user_row['id'],
                signup_data.software_level,
                signup_data.hardware_access,
                signup_data.preferred_language
            )
        
        # Build response
        user = UserResponse(
            id=user_row['id'],
            email=user_row['email'],
            name=user_row['name'],
            is_active=user_row['is_active'],
            is_verified=user_row['is_verified'],
            created_at=user_row['created_at'],
            last_login=user_row['last_login']
        )
        
        preferences = UserPreferencesResponse(
            software_level=prefs_row['software_level'],
            hardware_access=prefs_row['hardware_access'],
            preferred_language=prefs_row['preferred_language'],
            updated_at=prefs_row['updated_at']
        )
        
        profile = UserProfileResponse(user=user, preferences=preferences)
        
        # Log security event
        security_logger.info(
            f"Account created: user_id={user_row['id']}, email={signup_data.email}",
            extra={"event": "account_created", "user_id": str(user_row['id']), "email": signup_data.email}
        )
        
        return profile, None
        
    except asyncpg.UniqueViolationError:
        return None, "Account with this email already exists"
    except asyncpg.PostgresError as e:
        # Log database error for monitoring
        print(f"Database error during signup: {e}")
        traceback.print_exc()
        return None, "Unable to create account at this time, please try again"
    except Exception as e:
        # Log unexpected error
        print(f"Unexpected error during signup: {e}")
        print(f"Error type: {type(e).__name__}")
        traceback.print_exc()
        return None, "An unexpected error occurred"


async def login_user(
    email: str,
    password: str,
    db_conn: asyncpg.Connection
) -> Tuple[Optional[UserProfileResponse], Optional[str]]:
    """
    Authenticate user with email and password.
    
    Args:
        email: User's email address
        password: Plain text password
        db_conn: Database connection
        
    Returns:
        Tuple of (UserProfileResponse, error_message)
        Returns (profile, None) on success
        Returns (None, error_message) on failure
    """
    try:
        # Fetch user with preferences (JOIN query)
        row = await db_conn.fetchrow(
            """
            SELECT 
                u.id, u.email, u.hashed_password, u.name, u.is_active, u.is_verified,
                u.created_at, u.updated_at, u.last_login,
                p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
            FROM users u
            LEFT JOIN user_preferences p ON u.id = p.user_id
            WHERE u.email = $1 AND u.is_active = TRUE
            """,
            email
        )
        
        if not row:
            # Generic error message (don't reveal if email exists)
            security_logger.warning(
                f"Failed login attempt: email={email} (user not found)",
                extra={"event": "login_failed", "email": email, "reason": "user_not_found"}
            )
            return None, "Invalid email or password"
        
        # Verify password
        if not verify_password(password, row['hashed_password']):
            security_logger.warning(
                f"Failed login attempt: email={email} (invalid password)",
                extra={"event": "login_failed", "email": email, "reason": "invalid_password"}
            )
            return None, "Invalid email or password"
        
        # Update last_login timestamp
        await db_conn.execute(
            "UPDATE users SET last_login = $1 WHERE id = $2",
            datetime.utcnow(),
            row['id']
        )
        
        # Build response
        user = UserResponse(
            id=row['id'],
            email=row['email'],
            name=row['name'],
            is_active=row['is_active'],
            is_verified=row['is_verified'],
            created_at=row['created_at'],
            last_login=datetime.utcnow()  # Use updated timestamp
        )
        
        preferences = UserPreferencesResponse(
            software_level=row['software_level'],
            hardware_access=row['hardware_access'],
            preferred_language=row['preferred_language'],
            updated_at=row['pref_updated_at']
        )
        
        profile = UserProfileResponse(user=user, preferences=preferences)
        
        # Log security event
        security_logger.info(
            f"Successful login: user_id={row['id']}, email={email}",
            extra={"event": "login_success", "user_id": str(row['id']), "email": email}
        )
        
        return profile, None
        
    except asyncpg.PostgresError as e:
        print(f"Database error during login: {e}")
        return None, "Unable to log in at this time, please try again"
    except Exception as e:
        print(f"Unexpected error during login: {e}")
        return None, "An unexpected error occurred"


async def get_user_profile(
    user_id: UUID,
    db_conn: asyncpg.Connection
) -> Optional[UserProfileResponse]:
    """
    Get user profile with preferences by user ID.
    
    Args:
        user_id: User's UUID
        db_conn: Database connection
        
    Returns:
        UserProfileResponse if found, None otherwise
    """
    try:
        row = await db_conn.fetchrow(
            """
            SELECT 
                u.id, u.email, u.name, u.is_active, u.is_verified,
                u.created_at, u.last_login,
                p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
            FROM users u
            LEFT JOIN user_preferences p ON u.id = p.user_id
            WHERE u.id = $1 AND u.is_active = TRUE
            """,
            user_id
        )
        
        if not row:
            return None
        
        user = UserResponse(
            id=row['id'],
            email=row['email'],
            name=row['name'],
            is_active=row['is_active'],
            is_verified=row['is_verified'],
            created_at=row['created_at'],
            last_login=row['last_login']
        )
        
        preferences = UserPreferencesResponse(
            software_level=row['software_level'],
            hardware_access=row['hardware_access'],
            preferred_language=row['preferred_language'],
            updated_at=row['pref_updated_at']
        )
        
        return UserProfileResponse(user=user, preferences=preferences)
        
    except Exception as e:
        print(f"Error fetching user profile: {e}")
        return None


async def update_user_preferences(
    user_id: UUID,
    update_data: ProfileUpdateRequest,
    db_conn: asyncpg.Connection
) -> Tuple[Optional[UserProfileResponse], Optional[str]]:
    """
    Update user profile and preferences.
    
    Args:
        user_id: User's UUID
        update_data: Profile update request (all fields optional)
        db_conn: Database connection
        
    Returns:
        Tuple of (UserProfileResponse, error_message)
        Returns (profile, None) on success
        Returns (None, error_message) on failure
    """
    try:
        async with db_conn.transaction():
            # Update user name if provided
            if update_data.name is not None:
                await db_conn.execute(
                    "UPDATE users SET name = $1, updated_at = $2 WHERE id = $3",
                    update_data.name,
                    datetime.utcnow(),
                    user_id
                )
            
            # Update preferences if any provided
            pref_updates = []
            pref_values = []
            pref_index = 1
            
            if update_data.software_level is not None:
                pref_updates.append(f"software_level = ${pref_index}")
                pref_values.append(update_data.software_level)
                pref_index += 1
            
            if update_data.hardware_access is not None:
                pref_updates.append(f"hardware_access = ${pref_index}")
                pref_values.append(update_data.hardware_access)
                pref_index += 1
            
            if update_data.preferred_language is not None:
                pref_updates.append(f"preferred_language = ${pref_index}")
                pref_values.append(update_data.preferred_language)
                pref_index += 1
            
            if pref_updates:
                pref_updates.append(f"updated_at = ${pref_index}")
                pref_values.append(datetime.utcnow())
                pref_index += 1
                
                pref_values.append(user_id)
                
                query = f"""
                    UPDATE user_preferences
                    SET {', '.join(pref_updates)}
                    WHERE user_id = ${pref_index}
                """
                await db_conn.execute(query, *pref_values)
        
        # Fetch updated profile
        updated_profile = await get_user_profile(user_id, db_conn)
        
        if not updated_profile:
            return None, "User not found"
        
        # Log security event
        updated_fields = []
        if update_data.name is not None:
            updated_fields.append("name")
        if update_data.software_level is not None:
            updated_fields.append("software_level")
        if update_data.hardware_access is not None:
            updated_fields.append("hardware_access")
        if update_data.preferred_language is not None:
            updated_fields.append("preferred_language")
        
        security_logger.info(
            f"Profile updated: user_id={user_id}, fields={','.join(updated_fields)}",
            extra={"event": "profile_updated", "user_id": str(user_id), "updated_fields": updated_fields}
        )
        
        return updated_profile, None
        
    except asyncpg.PostgresError as e:
        print(f"Database error during profile update: {e}")
        traceback.print_exc()
        return None, "Unable to update profile at this time"
    except Exception as e:
        print(f"Unexpected error during profile update: {e}")
        traceback.print_exc()
        return None, "An unexpected error occurred"

