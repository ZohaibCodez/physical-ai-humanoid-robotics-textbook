# Data Model: User Authentication with Background Profiling

**Feature**: 003-user-authentication  
**Date**: 2025-12-06  
**Phase**: 1 (Design & Contracts)

## Overview

This document defines the data models, database schema, and entity relationships for the user authentication and background profiling feature. The design follows a normalized two-table structure separating authentication data from user preferences.

---

## Entity-Relationship Diagram

```
┌─────────────────────────┐
│       users             │
├─────────────────────────┤
│ id (UUID, PK)           │
│ email (VARCHAR, UNIQUE) │
│ hashed_password         │
│ name                    │
│ is_active               │
│ is_verified             │
│ created_at              │
│ updated_at              │
│ last_login              │
└───────────┬─────────────┘
            │ 1
            │
            │ 1
┌───────────┴─────────────┐
│   user_preferences      │
├─────────────────────────┤
│ user_id (UUID, PK, FK)  │
│ software_level          │
│ hardware_access         │
│ preferred_language      │
│ created_at              │
│ updated_at              │
└─────────────────────────┘
```

**Relationship**: One-to-One (each user has exactly one preferences record)

---

## Entity Definitions

### 1. User Entity

**Purpose**: Stores core authentication data and account metadata.

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique user identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL | User's email address (login identifier) |
| `hashed_password` | VARCHAR(255) | NOT NULL | Bcrypt hashed password (never store plaintext) |
| `name` | VARCHAR(255) | NULL | User's display name |
| `is_active` | BOOLEAN | DEFAULT TRUE | Account active status (soft delete flag) |
| `is_verified` | BOOLEAN | DEFAULT FALSE | Email verification status (future use) |
| `created_at` | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | Account creation timestamp |
| `updated_at` | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | Last profile update timestamp |
| `last_login` | TIMESTAMP | NULL | Most recent successful login timestamp |

**Indexes**:
- `idx_users_email` on `email` (fast login lookups)
- Primary key index on `id` (automatic)

**Business Rules**:
1. Email MUST be unique (enforced by database UNIQUE constraint)
2. Password MUST be hashed before storage (application layer responsibility)
3. `is_active=FALSE` prevents login (soft delete, preserves data)
4. `updated_at` timestamp updated on any profile change
5. `last_login` timestamp updated on successful authentication

**Validation Rules** (Pydantic model):
```python
class User(BaseModel):
    id: UUID
    email: EmailStr  # Validates email format
    name: str = Field(max_length=255)
    is_active: bool = True
    is_verified: bool = False
    created_at: datetime
    updated_at: datetime
    last_login: Optional[datetime] = None
```

---

### 2. UserPreferences Entity

**Purpose**: Stores user background profiling data for content personalization.

**Attributes**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `user_id` | UUID | PRIMARY KEY, FOREIGN KEY → users(id) | References parent user record |
| `software_level` | VARCHAR(50) | CHECK IN ('beginner', 'intermediate', 'advanced') | Programming experience level |
| `hardware_access` | VARCHAR(50) | CHECK IN ('cloud_only', 'basic', 'full_lab') | Available hardware resources |
| `preferred_language` | VARCHAR(10) | CHECK IN ('en', 'ur', 'both') | Content language preference |
| `created_at` | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | Preferences created timestamp |
| `updated_at` | TIMESTAMP | DEFAULT CURRENT_TIMESTAMP | Last preferences update timestamp |

**Foreign Key Constraint**:
- `user_id` REFERENCES `users(id)` ON DELETE CASCADE
- Cascade delete ensures preferences removed when user account deleted

**CHECK Constraints**:
- Enforce valid enum values at database level (prevents invalid data)

**Business Rules**:
1. One-to-One relationship with User (enforced by PRIMARY KEY on `user_id`)
2. All three preference fields MUST be set during signup
3. Preferences can be updated independently after account creation
4. `updated_at` timestamp updated on any preference change

**Validation Rules** (Pydantic model):
```python
class UserPreferences(BaseModel):
    user_id: UUID
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]
    created_at: datetime
    updated_at: datetime
```

---

## Database Schema (SQL)

### Migration Script: `backend/scripts/create_auth_tables.sql`

```sql
-- ===========================================
-- User Authentication Tables Migration
-- Feature: 003-user-authentication
-- Date: 2025-12-06
-- ===========================================

-- Enable UUID extension if not already enabled
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Users table (authentication data)
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    name VARCHAR(255),
    is_active BOOLEAN DEFAULT TRUE,
    is_verified BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP
);

-- Create index on email for fast lookups
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Trigger to automatically update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER users_updated_at_trigger
    BEFORE UPDATE ON users
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- User preferences table (profiling data)
CREATE TABLE IF NOT EXISTS user_preferences (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    software_level VARCHAR(50) NOT NULL 
        CHECK (software_level IN ('beginner', 'intermediate', 'advanced')),
    hardware_access VARCHAR(50) NOT NULL 
        CHECK (hardware_access IN ('cloud_only', 'basic', 'full_lab')),
    preferred_language VARCHAR(10) NOT NULL 
        CHECK (preferred_language IN ('en', 'ur', 'both')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Trigger to automatically update updated_at timestamp
CREATE TRIGGER user_preferences_updated_at_trigger
    BEFORE UPDATE ON user_preferences
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Grant permissions (if needed for application user)
-- GRANT SELECT, INSERT, UPDATE, DELETE ON users TO your_app_user;
-- GRANT SELECT, INSERT, UPDATE, DELETE ON user_preferences TO your_app_user;

-- ===========================================
-- Verification Queries
-- ===========================================
-- Run these to verify migration:

-- Check tables created
-- SELECT table_name FROM information_schema.tables WHERE table_schema = 'public' AND table_name IN ('users', 'user_preferences');

-- Check constraints
-- SELECT constraint_name, table_name FROM information_schema.table_constraints WHERE table_name IN ('users', 'user_preferences');

-- Check indexes
-- SELECT indexname FROM pg_indexes WHERE tablename IN ('users', 'user_preferences');
```

---

## Pydantic Models (Application Layer)

### Base Models (`backend/app/models/user.py`)

```python
"""
User and UserPreferences Pydantic models for request/response validation.
"""

from datetime import datetime
from typing import Optional, Literal
from uuid import UUID
from pydantic import BaseModel, EmailStr, Field


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
    password: str = Field(min_length=8, description="Minimum 8 characters")
    name: str = Field(max_length=255)
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]


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
    new_password: str = Field(min_length=8, description="Minimum 8 characters")


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
```

---

## Data Access Patterns

### Common Queries

**1. User Signup (Create user + preferences atomically)**:
```sql
BEGIN;

INSERT INTO users (email, hashed_password, name)
VALUES ($1, $2, $3)
RETURNING id;

INSERT INTO user_preferences (user_id, software_level, hardware_access, preferred_language)
VALUES ($4, $5, $6, $7);

COMMIT;
```

**2. User Login (Fetch user by email)**:
```sql
SELECT id, email, hashed_password, is_active, is_verified
FROM users
WHERE email = $1 AND is_active = TRUE;
```

**3. Get User Profile (User + Preferences join)**:
```sql
SELECT 
    u.id, u.email, u.name, u.is_active, u.is_verified, u.created_at, u.last_login,
    p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
FROM users u
LEFT JOIN user_preferences p ON u.id = p.user_id
WHERE u.id = $1 AND u.is_active = TRUE;
```

**4. Update Preferences**:
```sql
UPDATE user_preferences
SET 
    software_level = COALESCE($2, software_level),
    hardware_access = COALESCE($3, hardware_access),
    preferred_language = COALESCE($4, preferred_language)
WHERE user_id = $1
RETURNING *;
```

**5. Update Last Login Timestamp**:
```sql
UPDATE users
SET last_login = CURRENT_TIMESTAMP
WHERE id = $1;
```

---

## State Transitions

### User Account State Machine

```
┌─────────┐
│ Initial │
└────┬────┘
     │
     │ signup()
     ▼
┌─────────────┐
│   Active    │ ◄─────────────┐
│ is_active=T │               │
└──┬──────┬───┘               │
   │      │                   │
   │      │ deactivate()      │ reactivate()
   │      ▼                   │
   │  ┌──────────────┐        │
   │  │  Deactivated │────────┘
   │  │ is_active=F  │
   │  └──────────────┘
   │
   │ verify_email() (future)
   ▼
┌─────────────┐
│   Verified  │
│ is_verified=T│
└─────────────┘
```

**State Definitions**:
- **Active**: User can log in, access all features (default after signup)
- **Deactivated**: User cannot log in, preferences preserved (soft delete)
- **Verified**: Email verified, may unlock additional features (future)

**State Transitions**:
- `signup()`: Initial → Active (is_active=TRUE, is_verified=FALSE)
- `deactivate()`: Active → Deactivated (is_active=FALSE)
- `reactivate()`: Deactivated → Active (is_active=TRUE)
- `verify_email()`: Active → Verified (is_verified=TRUE) [future feature]

---

## Data Integrity Rules

### Database-Level Constraints
1. **Email Uniqueness**: UNIQUE constraint on `users.email` (prevents duplicate accounts)
2. **Foreign Key Integrity**: CASCADE DELETE on `user_preferences.user_id` (orphan prevention)
3. **CHECK Constraints**: Enum value validation on preference fields
4. **NOT NULL**: Required fields enforced at database level

### Application-Level Validation
1. **Email Format**: Pydantic `EmailStr` validates format before database insert
2. **Password Strength**: Minimum 8 characters (enforced in Pydantic model, can add complexity rules)
3. **Name Length**: Max 255 characters (prevents overflow)
4. **Atomic Operations**: Signup transaction ensures user + preferences created together (rollback on failure)

### Security Constraints
1. **Never Return hashed_password**: Pydantic response models exclude password field
2. **Soft Delete Only**: Set `is_active=FALSE` instead of DELETE (preserves audit trail)
3. **Last Login Tracking**: Update timestamp asynchronously (doesn't block auth response)

---

## Performance Considerations

### Indexing Strategy
- **Primary Indexes**: Automatic on primary keys (`users.id`, `user_preferences.user_id`)
- **Email Lookup Index**: `idx_users_email` for fast login queries (most common query)
- **No Additional Indexes Needed**: Small table size (<100K users expected), limited query patterns

### Query Optimization
- **JOIN for Profile**: Single query instead of two separate calls (reduces latency)
- **Async Database Ops**: Use `asyncpg` for non-blocking I/O (FastAPI native)
- **Connection Pooling**: Neon Postgres handles pooling (no application-side needed)

### Scalability Notes
- **Horizontal Scaling**: Stateless auth (JWT) allows multiple backend instances
- **Database Scaling**: Neon Postgres auto-scales compute (free tier: 10GB storage, 100 hours/month)
- **Cache Strategy (Future)**: Redis for JWT blacklist, rate limiting (not needed for MVP)

---

## Privacy & Compliance

### Data Classification
- **Sensitive Data**: Email, hashed_password (PII, must be protected)
- **Non-Sensitive Data**: name, preferences (PII but low risk)
- **Audit Data**: Timestamps (logs, analytics)

### GDPR Considerations (Future)
- **Right to Access**: Provide user profile endpoint (GET /v1/user/profile)
- **Right to Deletion**: Implement soft delete (is_active=FALSE) or hard delete on request
- **Data Minimization**: Only collect necessary fields (no unnecessary tracking)
- **Consent**: Clearly communicate data usage during signup (future: terms of service checkbox)

### Data Retention
- **Active Users**: Retain indefinitely (educational platform, no automatic deletion)
- **Inactive Users**: Retain preferences for reactivation (no automatic purge)
- **Deleted Users**: Hard delete option available (requires manual process for MVP)

---

## Summary

| **Aspect** | **Details** |
|------------|-------------|
| **Tables** | 2 (users, user_preferences) |
| **Relationship** | One-to-One (cascade delete) |
| **Primary Keys** | UUID (gen_random_uuid()) |
| **Indexes** | 2 (email lookup, auto primary keys) |
| **Constraints** | UNIQUE (email), CHECK (enum values), NOT NULL (required fields) |
| **Triggers** | 2 (auto-update updated_at timestamps) |
| **Pydantic Models** | 13 (request, response, database models) |
| **State Machine** | 3 states (Active, Deactivated, Verified) |
| **Data Integrity** | Database + application layer validation |
| **Performance** | Indexed email lookups, async queries, single-join profile fetch |
| **Privacy** | Soft delete, no password exposure, GDPR-ready architecture |

**Status**: ✅ Data model complete. Ready for API contract definition (contracts/).
