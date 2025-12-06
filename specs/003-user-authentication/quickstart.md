# Quickstart Guide: User Authentication Implementation

**Feature**: 003-user-authentication  
**Date**: 2025-12-06  
**Estimated Time**: 4-6 hours for MVP implementation

## Overview

This guide provides step-by-step instructions for implementing user authentication with background profiling. Follow phases in order for systematic implementation.

---

## Prerequisites

### Environment Setup
```bash
# 1. Ensure you're on the feature branch
git checkout 003-user-authentication

# 2. Backend: Create virtual environment with uv (recommended)
cd backend
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# 3. Install Python dependencies (add new auth packages)
uv pip install python-jose[cryptography] passlib[bcrypt] python-multipart

# 4. Frontend: Ensure Node.js 18+ installed
node --version  # Should be >=18.0

# 5. Verify Neon Postgres connection
echo $NEON_DATABASE_URL  # Should be set in .env
```

### Required Tools
- Python 3.10+ with uv package manager
- Node.js 18+
- PostgreSQL client (psql) or Neon console access
- Git
- Code editor (VS Code recommended)

---

## Phase 1: Database Setup (30 minutes)

### 1.1 Run Migration Script

**Create migration file**:
```bash
# File: backend/scripts/create_auth_tables.sql
# (Copy from data-model.md)
```

**Execute migration**:
```bash
# Option 1: Via psql
psql $NEON_DATABASE_URL -f backend/scripts/create_auth_tables.sql

# Option 2: Via Neon console
# Copy SQL content and paste into Neon SQL editor, then execute
```

**Verify tables created**:
```sql
-- Run in Neon console or psql
SELECT table_name FROM information_schema.tables 
WHERE table_schema = 'public' AND table_name IN ('users', 'user_preferences');

-- Expected output: users, user_preferences
```

### 1.2 Test Database Connection

```python
# Test script: backend/scripts/test_db_connection.py
import asyncio
import asyncpg
from app.config import settings

async def test_connection():
    conn = await asyncpg.connect(settings.neon_database_url)
    result = await conn.fetchval("SELECT COUNT(*) FROM users")
    print(f"✅ Connection successful. Users count: {result}")
    await conn.close()

asyncio.run(test_connection())
```

Run test:
```bash
cd backend
python scripts/test_db_connection.py
```

---

## Phase 2: Backend Implementation (2-3 hours)

### 2.1 Update Configuration (`backend/app/config.py`)

**Add auth settings**:
```python
# Add to Settings class
class Settings(BaseSettings):
    # ... existing fields ...
    
    # Authentication
    jwt_secret_key: str = Field(..., alias="JWT_SECRET_KEY")
    jwt_algorithm: str = Field(default="HS256", alias="JWT_ALGORITHM")
    access_token_expire_minutes: int = Field(default=30, alias="ACCESS_TOKEN_EXPIRE_MINUTES")
    refresh_token_expire_days: int = Field(default=30, alias="REFRESH_TOKEN_EXPIRE_DAYS")
    password_min_length: int = Field(default=8, alias="PASSWORD_MIN_LENGTH")
```

**Update `.env` file**:
```bash
# Generate secret key (run once)
openssl rand -hex 32

# Add to .env
JWT_SECRET_KEY=<generated_secret_from_above>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=30
PASSWORD_MIN_LENGTH=8
```

### 2.2 Create Pydantic Models (`backend/app/models/user.py`)

```python
# Copy all models from data-model.md
# Key models:
# - UserDB, UserPreferencesDB (database models)
# - SignupRequest, LoginRequest, ProfileUpdateRequest (request models)
# - UserResponse, UserProfileResponse, AuthResponse (response models)
# - TokenData (JWT payload)
```

### 2.3 Create Security Utilities (`backend/app/utils/security.py`)

```python
"""Security utilities for password hashing and JWT tokens."""
from datetime import datetime, timedelta
from typing import Optional
from uuid import UUID
from jose import JWTError, jwt
from passlib.context import CryptContext
from app.config import settings
from app.models.user import TokenData

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify plain password against hashed password."""
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """Hash a password using bcrypt."""
    return pwd_context.hash(password)

def create_access_token(user_id: UUID, email: str) -> str:
    """Create JWT access token."""
    expires = datetime.utcnow() + timedelta(minutes=settings.access_token_expire_minutes)
    payload = {
        "user_id": str(user_id),
        "email": email,
        "exp": expires,
        "iat": datetime.utcnow(),
        "token_type": "access"
    }
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)

def create_refresh_token(user_id: UUID, email: str) -> str:
    """Create JWT refresh token."""
    expires = datetime.utcnow() + timedelta(days=settings.refresh_token_expire_days)
    payload = {
        "user_id": str(user_id),
        "email": email,
        "exp": expires,
        "iat": datetime.utcnow(),
        "token_type": "refresh"
    }
    return jwt.encode(payload, settings.jwt_secret_key, algorithm=settings.jwt_algorithm)

def verify_token(token: str, token_type: str = "access") -> Optional[TokenData]:
    """Verify and decode JWT token."""
    try:
        payload = jwt.decode(token, settings.jwt_secret_key, algorithms=[settings.jwt_algorithm])
        if payload.get("token_type") != token_type:
            return None
        return TokenData(**payload)
    except JWTError:
        return None
```

### 2.4 Create Auth Dependencies (`backend/app/utils/dependencies.py`)

```python
"""FastAPI dependencies for authentication."""
from fastapi import Depends, HTTPException, status, Cookie
from fastapi.security import OAuth2PasswordBearer
from typing import Optional
from uuid import UUID
import asyncpg
from app.config import settings
from app.utils.security import verify_token
from app.models.user import UserDB, UserPreferencesDB, UserProfileResponse

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/v1/auth/login", auto_error=False)

async def get_current_user(
    access_token: Optional[str] = Cookie(None),
    authorization: Optional[str] = Depends(oauth2_scheme)
) -> UserProfileResponse:
    """Get current authenticated user from JWT token (cookie or header)."""
    # Try cookie first, then Authorization header
    token = access_token or authorization
    
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"message": "Authentication required", "code": "MISSING_TOKEN"}
        )
    
    # Verify token
    token_data = verify_token(token, token_type="access")
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"message": "Invalid or expired token", "code": "INVALID_TOKEN"}
        )
    
    # Fetch user from database
    conn = await asyncpg.connect(settings.neon_database_url)
    try:
        user_row = await conn.fetchrow("""
            SELECT u.*, p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
            FROM users u
            LEFT JOIN user_preferences p ON u.id = p.user_id
            WHERE u.id = $1 AND u.is_active = TRUE
        """, token_data.user_id)
        
        if not user_row:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"message": "User not found or inactive", "code": "USER_NOT_FOUND"}
            )
        
        # Parse into response model
        user = UserDB(**dict(user_row))
        preferences = UserPreferencesDB(
            user_id=user.id,
            software_level=user_row["software_level"],
            hardware_access=user_row["hardware_access"],
            preferred_language=user_row["preferred_language"],
            created_at=user.created_at,
            updated_at=user_row["pref_updated_at"]
        )
        
        return UserProfileResponse(user=user, preferences=preferences)
    finally:
        await conn.close()

async def get_current_user_optional(
    access_token: Optional[str] = Cookie(None),
    authorization: Optional[str] = Depends(oauth2_scheme)
) -> Optional[UserProfileResponse]:
    """Optional authentication (returns None if not authenticated)."""
    try:
        return await get_current_user(access_token, authorization)
    except HTTPException:
        return None
```

### 2.5 Create Auth Service (`backend/app/services/auth_service.py`)

```python
"""Authentication service logic."""
import asyncpg
from typing import Tuple
from uuid import UUID
from app.config import settings
from app.models.user import SignupRequest, UserDB, UserPreferencesDB, UserProfileResponse
from app.utils.security import get_password_hash, verify_password, create_access_token, create_refresh_token
from fastapi import HTTPException, status

async def signup_user(signup_data: SignupRequest) -> Tuple[UserProfileResponse, dict]:
    """Create new user account with preferences. Returns (user_profile, tokens)."""
    conn = await asyncpg.connect(settings.neon_database_url)
    
    try:
        # Start transaction
        async with conn.transaction():
            # Check if email already exists
            existing = await conn.fetchval("SELECT id FROM users WHERE email = $1", signup_data.email)
            if existing:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail={"message": "Account with this email already exists", "code": "EMAIL_EXISTS"}
                )
            
            # Hash password
            hashed_password = get_password_hash(signup_data.password)
            
            # Insert user
            user_id = await conn.fetchval("""
                INSERT INTO users (email, hashed_password, name)
                VALUES ($1, $2, $3)
                RETURNING id
            """, signup_data.email, hashed_password, signup_data.name)
            
            # Insert preferences
            await conn.execute("""
                INSERT INTO user_preferences (user_id, software_level, hardware_access, preferred_language)
                VALUES ($1, $2, $3, $4)
            """, user_id, signup_data.software_level, signup_data.hardware_access, signup_data.preferred_language)
            
            # Fetch created user
            user_row = await conn.fetchrow("""
                SELECT u.*, p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
                FROM users u
                JOIN user_preferences p ON u.id = p.user_id
                WHERE u.id = $1
            """, user_id)
        
        # Create tokens
        access_token = create_access_token(user_id, signup_data.email)
        refresh_token = create_refresh_token(user_id, signup_data.email)
        
        # Parse response
        user = UserDB(**dict(user_row))
        preferences = UserPreferencesDB(
            user_id=user.id,
            software_level=user_row["software_level"],
            hardware_access=user_row["hardware_access"],
            preferred_language=user_row["preferred_language"],
            created_at=user.created_at,
            updated_at=user_row["pref_updated_at"]
        )
        
        tokens = {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer",
            "expires_in": settings.access_token_expire_minutes * 60
        }
        
        return UserProfileResponse(user=user, preferences=preferences), tokens
    
    finally:
        await conn.close()

async def login_user(email: str, password: str) -> Tuple[UserProfileResponse, dict]:
    """Authenticate user. Returns (user_profile, tokens)."""
    conn = await asyncpg.connect(settings.neon_database_url)
    
    try:
        # Fetch user with hashed password
        user_row = await conn.fetchrow("""
            SELECT u.*, p.software_level, p.hardware_access, p.preferred_language, p.updated_at AS pref_updated_at
            FROM users u
            LEFT JOIN user_preferences p ON u.id = p.user_id
            WHERE u.email = $1
        """, email)
        
        if not user_row:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"message": "Invalid email or password", "code": "INVALID_CREDENTIALS"}
            )
        
        # Verify password
        if not verify_password(password, user_row["hashed_password"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"message": "Invalid email or password", "code": "INVALID_CREDENTIALS"}
            )
        
        # Check if account active
        if not user_row["is_active"]:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail={"message": "Account has been deactivated", "code": "ACCOUNT_DEACTIVATED"}
            )
        
        # Update last_login timestamp
        await conn.execute("UPDATE users SET last_login = CURRENT_TIMESTAMP WHERE id = $1", user_row["id"])
        
        # Create tokens
        access_token = create_access_token(user_row["id"], email)
        refresh_token = create_refresh_token(user_row["id"], email)
        
        # Parse response
        user = UserDB(**dict(user_row))
        preferences = UserPreferencesDB(
            user_id=user.id,
            software_level=user_row["software_level"],
            hardware_access=user_row["hardware_access"],
            preferred_language=user_row["preferred_language"],
            created_at=user.created_at,
            updated_at=user_row["pref_updated_at"]
        )
        
        tokens = {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer",
            "expires_in": settings.access_token_expire_minutes * 60
        }
        
        return UserProfileResponse(user=user, preferences=preferences), tokens
    
    finally:
        await conn.close()
```

### 2.6 Create Auth Routes (`backend/app/api/routes/auth.py`)

```python
"""Authentication API routes."""
from fastapi import APIRouter, HTTPException, Response, Cookie, Depends, status
from app.models.user import SignupRequest, LoginRequest, AuthResponse
from app.services.auth_service import signup_user, login_user
from app.utils.dependencies import get_current_user
from app.config import settings

router = APIRouter(prefix="/auth", tags=["authentication"])

@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(signup_data: SignupRequest, response: Response):
    """Create new user account with background profiling."""
    user_profile, tokens = await signup_user(signup_data)
    
    # Set HTTP-only cookies
    response.set_cookie(
        key="access_token",
        value=tokens["access_token"],
        httponly=True,
        secure=settings.environment == "production",
        samesite="strict",
        max_age=settings.access_token_expire_minutes * 60
    )
    response.set_cookie(
        key="refresh_token",
        value=tokens["refresh_token"],
        httponly=True,
        secure=settings.environment == "production",
        samesite="strict",
        max_age=settings.refresh_token_expire_days * 24 * 60 * 60
    )
    
    return AuthResponse(
        user=user_profile.user,
        preferences=user_profile.preferences,
        tokens=tokens
    )

@router.post("/login", response_model=AuthResponse)
async def login(login_data: LoginRequest, response: Response):
    """Authenticate user and return JWT tokens."""
    user_profile, tokens = await login_user(login_data.email, login_data.password)
    
    # Set HTTP-only cookies
    response.set_cookie(
        key="access_token",
        value=tokens["access_token"],
        httponly=True,
        secure=settings.environment == "production",
        samesite="strict",
        max_age=settings.access_token_expire_minutes * 60
    )
    response.set_cookie(
        key="refresh_token",
        value=tokens["refresh_token"],
        httponly=True,
        secure=settings.environment == "production",
        samesite="strict",
        max_age=settings.refresh_token_expire_days * 24 * 60 * 60
    )
    
    return AuthResponse(
        user=user_profile.user,
        preferences=user_profile.preferences,
        tokens=tokens
    )

@router.post("/logout")
async def logout(response: Response):
    """Logout user by clearing cookies."""
    response.delete_cookie("access_token")
    response.delete_cookie("refresh_token")
    return {"message": "Logged out successfully"}

@router.get("/session")
async def get_session(current_user=Depends(get_current_user)):
    """Get current session user data."""
    return {
        "user": current_user.user,
        "preferences": current_user.preferences,
        "authenticated": True
    }
```

### 2.7 Register Auth Router (`backend/app/main.py`)

```python
# Add import
from app.api.routes import chat, health, auth  # Add auth

# Register router
app.include_router(auth.router, prefix="/v1", tags=["auth"])
```

### 2.8 Update Requirements (`backend/requirements.txt`)

```txt
# Add after existing packages
python-jose[cryptography]==3.3.0   # JWT tokens
passlib[bcrypt]==1.7.4             # Password hashing
python-multipart>=0.0.9            # Already present (form data)
```

Install new dependencies:
```bash
cd backend
uv pip install python-jose[cryptography] passlib[bcrypt]
```

---

## Phase 3: Frontend Implementation (1.5-2 hours)

### 3.1 Create Auth Context (`src/components/auth/AuthProvider.tsx`)

```typescript
// Simplified implementation - see contracts/api-contracts.md for full details
import React, { createContext, useContext, useState, useEffect } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (data: SignupData) => Promise<void>;
  logout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check session on mount
    checkSession();
  }, []);

  const checkSession = async () => {
    try {
      const response = await fetch('/v1/auth/session', { credentials: 'include' });
      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
      }
    } catch (error) {
      console.error('Session check failed:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const login = async (email: string, password: string) => {
    const response = await fetch('/v1/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
      credentials: 'include'
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail.message);
    }

    const data = await response.json();
    setUser(data.user);
  };

  const signup = async (signupData: SignupData) => {
    const response = await fetch('/v1/auth/signup', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(signupData),
      credentials: 'include'
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail.message);
    }

    const data = await response.json();
    setUser(data.user);
  };

  const logout = async () => {
    await fetch('/v1/auth/logout', { method: 'POST', credentials: 'include' });
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, isAuthenticated: !!user, isLoading, login, signup, logout }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};
```

### 3.2 Create Login Form (`src/pages/login.tsx`)

```typescript
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/auth/AuthProvider';

export default function Login(): JSX.Element {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    try {
      await login(email, password);
      window.location.href = '/';  // Redirect to home
    } catch (err) {
      setError(err.message);
    }
  };

  return (
    <Layout title="Login">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Login</h1>
            <form onSubmit={handleSubmit}>
              {error && <div className="alert alert--danger">{error}</div>}
              
              <div className="margin-bottom--md">
                <label htmlFor="email">Email</label>
                <input
                  type="email"
                  id="email"
                  className="form-control"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>

              <div className="margin-bottom--md">
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  className="form-control"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>

              <button type="submit" className="button button--primary button--block">
                Login
              </button>
            </form>

            <p className="margin-top--md">
              Don't have an account? <a href="/signup">Sign up</a>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
```

### 3.3 Create Signup Form (`src/pages/signup.tsx`)

```typescript
// Similar structure to login.tsx
// Add fields for name, software_level, hardware_access, preferred_language
// Use select/radio inputs for enum fields
// Call useAuth().signup(data)
```

### 3.4 Wrap Root with AuthProvider (`docusaurus.config.js`)

```javascript
// Add to themeConfig or use custom Root component
// Import and wrap app with <AuthProvider>
```

---

## Phase 4: Testing (1 hour)

### 4.1 Manual Testing Checklist

**Signup Flow**:
- [ ] Open `/signup` page
- [ ] Fill form with valid data
- [ ] Submit and verify redirect to home
- [ ] Check cookies in browser DevTools (access_token, refresh_token)
- [ ] Verify user created in database

**Login Flow**:
- [ ] Open `/login` page
- [ ] Enter credentials
- [ ] Submit and verify authentication
- [ ] Check session persists after page refresh

**Profile Page**:
- [ ] Navigate to `/profile` while authenticated
- [ ] Update preferences
- [ ] Verify changes saved

**Guest Access**:
- [ ] Open textbook content pages without login
- [ ] Verify content readable
- [ ] Verify no forced redirects

### 4.2 Automated Testing

**Run pytest tests**:
```bash
cd backend
pytest tests/contract/test_auth_api.py -v
pytest tests/integration/test_auth_flow.py -v
pytest tests/unit/test_auth_service.py -v
```

**Coverage report**:
```bash
pytest --cov=app --cov-report=html
open htmlcov/index.html
```

---

## Phase 5: Deployment (30 minutes)

### 5.1 Update Environment Variables

**Vercel/Railway**:
- Add `JWT_SECRET_KEY` (generate with `openssl rand -hex 32`)
- Add other auth config vars from `.env.example`

### 5.2 Deploy Backend

```bash
# Push to main branch (triggers auto-deploy)
git add .
git commit -m "feat: implement user authentication with background profiling"
git push origin 003-user-authentication

# Create PR and merge to main
```

### 5.3 Deploy Frontend

```bash
# Docusaurus build
npm run build

# Deploy to GitHub Pages (if configured)
npm run deploy
```

---

## Troubleshooting

**Issue**: JWT token verification fails  
**Solution**: Ensure `JWT_SECRET_KEY` is same in all environments. Check token expiry time.

**Issue**: Database connection fails  
**Solution**: Verify `NEON_DATABASE_URL` is set correctly. Check Neon console for connection limits.

**Issue**: CORS errors in frontend  
**Solution**: Add frontend URL to `CORS_ORIGINS` in backend config.

**Issue**: Cookies not set  
**Solution**: Check `secure` flag (should be `False` in development, `True` in production). Ensure `credentials: 'include'` in frontend fetch calls.

---

## Next Steps

After MVP implementation:
1. Add email verification workflow
2. Implement password reset functionality
3. Add OAuth providers (Google, GitHub)
4. Implement rate limiting with Redis
5. Add JWT token blacklist for logout
6. Enhance password requirements (complexity rules)
7. Add user activity logging for analytics

---

## Resources

- [FastAPI Security Tutorial](https://fastapi.tiangolo.com/tutorial/security/)
- [python-jose Documentation](https://python-jose.readthedocs.io/)
- [Passlib Documentation](https://passlib.readthedocs.io/)
- [JWT Best Practices](https://datatracker.ietf.org/doc/html/rfc8725)
- [React Context API](https://react.dev/reference/react/useContext)

**Estimated Total Time**: 4-6 hours for complete MVP implementation.
