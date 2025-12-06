# Research: User Authentication with Background Profiling

**Feature**: 003-user-authentication  
**Date**: 2025-12-06  
**Phase**: 0 (Research & Technology Selection)

## Purpose

Research and document technology choices, best practices, and implementation patterns for secure user authentication with background profiling. Resolve any technical uncertainties before proceeding to design phase.

---

## 1. Authentication Framework Selection

### Decision: Better-Auth Python Integration

**What was chosen**: While "Better-Auth" is specified, research reveals Better-Auth is primarily a TypeScript/JavaScript library. For Python/FastAPI backend, we will use **established Python auth patterns** with FastAPI + JWT + OAuth libraries.

**Recommended Stack**:
- **FastAPI Security utilities** (built-in `OAuth2PasswordBearer`, `Security` dependencies)
- **python-jose** for JWT token generation and validation
- **passlib** with bcrypt for password hashing
- **python-multipart** for form data handling (already in requirements.txt)
- **Optional**: `fastapi-users` or `authlib` for OAuth provider integration

**Rationale**:
- Better-Auth (TypeScript) doesn't have a stable Python port
- FastAPI's built-in security patterns are well-documented and production-ready
- python-jose is industry standard for JWT in Python (used by major projects)
- passlib with bcrypt provides secure password hashing (OWASP recommended)
- Lightweight: adds only ~5-10MB to deployment (meets free-tier constraint)
- Native async/await support (matches existing FastAPI codebase)

**Alternatives Considered**:
1. **Django REST Framework auth**: Too heavyweight, requires Django (adds 50MB+)
2. **Flask-Login**: Not async-compatible, less suitable for FastAPI
3. **Auth0/Supabase**: Third-party SaaS, adds external dependency and cost
4. **fastapi-users**: Comprehensive but opinionated, may be overkill for MVP

**Trade-offs**:
- ✅ Pros: Lightweight, FastAPI-native, well-documented, free-tier compatible
- ❌ Cons: More manual implementation vs. all-in-one library, need to handle OAuth separately if needed later

---

## 2. Session Management Strategy

### Decision: JWT Tokens with Stateless Authentication

**What was chosen**: JWT (JSON Web Tokens) stored in HTTP-only cookies with refresh token pattern.

**Implementation Pattern**:
```python
# Access Token: Short-lived (15-60 minutes)
# Refresh Token: Long-lived (7-30 days)
# Storage: HTTP-only cookies (XSS protection)
# Validation: JWT signature verification on each request
```

**Rationale**:
- Stateless authentication (no session store needed, scales horizontally)
- Works with serverless/free-tier deployments (Vercel, Railway)
- HTTP-only cookies prevent XSS attacks
- Refresh tokens allow long sessions without compromising security
- Industry standard for modern web APIs

**Alternatives Considered**:
1. **Server-side sessions (Redis/Database)**: Requires additional infrastructure, not free-tier friendly
2. **Local storage JWT**: Vulnerable to XSS attacks
3. **Session cookies (Flask/Django style)**: Doesn't scale with serverless deployments

**Best Practices**:
- Set secure, HTTP-only, SameSite=Strict flags on cookies
- Rotate refresh tokens on use
- Implement token blacklist for logout (optional, can defer to post-MVP)
- Use short access token expiry (15-60 min) with automatic refresh

---

## 3. Password Security Standards

### Decision: Passlib with Bcrypt Hashing

**What was chosen**: Passlib library with bcrypt algorithm for password hashing.

```python
from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Hashing
hashed_password = pwd_context.hash(plain_password)

# Verification
is_valid = pwd_context.verify(plain_password, hashed_password)
```

**Rationale**:
- Bcrypt is OWASP-recommended for password storage
- Built-in salt generation (prevents rainbow table attacks)
- Adaptive cost factor (can increase rounds as hardware improves)
- Passlib provides future-proof API (easy to migrate to argon2 later)
- Lightweight dependency (~2MB)

**Security Requirements** (implemented):
- Minimum password length: 8 characters (enforced in Pydantic model)
- Automatic salting (bcrypt default)
- Cost factor: 12 rounds (bcrypt default, ~0.3 seconds per hash)
- Never store plaintext passwords
- No password hints or recovery questions

**Alternatives Considered**:
1. **Argon2**: More secure but slower, better for post-MVP optimization
2. **PBKDF2**: Older standard, bcrypt preferred
3. **SHA-256/SHA-512**: NOT suitable for passwords (too fast, no salt)

---

## 4. Database Schema Design

### Decision: Two-Table Normalized Structure

**Schema**:

```sql
-- Users table (authentication data)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    name VARCHAR(255),
    is_active BOOLEAN DEFAULT TRUE,
    is_verified BOOLEAN DEFAULT FALSE,  -- For future email verification
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);

-- User preferences table (profiling data)
CREATE TABLE user_preferences (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    software_level VARCHAR(50) CHECK (software_level IN ('beginner', 'intermediate', 'advanced')),
    hardware_access VARCHAR(50) CHECK (hardware_access IN ('cloud_only', 'basic', 'full_lab')),
    preferred_language VARCHAR(10) CHECK (preferred_language IN ('en', 'ur', 'both')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Rationale**:
- Separation of concerns: auth data separate from profile data
- One-to-one relationship (enforced by PRIMARY KEY on user_id)
- CASCADE DELETE ensures preferences deleted with user account
- CHECK constraints enforce valid enum values at database level
- Indexes on email for fast lookup during login
- UUID primary keys (better for distributed systems, no enumeration risk)
- Timestamps for audit trail (created_at, updated_at, last_login)

**Future Extensions** (deferred to post-MVP):
- Email verification workflow (is_verified flag prepared)
- OAuth provider accounts (oauth_provider, oauth_id columns)
- User roles/permissions (role enum column)
- Account deletion/deactivation logs

---

## 5. Frontend State Management

### Decision: React Context API with Custom Hook

**Pattern**:

```typescript
// AuthContext provides:
// - user: User | null
// - isAuthenticated: boolean
// - isLoading: boolean
// - login(email, password): Promise<void>
// - signup(data): Promise<void>
// - logout(): Promise<void>
// - updatePreferences(prefs): Promise<void>

// Usage:
const { user, isAuthenticated, login } = useAuth();
```

**Rationale**:
- Context API is built into React (no additional library needed)
- Sufficient for MVP auth state (no complex state transitions)
- Integrates well with Docusaurus (supports custom React contexts)
- Lightweight (no Redux/MobX overhead)
- Easy to test and reason about

**Alternatives Considered**:
1. **Redux/Redux Toolkit**: Overkill for auth state, adds complexity
2. **Zustand/Jotai**: External dependency, not needed for simple use case
3. **SWR/React Query**: Good for data fetching, but auth state fits Context better

**Implementation Details**:
- Auth state persists in `localStorage` (user object, tokens if needed client-side)
- Automatic token refresh on API calls
- Protected routes use `useAuth` hook to check `isAuthenticated`
- Logout clears context + localStorage + calls backend logout endpoint

---

## 6. API Endpoint Design

### Decision: RESTful Authentication Endpoints

**Endpoints**:

```
POST   /v1/auth/signup       - Create new account with background profile
POST   /v1/auth/login        - Authenticate user, return JWT tokens
POST   /v1/auth/logout       - Invalidate session (optional blacklist)
GET    /v1/auth/session      - Verify current session, return user data
GET    /v1/auth/refresh      - Refresh access token using refresh token
GET    /v1/user/profile      - Get current user profile + preferences
PUT    /v1/user/profile      - Update user profile (name, preferences)
PUT    /v1/user/password     - Change password (requires current password)
```

**RESTful Patterns**:
- `/auth/*` for authentication operations (public)
- `/user/*` for user data operations (protected, requires authentication)
- Standard HTTP methods (POST for create/action, GET for read, PUT for update)
- Consistent JSON request/response format

**Request/Response Schemas** (Pydantic models):

```python
# Signup
class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(min_length=8)
    name: str
    software_level: Literal["beginner", "intermediate", "advanced"]
    hardware_access: Literal["cloud_only", "basic", "full_lab"]
    preferred_language: Literal["en", "ur", "both"]

class AuthResponse(BaseModel):
    user: UserResponse
    access_token: str
    refresh_token: str
    token_type: str = "bearer"

# Login
class LoginRequest(BaseModel):
    email: EmailStr
    password: str

# Profile Update
class ProfileUpdateRequest(BaseModel):
    name: Optional[str] = None
    software_level: Optional[Literal["beginner", "intermediate", "advanced"]] = None
    hardware_access: Optional[Literal["cloud_only", "basic", "full_lab"]] = None
    preferred_language: Optional[Literal["en", "ur", "both"]] = None
```

---

## 7. Error Handling & Security

### Decision: Comprehensive Error Handling with Security Best Practices

**Error Response Format**:

```json
{
  "detail": {
    "message": "User-friendly error message",
    "code": "AUTH_ERROR_CODE",
    "status": 400
  }
}
```

**Security Patterns**:

1. **Authentication Errors**: Generic messages to prevent account enumeration
   - ✅ "Invalid email or password" (don't reveal which is wrong)
   - ❌ "Email not found" or "Incorrect password"

2. **Rate Limiting**: Protect against brute force attacks
   - Max 5 failed login attempts per IP per 15 minutes
   - Max 10 signup attempts per IP per hour
   - Implement with in-memory cache or Redis (post-MVP)

3. **Input Validation**: Server-side validation always, client-side for UX
   - Pydantic models enforce types and constraints
   - Email format validation
   - Password strength requirements
   - Sanitize SQL inputs (ORM handles this)

4. **CORS Configuration**: Restrict origins
   - Only allow known frontend domains
   - Already configured in `backend/app/main.py`

5. **HTTPS Enforcement**: Required in production
   - Deployment platforms (Vercel, Railway) provide HTTPS automatically
   - Set secure cookie flags in production only

**Logging Strategy**:
- ✅ Log: Account creation, login attempts (success/fail), password changes
- ❌ Never log: Passwords, JWT tokens, sensitive user data
- Include: Request ID, timestamp, user ID (if authenticated), action, result

---

## 8. OAuth Integration (Future Ready)

### Decision: Architecture Prepared for OAuth, Deferred to Post-MVP

**Selected Libraries for Future Use**:
- **Authlib**: OAuth 1.0/2.0 client library for Python
- **Google OAuth**: `google-auth`, `google-auth-oauthlib`
- **GitHub OAuth**: `authlib` with GitHub provider

**Why Deferred**:
- OAuth adds complexity (callback routes, provider configuration, token exchange)
- Email/password authentication sufficient for MVP
- Can be added later without breaking existing auth system
- Spec marks OAuth as "optional enhancement" (FR-015)

**Preparedness**:
- Database schema includes `is_verified` flag (for OAuth-verified accounts)
- Auth service designed with provider-agnostic patterns
- Future: Add `oauth_provider`, `oauth_id` columns to users table

---

## 9. Testing Strategy

### Decision: Three-Tier Test Structure

**Test Types**:

1. **Unit Tests** (`tests/unit/test_auth_service.py`):
   - Password hashing/verification
   - JWT token generation/validation
   - User creation logic (mocked database)
   - Input validation
   - Coverage target: >80%

2. **Integration Tests** (`tests/integration/test_auth_flow.py`):
   - Full signup -> login -> profile update flow
   - Real database (test database or in-memory SQLite)
   - End-to-end scenarios from spec acceptance criteria

3. **Contract Tests** (`tests/contract/test_auth_api.py`):
   - API endpoint input/output schemas
   - HTTP status codes
   - Error response formats
   - OpenAPI spec compliance

**Testing Tools**:
- `pytest` (already in requirements-dev.txt)
- `pytest-asyncio` for async tests
- `httpx` for API testing (TestClient)
- `pytest-cov` for coverage reports

**Test Database Strategy**:
- Use `pytest` fixtures for database setup/teardown
- Option 1: Separate Neon test database (requires additional connection string)
- Option 2: SQLite in-memory for unit tests (fast, isolated)
- Integration tests use real Postgres behavior

---

## 10. Environment Configuration

### Decision: Extend Existing Settings Class

**New Configuration Variables** (add to `backend/app/config.py`):

```python
class Settings(BaseSettings):
    # ... existing config ...
    
    # Authentication
    jwt_secret_key: str = Field(..., alias="JWT_SECRET_KEY")
    jwt_algorithm: str = Field(default="HS256", alias="JWT_ALGORITHM")
    access_token_expire_minutes: int = Field(default=30, alias="ACCESS_TOKEN_EXPIRE_MINUTES")
    refresh_token_expire_days: int = Field(default=30, alias="REFRESH_TOKEN_EXPIRE_DAYS")
    
    # Password Policy
    password_min_length: int = Field(default=8, alias="PASSWORD_MIN_LENGTH")
    
    # Rate Limiting
    login_rate_limit_per_minute: int = Field(default=5, alias="LOGIN_RATE_LIMIT_PER_MINUTE")
    signup_rate_limit_per_hour: int = Field(default=10, alias="SIGNUP_RATE_LIMIT_PER_HOUR")
```

**Environment Variables** (add to `.env.example`):

```bash
# Authentication
JWT_SECRET_KEY=<generate-with-openssl-rand-hex-32>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=30

# Password Policy
PASSWORD_MIN_LENGTH=8

# Rate Limiting
LOGIN_RATE_LIMIT_PER_MINUTE=5
SIGNUP_RATE_LIMIT_PER_HOUR=10
```

**Secret Generation**:
```bash
# Generate JWT secret key (run once, add to .env)
openssl rand -hex 32
```

---

## 11. Deployment Considerations

### Decision: Zero-Downtime Migration Strategy

**Database Migration**:
1. Create migration script: `backend/scripts/create_auth_tables.sql`
2. Run migration in Neon console or via psql:
   ```bash
   psql $NEON_DATABASE_URL -f backend/scripts/create_auth_tables.sql
   ```
3. Verify tables created and indexes in place

**Deployment Steps**:
1. Merge feature branch to main (via PR)
2. CI/CD runs tests (GitHub Actions)
3. Deploy backend (Vercel/Railway picks up changes automatically)
4. Deploy frontend (Docusaurus build, GitHub Pages)
5. No downtime (new routes don't affect existing functionality)

**Rollback Plan**:
- Database: Don't drop tables (no destructive changes in migration)
- Backend: Revert to previous deployment (Vercel has instant rollback)
- Frontend: Revert Git commit, rebuild Docusaurus

**Free-Tier Compatibility**:
- ✅ Vercel: 100GB-month bandwidth, serverless functions
- ✅ Railway: 500 hours/month, 5GB storage
- ✅ Neon Postgres: 10GB storage, 100 hours compute (free tier)
- ✅ Total backend size: +5-10MB (well within 150MB budget)

---

## 12. Frontend Styling & UX

### Decision: Match Existing Docusaurus Theme

**Styling Approach**:
- Use Docusaurus's CSS variables for consistent theming
- Import `@docusaurus/theme-common` for styled components
- Create custom CSS modules for auth pages (`src/css/auth.module.css`)
- Responsive design (mobile-first, follows Docusaurus breakpoints)

**Theme Integration**:
```css
/* Use Docusaurus CSS variables */
.authContainer {
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
  border: 1px solid var(--ifm-color-emphasis-300);
}

.authButton {
  background: var(--ifm-color-primary);
  color: var(--ifm-font-color-base-inverse);
}
```

**Accessibility**:
- WCAG 2.1 Level AA compliance (Constitution Principle IV)
- Keyboard navigation support
- Screen reader friendly (ARIA labels)
- Form validation with clear error messages

---

## Summary of Research Findings

| **Topic** | **Decision** | **Status** |
|-----------|--------------|------------|
| Auth Framework | FastAPI Security + python-jose + passlib | ✅ Resolved |
| Session Management | JWT tokens with refresh pattern | ✅ Resolved |
| Password Security | Bcrypt hashing via passlib | ✅ Resolved |
| Database Schema | Two tables (users, user_preferences) | ✅ Resolved |
| Frontend State | React Context API + useAuth hook | ✅ Resolved |
| API Design | RESTful endpoints under /v1/auth and /v1/user | ✅ Resolved |
| Error Handling | Generic auth errors, comprehensive logging | ✅ Resolved |
| OAuth Support | Architecture ready, implementation deferred | ✅ Resolved |
| Testing Strategy | Three-tier (unit/integration/contract) | ✅ Resolved |
| Environment Config | Extend Settings class, add JWT config | ✅ Resolved |
| Deployment | Zero-downtime migration, free-tier compatible | ✅ Resolved |
| Frontend Styling | Match Docusaurus theme, responsive design | ✅ Resolved |

**All NEEDS CLARIFICATION items resolved. Ready for Phase 1 (Design & Contracts).**

---

## References

- [FastAPI Security Documentation](https://fastapi.tiangolo.com/tutorial/security/)
- [python-jose GitHub](https://github.com/mpdavis/python-jose)
- [Passlib Documentation](https://passlib.readthedocs.io/)
- [JWT Best Practices (RFC 8725)](https://datatracker.ietf.org/doc/html/rfc8725)
- [OWASP Password Storage Cheat Sheet](https://cheatsheetsecure.org/Cheatsheets/Password_Storage_Cheat_Sheet.html)
- [Docusaurus Theming Guide](https://docusaurus.io/docs/styling-layout)
