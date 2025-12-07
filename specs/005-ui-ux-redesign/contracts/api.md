# API Contracts: Backend Integration

**Feature**: 005-ui-ux-redesign  
**Date**: 2025-12-07  
**Purpose**: Define API contracts for authentication and user profile updates

---

## Base URL

```
Development: http://localhost:8000
Production: https://your-app.railway.app
```

All endpoints prefixed with `/v1/`

---

## Authentication Endpoints

### 1. POST /v1/auth/signup

Create new user account.

**Request**:
```typescript
interface SignupRequest {
  email: string;              // Valid email, max 255 chars
  password: string;           // Min 8 chars
  name: string;               // Min 2 chars, max 100 chars
  software_level: 'beginner' | 'intermediate' | 'advanced';
  hardware_access: 'none' | 'basic' | 'full_lab';
  preferred_language: 'en' | 'ur' | 'both';
}
```

**Response (201 Created)**:
```typescript
interface SignupResponse {
  user: UserProfile;
  preferences: UserPreferences;
  tokens: TokenResponse;
}

interface UserProfile {
  id: string;
  email: string;
  name: string;
  is_active: boolean;
  is_verified: boolean;
  created_at: string;          // ISO 8601
  last_login: string | null;
}

interface UserPreferences {
  software_level: string;
  hardware_access: string;
  preferred_language: string;
  updated_at: string;          // ISO 8601
}

interface TokenResponse {
  access_token: string;        // JWT
  refresh_token: string;       // JWT
  token_type: 'bearer';
  expires_in: number;          // Seconds (1800 = 30min)
}
```

**Error Responses**:
- `400 Bad Request`: Validation errors
  ```json
  {
    "detail": [
      {
        "loc": ["body", "email"],
        "msg": "Invalid email format",
        "type": "value_error.email"
      }
    ]
  }
  ```
- `409 Conflict`: Email already exists
  ```json
  {
    "detail": "User with this email already exists"
  }
  ```

**Side Effects**:
- Sets httpOnly cookies: `access_token`, `refresh_token`
- Creates user record in database
- Creates user_preferences record

---

### 2. POST /v1/auth/login

Authenticate user.

**Request**:
```typescript
interface LoginRequest {
  email: string;
  password: string;
}
```

**Response (200 OK)**:
```typescript
interface LoginResponse {
  user: UserProfile;
  preferences: UserPreferences;
  tokens: TokenResponse;
}
```

**Error Responses**:
- `401 Unauthorized`: Invalid credentials
  ```json
  {
    "detail": "Invalid email or password"
  }
  ```
- `403 Forbidden`: Account deactivated
  ```json
  {
    "detail": "Account has been deactivated"
  }
  ```

**Side Effects**:
- Sets httpOnly cookies
- Updates `last_login` timestamp

---

### 3. POST /v1/auth/logout

Logout user.

**Request**: None (uses cookie for authentication)

**Response (200 OK)**:
```json
{
  "message": "Logged out successfully"
}
```

**Side Effects**:
- Clears httpOnly cookies
- Optionally invalidates refresh token (if token blacklist implemented)

---

### 4. GET /v1/auth/session

Get current user session.

**Request**: None (requires authentication cookie)

**Response (200 OK)**:
```typescript
interface SessionResponse {
  user: UserProfile;
  preferences: UserPreferences;
}
```

**Error Responses**:
- `401 Unauthorized`: No valid session
  ```json
  {
    "detail": "Not authenticated"
  }
  ```

---

### 5. POST /v1/auth/refresh

Refresh access token.

**Request**: None (uses refresh_token cookie)

**Response (200 OK)**:
```typescript
interface RefreshResponse {
  access_token: string;
  refresh_token: string;
  token_type: 'bearer';
  expires_in: number;
}
```

**Error Responses**:
- `401 Unauthorized`: Invalid or expired refresh token
  ```json
  {
    "detail": "Invalid refresh token"
  }
  ```

**Side Effects**:
- Sets new httpOnly cookies

---

## User Profile Endpoints

### 6. GET /v1/user/profile

Get current user profile.

**Request**: None (requires authentication)

**Response (200 OK)**:
```typescript
interface ProfileResponse {
  user: UserProfile;
  preferences: UserPreferences;
}
```

**Error Responses**:
- `401 Unauthorized`: Not authenticated

---

### 7. PUT /v1/user/profile

Update user profile.

**Request**:
```typescript
interface UpdateProfileRequest {
  name?: string;                                          // Optional updates
  software_level?: 'beginner' | 'intermediate' | 'advanced';
  hardware_access?: 'none' | 'basic' | 'full_lab';
  preferred_language?: 'en' | 'ur' | 'both';
}
```

**Response (200 OK)**:
```typescript
interface UpdateProfileResponse {
  user: UserProfile;
  preferences: UserPreferences;
}
```

**Error Responses**:
- `400 Bad Request`: Validation error
- `401 Unauthorized`: Not authenticated

**Side Effects**:
- Updates user and/or user_preferences tables
- Sets `updated_at` timestamp

---

## Chatbot Endpoints (Reference from Feature 002)

### 8. POST /v1/chat/ask

Ask chatbot a question.

**Request**:
```typescript
interface ChatRequest {
  session_id: string;         // Unique session identifier
  question_text: string;      // User's question
  context_mode: 'full' | 'selected';  // Context type
  selected_text?: string;     // If context_mode === 'selected'
}
```

**Response (200 OK)**:
```typescript
interface ChatResponse {
  answer: string;
  citations: Citation[];
  confidence_score: number;   // 0-1
  processing_time_ms: number;
  context_used: 'full' | 'selected';
  tokens_used: number;
}

interface Citation {
  text: string;               // Chapter/section reference
  url: string;                // Link to content
  relevance_score: number;    // 0-1
}
```

**Error Responses**:
- `400 Bad Request`: Invalid request format
- `429 Too Many Requests`: Rate limit exceeded
  ```json
  {
    "detail": "Rate limit exceeded. Try again in 60 seconds."
  }
  ```
- `500 Internal Server Error`: Processing error
  ```json
  {
    "detail": "Failed to process question. Please try again."
  }
  ```

---

## Error Response Format

All error responses follow this structure:

```typescript
interface ErrorResponse {
  detail: string | ValidationError[];
}

interface ValidationError {
  loc: string[];              // Location of error (e.g., ["body", "email"])
  msg: string;                // Human-readable message
  type: string;               // Error type
}
```

---

## Authentication Flow

### Client-Side Token Management

```typescript
// Tokens stored in httpOnly cookies (server-managed)
// Client doesn't need to handle tokens directly

// Authentication state managed via session check
async function checkAuth(): Promise<User | null> {
  try {
    const response = await fetch('/v1/auth/session', {
      credentials: 'include',  // Include cookies
    });
    
    if (response.ok) {
      const { user } = await response.json();
      return user;
    }
    return null;
  } catch (error) {
    return null;
  }
}

// Login flow
async function login(email: string, password: string) {
  const response = await fetch('/v1/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include',  // Include cookies
    body: JSON.stringify({ email, password }),
  });
  
  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail);
  }
  
  return await response.json();
}

// Logout flow
async function logout() {
  await fetch('/v1/auth/logout', {
    method: 'POST',
    credentials: 'include',
  });
}
```

### Token Refresh Strategy

```typescript
// Automatic token refresh on 401 responses
async function fetchWithAuth(url: string, options: RequestInit = {}) {
  let response = await fetch(url, {
    ...options,
    credentials: 'include',
  });
  
  // If unauthorized, try to refresh token
  if (response.status === 401) {
    const refreshResponse = await fetch('/v1/auth/refresh', {
      method: 'POST',
      credentials: 'include',
    });
    
    if (refreshResponse.ok) {
      // Retry original request with new token
      response = await fetch(url, {
        ...options,
        credentials: 'include',
      });
    }
  }
  
  return response;
}
```

---

## Rate Limiting

All API endpoints have rate limiting:

| Endpoint | Rate Limit | Window |
|----------|------------|--------|
| /v1/auth/signup | 5 requests | 15 minutes |
| /v1/auth/login | 10 requests | 15 minutes |
| /v1/auth/refresh | 20 requests | 15 minutes |
| /v1/chat/ask | 10 requests | 1 minute |
| /v1/user/profile (GET) | 30 requests | 1 minute |
| /v1/user/profile (PUT) | 10 requests | 1 minute |

**Rate Limit Headers**:
```
X-RateLimit-Limit: 10
X-RateLimit-Remaining: 7
X-RateLimit-Reset: 1638360000
```

**Rate Limit Exceeded Response (429)**:
```json
{
  "detail": "Rate limit exceeded. Try again in 45 seconds."
}
```

---

## CORS Configuration

**Allowed Origins** (configured in backend):
```
http://localhost:3000
https://yourusername.github.io
https://your-app.railway.app
```

**Allowed Methods**:
```
GET, POST, PUT, DELETE, OPTIONS
```

**Allowed Headers**:
```
Content-Type, Authorization
```

**Exposed Headers**:
```
X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset
```

**Credentials**: `true` (cookies allowed)

---

## Response Headers

All responses include:

```
Content-Type: application/json
X-Request-ID: uuid
```

Authentication responses also include:
```
Set-Cookie: access_token=...; HttpOnly; Secure; SameSite=Lax
Set-Cookie: refresh_token=...; HttpOnly; Secure; SameSite=Lax
```

---

## Health Check

### GET /v1/health

Check API health status.

**Request**: None

**Response (200 OK)**:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-07T10:30:00Z"
}
```

---

**API Contracts Complete**: All endpoints defined with request/response formats, error handling, and authentication flows.
