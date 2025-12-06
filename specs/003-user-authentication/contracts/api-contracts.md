# API Contracts: User Authentication

**Feature**: 003-user-authentication  
**Date**: 2025-12-06  
**API Version**: v1  
**Base URL**: `/v1`

## Overview

RESTful API contracts for user authentication and profile management. All endpoints use JSON request/response format. Authentication uses JWT tokens in HTTP-only cookies or Authorization header.

---

## Authentication Endpoints

### 1. POST /v1/auth/signup

**Description**: Create new user account with background profiling.

**Request**:
```http
POST /v1/auth/signup HTTP/1.1
Content-Type: application/json

{
  "email": "student@example.com",
  "password": "securepassword123",
  "name": "Jane Doe",
  "software_level": "intermediate",
  "hardware_access": "basic",
  "preferred_language": "en"
}
```

**Response (201 Created)**:
```http
HTTP/1.1 201 Created
Content-Type: application/json
Set-Cookie: access_token=<JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=1800
Set-Cookie: refresh_token=<JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=2592000

{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "name": "Jane Doe",
    "is_active": true,
    "is_verified": false,
    "created_at": "2025-12-06T10:30:00Z",
    "last_login": null
  },
  "preferences": {
    "software_level": "intermediate",
    "hardware_access": "basic",
    "preferred_language": "en",
    "updated_at": "2025-12-06T10:30:00Z"
  },
  "tokens": {
    "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "token_type": "bearer",
    "expires_in": 1800
  }
}
```

**Error Responses**:

*400 Bad Request - Validation Error*:
```json
{
  "detail": {
    "message": "Validation error",
    "code": "VALIDATION_ERROR",
    "errors": [
      {
        "field": "password",
        "message": "Password must be at least 8 characters"
      }
    ]
  }
}
```

*409 Conflict - Email Already Exists*:
```json
{
  "detail": {
    "message": "Account with this email already exists",
    "code": "EMAIL_EXISTS"
  }
}
```

*500 Internal Server Error - Database Error*:
```json
{
  "detail": {
    "message": "Unable to create account at this time, please try again",
    "code": "SERVER_ERROR"
  }
}
```

**Field Constraints**:
- `email`: Valid email format, max 255 characters
- `password`: Min 8 characters, max 255 characters
- `name`: Max 255 characters
- `software_level`: Enum ("beginner", "intermediate", "advanced")
- `hardware_access`: Enum ("cloud_only", "basic", "full_lab")
- `preferred_language`: Enum ("en", "ur", "both")

---

### 2. POST /v1/auth/login

**Description**: Authenticate existing user, return JWT tokens.

**Request**:
```http
POST /v1/auth/login HTTP/1.1
Content-Type: application/json

{
  "email": "student@example.com",
  "password": "securepassword123"
}
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json
Set-Cookie: access_token=<JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=1800
Set-Cookie: refresh_token=<JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=2592000

{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "name": "Jane Doe",
    "is_active": true,
    "is_verified": false,
    "created_at": "2025-12-06T10:30:00Z",
    "last_login": "2025-12-06T15:45:00Z"
  },
  "preferences": {
    "software_level": "intermediate",
    "hardware_access": "basic",
    "preferred_language": "en",
    "updated_at": "2025-12-06T10:30:00Z"
  },
  "tokens": {
    "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "token_type": "bearer",
    "expires_in": 1800
  }
}
```

**Error Responses**:

*401 Unauthorized - Invalid Credentials*:
```json
{
  "detail": {
    "message": "Invalid email or password",
    "code": "INVALID_CREDENTIALS"
  }
}
```
*Note: Generic error message prevents account enumeration (don't reveal if email exists).*

*403 Forbidden - Account Deactivated*:
```json
{
  "detail": {
    "message": "Account has been deactivated",
    "code": "ACCOUNT_DEACTIVATED"
  }
}
```

---

### 3. POST /v1/auth/logout

**Description**: Invalidate current session (clear cookies, optionally blacklist token).

**Request**:
```http
POST /v1/auth/logout HTTP/1.1
Authorization: Bearer <access_token>
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json
Set-Cookie: access_token=; HttpOnly; Secure; SameSite=Strict; Max-Age=0
Set-Cookie: refresh_token=; HttpOnly; Secure; SameSite=Strict; Max-Age=0

{
  "message": "Logged out successfully"
}
```

**Error Responses**:

*401 Unauthorized - No Token*:
```json
{
  "detail": {
    "message": "Authentication required",
    "code": "MISSING_TOKEN"
  }
}
```

---

### 4. GET /v1/auth/session

**Description**: Verify current session, return user data if authenticated.

**Request**:
```http
GET /v1/auth/session HTTP/1.1
Authorization: Bearer <access_token>
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "name": "Jane Doe",
    "is_active": true,
    "is_verified": false,
    "created_at": "2025-12-06T10:30:00Z",
    "last_login": "2025-12-06T15:45:00Z"
  },
  "preferences": {
    "software_level": "intermediate",
    "hardware_access": "basic",
    "preferred_language": "en",
    "updated_at": "2025-12-06T10:30:00Z"
  },
  "authenticated": true
}
```

**Error Responses**:

*401 Unauthorized - Invalid/Expired Token*:
```json
{
  "detail": {
    "message": "Session expired, please log in again",
    "code": "TOKEN_EXPIRED"
  }
}
```

---

### 5. GET /v1/auth/refresh

**Description**: Refresh access token using refresh token.

**Request**:
```http
GET /v1/auth/refresh HTTP/1.1
Cookie: refresh_token=<refresh_JWT>
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json
Set-Cookie: access_token=<new_JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=1800
Set-Cookie: refresh_token=<new_JWT>; HttpOnly; Secure; SameSite=Strict; Max-Age=2592000

{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "expires_in": 1800
}
```

**Error Responses**:

*401 Unauthorized - Invalid Refresh Token*:
```json
{
  "detail": {
    "message": "Invalid refresh token, please log in again",
    "code": "INVALID_REFRESH_TOKEN"
  }
}
```

---

## User Profile Endpoints

### 6. GET /v1/user/profile

**Description**: Get current user's profile and preferences.

**Authentication**: Required (JWT token)

**Request**:
```http
GET /v1/user/profile HTTP/1.1
Authorization: Bearer <access_token>
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "name": "Jane Doe",
    "is_active": true,
    "is_verified": false,
    "created_at": "2025-12-06T10:30:00Z",
    "last_login": "2025-12-06T15:45:00Z"
  },
  "preferences": {
    "software_level": "intermediate",
    "hardware_access": "basic",
    "preferred_language": "en",
    "updated_at": "2025-12-06T10:30:00Z"
  }
}
```

**Error Responses**:

*401 Unauthorized*:
```json
{
  "detail": {
    "message": "Authentication required",
    "code": "MISSING_TOKEN"
  }
}
```

---

### 7. PUT /v1/user/profile

**Description**: Update user profile and/or preferences (partial update supported).

**Authentication**: Required (JWT token)

**Request**:
```http
PUT /v1/user/profile HTTP/1.1
Authorization: Bearer <access_token>
Content-Type: application/json

{
  "name": "Jane Smith",
  "software_level": "advanced",
  "hardware_access": "full_lab"
}
```
*Note: All fields optional (partial update). Only include fields to update.*

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "student@example.com",
    "name": "Jane Smith",
    "is_active": true,
    "is_verified": false,
    "created_at": "2025-12-06T10:30:00Z",
    "last_login": "2025-12-06T15:45:00Z"
  },
  "preferences": {
    "software_level": "advanced",
    "hardware_access": "full_lab",
    "preferred_language": "en",
    "updated_at": "2025-12-06T16:00:00Z"
  }
}
```

**Error Responses**:

*400 Bad Request - Validation Error*:
```json
{
  "detail": {
    "message": "Validation error",
    "code": "VALIDATION_ERROR",
    "errors": [
      {
        "field": "software_level",
        "message": "Must be one of: beginner, intermediate, advanced"
      }
    ]
  }
}
```

*401 Unauthorized*:
```json
{
  "detail": {
    "message": "Authentication required",
    "code": "MISSING_TOKEN"
  }
}
```

---

### 8. PUT /v1/user/password

**Description**: Change user password (requires current password).

**Authentication**: Required (JWT token)

**Request**:
```http
PUT /v1/user/password HTTP/1.1
Authorization: Bearer <access_token>
Content-Type: application/json

{
  "current_password": "securepassword123",
  "new_password": "newsecurepassword456"
}
```

**Response (200 OK)**:
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "message": "Password updated successfully"
}
```

**Error Responses**:

*400 Bad Request - New Password Too Short*:
```json
{
  "detail": {
    "message": "Validation error",
    "code": "VALIDATION_ERROR",
    "errors": [
      {
        "field": "new_password",
        "message": "Password must be at least 8 characters"
      }
    ]
  }
}
```

*401 Unauthorized - Incorrect Current Password*:
```json
{
  "detail": {
    "message": "Current password is incorrect",
    "code": "INVALID_PASSWORD"
  }
}
```

---

## Common Response Headers

All authenticated endpoints return these headers:

```http
X-Request-ID: <uuid>          # Unique request identifier for tracing
X-Process-Time: <ms>           # Request processing time in milliseconds
Content-Type: application/json
```

---

## Authentication Flow

### Token-Based Authentication

**Access Token**:
- Short-lived (30 minutes default)
- Used for API requests
- Stored in HTTP-only cookie OR Authorization header
- Format: `Bearer <JWT>`

**Refresh Token**:
- Long-lived (30 days default)
- Used to obtain new access token
- Stored in HTTP-only cookie only (more secure)
- Rotated on each refresh

**JWT Payload Structure**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "student@example.com",
  "exp": 1701874800,
  "iat": 1701873000,
  "token_type": "access"
}
```

---

## Rate Limiting

**Signup Endpoint**: 10 requests per hour per IP  
**Login Endpoint**: 5 requests per minute per IP  
**Other Endpoints**: 60 requests per minute per user

**Rate Limit Headers** (included in response):
```http
X-RateLimit-Limit: 5
X-RateLimit-Remaining: 3
X-RateLimit-Reset: 1701873600
```

**Rate Limit Exceeded Response (429)**:
```json
{
  "detail": {
    "message": "Too many requests. Please try again later.",
    "code": "RATE_LIMIT_EXCEEDED",
    "retry_after": 120
  }
}
```

---

## Security Considerations

### Cookie Security Flags
- `HttpOnly`: Prevents JavaScript access (XSS protection)
- `Secure`: HTTPS-only (production)
- `SameSite=Strict`: CSRF protection
- `Max-Age`: Token expiration time

### Password Requirements
- Minimum 8 characters
- Hashed with bcrypt (12 rounds)
- Never transmitted in plain (HTTPS required)
- Never logged or stored in plain text

### Error Message Security
- Generic authentication errors (prevent account enumeration)
- No stack traces in production responses
- Detailed errors logged server-side only

---

## Frontend Integration Example

```typescript
// TypeScript example for frontend integration

interface AuthResponse {
  user: {
    id: string;
    email: string;
    name: string;
    is_active: boolean;
    is_verified: boolean;
    created_at: string;
    last_login: string | null;
  };
  preferences: {
    software_level: "beginner" | "intermediate" | "advanced";
    hardware_access: "cloud_only" | "basic" | "full_lab";
    preferred_language: "en" | "ur" | "both";
    updated_at: string;
  };
  tokens: {
    access_token: string;
    refresh_token: string;
    token_type: string;
    expires_in: number;
  };
}

// Signup
const signup = async (data: SignupRequest): Promise<AuthResponse> => {
  const response = await fetch('/v1/auth/signup', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
    credentials: 'include'  // Include cookies
  });
  
  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail.message);
  }
  
  return response.json();
};

// Login
const login = async (email: string, password: string): Promise<AuthResponse> => {
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
  
  return response.json();
};

// Get session (check if authenticated)
const getSession = async (): Promise<AuthResponse | null> => {
  const response = await fetch('/v1/auth/session', {
    credentials: 'include'
  });
  
  if (!response.ok) {
    return null;  // Not authenticated
  }
  
  return response.json();
};

// Logout
const logout = async (): Promise<void> => {
  await fetch('/v1/auth/logout', {
    method: 'POST',
    credentials: 'include'
  });
};
```

---

## Testing Checklist

### Contract Tests (Required)
- [ ] POST /v1/auth/signup - Success (201)
- [ ] POST /v1/auth/signup - Email already exists (409)
- [ ] POST /v1/auth/signup - Invalid email format (400)
- [ ] POST /v1/auth/signup - Password too short (400)
- [ ] POST /v1/auth/login - Success (200)
- [ ] POST /v1/auth/login - Invalid credentials (401)
- [ ] POST /v1/auth/login - Account deactivated (403)
- [ ] POST /v1/auth/logout - Success (200)
- [ ] GET /v1/auth/session - Authenticated (200)
- [ ] GET /v1/auth/session - Not authenticated (401)
- [ ] GET /v1/auth/refresh - Success (200)
- [ ] GET /v1/auth/refresh - Invalid refresh token (401)
- [ ] GET /v1/user/profile - Success (200)
- [ ] GET /v1/user/profile - Not authenticated (401)
- [ ] PUT /v1/user/profile - Success (200)
- [ ] PUT /v1/user/profile - Validation error (400)
- [ ] PUT /v1/user/password - Success (200)
- [ ] PUT /v1/user/password - Incorrect current password (401)

---

## Summary

| **Endpoint** | **Method** | **Auth Required** | **Purpose** |
|--------------|------------|-------------------|-------------|
| `/v1/auth/signup` | POST | No | Create account with profiling |
| `/v1/auth/login` | POST | No | Authenticate user |
| `/v1/auth/logout` | POST | Yes | End session |
| `/v1/auth/session` | GET | Yes | Verify session |
| `/v1/auth/refresh` | GET | Refresh token | Get new access token |
| `/v1/user/profile` | GET | Yes | Get user profile |
| `/v1/user/profile` | PUT | Yes | Update profile/preferences |
| `/v1/user/password` | PUT | Yes | Change password |

**Total Endpoints**: 8  
**Public Endpoints**: 2 (signup, login)  
**Protected Endpoints**: 6 (require JWT authentication)

**Status**: ✅ API contracts complete. Ready for quickstart documentation.
