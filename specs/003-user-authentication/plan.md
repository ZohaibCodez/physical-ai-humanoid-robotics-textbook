# Implementation Plan: User Authentication with Background Profiling

**Branch**: `003-user-authentication` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-user-authentication/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement secure user authentication using Better-Auth library integrated with the existing FastAPI backend and Docusaurus frontend. During signup, collect user background information (software experience level, hardware access, preferred language) to enable personalized content delivery per Constitution Principles IX, X, and XI. System supports both authenticated users (with personalization) and guest users (public content access). Authentication uses JWT sessions stored in Better-Auth with user data and preferences persisted in Neon Postgres.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript/JavaScript (frontend)  
**Primary Dependencies**: 
- Backend: FastAPI >=0.115.0, Better-Auth (Python adapter), psycopg2-binary 2.9.10, asyncpg 0.29.0, pydantic >=2.10.3, python-jose (JWT), passlib (password hashing), python-multipart
- Frontend: React 18, Docusaurus 3.0, Context API for auth state management
**Storage**: Neon Postgres (existing connection via NEON_DATABASE_URL)  
**Testing**: pytest (backend unit/integration tests), React Testing Library (frontend component tests)  
**Target Platform**: Web application - Linux/containerized backend (Vercel/Railway deployment), browser-based frontend (static Docusaurus build)
**Project Type**: Web application (backend + frontend architecture)  
**Performance Goals**: 
- Signup/login response time: <2 seconds (p95)
- Support 100 concurrent authentication requests without degradation
- JWT token validation: <50ms
- Cold start time (serverless): <3 seconds
**Constraints**: 
- Free-tier deployment compatible (Vercel/Railway/Render)
- Total backend size budget: 100-150MB (existing constraint from requirements.txt)
- No heavy ML frameworks (maintain lightweight backend)
- Session timeout: 7-30 days (TBD, industry standard)
- GDPR/privacy considerations for user data storage
**Scale/Scope**: 
- Initial target: 1,000-10,000 users (educational platform)
- Database: 2 new tables (users, user_preferences)
- API endpoints: 4-6 new auth/user routes
- Frontend: 3-4 new pages/components (Login, Signup, Profile, AuthProvider)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle IX: Secure Authentication ✅
- **Requirement**: Use Better-Auth for modern, secure authentication. Collect user background during signup. Protect user data and privacy.
- **Compliance**: 
  - Better-Auth library selected for authentication framework
  - Background profiling integrated into signup flow (software level, hardware access, language preference)
  - Password hashing with passlib (bcrypt/argon2)
  - JWT tokens for secure session management
  - User data stored in Neon Postgres with proper access controls
  - No plaintext password storage
- **Status**: PASS ✅

### Principle X: Background Profiling ✅
- **Requirement**: Ask about software experience, hardware access. Store preferences for content adaptation.
- **Compliance**:
  - Signup form collects three profiling dimensions:
    1. Software Experience: beginner/intermediate/advanced
    2. Hardware Access: cloud_only/basic/full_lab
    3. Preferred Language: en/ur/both
  - Preferences stored in dedicated user_preferences table
  - Data available for RAG chatbot personalization (future integration)
- **Status**: PASS ✅

### Principle XI: Seamless UX ✅
- **Requirement**: Optional authentication, SSO-ready, fast non-intrusive signup flow.
- **Compliance**:
  - Guest users can access all public textbook content without authentication (no forced signup walls)
  - Protected routes (profile, preferences) only require auth when accessed
  - Signup flow: 2-step process (<3 minutes per success criteria)
  - Architecture supports future OAuth providers (Google, GitHub) via Better-Auth
  - SSO-ready: Better-Auth supports OAuth flows natively
- **Status**: PASS ✅

### Principle VII: Code Quality ✅
- **Requirement**: Tested, functional code. Industry best practices.
- **Compliance**:
  - pytest test suite for auth endpoints (contract, integration, unit tests)
  - Password validation, email format validation
  - Error handling for edge cases (duplicate email, invalid credentials, database failures)
  - Logging for security events (failed login attempts, account creation)
- **Status**: PASS ✅

### Principle VIII: Maintainability ✅
- **Requirement**: Clear structure, reusable components, well-documented.
- **Compliance**:
  - Separate auth routes module (app/api/routes/auth.py)
  - Auth models in app/models/user.py
  - Auth service layer in app/services/auth_service.py
  - Reusable auth middleware for protected routes
  - Clear separation: backend (FastAPI), frontend (React/Docusaurus)
- **Status**: PASS ✅

### Performance & Deployment Readiness (Principles VI) ✅
- **Requirement**: Deploy to free-tier platforms, lightweight, fast.
- **Compliance**:
  - Better-Auth is lightweight (adds ~5-10MB to deployment)
  - No heavy ML frameworks added (maintains 100-150MB budget)
  - Vercel/Railway compatible
  - Async database operations (asyncpg) for performance
  - JWT validation caching for speed
- **Status**: PASS ✅

**Overall Gate Status**: ✅ PASS - All constitution requirements satisfied. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── models/
│   │   └── user.py              # NEW: User, UserPreferences Pydantic models
│   ├── services/
│   │   └── auth_service.py      # NEW: Auth business logic (signup, login, session mgmt)
│   ├── api/
│   │   └── routes/
│   │       └── auth.py          # NEW: Auth endpoints (POST /signup, POST /login, GET /session, etc.)
│   ├── utils/
│   │   ├── security.py          # NEW: Password hashing, JWT token generation/validation
│   │   └── dependencies.py      # NEW: Auth middleware (get_current_user dependency)
│   ├── config.py                # MODIFY: Add auth-related config (JWT secret, token expiry)
│   └── main.py                  # MODIFY: Include auth router
├── tests/
│   ├── contract/
│   │   └── test_auth_api.py     # NEW: API contract tests for auth endpoints
│   ├── integration/
│   │   └── test_auth_flow.py    # NEW: End-to-end auth flow tests
│   └── unit/
│       └── test_auth_service.py # NEW: Unit tests for auth service logic
├── scripts/
│   └── create_auth_tables.sql   # NEW: SQL migration for users & user_preferences tables
└── requirements.txt             # MODIFY: Add better-auth, python-jose, passlib

src/ (frontend)
├── components/
│   ├── auth/
│   │   ├── LoginForm.tsx        # NEW: Login form component
│   │   ├── SignupForm.tsx       # NEW: Signup form with background questions
│   │   ├── ProfileForm.tsx      # NEW: Profile/preferences update form
│   │   └── AuthProvider.tsx     # NEW: React Context for auth state
│   └── common/
│       └── ProtectedRoute.tsx   # NEW: Route wrapper requiring authentication
├── pages/
│   ├── login.tsx                # NEW: Login page
│   ├── signup.tsx               # NEW: Signup page
│   └── profile.tsx              # NEW: User profile page
├── hooks/
│   └── useAuth.ts               # NEW: Custom hook for auth state/actions
└── theme/                       # MODIFY: Style auth pages to match Docusaurus theme
```

**Structure Decision**: Web application architecture (Option 2) selected. The repository has an existing `backend/` directory with FastAPI structure and `src/` frontend directory for Docusaurus/React components. This feature extends both sides:

- **Backend**: Add auth-specific models, services, routes, and utilities to existing `backend/app/` structure
- **Frontend**: Add auth components, pages, and hooks to existing `src/` Docusaurus structure
- **Database**: New tables via SQL migration scripts (follows existing pattern in `backend/scripts/`)
- **Tests**: Follow existing three-tier structure (contract/integration/unit)

## Complexity Tracking

No constitution violations. All complexity justified by feature requirements and existing architecture patterns.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
