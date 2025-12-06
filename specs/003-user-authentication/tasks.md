---
description: "Implementation tasks for User Authentication with Background Profiling"
---

# Tasks: User Authentication with Background Profiling

**Feature**: 003-user-authentication  
**Input**: Design documents from `/specs/003-user-authentication/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-contracts.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [ ] T001 Create Python virtual environment using uv: `cd backend && uv venv`
- [ ] T002 Install authentication dependencies in backend/requirements.txt: python-jose[cryptography]==3.3.0, passlib[bcrypt]==1.7.4, python-multipart>=0.0.9
- [ ] T003 [P] Update backend/app/config.py to add JWT configuration: JWT_SECRET_KEY, JWT_ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES, REFRESH_TOKEN_EXPIRE_DAYS
- [ ] T004 Create database migration script backend/scripts/create_auth_tables.sql with users and user_preferences tables per data-model.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core authentication infrastructure that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Execute database migration: Run backend/scripts/create_auth_tables.sql on Neon Postgres to create users and user_preferences tables
- [ ] T006 Create backend/app/models/user.py with Pydantic models: UserDB, UserPreferencesDB, UserResponse, UserPreferencesResponse, UserProfileResponse
- [ ] T007 Create backend/app/utils/security.py with functions: verify_password(), get_password_hash(), create_access_token(), create_refresh_token(), verify_token()
- [ ] T008 Create backend/app/utils/dependencies.py with get_current_user() dependency for protected route authentication
- [ ] T009 [P] Create src/contexts/AuthContext.jsx with React Context for auth state management (user, isAuthenticated, isLoading)
- [ ] T010 [P] Create src/hooks/useAuth.js custom hook to access AuthContext

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New User Signup with Profiling (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with email/password and complete background profiling questions (software level, hardware access, preferred language) during signup flow.

**Independent Test**: Navigate to /signup, enter valid email/password/name, complete background questions (software_level: intermediate, hardware_access: basic, preferred_language: en), submit form, verify account created in database, verify redirect to dashboard with authenticated session, logout and login again successfully.

### Backend Implementation

- [ ] T011 [US1] Create SignupRequest Pydantic model in backend/app/models/user.py with fields: email (EmailStr), password (min 8 chars), name (str), software_level (enum), hardware_access (enum), preferred_language (enum)
- [ ] T012 [US1] Implement signup_user() function in backend/app/services/auth_service.py: validate email uniqueness, hash password, create user record, create preferences record, return user data
- [ ] T013 [US1] Create POST /v1/auth/signup endpoint in backend/app/api/routes/auth.py: accept SignupRequest, call signup_user(), generate JWT tokens, set HTTP-only cookies, return AuthResponse
- [ ] T014 [US1] Add email validation logic in backend/app/utils/validators.py: check email format, domain validity
- [ ] T015 [US1] Add password validation logic in backend/app/utils/validators.py: enforce minimum 8 characters, optional complexity requirements

### Frontend Implementation

- [ ] T016 [P] [US1] Create src/components/auth/SignupForm.jsx with form fields: email (input), password (password input), name (input), software_level (select), hardware_access (select), preferred_language (select)
- [ ] T017 [P] [US1] Create src/pages/Signup.jsx page component wrapping SignupForm with Docusaurus theme layout
- [ ] T018 [US1] Implement handleSignup() function in SignupForm.jsx: call POST /v1/auth/signup API, handle success (store tokens, update AuthContext, redirect to home), handle errors (display validation messages)
- [ ] T019 [US1] Add client-side validation to SignupForm.jsx: email format check, password length check, required fields check before submission
- [ ] T020 [US1] Style SignupForm.jsx to match Docusaurus theme: use Infima CSS classes, responsive design, WCAG 2.1 AA compliant

### Integration

- [ ] T021 [US1] Register auth router in backend/app/main.py: app.include_router(auth_router, prefix="/v1/auth", tags=["auth"])
- [ ] T022 [US1] Update src/theme/Root.jsx to wrap app with AuthProvider component for global auth state
- [ ] T023 [US1] Add /signup route to Docusaurus routing in docusaurus.config.js

---

## Phase 4: User Story 2 - Returning User Login (Priority: P1) 🎯 MVP

**Goal**: Enable existing users to authenticate using email and password, establish session, and access personalized content.

**Independent Test**: Navigate to /login, enter existing account credentials (email and password), submit form, verify session established (check AuthContext state), verify preferences loaded, verify redirect to home page, verify protected routes accessible.

### Backend Implementation

- [ ] T024 [US2] Create LoginRequest Pydantic model in backend/app/models/user.py with fields: email (EmailStr), password (str)
- [ ] T025 [US2] Implement login_user() function in backend/app/services/auth_service.py: verify email exists, verify password hash, update last_login timestamp, return user data with preferences
- [ ] T026 [US2] Create POST /v1/auth/login endpoint in backend/app/api/routes/auth.py: accept LoginRequest, call login_user(), generate JWT tokens, set HTTP-only cookies, return AuthResponse
- [ ] T027 [US2] Create GET /v1/auth/session endpoint in backend/app/api/routes/auth.py: verify JWT token, return current user data with preferences (used for session restoration)
- [ ] T028 [US2] Create POST /v1/auth/logout endpoint in backend/app/api/routes/auth.py: clear HTTP-only cookies, return success response
- [ ] T029 [US2] Create GET /v1/auth/refresh endpoint in backend/app/api/routes/auth.py: verify refresh token, generate new access token, return new tokens

### Frontend Implementation

- [ ] T030 [P] [US2] Create src/components/auth/LoginForm.jsx with form fields: email (input), password (password input)
- [ ] T031 [P] [US2] Create src/pages/Login.jsx page component wrapping LoginForm with Docusaurus theme layout
- [ ] T032 [US2] Implement handleLogin() function in LoginForm.jsx: call POST /v1/auth/login API, handle success (update AuthContext, redirect to intended destination or home), handle errors (display generic "Invalid email or password" message)
- [ ] T033 [US2] Implement session restoration in AuthProvider: call GET /v1/auth/session on app mount, restore user state if valid token exists
- [ ] T034 [US2] Implement logout functionality in AuthProvider: call POST /v1/auth/logout API, clear auth state, redirect to home
- [ ] T035 [US2] Style LoginForm.jsx to match Docusaurus theme: use Infima CSS classes, responsive design, WCAG 2.1 AA compliant
- [ ] T036 [US2] Add redirect logic to Login page: if user already authenticated, redirect to home page automatically

### Integration

- [ ] T037 [US2] Add /login route to Docusaurus routing in docusaurus.config.js
- [ ] T038 [US2] Create src/components/ProtectedRoute.jsx wrapper component: check AuthContext.isAuthenticated, redirect to /login with returnUrl if not authenticated
- [ ] T039 [US2] Implement token refresh mechanism in AuthProvider: automatically call /v1/auth/refresh when access token expires

---

## Phase 5: User Story 4 - Guest User Experience (Priority: P1) 🎯 MVP

**Goal**: Allow unauthenticated visitors to browse all public textbook content without forced signup, with optional non-intrusive signup prompts.

**Independent Test**: Open textbook in incognito/private browser window (no cookies), navigate to course modules and content pages, verify all educational content readable without login prompts blocking access, click profile link and verify friendly message "Create an account to unlock personalized features" appears without blocking navigation.

### Backend Implementation

- [ ] T040 [US4] Update backend/app/utils/dependencies.py to create get_current_user_optional() dependency: return user if valid token exists, return None if not authenticated (no exception raised)
- [ ] T041 [US4] Audit all API endpoints in backend/app/api/routes/ to ensure public content routes do NOT require authentication (remove Depends(get_current_user) where applicable)

### Frontend Implementation

- [ ] T042 [P] [US4] Update AuthProvider to handle guest state: set isAuthenticated=false, user=null as default state
- [ ] T043 [P] [US4] Create src/components/auth/SignupPrompt.jsx component: dismissible, non-intrusive message "Track your progress by creating a free account" with signup link
- [ ] T044 [US4] Update protected pages (Profile, Preferences) to show friendly message for guests: "Create an account to unlock personalized features" with signup/login links instead of hard redirect
- [ ] T045 [US4] Add conditional rendering in navigation bar: show "Login" and "Signup" buttons for guests, show "Profile" and "Logout" for authenticated users
- [ ] T046 [US4] Verify all textbook content pages (docs/) are accessible without authentication checks in src/theme/DocPage/ or src/pages/

### Integration

- [ ] T047 [US4] Test guest user flow: clear all cookies, navigate to multiple content pages, confirm no forced login redirects, confirm content fully readable

---

## Phase 6: User Story 3 - Profile Preferences Update (Priority: P2)

**Goal**: Enable authenticated users to update their background preferences (software level, hardware access, language) from profile page and persist changes across sessions.

**Independent Test**: Log in with existing account, navigate to /profile, change software_level from "beginner" to "intermediate", change hardware_access to "full_lab", save changes, verify success message, logout and login again, navigate to /profile, verify changes persisted.

### Backend Implementation

- [ ] T048 [US3] Create ProfileUpdateRequest Pydantic model in backend/app/models/user.py with optional fields: name (Optional[str]), software_level (Optional[enum]), hardware_access (Optional[enum]), preferred_language (Optional[enum])
- [ ] T049 [US3] Implement update_user_preferences() function in backend/app/services/auth_service.py: update user_preferences table, update updated_at timestamp, return updated preferences
- [ ] T050 [US3] Create GET /v1/user/profile endpoint in backend/app/api/routes/auth.py: require authentication with Depends(get_current_user), return UserProfileResponse with user data and preferences
- [ ] T051 [US3] Create PUT /v1/user/profile endpoint in backend/app/api/routes/auth.py: require authentication, accept ProfileUpdateRequest, call update_user_preferences(), return updated profile

### Frontend Implementation

- [ ] T052 [P] [US3] Create src/components/profile/ProfileForm.jsx with form fields: name (input), software_level (select with current value), hardware_access (select with current value), preferred_language (select with current value)
- [ ] T053 [P] [US3] Create src/pages/Profile.jsx page component: wrap with ProtectedRoute, fetch profile data from GET /v1/user/profile on mount, render ProfileForm with current values
- [ ] T054 [US3] Implement handleUpdateProfile() function in ProfileForm.jsx: call PUT /v1/user/profile API with changed fields only, handle success (show toast notification, update AuthContext), handle errors (display validation messages)
- [ ] T055 [US3] Style ProfileForm.jsx to match Docusaurus theme: use Infima CSS classes, responsive design, show loading state during API calls
- [ ] T056 [US3] Add success/error toast notifications using react-toastify or similar library for user feedback

### Integration

- [ ] T057 [US3] Add /profile route to Docusaurus routing in docusaurus.config.js
- [ ] T058 [US3] Add "Profile" link to navigation bar for authenticated users (conditionally rendered in src/theme/Navbar/)

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, security hardening, performance optimization, documentation

- [ ] T059 [P] Add rate limiting middleware to auth endpoints in backend/app/main.py: limit POST /v1/auth/signup and POST /v1/auth/login to 5 requests per minute per IP
- [ ] T060 [P] Implement comprehensive error handling in backend/app/api/routes/auth.py: catch database errors, return user-friendly messages, log errors for monitoring
- [ ] T061 [P] Add password reset flow preparation: create PasswordResetRequest model, stub POST /v1/auth/reset-password endpoint (full implementation deferred per spec out-of-scope)
- [ ] T062 [P] Add logging for security events in backend/app/services/auth_service.py: log failed login attempts, account creation, preference updates
- [ ] T063 [P] Create API documentation for auth endpoints: add OpenAPI/Swagger annotations to all routes in backend/app/api/routes/auth.py
- [ ] T064 [P] Add loading states to all forms: show spinner during API calls in SignupForm, LoginForm, ProfileForm
- [ ] T065 [P] Implement form field accessibility: add proper labels, ARIA attributes, keyboard navigation support in all auth forms
- [ ] T066 [P] Add environment variable validation in backend/app/config.py: ensure JWT_SECRET_KEY is set and strong (min 32 characters)
- [ ] T067 Write README section in backend/README.md documenting authentication setup: environment variables, database migration steps, testing instructions

---

## Implementation Strategy

### MVP Scope (Recommended First Iteration)

The MVP should include **User Stories 1, 2, and 4 only** (all P1 priority):
- ✅ User Story 1: New User Signup with Profiling (T011-T023)
- ✅ User Story 2: Returning User Login (T024-T039)
- ✅ User Story 4: Guest User Experience (T040-T047)

**MVP Completion Criteria**:
1. Users can create accounts with background profiling
2. Users can log in and maintain sessions
3. Guest users can access public content freely
4. All foundational infrastructure complete (Phase 1-2)

**Deferred to Post-MVP**:
- ❌ User Story 3: Profile Preferences Update (P2 priority, can be added after MVP validation)
- ❌ Password reset flow (out of scope per spec.md)
- ❌ OAuth providers (out of scope per spec.md)
- ❌ Email verification (out of scope per spec.md)

### Parallel Execution Opportunities

**Within User Story 1** (can parallelize):
- T016 (SignupForm.jsx) + T014 (email validation) + T015 (password validation) - different files, no dependencies
- T017 (Signup.jsx page) + T016 (SignupForm component) - backend tasks T011-T013 completed first

**Within User Story 2** (can parallelize):
- T030 (LoginForm.jsx) + T031 (Login.jsx) - after backend endpoints ready
- T033 (session restoration) + T034 (logout) + T039 (token refresh) - independent AuthProvider features

**Within User Story 4** (can parallelize):
- T042 (guest state) + T043 (SignupPrompt) + T045 (navigation) - different components

**Phase 7 Polish** (mostly parallel):
- All T059-T066 tasks can run in parallel (different files/concerns)

### Dependency Graph (User Story Completion Order)

```
Phase 1 (Setup) → Phase 2 (Foundational)
                       ↓
        ┌──────────────┴──────────────┬──────────────┐
        ↓                             ↓              ↓
    Phase 3 (US1)              Phase 4 (US2)   Phase 5 (US4)
    Signup Flow                Login Flow       Guest Access
        ↓                             ↓              ↓
        └──────────────┬──────────────┴──────────────┘
                       ↓
                 Phase 6 (US3)
              Profile Update (P2)
                       ↓
                 Phase 7 (Polish)
```

**Key Dependencies**:
- Phase 2 MUST complete before ANY user story work
- User Story 1 (Signup) should complete before User Story 2 (Login) for testing purposes, but technically independent
- User Story 3 (Profile Update) depends on User Story 2 (Login) for authentication
- User Story 4 (Guest Access) can start immediately after Phase 2, independent of US1/US2

**Critical Path**: Phase 1 → Phase 2 → US1 → US2 → US3 → Phase 7 (approximately 6-8 hours for MVP)

---

## Task Summary

**Total Tasks**: 67  
**MVP Tasks** (US1, US2, US4 only): 47 tasks  
**Post-MVP Tasks** (US3, Polish): 20 tasks

**Task Count by User Story**:
- Setup (Phase 1): 4 tasks
- Foundational (Phase 2): 6 tasks
- User Story 1 (Signup - P1): 13 tasks
- User Story 2 (Login - P1): 16 tasks
- User Story 4 (Guest - P1): 8 tasks
- User Story 3 (Profile - P2): 11 tasks
- Polish (Phase 7): 9 tasks

**Parallel Opportunities**: 18 tasks marked with [P] can run in parallel

**Estimated Effort**:
- MVP (US1, US2, US4): 4-6 hours
- Post-MVP (US3): 1.5-2 hours
- Polish (Phase 7): 1-1.5 hours
- **Total**: 6.5-9.5 hours

---

## Format Validation ✅

All tasks follow the required checklist format:
- ✅ Checkbox: All tasks start with `- [ ]`
- ✅ Task ID: Sequential T001-T067
- ✅ [P] marker: 18 tasks marked parallelizable
- ✅ [Story] label: All user story tasks labeled (US1, US2, US3, US4)
- ✅ File paths: All implementation tasks include exact file paths
- ✅ Description: Clear, actionable descriptions

**Ready for execution**: Each task can be completed by an LLM without additional context.
