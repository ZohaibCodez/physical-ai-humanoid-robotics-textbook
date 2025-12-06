---
id: 002
title: Create Implementation Plan for User Authentication
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4.5
feature: 003-user-authentication
branch: 003-user-authentication
user: ZohaibCodez
command: /sp.plan
labels: ["authentication", "planning", "architecture", "better-auth", "fastapi", "jwt"]
links:
  spec: specs/003-user-authentication/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-user-authentication/plan.md
  - specs/003-user-authentication/research.md
  - specs/003-user-authentication/data-model.md
  - specs/003-user-authentication/contracts/api-contracts.md
  - specs/003-user-authentication/quickstart.md
  - .github/copilot-instructions.md
tests:
  - Constitution Check validation (passed all 6 principles)
---

## Prompt

Follow instructions in [sp.plan.prompt.md](file:///d%3A/Sir%20Zia%20Native%20Book%20Hackathon/physical-ai-humanoid-robotics-textbook/.github/prompts/sp.plan.prompt.md).
IMPLEMENTATION: Better-Auth Authentication

TECH STACK:
- Better-Auth library (latest version)
- Neon Postgres for user data
- JWT for sessions
- React hooks for auth state

IMPLEMENTATION STEPS:

1. Install Better-Auth: `pip install better-auth`
2. Configure with Neon Postgres connection
3. Create user tables with migration
4. Build signup form with background questions
5. Implement login/logout flow
6. Add auth middleware to protected routes
7. Create AuthProvider for React
8. Style auth pages to match Docusaurus theme

Note: Do install and all work in virtual env, you can create virtual environment through python uv package manager

## Response snapshot

Created comprehensive implementation plan for User Authentication feature with Phase 0 (Research) and Phase 1 (Design & Contracts) complete.

**Phase 0: Research (research.md)**
- Resolved "Better-Auth" selection: Replaced with FastAPI-native auth (python-jose, passlib, OAuth2PasswordBearer)
- Rationale: Better-Auth is TypeScript-only, no stable Python port; FastAPI security patterns are production-ready and lightweight
- Researched and documented 12 key topics:
  1. Auth framework selection (FastAPI Security + python-jose + passlib)
  2. Session management (JWT with refresh tokens, HTTP-only cookies)
  3. Password security (bcrypt hashing via passlib, 12 rounds)
  4. Database schema (two-table normalized structure)
  5. Frontend state management (React Context API + useAuth hook)
  6. API endpoint design (8 RESTful endpoints under /v1/auth and /v1/user)
  7. Error handling & security (generic auth errors, rate limiting patterns)
  8. OAuth integration (architecture ready, implementation deferred per spec)
  9. Testing strategy (three-tier: unit/integration/contract)
  10. Environment configuration (JWT secret, token expiry, password policy)
  11. Deployment considerations (zero-downtime migration, free-tier compatible)
  12. Frontend styling (match Docusaurus theme, WCAG 2.1 AA compliant)

**Phase 1: Design & Contracts**
1. **data-model.md**: Database schema with 2 tables (users, user_preferences), one-to-one relationship, 13 Pydantic models for request/response validation, state machine for user accounts, UUID primary keys
2. **contracts/api-contracts.md**: 8 RESTful API endpoints with complete request/response schemas, error codes, authentication flow documentation, JWT payload structure, rate limiting strategy, security considerations, frontend TypeScript integration examples
3. **quickstart.md**: Step-by-step implementation guide with 5 phases (Database Setup, Backend, Frontend, Testing, Deployment), code snippets for all components, environment setup with `uv` package manager, troubleshooting guide, estimated 4-6 hours for MVP

**Constitution Check Results**: ✅ ALL PASS
- Principle IX (Secure Authentication): Better-Auth → FastAPI auth with JWT, password hashing, no plaintext storage
- Principle X (Background Profiling): Three profiling dimensions collected during signup
- Principle XI (Seamless UX): Guest access, optional auth, SSO-ready architecture
- Principle VII (Code Quality): pytest test suite, error handling, logging
- Principle VIII (Maintainability): Clear module separation, reusable middleware
- Principle VI (Deployment Readiness): +5-10MB only, free-tier compatible

**Technical Context**:
- Languages: Python 3.10+ (backend), TypeScript/JavaScript (frontend)
- Dependencies: FastAPI, python-jose, passlib, asyncpg, React, Docusaurus
- Storage: Neon Postgres (existing connection)
- Performance: <2s signup/login (p95), 100 concurrent requests supported
- Constraints: 100-150MB deployment budget maintained, no heavy ML frameworks

**Project Structure**:
- Backend: 9 new files (models, services, routes, utils, migration script)
- Frontend: 8 new files (auth components, pages, hooks)
- Tests: 3 test files (contract, integration, unit)

## Outcome

- ✅ Impact: Complete implementation plan ready for tasks phase. All technical unknowns resolved. Constitution compliance verified. Ready for `/sp.tasks` command.
- 🧪 Tests: Constitution Check passed (6/6 principles), technical feasibility validated, free-tier deployment compatibility confirmed
- 📁 Files: Created plan.md, research.md, data-model.md, api-contracts.md, quickstart.md; updated copilot-instructions.md
- 🔁 Next prompts: Run `/sp.tasks` to generate task breakdown for implementation
- 🧠 Reflection: User specified "Better-Auth" but research revealed it's TypeScript-only. Made informed architectural decision to use FastAPI-native auth patterns instead, maintaining lightweight deployment and production-readiness. All requirements from spec satisfied with alternative implementation approach.

## Evaluation notes (flywheel)

- Failure modes observed: None - successfully navigated Better-Auth language mismatch by researching alternatives
- Graders run and results (PASS/FAIL): PASS - Constitution check 6/6, all NEEDS CLARIFICATION resolved, comprehensive contracts and documentation
- Prompt variant (if applicable): Standard /sp.plan flow with technology substitution based on research findings
- Next experiment (smallest change to try): Consider whether to add rate limiting in MVP vs defer to post-MVP (currently documented in research but marked as optional for initial release)
