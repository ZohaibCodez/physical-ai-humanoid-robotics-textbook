---
description: "Implementation tasks for RAG chatbot feature"
---

# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: Tests are NOT included in this task list as they were not explicitly requested in the feature specification. Focus is on implementation and manual validation per user story acceptance scenarios.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a web application with separate backend (Python/FastAPI) and frontend (React/TypeScript):
- **Backend**: `backend/app/`, `backend/tests/`, `backend/scripts/`
- **Frontend**: `frontend/src/`, integrated with existing Docusaurus in `src/components/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both backend and frontend

- [ ] T001 Create backend project structure per plan.md (backend/app/, backend/tests/, backend/scripts/)
- [ ] T002 Initialize Python 3.11+ project with requirements.txt (FastAPI, google-generativeai, qdrant-client, psycopg2-binary)
- [ ] T003 [P] Create backend/.env.example with all required environment variables (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL)
- [ ] T004 [P] Create backend/Dockerfile for containerized deployment
- [ ] T005 [P] Initialize backend/app/__init__.py and backend/app/main.py with FastAPI app
- [ ] T006 [P] Create requirements-dev.txt for development dependencies (pytest, pytest-asyncio, httpx, black, flake8)
- [ ] T007 Create frontend ChatWidget component structure in src/components/ChatWidget/
- [ ] T008 Install frontend dependencies (npm install @chatscope/chat-ui-kit-react react-markdown remark-gfm)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Create backend/app/config.py for environment configuration (load from .env, validate required keys)
- [ ] T010 [P] Implement backend/app/utils/logger.py with structured JSON logging
- [ ] T011 [P] Implement backend/app/utils/exceptions.py with custom exception classes (RateLimitError, EmbeddingError, VectorStoreError)
- [ ] T012 Create database schema in backend/scripts/migrate_db.py (conversations, questions, answers tables with indexes)
- [ ] T013 Implement backend/app/services/postgres_service.py with Neon Postgres connection pooling and CRUD operations
- [ ] T014 [P] Implement backend/app/services/embeddings.py for Google text-embedding-004 API client
- [ ] T015 [P] Implement backend/app/services/embeddings_local.py for Sentence Transformers fallback (all-MiniLM-L6-v2)
- [ ] T016 Implement backend/app/services/vector_store.py with Qdrant client (create collection with named vectors: google 768-dim, local 384-dim)
- [ ] T017 [P] Implement backend/app/services/rate_limiter.py with token bucket algorithm (10 req/min per session, 50 req/hour per IP)
- [ ] T018 Implement backend/app/services/indexer.py for textbook chunking (300-500 tokens, 50-token overlap) and embedding generation
- [ ] T019 Create backend/scripts/index_textbook.py CLI script to index docs/ directory into Qdrant
- [ ] T020 [P] Implement backend/app/api/dependencies.py with FastAPI dependency injection for services
- [ ] T021 Configure CORS middleware in backend/app/main.py for frontend origins
- [ ] T022 Setup API versioning and prefix (/v1) in backend/app/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Question from Full Textbook (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about any textbook content and receive accurate answers with citations from the full textbook corpus

**Independent Test**: Ask 10 representative questions from different chapters (e.g., "What are the key components of ROS2?", "Explain forward kinematics") and verify:
1. Answers are accurate and relevant (manually validated against textbook)
2. Each answer includes at least one citation
3. Citations reference correct textbook sections
4. Out-of-scope questions (e.g., "What is quantum computing?") are identified as not covered

### Data Models for User Story 1

- [ ] T023 [P] [US1] Create backend/app/models/conversation.py with Conversation Pydantic model (id, session_id, user_ip, context_mode, created_at, updated_at)
- [ ] T024 [P] [US1] Create backend/app/models/textbook.py with TextbookChunk and Citation Pydantic models (chunk_id, text, chapter, section, embeddings, metadata)
- [ ] T025 [P] [US1] Create backend/app/types/chat.py with QuestionRequest and AnswerResponse schemas matching OpenAPI spec

### Services for User Story 1

- [ ] T026 [US1] Implement backend/app/services/gemini_service.py with Gemini 1.5 Flash API wrapper (generate_answer method with context, error handling, rate limit retry)
- [ ] T027 [US1] Implement backend/app/services/citation_resolver.py to generate Docusaurus URLs from chunk metadata (format: /week-##-##/topic#heading-slug)
- [ ] T028 [US1] Extend backend/app/services/vector_store.py with semantic search method (search by query embedding, return top-5 chunks with relevance scores)
- [ ] T029 [US1] Extend backend/app/services/postgres_service.py with methods to create/retrieve conversations, save questions, save answers

### API Endpoints for User Story 1

- [ ] T030 [US1] Implement POST /v1/chat/ask endpoint in backend/app/api/routes/chat.py for full-textbook mode
  - Validate QuestionRequest (10-1000 chars, session_id format)
  - Apply rate limiting (call rate_limiter service)
  - Generate query embedding (call embeddings service with fallback)
  - Search vector store for relevant chunks (top-5)
  - Assemble context from chunks
  - Call Gemini API to generate answer (max 500 words)
  - Generate citations with links (call citation_resolver)
  - Save question and answer to Postgres
  - Return AnswerResponse with answer, citations, confidence, processing_time_ms
- [ ] T031 [US1] Add error handling for all failure modes in chat.py (no relevant content found, Gemini timeout, rate limit exceeded, embedding API failure)
- [ ] T032 [US1] Implement GET /v1/health endpoint in backend/app/api/routes/health.py (check Gemini API, Qdrant, Postgres health)

### Frontend Integration for User Story 1

- [ ] T033 [US1] Create src/components/ChatWidget/index.tsx with chat UI using @chatscope/chat-ui-kit-react
  - State management for messages, typing indicator, session ID
  - handleSend function to call POST /v1/chat/ask
  - Display user messages and assistant responses
  - Show typing indicator during API call
  - Error handling with user-friendly messages
- [ ] T034 [US1] Create src/components/ChatWidget/styles.css for chat widget styling (fixed position bottom-right, toggle button, responsive design)
- [ ] T035 [US1] Create src/components/CitationLink.tsx to render clickable citations that navigate to textbook sections
- [ ] T036 [US1] Integrate ChatWidget in src/theme/Layout/index.tsx to appear on all Docusaurus pages
- [ ] T037 [US1] Configure environment variables in .env and docusaurus.config.js for REACT_APP_CHAT_API_URL

### Setup & Validation for User Story 1

- [ ] T038 [US1] Run backend/scripts/index_textbook.py to embed and index all docs/ content into Qdrant (~500-1000 chunks expected)
- [ ] T039 [US1] Run backend/scripts/migrate_db.py to initialize Neon Postgres schema
- [ ] T040 [US1] Start backend server (uvicorn app.main:app --reload) and verify /v1/health returns healthy
- [ ] T041 [US1] Test POST /v1/chat/ask with curl/Postman using 3 sample questions (verify 200 response, citations present)
- [ ] T042 [US1] Start Docusaurus frontend (npm start) and verify chat widget appears and responds to questions
- [ ] T043 [US1] Manual validation against acceptance scenarios:
  - Test textbook-covered question (e.g., "What are ROS2 components?") → Verify accurate answer with citations
  - Test out-of-scope question (e.g., "What is quantum computing?") → Verify "not covered" response
  - Test vague question (e.g., "Tell me about robots") → Verify structured overview or clarifying response
  - Click citation links → Verify navigation to correct textbook sections

**Checkpoint**: User Story 1 (MVP) is fully functional - students can ask questions and receive cited answers from full textbook

---

## Phase 4: User Story 2 - Ask Question from Selected Text (Priority: P2)

**Goal**: Enable students to highlight specific textbook sections and ask questions scoped only to that selected content for focused learning

**Independent Test**: 
1. Select a 2-page section (e.g., "TF2 Transformations")
2. Ask 5 questions answerable from selection (e.g., "How do coordinate frames work?") → Verify answers reference only selected section
3. Ask 2 questions NOT answerable from selection (e.g., "What is motion planning?") → Verify "cannot answer from selected content" response
4. Clear selection and ask same questions → Verify answers now reference full textbook

### Data Models for User Story 2

- [ ] T044 [P] [US2] Create backend/app/models/context.py with SelectedContext Pydantic model (text, range with startOffset/endOffset, metadata with chapter/section/file_path)
- [ ] T045 [P] [US2] Extend QuestionRequest schema in backend/app/types/chat.py to include optional selected_text and selected_metadata fields

### Services for User Story 2

- [ ] T046 [US2] Extend backend/app/services/vector_store.py with filtered search method (search within specific chapter/section using Qdrant payload filters)
- [ ] T047 [US2] Update backend/app/services/postgres_service.py to store selected_context JSONB in questions table

### API Endpoints for User Story 2

- [ ] T048 [US2] Extend POST /v1/chat/ask endpoint in backend/app/api/routes/chat.py to handle selected-text mode
  - Validate selected_text (50-5000 chars if provided)
  - Validate selected_metadata (chapter 1-13, section, file_path format)
  - If context_mode='selected': embed selected_text, search vector store with chapter/section filter
  - Generate answer from filtered context only
  - Add "context_used" field to response indicating which mode was used
- [ ] T049 [US2] Add validation to reject questions where selected_text is too short (<50 chars) with helpful error message

### Frontend Integration for User Story 2

- [ ] T050 [US2] Create frontend/src/hooks/useTextSelection.ts hook to capture DOM selections
  - Listen for mouseup events on document
  - Use window.getSelection() to get selected text
  - Extract metadata from parent elements (data-chapter, data-section attributes)
  - Validate selection length (minimum 50 chars)
  - Return selectedText and metadata
- [ ] T051 [US2] Create src/components/ContextSelector.tsx component to display selected text and mode toggle
  - Show selected text snippet with character count
  - Toggle button to switch between full/selected mode
  - Clear selection button
  - Visual indicator of current mode
- [ ] T052 [US2] Integrate useTextSelection hook in ChatWidget/index.tsx
  - Pass selected_text and selected_metadata to API when mode is 'selected'
  - Display context mode indicator in chat interface
  - Show warning if selection is too short
- [ ] T053 [US2] Update Docusaurus markdown pages to include data attributes (data-chapter, data-section) in section wrappers

### Validation for User Story 2

- [ ] T054 [US2] Test selected-text mode with curl/Postman (send selected_text and selected_metadata in request body)
- [ ] T055 [US2] Manual validation against acceptance scenarios:
  - Select "TF2 Transformations" section → Ask "How do coordinate frames work?" → Verify answer references only that section
  - Same selection → Ask about unrelated topic → Verify "cannot answer from selected content" response
  - Clear selection → Ask same question → Verify answer now references full textbook
- [ ] T056 [US2] Test edge case: Select very short text (1 sentence) → Verify warning displayed

**Checkpoint**: User Stories 1 AND 2 both work independently - students can use either full or selected-text modes

---

## Phase 5: User Story 3 - View Source Citations (Priority: P3)

**Goal**: Build trust in chatbot answers by providing clickable citations that navigate directly to referenced textbook sections

**Independent Test**: 
1. Ask 5 questions from different chapters
2. Verify each answer includes at least one citation
3. Click each citation link
4. Verify navigation to correct textbook section (page scrolls to exact heading)
5. Verify citations are ordered by relevance score

### Services for User Story 3

- [ ] T057 [P] [US3] Enhance backend/app/services/citation_resolver.py to generate precise URL fragments matching Docusaurus heading IDs
- [ ] T058 [P] [US3] Add citation ranking logic in chat.py to order citations by relevance_score descending

### Frontend Integration for User Story 3

- [ ] T059 [US3] Enhance src/components/CitationLink.tsx to handle navigation
  - Render citation as clickable link with chapter, section, page info
  - Show relevance score on hover (optional)
  - Handle click to navigate within Docusaurus (use window.location.hash or React Router)
  - Smooth scroll to target section
- [ ] T060 [US3] Update ChatWidget message rendering to parse and display citations from AnswerResponse
  - Render citations as list below answer text
  - Use CitationLink component for each citation
  - Display citation format: "[Chapter X, Section Y] - Relevance: 94%"
- [ ] T061 [US3] Add text_snippet to Citation model (first 100-200 chars of cited chunk) for preview on hover

### Validation for User Story 3

- [ ] T062 [US3] Manual validation against acceptance scenarios:
  - Ask question about inverse kinematics → Verify citation in format "[Chapter 11, Humanoid Kinematics]" is clickable
  - Click citation → Verify navigation to /week-11-12/humanoid-kinematics#inverse-kinematics and scroll to section
  - Ask question drawing from multiple sections → Verify all sections cited in relevance order
- [ ] T063 [US3] Test citation URL generation for all textbook sections (iterate through docs/ files, verify URL format correctness)

**Checkpoint**: All user stories (US1, US2, US3) are independently functional - complete chatbot experience delivered

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

### Documentation & Deployment

- [ ] T064 [P] Create backend/README.md with setup instructions, API documentation, environment variables
- [ ] T065 [P] Create frontend/README.md with component documentation and integration guide
- [ ] T066 [P] Update root README.md to reference chatbot feature and quickstart.md
- [ ] T067 [P] Create .github/workflows/backend-deploy.yml for Railway deployment automation
- [ ] T068 [P] Create .github/workflows/frontend-deploy.yml for GitHub Pages deployment (if separate from main Docusaurus workflow)

### Error Handling & Edge Cases

- [ ] T069 Add comprehensive error handling for embedding API rate limits (daily quota of 1500 requests)
  - Implement fallback to Sentence Transformers
  - Display user-friendly message: "High demand - using local embeddings"
- [ ] T070 Handle Qdrant connection timeouts with retry logic (max 3 retries with exponential backoff)
- [ ] T071 Handle Neon Postgres cold start delays (auto-suspend after inactivity)
  - Add keep-alive ping every 5 minutes
  - Show loading state during cold start
- [ ] T072 Implement conversation cleanup job in backend/scripts/cleanup_db.py
  - Delete conversations older than 30 days
  - Cascade delete questions and answers
  - Log cleanup statistics

### Performance Optimization

- [ ] T073 Add response caching for frequently asked questions
  - Use in-memory cache (LRU with 1-hour TTL)
  - Cache key: sha256(question_text + context_mode)
  - Invalidate on textbook updates
- [ ] T074 Optimize Qdrant search parameters (adjust HNSW ef_construct for speed/accuracy balance)
- [ ] T075 Add database connection pooling configuration in postgres_service.py (max 20 connections)
- [ ] T076 Implement batch embedding generation for indexing (process 10 chunks per API call)

### Monitoring & Observability

- [ ] T077 [P] Add structured logging for all API requests (request ID, user IP, processing time, errors)
- [ ] T078 [P] Implement metrics endpoint GET /v1/metrics (optional, for future Prometheus integration)
- [ ] T079 Add health check details to /v1/health endpoint
  - Gemini API status (test with simple prompt)
  - Qdrant collection count and storage usage
  - Postgres connection pool status
  - Last successful embedding API call timestamp

### Security Hardening

- [ ] T080 Validate and sanitize all user inputs (question_text, selected_text)
  - Prevent SQL injection (already handled by parameterized queries)
  - Prevent XSS attacks (React auto-escaping)
  - Limit question length strictly (max 1000 chars)
- [ ] T081 Add request ID to all logs and responses for traceability
- [ ] T082 Implement IP-based rate limiting middleware in main.py
- [ ] T083 Add HTTPS enforcement in production (handled by Railway/GitHub Pages)

### Testing & Validation

- [ ] T084 Run quickstart.md validation end-to-end
  - Follow setup steps from scratch (fresh environment)
  - Verify all prerequisites are documented correctly
  - Time setup process (target: 45-60 minutes)
  - Document any issues or missing steps
- [ ] T085 Perform load testing with 50 concurrent users
  - Verify <5 second response time maintained
  - Monitor rate limit behavior
  - Check for memory leaks or resource exhaustion
- [ ] T086 Validate all acceptance criteria from spec.md
  - SC-001: 90%+ relevant answers (test with 50 questions)
  - SC-002: <5 second response for 95%+ queries (measure with load test)
  - SC-003: All answers have at least one citation (automated check)
  - SC-004: Selected-text mode works (manual test scenarios)
  - SC-005: 85%+ out-of-scope questions identified (test with 20 non-textbook questions)
  - SC-006: Conversation history maintained across 10+ turns (integration test)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - MVP deliverable
- **User Story 2 (Phase 4)**: Depends on Foundational (Phase 2) - Can start in parallel with US1 if team capacity allows, but US1 is higher priority
- **User Story 3 (Phase 5)**: Depends on Foundational (Phase 2) and partially on US1 (citation data structure) - Enhances existing functionality
- **Polish (Phase 6)**: Depends on desired user stories being complete - Can start after US1 for MVP, or wait for all stories

### User Story Dependencies

- **User Story 1 (P1 - MVP)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Extends US1 but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Relies on citation structure from US1 but adds display/navigation only

### Critical Path (MVP)

1. Phase 1: Setup (T001-T008) - ~2-3 hours
2. Phase 2: Foundational (T009-T022) - ~8-10 hours
3. Phase 3: User Story 1 (T023-T043) - ~20-25 hours
4. Phase 6 (selected): T064, T066, T084 (documentation & validation) - ~3-4 hours

**Total MVP Estimate**: 35-45 hours

### Parallel Opportunities

**Phase 1 (Setup)**: T003, T004, T005, T006, T007 can run in parallel after T001-T002

**Phase 2 (Foundational)**: 
- T010, T011, T014, T015, T017, T020 can all run in parallel (different services/files)
- T013 depends on T012 (database schema before service)
- T016 depends on T014, T015 (embeddings before vector store)

**Phase 3 (User Story 1)**:
- Models: T023, T024, T025 can run in parallel
- Services: T026, T027 can run after models, in parallel with each other
- T028, T029 extend existing services - can run in parallel
- Frontend: T033, T034, T035 can run in parallel after T030-T032 complete

**Phase 4 (User Story 2)**:
- T044, T045 can run in parallel
- T050, T051, T052 (frontend tasks) can run in parallel

**Phase 5 (User Story 3)**:
- T057, T058, T061 can run in parallel
- T059, T060 (frontend tasks) can run in parallel

**Phase 6 (Polish)**:
- T064, T065, T066 (documentation) can run in parallel
- T067, T068 (CI/CD) can run in parallel
- T077, T078, T079 (monitoring) can run in parallel

---

## Parallel Example: User Story 1 Core Implementation

After models are complete (T023-T025), these can run simultaneously:

```bash
# Launch service implementations in parallel:
Task T026: "Implement backend/app/services/gemini_service.py"
Task T027: "Implement backend/app/services/citation_resolver.py"

# After services, launch API endpoint and frontend in parallel:
Task T030: "Implement POST /v1/chat/ask endpoint in backend/app/api/routes/chat.py"
Task T033: "Create src/components/ChatWidget/index.tsx with chat UI"
Task T034: "Create src/components/ChatWidget/styles.css"
Task T035: "Create src/components/CitationLink.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only) - RECOMMENDED

**Timeline**: 2-3 weeks for one developer

1. **Week 1**: Complete Phase 1 (Setup) + Phase 2 (Foundational)
   - Days 1-2: Setup backend and frontend structure (T001-T008)
   - Days 3-5: Build foundational services (T009-T022)
   - Checkpoint: Run migrate_db.py and index_textbook.py successfully

2. **Week 2**: Complete Phase 3 (User Story 1) - Backend
   - Days 1-2: Build data models and services (T023-T029)
   - Days 3-4: Implement API endpoints and error handling (T030-T032)
   - Checkpoint: Test API with curl, verify health endpoint

3. **Week 3**: Complete Phase 3 (User Story 1) - Frontend + Polish
   - Days 1-2: Build ChatWidget components (T033-T037)
   - Day 3: Setup and validation (T038-T042)
   - Day 4: Manual testing and bug fixes (T043)
   - Day 5: Documentation and deployment (T064-T066, T084)
   - **STOP and DEMO MVP**

### Incremental Delivery (All User Stories)

**Timeline**: 4-5 weeks for one developer

1. **Weeks 1-3**: MVP (User Story 1) as above
2. **Week 4**: Add User Story 2 (Selected-Text Mode)
   - Days 1-2: Backend implementation (T044-T049)
   - Days 3-4: Frontend implementation (T050-T053)
   - Day 5: Testing and validation (T054-T056)
   - **Demo US1 + US2**

3. **Week 5**: Add User Story 3 (Citations) + Polish
   - Days 1-2: Citation enhancements (T057-T061)
   - Day 3: Testing (T062-T063)
   - Days 4-5: Polish and production readiness (Phase 6 selected tasks)
   - **Final Demo: Complete Feature**

### Parallel Team Strategy

With 2-3 developers:

1. **All**: Complete Setup + Foundational together (Week 1)
2. **Once Foundational is done:**
   - Developer A: User Story 1 backend (T023-T032)
   - Developer B: User Story 1 frontend (T033-T037)
   - Developer C: User Story 2 prep (can start T044-T047 in parallel)
3. **Integration**: Merge US1, validate, then proceed with US2 and US3 in parallel

---

## Task Count Summary

- **Phase 1 (Setup)**: 8 tasks
- **Phase 2 (Foundational)**: 14 tasks (CRITICAL - blocks all stories)
- **Phase 3 (User Story 1 - MVP)**: 21 tasks
- **Phase 4 (User Story 2)**: 13 tasks
- **Phase 5 (User Story 3)**: 7 tasks
- **Phase 6 (Polish)**: 23 tasks

**Total Tasks**: 86 tasks

**MVP Scope** (Phases 1-3 + minimal Phase 6): 35-40 tasks (~35-45 hours)
**Full Feature** (All Phases): 86 tasks (~60-80 hours)

---

## Suggested MVP Scope

**Include:**
- Phase 1: Setup (all tasks)
- Phase 2: Foundational (all tasks) 
- Phase 3: User Story 1 (all tasks)
- Phase 6: T064, T066, T069, T070, T071, T080-T081, T084 (essential polish)

**Defer to Post-MVP:**
- Phase 4: User Story 2 (selected-text mode)
- Phase 5: User Story 3 (enhanced citations)
- Phase 6: Performance optimization, caching, advanced monitoring

**Rationale**: User Story 1 delivers the core value proposition (instant answers from textbook with citations). US2 and US3 are valuable enhancements but not essential for initial launch. Students can start benefiting immediately from MVP.

---

## Notes

- **[P] tasks**: Different files, no dependencies - safe to parallelize
- **[Story] label**: Maps each task to specific user story for traceability and independent testing
- **File paths**: All paths are explicit (backend/app/..., src/components/...) for immediate action
- **Checkpoints**: Validate each user story independently before proceeding
- **Manual testing**: Acceptance scenarios from spec.md guide validation at each checkpoint
- **No automated tests**: Per specification, focus is on implementation and manual validation
- **Free-tier constraints**: Rate limiting and fallback strategies built into foundational phase
- **Constitution compliance**: Documentation tasks (T064-T066) fulfill Technical Depth and Deployment Readiness requirements

---

## Validation Checklist (Final)

Before considering feature complete, verify:

- [ ] All 3 user stories pass their independent tests
- [ ] quickstart.md can be followed successfully (T084)
- [ ] All 6 success criteria from spec.md are met (T086)
- [ ] Backend deploys to Railway successfully
- [ ] Frontend integrates with Docusaurus on GitHub Pages
- [ ] Rate limiting protects free-tier API quotas
- [ ] Error messages are user-friendly (no technical jargon)
- [ ] Citations navigate to correct textbook sections
- [ ] Conversation history persists across page navigation
- [ ] Mobile responsive (chat widget works on phones/tablets)
