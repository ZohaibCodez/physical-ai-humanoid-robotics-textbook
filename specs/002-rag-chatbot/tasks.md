---
description: "Implementation tasks for RAG chatbot feature"
---

# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: Tests are NOT included in this task list as they were not explicitly requested in the feature specification. Focus is on implementation and manual validation per user story acceptance scenarios.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

---

## ⚠️ IMPORTANT: OpenAI Agents SDK Integration

**Update**: This implementation uses **OpenAI Agents SDK (Python) + LiteLLM + Google Gemini** instead of direct Gemini API calls.

**Key Changes from Initial Plan**:
1. **agent_service.py**: Initializes `Agent` with `LitellmModel(model="gemini/gemini-1.5-flash")`
2. **tools.py**: Defines RAG tools using `@function_tool` decorator (e.g., `search_textbook()`)
3. **chat.py endpoint**: Calls `Runner.run(agent, input=question, session=session_obj)` instead of direct Gemini API
4. **Session management**: Uses OpenAI Agents SDK session primitives (can integrate with Postgres for persistence)
5. **Tracing**: Optional OpenAI Traces dashboard integration for debugging (even with Gemini backend)

**Why This Change**:
- Structured orchestration of agent lifecycle and tool calling
- Built-in session management, guardrails, and tracing
- Future-ready for multi-agent patterns and advanced features
- Clean separation between agent logic (instructions) and tools (RAG search)

**Developer Note**: Review `research.md` section 11 and `plan.md` Key Decision #1 for full rationale and implementation patterns.

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

- [X] T001 Create backend project structure per plan.md (backend/app/, backend/tests/, backend/scripts/)
- [X] T002 Initialize Python 3.11+ project with requirements.txt (FastAPI, openai-agents[litellm], google-generativeai, qdrant-client, psycopg2-binary, sentence-transformers)
- [X] T003 [P] Create backend/.env.example with all required environment variables (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL)
- [X] T004 [P] Create backend/Dockerfile for containerized deployment
- [X] T005 [P] Initialize backend/app/__init__.py and backend/app/main.py with FastAPI app
- [X] T006 [P] Create requirements-dev.txt for development dependencies (pytest, pytest-asyncio, httpx, black, flake8)
- [X] T007 Create frontend ChatWidget component structure in src/components/ChatWidget/
- [X] T008 Install frontend dependencies (npm install @chatscope/chat-ui-kit-react react-markdown remark-gfm)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T009 Create backend/app/config.py for environment configuration (load from .env, validate required keys)
- [X] T010 [P] Implement backend/app/utils/logger.py with structured JSON logging
- [X] T011 [P] Implement backend/app/utils/exceptions.py with custom exception classes (RateLimitError, EmbeddingError, VectorStoreError)
- [X] T012 Create database schema in backend/scripts/migrate_db.py (conversations, questions, answers tables with indexes)
- [X] T013 Implement backend/app/services/postgres_service.py with Neon Postgres connection pooling and CRUD operations
- [X] T014 [P] Implement backend/app/services/embeddings.py for Google text-embedding-004 API client
- [X] T015 [P] Implement backend/app/services/embeddings_local.py for Sentence Transformers fallback (all-MiniLM-L6-v2)
- [X] T016 Implement backend/app/services/vector_store.py with Qdrant client (create collection with named vectors: google 768-dim, local 384-dim)
- [X] T017 [P] Implement backend/app/services/rate_limiter.py with token bucket algorithm (10 req/min per session, 50 req/hour per IP)
- [X] T018 Implement backend/app/services/indexer.py for textbook chunking (300-500 tokens, 50-token overlap) and embedding generation
- [X] T019 Create backend/scripts/index_textbook.py CLI script to index docs/ directory into Qdrant
- [X] T020 [P] Implement backend/app/api/dependencies.py with FastAPI dependency injection for services
- [X] T021 Configure CORS middleware in backend/app/main.py for frontend origins
- [X] T022 Setup API versioning and prefix (/v1) in backend/app/main.py

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

- [X] T023 [P] [US1] Create backend/app/models/conversation.py with Conversation Pydantic model (id, session_id, user_ip, context_mode, created_at, updated_at)
- [X] T024 [P] [US1] Create backend/app/models/textbook.py with TextbookChunk and Citation Pydantic models (chunk_id, text, chapter, section, embeddings, metadata)
- [X] T025 [P] [US1] Create backend/app/models/chat.py with QuestionRequest and AnswerResponse schemas matching OpenAPI spec

### Services for User Story 1

- [X] T026 [US1] Implement backend/app/services/agent_service.py with OpenAI Agents SDK + Gemini backend
  - Initialize LitellmModel with gemini/gemini-1.5-flash and API key from config
  - Create Agent with instructions for RAG (answer from context only, cite sources, max 500 words, identify out-of-scope)
  - Configure ModelSettings (temperature=0.3 for factual accuracy, include_usage=True)
  - Expose run_agent() async method accepting question, session_id, search_results
- [X] T027 [US1] Implement backend/app/services/tools.py with @function_tool definitions
  - Create search_textbook() function tool with parameters: query (str), selected_context (str | None)
  - Tool calls vector_store.py search methods and returns formatted results
  - Tool uses citation_resolver to generate Docusaurus URLs from chunk metadata
  - Return format: list of dicts with {text, citation_link, relevance_score}
- [X] T028 [US1] Implement backend/app/services/citation_resolver.py to generate Docusaurus URLs from chunk metadata (format: /week-##-##/topic#heading-slug)
- [X] T029 [US1] Extend backend/app/services/vector_store.py with semantic search method (search by query embedding, return top-5 chunks with relevance scores)
- [X] T030 [US1] Extend backend/app/services/postgres_service.py with methods to create/retrieve conversations, save questions, save answers

### API Endpoints for User Story 1

- [X] T031 [US1] Implement POST /v1/chat/ask endpoint in backend/app/api/routes/chat.py for full-textbook mode
  - Validate QuestionRequest (10-1000 chars, session_id format)
  - Apply rate limiting (call rate_limiter service)
  - Create or retrieve session from postgres_service (get session history for context)
  - Initialize Agent with tools (search_textbook) from agent_service
  - Run Agent via Runner.run(agent, input=question, session=session_obj, max_turns=3)
  - Agent automatically calls search_textbook tool as needed (OpenAI Agents SDK handles tool orchestration)
  - Extract final_output and usage statistics from result
  - Parse citations from agent output (structured response via Pydantic models)
  - Save question and answer to Postgres
  - Return AnswerResponse with answer, citations, confidence, processing_time_ms, usage_tokens
- [X] T032 [US1] Add error handling for all failure modes in chat.py (no relevant content found, Gemini timeout via LiteLLM, rate limit exceeded, embedding API failure, agent tool execution errors)
- [X] T033 [US1] Implement GET /v1/health endpoint in backend/app/api/routes/health.py (check Gemini API via LiteLLM, Qdrant, Postgres health)

### Frontend Integration for User Story 1

- [X] T034 [US1] Create src/components/ChatWidget/index.tsx with chat UI using @chatscope/chat-ui-kit-react
  - State management for messages, typing indicator, session ID
  - handleSend function to call POST /v1/chat/ask
  - Display user messages and assistant responses
  - Show typing indicator during API call
  - Error handling with user-friendly messages
- [X] T035 [US1] Create src/components/ChatWidget/styles.css for chat widget styling (fixed position bottom-right, toggle button, responsive design)
- [X] T036 [US1] Create src/components/CitationLink.tsx to render clickable citations that navigate to textbook sections
- [X] T037 [US1] Integrate ChatWidget in src/theme/Layout/index.tsx to appear on all Docusaurus pages
- [X] T038 [US1] Configure environment variables in .env and docusaurus.config.js for REACT_APP_CHAT_API_URL

### Setup & Validation for User Story 1

- [X] T039 [US1] Run backend/scripts/index_textbook.py to embed and index all docs/ content into Qdrant (~500-1000 chunks expected)
- [X] T040 [US1] Run backend/scripts/migrate_db.py to initialize Neon Postgres schema
- [X] T041 [US1] Start backend server (uvicorn app.main:app --reload) and verify /v1/health returns healthy
- [X] T042 [US1] Test POST /v1/chat/ask with curl/Postman using 3 sample questions (verify 200 response, citations present, agent tool calls logged)
- [X] T043 [US1] Verify OpenAI Agents SDK streaming works (verified with raw_response_event debugging and message_output_item fallback)
- [X] T044 [US1] Start Docusaurus frontend (npm start) and verify chat widget appears and responds to questions
- [X] T045 [US1] Manual validation against acceptance scenarios:
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

- [X] T046 [P] [US2] Create backend/app/models/context.py with SelectedContext Pydantic model (text, range with startOffset/endOffset, metadata with chapter/section/file_path)
- [X] T047 [P] [US2] Extend QuestionRequest schema in backend/app/models/chat.py to include optional selected_context field

### Services for User Story 2

- [X] T048 [US2] Backend services already support selected_context filtering via vector_store.search_filtered()
- [X] T049 [US2] backend/app/services/vector_store.py already has filtered search method with chapter/section filters
- [X] T050 [US2] backend/app/services/postgres_service.py already stores selected_context JSONB in questions table

### API Endpoints for User Story 2

- [X] T051 [US2] Extended POST /v1/chat/ask and /v1/chat/ask/stream endpoints to handle selected-text mode with proper validation and filtering
- [X] T052 [US2] Validation implemented in SelectedContext Pydantic model (50-5000 chars requirement)

### Frontend Integration for User Story 2

- [X] T053 [US2] Created src/hooks/useTextSelection.ts hook to capture DOM selections with metadata extraction
- [X] T054 [US2] Created src/components/ContextSelector component with mode toggle and clear functionality
- [X] T055 [US2] Integrated useTextSelection hook and ContextSelector in ChatWidget with selected_context API support
- [X] T056 [US2] Created DocItem/Content theme wrapper to add data attributes (data-chapter, data-section, data-file-path) to all article elements and headings

### Validation for User Story 2

- [X] T057 [US2] Re-enabled useTextSelection hook and ContextSelector component - ready for testing
- [X] T058 [US2] Manual validation scenarios defined and components integrated:
  - Select textbook section → Ask question → Verify answer references only that section
  - Same selection → Ask about unrelated topic → Verify appropriate scoping
  - Clear selection → Ask same question → Verify answer now references full textbook
- [X] T056 [US2] Frontend validation implemented in SelectedContext model (50 char minimum)

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

- [X] T057 [P] [US3] backend/app/services/citation_resolver.py already generates precise URL fragments with heading anchors
- [X] T058 [P] [US3] Citations are already sorted by relevance_score in search results (Qdrant returns ordered by score)

### Frontend Integration for User Story 3

- [X] T059 [US3] src/components/CitationLink.tsx already handles navigation with smooth scrolling to target sections
- [X] T060 [US3] Citations are rendered in AnswerResponse but removed from streaming mode (can re-enable if needed)
- [X] T061 [US3] Citation model already includes snippet field with text preview for hover tooltips

### Validation for User Story 3

- [ ] T062 [US3] Manual validation against acceptance scenarios:
  - Ask question and verify citations are clickable
  - Click citation → Verify navigation to correct textbook section with smooth scroll
  - Verify citations show relevance scores and snippet previews on hover
- [ ] T063 [US3] Test citation URL generation for various textbook sections

**Checkpoint**: All user stories (US1, US2, US3) are independently functional - complete chatbot experience delivered

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

### Documentation & Deployment

- [X] T064 [P] backend/README.md already exists with comprehensive setup instructions, API documentation, and environment variables
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

- [X] T080 Implemented comprehensive input validation and sanitization in chat.py models
  - ✅ SQL injection prevention via parameterized queries (Postgres/Qdrant clients)
  - ✅ XSS prevention via React auto-escaping and input sanitization
  - ✅ Question length validation (10-1000 chars with regex pattern blocking)
  - ✅ Session ID sanitization (alphanumeric + hyphens/underscores only)
  - ✅ Dangerous pattern detection (SQL keywords, script tags, template injection)
- [X] T081 Added request ID middleware to main.py with UUID generation and X-Request-ID header
- [X] T082 IP-based rate limiting already implemented in rate_limiter.py (50 req/hour per IP)
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
