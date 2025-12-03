# Implementation Plan: Integrated RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build an integrated RAG (Retrieval-Augmented Generation) chatbot that answers student questions from the Physical AI & Humanoid Robotics textbook using free AI models. The system uses FastAPI backend with Google Gemini API (free tier) for conversational AI, Google's text-embedding-004 model for vector embeddings, Qdrant for vector storage, and Neon Postgres for conversation history. The chatbot supports two modes: full-textbook QA and selected-text-only QA. ChatKit UI will provide the frontend interface. All citations link back to specific textbook sections, and answers are limited to 500 words maximum.

## Technical Context

**Language/Version**: Python 3.11+  
**Primary Dependencies**: 
- Backend: FastAPI 0.104+, Uvicorn, Pydantic v2
- AI/ML: google-generativeai 0.3.2 (Gemini API SDK), sentence-transformers 2.2.2 (fallback embeddings)
- Storage: qdrant-client 1.7.0 (vector DB), psycopg2-binary 2.9.9 (Postgres client), neon-serverless (Postgres provider)
- Frontend: ChatKit UI (OpenAI SDK), React, TypeScript

**Storage**: 
- Qdrant Cloud (free tier) - vector embeddings (768-dimensional for text-embedding-004)
- Neon Postgres (free tier) - conversation history, user sessions, metadata

**Testing**: pytest 7.4+, pytest-asyncio for async tests, httpx for API testing  

**Target Platform**: 
- Backend: Cloud-hosted Linux server (Docker container on Railway/Render/Fly.io)
- Frontend: Static site deployed alongside Docusaurus (GitHub Pages)

**Project Type**: Web application (separate frontend + backend)

**Performance Goals**: 
- Answer generation: <5 seconds for 95% of queries
- Embedding generation: <1 second per query
- Vector search: <500ms for top-k retrieval (k=5)
- API throughput: 100 concurrent users sustained

**Constraints**: 
- Free tier limits: Gemini API (60 requests/minute), embedding API (1500 requests/day)
- Answer length: max 500 words
- Conversation history: retain last 10 turns per session
- Rate limiting: 10 requests/minute per user IP
- Memory: <512MB RAM per container instance

**Scale/Scope**: 
- Textbook content: ~300 pages, ~150 sections, ~50,000 tokens total
- Expected users: 100-500 students per semester
- Embedding chunks: ~500-1000 chunks (300-500 tokens each)
- Concurrent sessions: 50 peak

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Educational Excellence ✅
- **Compliance**: RAG chatbot enhances learning by providing instant, contextual answers from textbook content
- **Technical Accuracy**: Uses research-backed AI models (Gemini, embeddings) with proper citation mechanisms
- **Clear Explanations**: 500-word limit ensures concise, digestible answers suitable for students

### II. Curriculum Alignment ✅
- **Compliance**: Chatbot serves the 13-week curriculum by answering questions from all modules
- **No Deviation**: Does not alter course structure; purely additive learning support feature

### III. Technical Depth ⚠️ CONDITIONAL PASS
- **Compliance**: Provides working code examples in backend setup documentation
- **Condition**: Must include complete setup instructions for Google Gemini API, Qdrant, and Neon Postgres in quickstart.md

### IV. Docusaurus Best Practices ✅
- **Compliance**: ChatKit UI integrates seamlessly with existing Docusaurus site
- **Responsive Design**: Frontend will follow mobile-first approach with accessibility standards

### V. AI-Native Design ✅ EXEMPLARY
- **Compliance**: This feature IS the AI-native enhancement
- **RAG Architecture**: Structured embeddings with semantic search enable future personalization

### VI. Deployment Readiness ⚠️ CONDITIONAL PASS
- **Compliance**: Backend deployable to cloud platforms; frontend to GitHub Pages
- **Condition**: Must document deployment configuration and CI/CD pipeline in quickstart.md

### VII. Code Quality ✅
- **Compliance**: All code will be tested with pytest; API contracts documented in OpenAPI
- **Best Practices**: Follow FastAPI conventions, type hints, async patterns

### VIII. Maintainability ✅
- **Compliance**: Clear separation of concerns (embedding service, vector store, API routes)
- **Extensibility**: Modular design allows future model swaps (Gemini → other LLMs)

**Overall Status**: ✅ PASS with 2 conditions to fulfill in Phase 1 (quickstart.md documentation)

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
│   ├── __init__.py
│   ├── main.py                      # FastAPI app entry point
│   ├── config.py                    # Environment configuration
│   ├── models/
│   │   ├── __init__.py
│   │   ├── conversation.py          # Conversation, Question, Answer models
│   │   ├── textbook.py              # TextbookChunk, Citation models
│   │   └── context.py               # SelectedContext model
│   ├── services/
│   │   ├── __init__.py
│   │   ├── gemini_service.py        # Gemini API wrapper for chat
│   │   ├── embeddings.py            # Google embedding API client
│   │   ├── embeddings_local.py      # Sentence Transformers fallback
│   │   ├── vector_store.py          # Qdrant client and search
│   │   ├── postgres_service.py      # Neon Postgres client
│   │   ├── rate_limiter.py          # Rate limiting logic
│   │   ├── indexer.py               # Textbook content indexing
│   │   └── citation_resolver.py     # Citation generation and linking
│   ├── api/
│   │   ├── __init__.py
│   │   ├── routes/
│   │   │   ├── __init__.py
│   │   │   ├── chat.py              # POST /chat/ask endpoint
│   │   │   ├── history.py           # GET /chat/history/{session_id}
│   │   │   └── health.py            # GET /health
│   │   └── dependencies.py          # FastAPI dependency injection
│   └── utils/
│       ├── __init__.py
│       ├── logger.py                # Structured logging
│       └── exceptions.py            # Custom exception classes
├── tests/
│   ├── __init__.py
│   ├── conftest.py                  # Pytest fixtures
│   ├── unit/
│   │   ├── test_gemini_service.py
│   │   ├── test_embeddings.py
│   │   └── test_vector_store.py
│   ├── integration/
│   │   ├── test_chat_api.py
│   │   └── test_indexer.py
│   └── contract/
│       └── test_openapi_spec.py
├── scripts/
│   ├── index_textbook.py            # CLI to index textbook content
│   └── migrate_db.py                # Database migration script
├── requirements.txt
├── requirements-dev.txt
├── .env.example
├── Dockerfile
└── README.md

frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface.tsx        # Main chat UI component
│   │   ├── MessageList.tsx          # Message display component
│   │   ├── InputBox.tsx             # Question input component
│   │   ├── CitationLink.tsx         # Clickable citation component
│   │   └── ContextSelector.tsx      # Text selection UI component
│   ├── services/
│   │   ├── chatService.ts           # API client for backend
│   │   └── textSelection.ts         # Text selection handling
│   ├── hooks/
│   │   ├── useChatSession.ts        # Chat state management
│   │   └── useTextSelection.ts      # Selection state management
│   ├── types/
│   │   └── chat.ts                  # TypeScript interfaces
│   └── App.tsx                      # Root component with ChatKit UI
├── public/
├── package.json
├── tsconfig.json
└── README.md
```

**Structure Decision**: Web application structure (Option 2) selected. The backend and frontend are separated to enable:
1. Independent deployment: Backend on cloud platforms (Railway/Render), frontend integrated with Docusaurus on GitHub Pages
2. Technology separation: Python/FastAPI for AI/ML backend, React/TypeScript for interactive UI
3. API-first design: Backend exposes RESTful endpoints consumable by frontend or other clients
4. Scalability: Backend can scale independently based on AI workload demands

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations** - All constitutional principles are met. The conditional passes (Technical Depth, Deployment Readiness) will be fulfilled in quickstart.md documentation.

---

## Phase 0: Research (COMPLETED ✅)

**Objective**: Resolve all NEEDS CLARIFICATION items from Technical Context and validate technology choices.

**Output**: `research.md` documenting decisions, rationales, and alternatives for:
1. Google Gemini API 1.5 Flash - free conversational AI
2. Google text-embedding-004 - free embedding model with 768-dim vectors
3. Qdrant Cloud - free vector database (1GB storage)
4. Neon Postgres - free serverless database (0.5GB storage)
5. FastAPI architecture - async patterns for 100+ concurrent users
6. ChatKit UI implementation - custom React chat component using @chatscope library
7. Text selection strategy - DOM Range API for highlighting
8. Textbook chunking - 300-500 tokens per chunk with 50-token overlap
9. Citation resolution - Docusaurus URL generation from chunk metadata
10. Deployment architecture - Railway (backend) + GitHub Pages (frontend)

**Status**: ✅ All unknowns resolved. See `research.md` for complete findings.

---

## Phase 1: Design & Contracts (COMPLETED ✅)

### Data Model (`data-model.md`)

**Entities Defined**:
1. **Conversation** - Chat session with context mode (full/selected)
2. **Question** - User query with optional selected text context
3. **Answer** - Generated response with citations and confidence
4. **TextbookChunk** - Embedded textbook segment (768-dim Google vectors + 384-dim fallback)
5. **SelectedContext** - User-highlighted text (embedded in Question)

**Relationships**:
- Conversation 1:N Question
- Question 1:1 Answer
- Answer N:M TextbookChunk (via citations)

**Validation Rules**: 35+ rules covering text length, timestamp ordering, rate limits, citation integrity

**Storage**: 
- Qdrant: TextbookChunk (vectors + payload)
- Neon Postgres: Conversation, Question, Answer

**Status**: ✅ Complete. See `data-model.md` for entity schemas, state transitions, and sample data.

### API Contracts (`contracts/openapi.yaml`)

**Endpoints**:
1. **POST /v1/chat/ask** - Submit question, get answer with citations
2. **GET /v1/chat/history/{session_id}** - Retrieve conversation history
3. **GET /v1/health** - Health check for dependencies

**Request/Response Models**:
- `QuestionRequest` - Validated user input (10-1000 chars, context mode, optional selection)
- `AnswerResponse` - Generated answer (max 500 words, 1-5 citations, confidence score)
- `HistoryResponse` - Ordered messages with pagination
- `ErrorResponse` - Standardized error format with retry hints

**Status**: ✅ Complete OpenAPI 3.0.3 spec. Auto-generates FastAPI docs at `/docs`.

### Quick Start Guide (`quickstart.md`)

**Sections**:
1. Prerequisites - Free account setup (Google AI Studio, Qdrant, Neon)
2. Backend Setup - Environment config, database init, textbook indexing
3. Frontend Integration - Chat widget component, Docusaurus theming
4. Deployment - Railway (backend), GitHub Pages (frontend)
5. Troubleshooting - Common issues and solutions
6. Maintenance - Re-indexing, cleanup, monitoring

**Estimated Setup Time**: 45-60 minutes

**Status**: ✅ Complete step-by-step guide. Fulfills Constitutional conditions for Technical Depth and Deployment Readiness.

---

## Phase 2: Tasks (NOT STARTED - Run `/sp.tasks`)

**Next Command**: `/sp.tasks` to generate testable implementation tasks from this plan.

**Expected Tasks**:
1. Backend infrastructure setup
2. Database schema implementation
3. Gemini service integration
4. Qdrant vector store implementation
5. API endpoint implementation
6. Frontend chat component
7. Text selection feature
8. Citation linking
9. Testing (unit, integration, contract)
10. Deployment configuration

---

## Key Design Decisions

### 1. Free-Tier Architecture

**Decision**: Use only free services with no credit card required.

**Rationale**: 
- Lowers barrier to entry for students and educators
- Sufficient capacity for 100-500 students per semester
- Graceful degradation when rate limits hit

**Tradeoffs**:
- Rate limits require queuing and backoff strategies
- Cannot handle viral/high-traffic scenarios
- Fallback embeddings (Sentence Transformers) have lower quality

**Alternatives Considered**:
- Paid OpenAI API (rejected: cost prohibitive for educational use)
- Self-hosted LLMs (rejected: complex infrastructure, GPU requirements)

### 2. Dual Embedding Strategy

**Decision**: Primary Google embeddings (768-dim) with Sentence Transformer fallback (384-dim).

**Rationale**:
- Google embedding API has daily quota (1500 requests/day)
- Fallback ensures continuous service during high usage
- Qdrant supports named vectors for multi-model storage

**Tradeoffs**:
- Increased storage complexity (two vector sets per chunk)
- Slight quality degradation in fallback mode

**Implementation**:
```python
# Try Google embeddings first
try:
    embedding = google_embeddings.embed(text, task_type="retrieval_query")
    vector_name = "google"
except RateLimitError:
    # Fallback to local model
    embedding = local_embeddings.embed(text)
    vector_name = "local"

# Search Qdrant
results = qdrant.search(
    collection_name="textbook_chunks",
    query_vector=(vector_name, embedding),
    limit=5
)
```

### 3. Selected-Text Mode Architecture

**Decision**: Store selected text in Question entity (JSONB), not as separate TextbookChunk.

**Rationale**:
- Selected text is transient (unique to each question)
- No need for persistent embedding (generated on-the-fly)
- Simpler data model (no separate SelectedChunk entity)

**Tradeoffs**:
- Cannot cache embeddings for frequently selected texts
- Slightly slower query processing (need to embed selection each time)

**Flow**:
1. User highlights text in UI → DOM Range API captures text + metadata
2. Frontend sends `selected_text` and `selected_metadata` in QuestionRequest
3. Backend embeds selected text, searches only within matching chunks (filtered by chapter/section)
4. Answer generated from filtered context only

### 4. Citation Linking Strategy

**Decision**: Generate Docusaurus URLs from chunk metadata (file path + heading).

**Rationale**:
- Docusaurus has predictable URL structure: `/week-##-##/topic#heading-slug`
- No need for external link registry or database table
- Citations automatically work across textbook updates (as long as headings stable)

**URL Generation Algorithm**:
```python
def generate_citation_link(chunk_metadata):
    # Example: docs/week-11-12/humanoid-kinematics.md
    file_path = chunk_metadata["file_path"]
    heading = chunk_metadata["heading"]
    
    # Remove docs/ prefix and .md suffix
    url_path = file_path.replace("docs/", "").replace(".md", "")
    # Result: week-11-12/humanoid-kinematics
    
    # Slugify heading: "Inverse Kinematics" → "inverse-kinematics"
    fragment = heading.lower().replace(" ", "-").replace("&", "and")
    
    return f"/{url_path}#{fragment}"
```

### 5. Rate Limiting Strategy

**Decision**: Two-tier rate limiting (per-session and per-IP).

**Rationale**:
- Protects free-tier API quotas from abuse
- Per-session limit prevents accidental spam from single user
- Per-IP limit prevents distributed abuse

**Limits**:
- Per session: 10 requests/minute (aligned with Gemini free tier: 15 RPM)
- Per IP: 50 requests/hour (prevents multi-session abuse from same client)

**Implementation**: Token bucket algorithm with Redis (optional) or in-memory cache.

### 6. Conversation Retention Policy

**Decision**: Auto-delete conversations after 30 days of inactivity.

**Rationale**:
- Prevents unbounded database growth
- Sufficient for semester-long course (13 weeks)
- Complies with data minimization principles

**Cleanup Strategy**:
```sql
-- Scheduled job (daily at 2 AM)
DELETE FROM conversations
WHERE updated_at < NOW() - INTERVAL '30 days'
RETURNING id;
-- Cascade delete will remove associated questions and answers
```

---

## Non-Functional Requirements (NFRs)

### Performance

| Metric | Target | Rationale |
|--------|--------|-----------|
| Answer generation latency (p95) | < 5 seconds | User patience threshold for chat |
| Embedding generation | < 1 second | Google API typical response time |
| Vector search (top-5) | < 500ms | Qdrant HNSW index performance |
| API throughput | 100 concurrent users | Free-tier capacity estimate |

### Reliability

| Metric | Target | Rationale |
|--------|--------|-----------|
| API uptime | > 99% | Railway/Render SLA |
| Answer relevance | > 90% | Measured by citation accuracy |
| Error rate | < 5% | Excluding rate limit errors |

### Scalability

| Dimension | Current | Future (Paid Tiers) |
|-----------|---------|---------------------|
| Textbook chunks | ~500-1000 | 10,000+ (multi-textbook support) |
| Concurrent users | 50 peak | 500+ with load balancing |
| Conversations/semester | 1,000-5,000 | 50,000+ with database sharding |

### Security

| Concern | Mitigation |
|---------|-----------|
| API key exposure | Environment variables, never in code |
| SQL injection | Parameterized queries (psycopg2) |
| XSS attacks | React auto-escaping, Content Security Policy |
| DDoS | Rate limiting, Cloudflare (if needed) |
| PII leakage | No user authentication, minimal data collection |

---

## Testing Strategy

### Unit Tests (pytest)

**Coverage Target**: > 80%

**Key Test Files**:
- `test_gemini_service.py` - Mock Gemini API calls, test error handling
- `test_embeddings.py` - Verify embedding generation, fallback logic
- `test_vector_store.py` - Mock Qdrant client, test search filtering
- `test_rate_limiter.py` - Validate token bucket algorithm
- `test_citation_resolver.py` - URL generation edge cases

**Example**:
```python
def test_gemini_service_rate_limit_retry():
    service = GeminiService(api_key="test")
    service.client = MagicMock()
    service.client.generate_content.side_effect = [
        RateLimitError("quota exceeded"),
        Mock(text="answer")
    ]
    
    result = service.generate_answer("question", ["context"])
    
    assert result == "answer"
    assert service.client.generate_content.call_count == 2
```

### Integration Tests (pytest + httpx)

**Coverage Target**: > 60%

**Key Test Files**:
- `test_chat_api.py` - End-to-end API flows (ask question, get history)
- `test_indexer.py` - Textbook chunking and embedding pipeline
- `test_database.py` - Postgres CRUD operations

**Example**:
```python
async def test_chat_ask_full_textbook_mode(async_client):
    response = await async_client.post("/v1/chat/ask", json={
        "session_id": "test-123",
        "question": "What is ROS2?",
        "context_mode": "full"
    })
    
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert len(data["citations"]) >= 1
    assert data["confidence"] > 0.5
```

### Contract Tests (Schemathesis)

**Purpose**: Validate OpenAPI spec compliance

**Example**:
```python
import schemathesis

schema = schemathesis.from_uri("http://localhost:8000/v1/openapi.json")

@schema.parametrize()
def test_api_contract(case):
    response = case.call()
    case.validate_response(response)
```

### Frontend Tests (Jest + React Testing Library)

**Coverage Target**: > 70%

**Key Test Files**:
- `ChatWidget.test.tsx` - Component rendering, message handling
- `useChatSession.test.ts` - State management, API integration
- `useTextSelection.test.ts` - DOM selection capture

---

## Deployment Architecture

### Production Environment

```
┌─────────────────────────────────────────────────┐
│              GitHub Pages (Frontend)            │
│  - Static Docusaurus site                       │
│  - React ChatWidget component                   │
│  - HTTPS enabled                                │
└──────────────────┬──────────────────────────────┘
                   │
                   │ HTTPS requests
                   │
                   ▼
┌─────────────────────────────────────────────────┐
│           Railway/Render (Backend)              │
│  - FastAPI app (4 Uvicorn workers)              │
│  - Auto-scaling: 1-3 instances                  │
│  - HTTPS + custom domain                        │
│  - Environment variables (secrets)              │
└──┬─────────┬─────────────┬──────────────────────┘
   │         │             │
   │         │             │ API calls
   │         │             │
   ▼         ▼             ▼
┌────────┐ ┌────────────┐ ┌─────────────────────┐
│ Gemini │ │   Qdrant   │ │   Neon Postgres     │
│  API   │ │   Cloud    │ │   (Serverless)      │
│ (Free) │ │   (Free)   │ │   (Free)            │
└────────┘ └────────────┘ └─────────────────────┘
```

### CI/CD Pipeline (GitHub Actions)

**Backend** (Railway auto-deploy):
```yaml
# .github/workflows/backend-deploy.yml
name: Deploy Backend
on:
  push:
    branches: [main]
    paths: ['backend/**']

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Deploy to Railway
        uses: railway/deploy@v1
        with:
          service-token: ${{ secrets.RAILWAY_TOKEN }}
```

**Frontend** (GitHub Pages):
```yaml
# .github/workflows/frontend-deploy.yml
name: Deploy Frontend
on:
  push:
    branches: [main]
    paths: ['src/**', 'docs/**', 'docusaurus.config.js']

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

---

## Monitoring & Observability

### Logs (Structured JSON)

```python
import logging
import json
from datetime import datetime

logger = logging.getLogger(__name__)

def log_chat_request(session_id, question, processing_time_ms):
    logger.info(json.dumps({
        "event": "chat_request",
        "session_id": session_id,
        "question_length": len(question),
        "processing_time_ms": processing_time_ms,
        "timestamp": datetime.utcnow().isoformat()
    }))
```

### Metrics (Optional: Prometheus + Grafana)

**Key Metrics**:
- `chat_requests_total` - Counter of total requests
- `chat_request_duration_seconds` - Histogram of latencies
- `gemini_api_errors_total` - Counter of API failures
- `qdrant_search_duration_seconds` - Vector search latency
- `active_conversations_gauge` - Current active sessions

### Alerts (Railway/Render)

- CPU usage > 80% for 5 minutes
- Memory usage > 90%
- Error rate > 10% over 15 minutes
- Gemini API rate limit exceeded

---

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Gemini API rate limit exhaustion | High | Medium | Implement queuing, show "high demand" message |
| Qdrant Cloud downtime | Low | High | Cache frequent queries, fallback error message |
| Neon Postgres cold start delay | Medium | Low | Keep-alive ping every 5 minutes |
| Textbook content errors (broken links) | Medium | Medium | Automated link checking in CI/CD |
| User spam/abuse | Medium | Low | Rate limiting + IP blocking |
| CORS misconfiguration | Low | High | Automated contract tests |

---

## Future Enhancements (Out of Scope for v1.0)

1. **Multi-language support** - Translate textbook + chatbot to Spanish, Chinese
2. **Voice input** - Speech-to-text for questions
3. **Diagram understanding** - Multimodal Gemini for image-based questions
4. **Personalized learning** - Track user knowledge gaps, adaptive questioning
5. **Instructor dashboard** - Analytics on common student questions
6. **Collaborative study** - Shared chat sessions for group learning
7. **Export conversations** - PDF/Markdown download of chat history
8. **Advanced RAG** - Hybrid search (semantic + keyword), re-ranking

---

## Acceptance Criteria (from Spec)

| ID | Criterion | Verification Method |
|----|-----------|---------------------|
| SC-001 | Users receive relevant answers to 90%+ of textbook questions | Manual testing with 50 sample questions across all chapters |
| SC-002 | Answer generation completes within 5 seconds for 95% of queries | Performance testing with concurrent load |
| SC-003 | Each answer includes at least one accurate citation | Automated test verifying citation presence and link validity |
| SC-004 | Users can successfully use selected-text mode | Integration test with mocked DOM selection |
| SC-005 | System correctly identifies out-of-scope questions 85%+ of the time | Test with 20 non-textbook questions (e.g., "What is quantum computing?") |
| SC-006 | Conversation history maintained across 10+ turns | Integration test with sequential API calls |

---

## Sign-Off

**Plan Status**: ✅ **COMPLETE** (Phase 0 + Phase 1)

**Artifacts Generated**:
1. ✅ `research.md` - All technical unknowns resolved
2. ✅ `data-model.md` - 5 entities, validation rules, state transitions
3. ✅ `contracts/openapi.yaml` - OpenAPI 3.0.3 spec (3 endpoints, 8 schemas)
4. ✅ `quickstart.md` - Step-by-step setup guide (45-60 min)
5. ✅ `plan.md` - This document (comprehensive implementation plan)

**Constitution Re-Check**: ✅ **PASS** (all conditions fulfilled via quickstart.md)

**Next Action**: Run `/sp.tasks` to generate implementation tasks from this plan.

**Branch**: `002-rag-chatbot`  
**Estimated Implementation Time**: 40-60 hours (2-3 weeks for one developer)  
**Priority**: High (enables AI-native learning experience)
