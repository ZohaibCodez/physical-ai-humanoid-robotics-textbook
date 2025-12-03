# Data Model: RAG Chatbot

**Feature**: 002-rag-chatbot  
**Date**: 2025-12-03  
**Status**: Phase 1 - Design Complete

## Overview

This document defines the data entities, relationships, validation rules, and state transitions for the RAG chatbot system. The data model supports two primary modes: full-textbook question-answering and selected-text-only question-answering.

---

## Entity Definitions

### 1. Conversation

Represents a chat session between a user and the chatbot.

**Attributes:**
- `id` (UUID, PK): Unique conversation identifier
- `session_id` (String, Unique, Indexed): Client-generated session identifier
- `user_ip` (String, Optional): User IP address for rate limiting
- `context_mode` (Enum): Current context mode
  - Values: `full`, `selected`
  - Default: `full`
- `created_at` (Timestamp): Session start time
- `updated_at` (Timestamp): Last activity time

**Validation Rules:**
- `session_id` must be a valid UUID v4 format
- `context_mode` must be one of: `full`, `selected`
- `user_ip` must be a valid IPv4 or IPv6 address (if provided)
- `created_at` must be <= `updated_at`

**Relationships:**
- Has many `Question` entities (1:N)

**Storage:**
- Neon Postgres: `conversations` table
- Retention: 30 days for inactive sessions

**Indexes:**
- Primary: `id`
- Unique: `session_id`
- Secondary: `created_at` (for cleanup queries)

---

### 2. Question

Represents a user's natural language question within a conversation.

**Attributes:**
- `id` (UUID, PK): Unique question identifier
- `conversation_id` (UUID, FK): Parent conversation
- `question_text` (Text): User's question (max 1000 characters)
- `timestamp` (Timestamp): When question was asked
- `selected_context` (JSONB, Optional): For selected-text mode
  ```json
  {
    "text": "highlighted text from textbook",
    "range": {
      "startOffset": 120,
      "endOffset": 450
    },
    "metadata": {
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "file_path": "docs/week-11-12/humanoid-kinematics.md"
    }
  }
  ```
- `embedding_vector` (BYTEA, Optional): Query embedding for analytics

**Validation Rules:**
- `question_text` must be 10-1000 characters
- `question_text` must not be empty or only whitespace
- `selected_context.text` (if provided) must be 50-5000 characters
- `timestamp` must be >= parent conversation's `created_at`

**Relationships:**
- Belongs to one `Conversation` (N:1)
- Has one `Answer` (1:1)

**Storage:**
- Neon Postgres: `questions` table
- Retention: Tied to parent conversation (cascade delete)

**Indexes:**
- Primary: `id`
- Foreign key: `conversation_id`

---

### 3. Answer

Represents the chatbot's generated response to a question.

**Attributes:**
- `id` (UUID, PK): Unique answer identifier
- `question_id` (UUID, FK): Parent question
- `answer_text` (Text): Generated response (max 500 words)
- `citations` (JSONB): Array of cited textbook chunks
  ```json
  [
    {
      "chunk_id": "uuid",
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "page": 245,
      "relevance_score": 0.92,
      "link": "/week-11-12/humanoid-kinematics#inverse-kinematics",
      "text_snippet": "first 100 chars of cited chunk..."
    }
  ]
  ```
- `confidence_score` (Float): Model confidence (0.0-1.0)
- `token_count` (Integer): Number of tokens in answer
- `generation_time_ms` (Integer): Time taken to generate answer
- `timestamp` (Timestamp): When answer was generated

**Validation Rules:**
- `answer_text` must be 1-500 words (approx 1-2500 characters)
- `citations` must contain 1-5 citation objects
- Each citation must have valid `link` (URL format)
- `confidence_score` must be between 0.0 and 1.0
- `token_count` must be <= 3000 (safety check)
- `generation_time_ms` must be > 0

**Relationships:**
- Belongs to one `Question` (1:1)
- References multiple `TextbookChunk` entities via `citations`

**Storage:**
- Neon Postgres: `answers` table
- Retention: Tied to parent question (cascade delete)

**Indexes:**
- Primary: `id`
- Foreign key: `question_id`

---

### 4. TextbookChunk

Represents a semantically meaningful segment of textbook content with vector embedding.

**Attributes:**
- `chunk_id` (String, PK): Unique chunk identifier (format: `{file_path}#{chunk_index}`)
- `text` (Text): Original chunk text (300-500 tokens)
- `chapter` (Integer): Chapter number (1-13)
- `section` (String): Section title
- `page` (Integer, Optional): Page number (if applicable)
- `tokens` (Integer): Token count for this chunk
- `metadata` (JSONB): Additional context
  ```json
  {
    "file_path": "docs/week-11-12/humanoid-kinematics.md",
    "heading": "Inverse Kinematics",
    "week_range": "11-12",
    "topic": "Humanoid Kinematics",
    "chunk_index": 3
  }
  ```
- `vector_google` (768-dim float array): Google embedding
- `vector_local` (384-dim float array, Optional): Sentence Transformer embedding (fallback)

**Validation Rules:**
- `text` must be 50-3000 characters
- `chapter` must be between 1 and 13
- `tokens` must be between 50 and 600
- `metadata.file_path` must match pattern: `docs/week-\d+-\d+/.+\.md`
- Embeddings must have correct dimensionality (768 or 384)

**Relationships:**
- Referenced by multiple `Answer` entities via citations (N:M implicit)

**Storage:**
- Qdrant Cloud: `textbook_chunks` collection
  - Named vectors: `google` (768-dim), `local` (384-dim)
  - Payload: all attributes except vectors
- Retention: Permanent (until textbook content updated)

**Indexes:**
- Primary: `chunk_id` (Qdrant payload)
- Filters: `chapter`, `section`, `metadata.file_path`

---

### 5. SelectedContext

Represents user-highlighted text for scoped question-answering. This is a **transient entity** (not persisted independently) but included in `Question.selected_context`.

**Attributes:**
- `text` (String): Highlighted text content
- `range` (Object): DOM selection range
  - `startOffset` (Integer): Start character position
  - `endOffset` (Integer): End character position
- `metadata` (Object): Source context
  - `chapter` (Integer): Chapter number
  - `section` (String): Section title
  - `file_path` (String): Source markdown file

**Validation Rules:**
- `text` must be 50-5000 characters
- `endOffset` must be > `startOffset`
- `metadata.chapter` must be between 1 and 13
- `metadata.file_path` must match textbook file pattern

**Relationships:**
- Embedded within `Question` entity (composition)

**Storage:**
- Embedded in `questions.selected_context` JSONB field

---

## Entity Relationships Diagram

```
┌─────────────────┐
│  Conversation   │
│  (Postgres)     │
│  - id (PK)      │
│  - session_id   │
│  - context_mode │
└────────┬────────┘
         │ 1:N
         │
         ▼
┌─────────────────┐
│    Question     │
│  (Postgres)     │
│  - id (PK)      │
│  - conversation │
│  - question_text│
│  - selected_ctx │◄──── SelectedContext
└────────┬────────┘       (embedded)
         │ 1:1
         │
         ▼
┌─────────────────┐       N:M (implicit)
│     Answer      ├───────────────────►┌──────────────────┐
│  (Postgres)     │      via citations  │  TextbookChunk   │
│  - id (PK)      │                    │  (Qdrant)        │
│  - question_id  │                    │  - chunk_id (PK) │
│  - answer_text  │                    │  - text          │
│  - citations[]  │                    │  - embeddings    │
└─────────────────┘                    └──────────────────┘
```

---

## State Transitions

### Conversation States

```
[Created] → [Active] → [Idle] → [Expired]
            ↓
         [Archived]
```

**State Definitions:**
- **Created**: New session initialized, no questions yet
- **Active**: User is actively asking questions (< 5 min since last activity)
- **Idle**: No activity for 5-30 minutes
- **Expired**: No activity for > 30 days (eligible for deletion)
- **Archived**: User explicitly ended session

**Transitions:**
- Created → Active: When first question is asked
- Active → Idle: After 5 minutes of inactivity
- Idle → Active: When new question is asked
- Idle → Expired: After 30 days
- Active → Archived: User closes chat widget
- Archived → Expired: After 30 days

### Question-Answer Flow

```
[Question Received] → [Embedding Generated] → [Context Retrieved] 
                                                      ↓
[Answer Generated] ← [Gemini API Call] ← [Context Assembled]
        ↓
[Answer Stored with Citations]
```

**State Definitions:**
1. **Question Received**: User submits question via API
2. **Embedding Generated**: Question converted to 768-dim vector
3. **Context Retrieved**: Top 5 relevant chunks fetched from Qdrant
4. **Context Assembled**: Chunks formatted into prompt for Gemini
5. **Gemini API Call**: Request sent to Gemini 1.5 Flash
6. **Answer Generated**: Model returns response with reasoning
7. **Answer Stored**: Response + citations persisted to Postgres

**Error States:**
- **Embedding Failed**: Use fallback Sentence Transformer model
- **No Context Found**: Return "question not covered in textbook" message
- **Gemini Timeout**: Retry once, then return error message
- **Rate Limit Exceeded**: Queue request with exponential backoff

---

## Validation Rules Summary

### Cross-Entity Constraints

1. **Conversation History Limit**:
   - Each conversation can have max 50 questions
   - Enforce in API layer before creating new question

2. **Selected Context Consistency**:
   - If `Question.selected_context` is provided, `Conversation.context_mode` must be `selected`
   - Validate in API layer before question creation

3. **Citation Integrity**:
   - All `Answer.citations[].chunk_id` must reference existing chunks in Qdrant
   - Validate during answer generation

4. **Timestamp Ordering**:
   - `Question.timestamp` >= `Conversation.created_at`
   - `Answer.timestamp` >= `Question.timestamp`
   - Enforce with database CHECK constraints

5. **Rate Limiting**:
   - Max 10 questions per conversation per minute
   - Max 50 questions per IP address per hour
   - Enforce in API middleware

---

## Indexing Strategy

### Qdrant Vector Indexes

```python
# Collection configuration
collection_config = {
    "vectors": {
        "google": {
            "size": 768,
            "distance": "Cosine"
        },
        "local": {
            "size": 384,
            "distance": "Cosine"
        }
    },
    "optimizers_config": {
        "indexing_threshold": 10000,  # Build HNSW index after 10k vectors
        "memmap_threshold": 20000
    }
}

# Payload indexes for filtering
payload_indexes = [
    {"field_name": "chapter", "field_schema": "integer"},
    {"field_name": "section", "field_schema": "keyword"},
    {"field_name": "metadata.file_path", "field_schema": "keyword"}
]
```

### Postgres Database Indexes

```sql
-- Conversations
CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_created_at ON conversations(created_at);
CREATE INDEX idx_conversations_updated_at ON conversations(updated_at);

-- Questions
CREATE INDEX idx_questions_conversation_id ON questions(conversation_id);
CREATE INDEX idx_questions_timestamp ON questions(timestamp);

-- Answers
CREATE INDEX idx_answers_question_id ON answers(question_id);
CREATE INDEX idx_answers_timestamp ON answers(timestamp);
CREATE INDEX idx_answers_confidence ON answers(confidence_score);

-- Composite indexes for common queries
CREATE INDEX idx_questions_conv_timestamp ON questions(conversation_id, timestamp DESC);
```

---

## Data Migration & Versioning

### Schema Version: 1.0.0

**Migration Path:**
1. Initial schema creation (this version)
2. Future migrations will be tracked in `backend/scripts/migrations/` directory

**Backward Compatibility:**
- API responses include `schema_version` field for clients to adapt
- Qdrant collection supports schema evolution via payload updates

**Rollback Strategy:**
- Database migrations include DOWN scripts
- Qdrant snapshots taken before schema changes

---

## Sample Data

### Example Conversation

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "client-generated-uuid-123",
  "user_ip": "192.168.1.100",
  "context_mode": "full",
  "created_at": "2025-12-03T10:00:00Z",
  "updated_at": "2025-12-03T10:15:00Z"
}
```

### Example Question (Full-Textbook Mode)

```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "question_text": "What is the difference between forward and inverse kinematics?",
  "timestamp": "2025-12-03T10:05:00Z",
  "selected_context": null
}
```

### Example Question (Selected-Text Mode)

```json
{
  "id": "770e8400-e29b-41d4-a716-446655440002",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "question_text": "How does the Jacobian matrix relate to this?",
  "timestamp": "2025-12-03T10:10:00Z",
  "selected_context": {
    "text": "Inverse kinematics solves for joint angles given a desired end-effector position...",
    "range": {
      "startOffset": 1250,
      "endOffset": 1580
    },
    "metadata": {
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "file_path": "docs/week-11-12/humanoid-kinematics.md"
    }
  }
}
```

### Example Answer

```json
{
  "id": "880e8400-e29b-41d4-a716-446655440003",
  "question_id": "660e8400-e29b-41d4-a716-446655440001",
  "answer_text": "Forward kinematics computes the position and orientation of a robot's end-effector given the joint angles...",
  "citations": [
    {
      "chunk_id": "docs/week-11-12/humanoid-kinematics.md#2",
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "page": 245,
      "relevance_score": 0.94,
      "link": "/week-11-12/humanoid-kinematics#forward-kinematics",
      "text_snippet": "Forward kinematics (FK) is the process of determining the position..."
    },
    {
      "chunk_id": "docs/week-11-12/humanoid-kinematics.md#3",
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "page": 246,
      "relevance_score": 0.89,
      "link": "/week-11-12/humanoid-kinematics#inverse-kinematics",
      "text_snippet": "Inverse kinematics (IK), conversely, solves for the joint angles..."
    }
  ],
  "confidence_score": 0.91,
  "token_count": 487,
  "generation_time_ms": 1450,
  "timestamp": "2025-12-03T10:05:02Z"
}
```

### Example TextbookChunk (Qdrant)

```json
{
  "id": "docs/week-11-12/humanoid-kinematics.md#2",
  "vector": {
    "google": [0.123, -0.456, ...],  // 768 dimensions
    "local": [0.789, 0.234, ...]     // 384 dimensions
  },
  "payload": {
    "chunk_id": "docs/week-11-12/humanoid-kinematics.md#2",
    "text": "Forward kinematics (FK) is the process of determining the position and orientation of a robot's end-effector...",
    "chapter": 11,
    "section": "Humanoid Kinematics",
    "page": 245,
    "tokens": 387,
    "metadata": {
      "file_path": "docs/week-11-12/humanoid-kinematics.md",
      "heading": "Forward Kinematics",
      "week_range": "11-12",
      "topic": "Humanoid Kinematics",
      "chunk_index": 2
    }
  }
}
```

---

## Performance Considerations

### Qdrant Query Optimization

- **Batch searches**: Use `search_batch()` for multiple queries
- **Filter before search**: Apply chapter/section filters to reduce search space
- **HNSW parameters**: `m=16, ef_construct=100` for balanced speed/accuracy

### Postgres Query Optimization

- **Pagination**: Limit conversation history to last 10 turns
- **Partitioning**: Partition `questions` and `answers` by month if scaling beyond 1M rows
- **Connection pooling**: Use `psycopg2.pool.ThreadedConnectionPool` with max 20 connections

### Caching Strategy

- **Redis cache** (optional): Cache frequently asked questions
  - Key: `sha256(question_text)`
  - Value: `{answer_text, citations, timestamp}`
  - TTL: 1 hour
- **In-memory cache**: Cache Qdrant client and Gemini model instances

---

**Next**: Proceed to API contract generation (`contracts/openapi.yaml`)
