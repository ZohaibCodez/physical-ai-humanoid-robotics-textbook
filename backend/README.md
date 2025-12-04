# RAG Chatbot Backend

Backend API for the Physical AI & Humanoid Robotics Textbook RAG chatbot using **OpenAI Agents SDK + Google Gemini (via LiteLLM)**.

## 🏗️ Architecture

```
FastAPI Backend
├── OpenAI Agents SDK (orchestration)
├── LiteLLM (Gemini 1.5 Flash backend)
├── Qdrant (vector database)
├── Neon Postgres (conversation history)
└── Google Text Embeddings (768-dim + fallback)
```

## 📦 Setup

### Prerequisites

- Python 3.11+
- Google AI Studio API key (free tier)
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)

### Installation

1. **Install dependencies:**

```bash
cd backend
pip install -r requirements.txt
pip install -r requirements-dev.txt  # For development
```

2. **Configure environment variables:**

```bash
cp .env.example .env
# Edit .env with your API keys
```

3. **Initialize database:**

```bash
python scripts/migrate_db.py
```

4. **Index textbook content:**

```bash
python scripts/index_textbook.py
```

### Run Development Server

```bash
uvicorn app.main:app --reload --port 8000
```

Access API docs at: http://localhost:8000/v1/docs

## 🔌 API Endpoints

### POST /v1/chat/ask

Ask a question and get an answer with citations.

**Request:**
```json
{
  "session_id": "sess_abc123",
  "question_text": "What are the key components of ROS2?",
  "context_mode": "full"
}
```

**Response:**
```json
{
  "answer": "ROS2 has several key components...",
  "citations": [
    {
      "text": "Chapter 3, Section: ROS2 Architecture",
      "url": "/week-03-05/ros2-architecture#key-components",
      "relevance_score": 0.95
    }
  ],
  "confidence_score": 0.92,
  "processing_time_ms": 1250,
  "context_used": "full",
  "tokens_used": 450
}
```

### GET /v1/chat/history/{session_id}

Retrieve conversation history for a session.

### GET /v1/health

Health check for monitoring.

## 🧪 Testing

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test file
pytest tests/unit/test_agent_service.py
```

## 📁 Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI app entry point
│   ├── config.py            # Environment configuration
│   ├── models/              # Pydantic models
│   ├── services/            # Business logic
│   │   ├── agent_service.py     # OpenAI Agents SDK + Gemini
│   │   ├── tools.py             # @function_tool definitions
│   │   ├── embeddings.py        # Google embeddings
│   │   ├── vector_store.py      # Qdrant client
│   │   └── postgres_service.py  # Neon Postgres
│   ├── api/                 # API routes
│   └── utils/               # Utilities
├── scripts/                 # CLI scripts
├── tests/                   # Test suite
└── requirements.txt         # Dependencies
```

## 🚀 Deployment

### Railway (Recommended)

1. Create new project on Railway
2. Connect GitHub repository
3. Set environment variables in Railway dashboard
4. Deploy automatically on push

### Docker

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

## 📊 Environment Variables

See `.env.example` for all required variables:

- `GOOGLE_API_KEY` - Google AI Studio API key
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DATABASE_URL` - Neon Postgres connection string

## 🔧 Development

### Code Quality

```bash
# Format code
black app/ tests/

# Lint
flake8 app/ tests/

# Type check
mypy app/
```

### Adding New Features

1. Create data models in `app/models/`
2. Implement services in `app/services/`
3. Add API routes in `app/api/routes/`
4. Write tests in `tests/`
5. Update OpenAPI documentation

## 📝 Implementation Status

**Phase 1: Setup** ✅
- [x] Project structure
- [x] Dependencies (requirements.txt)
- [x] Environment files (.env.example, Dockerfile)
- [x] FastAPI initialization

**Phase 2: Foundational** 🚧
- [x] Core utilities (config, logger, exceptions)
- [x] Data models (conversation, textbook, chat)
- [x] API routes (health, chat placeholders)
- [ ] Embedding services (TODO)
- [ ] Vector store (TODO)
- [ ] Database service (TODO)
- [ ] Rate limiting (TODO)

**Phase 3: MVP** 📋
- [ ] OpenAI Agents SDK integration (TODO)
- [ ] Agent tools (search_textbook) (TODO)
- [ ] Citation resolver (TODO)
- [ ] Complete chat endpoint (TODO)
- [ ] Database migrations (TODO)
- [ ] Textbook indexing script (TODO)

## 📚 Documentation

- [Specification](../specs/002-rag-chatbot/spec.md)
- [Technical Plan](../specs/002-rag-chatbot/plan.md)
- [Implementation Tasks](../specs/002-rag-chatbot/tasks.md)
- [Quickstart Guide](../specs/002-rag-chatbot/quickstart.md)

## 🤝 Contributing

1. Create feature branch from `002-rag-chatbot`
2. Implement changes following existing patterns
3. Add tests for new functionality
4. Update documentation
5. Submit pull request

## 📄 License

See main repository LICENSE file.
