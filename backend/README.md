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

## 🚀 Deployment (FREE Options)

### Option 1: Railway.app (Recommended - FREE Tier)

**Why Railway?**
- ✅ FREE $5/month credit (enough for this app)
- ✅ Automatic HTTPS
- ✅ Environment variables management
- ✅ Automatic deployments from GitHub
- ✅ Built-in PostgreSQL (if needed)

**Step-by-Step Deployment:**

1. **Sign up for Railway:**
   - Go to https://railway.app
   - Sign up with GitHub account (FREE)

2. **Create New Project:**
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Choose your `physical-ai-humanoid-robotics-textbook` repository
   - Select `002-rag-chatbot` branch

3. **Configure Root Directory:**
   - In project settings, set **Root Directory** to `backend`
   - This tells Railway to deploy only the backend folder

4. **Set Environment Variables:**
   Click "Variables" tab and add:
   ```env
   # Required
   GEMINI_API_KEY=your_google_ai_api_key_here
   QDRANT_URL=https://your-cluster.gcp.cloud.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION=robotics_textbook_embeddings
   
   # Optional (if using conversation history)
   NEON_DATABASE_URL=postgresql://user:pass@host/db
   
   # CORS - Add your GitHub Pages URL
   CORS_ORIGINS=https://yourusername.github.io,http://localhost:3000
   
   # API Settings
   LOG_LEVEL=INFO
   PORT=8000
   ```

5. **Deploy:**
   - Railway will automatically build and deploy
   - Wait 2-3 minutes for first deployment
   - You'll get a public URL like: `https://your-app.railway.app`

6. **Index Your Textbook:**
   After first deployment, run indexing script:
   ```bash
   # Connect to Railway console or run locally pointing to Railway
   python scripts/index_textbook.py
   ```

### Option 2: Render.com (FREE Tier Alternative)

**Step-by-Step:**

1. Go to https://render.com (sign up with GitHub)
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Name**: `rag-chatbot-backend`
   - **Root Directory**: `backend`
   - **Environment**: Python 3
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as Railway above)
6. Click "Create Web Service"

**Note**: Render free tier spins down after inactivity (first request takes 30s to wake up)

### Option 3: Python Anywhere (FREE but limited)

Good for testing, but has limitations (no async support on free tier).

### Connecting Backend to Frontend

Once your backend is deployed, you'll have a URL like:
```
https://your-app.railway.app
```

**Update your Docusaurus frontend:**

1. **Create `.env` file in root directory:**
   ```env
   REACT_APP_CHAT_API_URL=https://your-app.railway.app
   ```

2. **Update `docusaurus.config.js`:**
   ```js
   customFields: {
     chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000',
   }
   ```

3. **Update CORS in backend:**
   Make sure your GitHub Pages URL is in `CORS_ORIGINS` environment variable:
   ```
   CORS_ORIGINS=https://yourusername.github.io,https://your-app.railway.app
   ```

4. **Rebuild and deploy frontend:**
   ```bash
   npm run build
   git add .
   git commit -m "Connect to deployed backend"
   git push origin 002-rag-chatbot
   ```

### Deployment Checklist

Before deploying:
- [ ] Get Google AI API key from https://aistudio.google.com/app/apikey
- [ ] Create Qdrant Cloud cluster at https://cloud.qdrant.io (FREE tier)
- [ ] (Optional) Create Neon Postgres at https://neon.tech (FREE tier)
- [ ] Index textbook content into Qdrant
- [ ] Test backend locally first: `uvicorn app.main:app --reload`
- [ ] Update CORS_ORIGINS with your frontend URL
- [ ] Deploy backend to Railway/Render
- [ ] Update frontend .env with backend URL
- [ ] Deploy frontend to GitHub Pages

### Testing Your Deployment

1. **Test backend health:**
   ```bash
   curl https://your-app.railway.app/v1/health
   ```

2. **Test API endpoint:**
   ```bash
   curl -X POST https://your-app.railway.app/v1/chat/ask \
     -H "Content-Type: application/json" \
     -d '{
       "session_id": "test123",
       "question_text": "What are the key components of ROS2?",
       "context_mode": "full"
     }'
   ```

3. **Check logs:**
   - Railway: Click on deployment → "Logs" tab
   - Render: Dashboard → Service → "Logs"

### Docker (Alternative)

```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

### Troubleshooting Deployment

**Issue: "Module not found" error**
- Solution: Make sure `requirements.txt` includes all dependencies
- Check Railway build logs

**Issue: "Connection refused" from frontend**
- Solution: Verify CORS_ORIGINS includes your frontend URL
- Check browser console for CORS errors

**Issue: "Postgres connection timeout"**
- Solution: This is normal - app falls back to stateless mode
- Neon Postgres auto-suspends, first request may be slow

**Issue: "Qdrant returns 0 results"**
- Solution: Run `scripts/index_textbook.py` to populate vector database
- Check Qdrant dashboard for indexed documents (~240 chunks expected)

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
