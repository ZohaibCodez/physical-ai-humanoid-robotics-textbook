# Quick Start Guide: RAG Chatbot

**Feature**: 002-rag-chatbot  
**Date**: 2025-12-03  
**Estimated Setup Time**: 45-60 minutes

## Overview

This guide walks you through setting up the RAG chatbot for the Physical AI & Humanoid Robotics textbook. You'll configure free-tier services (Google Gemini, Qdrant Cloud, Neon Postgres), run the backend locally, and integrate the chat UI with Docusaurus.

---

## Prerequisites

### Required Accounts (All Free)

1. **Google AI Studio** - for Gemini API key
   - Sign up: https://makersuite.google.com/app/apikey
   - Free tier: 15 requests/minute, 1500 requests/day

2. **Qdrant Cloud** - for vector database
   - Sign up: https://cloud.qdrant.io/
   - Free tier: 1GB storage, unlimited API calls

3. **Neon** - for Postgres database
   - Sign up: https://neon.tech/
   - Free tier: 0.5GB storage, auto-suspend after inactivity

### Software Requirements

- **Python 3.11+** (with pip)
- **Node.js 18+** (with npm)
- **Git** (for cloning repository)
- **Code editor** (VS Code recommended)

### Optional Tools

- **Docker** (for containerized deployment)
- **Postman** or **curl** (for API testing)

---

## Part 1: Backend Setup (30 minutes)

### Step 1: Get API Keys

#### Google Gemini API Key
```bash
# 1. Visit https://makersuite.google.com/app/apikey
# 2. Click "Create API Key"
# 3. Copy the key (format: AIza...)
```

#### Qdrant Cloud
```bash
# 1. Visit https://cloud.qdrant.io/
# 2. Create a free cluster (choose a region close to you)
# 3. Copy the cluster URL (format: https://xyz.qdrant.io)
# 4. Copy the API key from cluster settings
```

#### Neon Postgres
```bash
# 1. Visit https://neon.tech/
# 2. Create a new project (choose a region)
# 3. Copy the connection string from dashboard
#    Format: postgresql://user:pass@host/dbname?sslmode=require
```

### Step 2: Clone Repository & Install Dependencies

```bash
# Clone the repository
git clone https://github.com/your-org/physical-ai-humanoid-robotics-textbook.git
cd physical-ai-humanoid-robotics-textbook

# Create backend directory (if not exists)
mkdir -p backend
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### Step 3: Configure Environment Variables

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your keys (use nano, vim, or any editor)
nano .env
```

**`.env` file contents:**
```bash
# Google Gemini Configuration
GEMINI_API_KEY=AIza...your_key_here
GEMINI_MODEL=gemini-1.5-flash

# Qdrant Configuration
QDRANT_URL=https://xyz.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
QDRANT_COLLECTION=textbook_chunks

# Neon Postgres Configuration
NEON_DATABASE_URL=postgresql://user:pass@host/dbname?sslmode=require

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_WORKERS=4
LOG_LEVEL=info

# Rate Limiting
RATE_LIMIT_PER_MINUTE=10
RATE_LIMIT_PER_HOUR=50

# CORS (Frontend URL)
CORS_ORIGINS=http://localhost:3000,https://your-github-pages-domain.io
```

### Step 4: Initialize Database

```bash
# Run database migration script
python scripts/migrate_db.py

# Expected output:
# ✓ Connected to Neon Postgres
# ✓ Created table: conversations
# ✓ Created table: questions
# ✓ Created table: answers
# ✓ Created indexes
# Database initialized successfully!
```

### Step 5: Index Textbook Content

```bash
# Run indexing script (this will take 5-10 minutes)
python scripts/index_textbook.py --source-dir ../docs --batch-size 10

# Expected output:
# Processing docs/week-01-02/...
# ✓ Generated 487 chunks
# ✓ Created embeddings (Google text-embedding-004)
# ✓ Uploaded to Qdrant collection: textbook_chunks
# Indexing complete! 487 chunks indexed.
```

**Troubleshooting indexing:**
```bash
# If Google embedding API hits rate limit, use fallback model:
python scripts/index_textbook.py --source-dir ../docs --use-local-embeddings

# To re-index specific chapters:
python scripts/index_textbook.py --source-dir ../docs/week-11-12 --overwrite
```

### Step 6: Start Backend Server

```bash
# Development mode (with auto-reload)
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Expected output:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Started reloader process
# INFO:     Started server process
# INFO:     Waiting for application startup.
# INFO:     Application startup complete.
```

### Step 7: Test Backend API

```bash
# Open a new terminal and test the health endpoint
curl http://localhost:8000/v1/health

# Expected response:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "timestamp": "2025-12-03T10:00:00Z",
#   "dependencies": {
#     "gemini_api": "healthy",
#     "qdrant": "healthy",
#     "postgres": "healthy"
#   }
# }
```

**Test asking a question:**
```bash
curl -X POST http://localhost:8000/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "test-session-123",
    "question": "What is forward kinematics?",
    "context_mode": "full"
  }'

# Expected response (truncated):
# {
#   "answer": "Forward kinematics is the process of...",
#   "citations": [...],
#   "confidence": 0.91,
#   "processing_time_ms": 1450
# }
```

---

## Part 2: Frontend Integration (15 minutes)

### Step 1: Install Frontend Dependencies

```bash
# Navigate to repository root
cd ..

# Install Docusaurus dependencies (if not already done)
npm install

# Install chat UI library
npm install @chatscope/chat-ui-kit-react react-markdown remark-gfm
```

### Step 2: Create Chat Widget Component

Create `src/components/ChatWidget/index.tsx`:

```tsx
import React, { useState, useEffect } from 'react';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import './styles.css';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  citations?: any[];
  timestamp: string;
}

export default function ChatWidget() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isTyping, setIsTyping] = useState(false);
  const [sessionId] = useState(() => crypto.randomUUID());
  const [isOpen, setIsOpen] = useState(false);

  const apiUrl = process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000/v1';

  const handleSend = async (text: string) => {
    // Add user message
    const userMessage: ChatMessage = {
      role: 'user',
      content: text,
      timestamp: new Date().toISOString()
    };
    setMessages(prev => [...prev, userMessage]);
    setIsTyping(true);

    try {
      const response = await fetch(`${apiUrl}/chat/ask`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          session_id: sessionId,
          question: text,
          context_mode: 'full'
        })
      });

      const data = await response.json();

      // Add assistant message
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      // Add error message
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      }]);
    } finally {
      setIsTyping(false);
    }
  };

  return (
    <div className={`chat-widget ${isOpen ? 'open' : ''}`}>
      <button 
        className="chat-toggle" 
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        💬
      </button>
      
      {isOpen && (
        <MainContainer>
          <ChatContainer>
            <MessageList typingIndicator={isTyping && <TypingIndicator content="Chatbot is thinking..." />}>
              {messages.map((msg, i) => (
                <Message
                  key={i}
                  model={{
                    message: msg.content,
                    sender: msg.role,
                    direction: msg.role === 'user' ? 'outgoing' : 'incoming',
                    position: 'single'
                  }}
                />
              ))}
            </MessageList>
            <MessageInput 
              placeholder="Ask about the textbook..." 
              onSend={handleSend}
              attachButton={false}
            />
          </ChatContainer>
        </MainContainer>
      )}
    </div>
  );
}
```

Create `src/components/ChatWidget/styles.css`:

```css
.chat-widget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 9999;
}

.chat-toggle {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  border: none;
  font-size: 28px;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  transition: transform 0.2s;
}

.chat-toggle:hover {
  transform: scale(1.1);
}

.chat-widget.open .cs-main-container {
  width: 400px;
  height: 600px;
  position: fixed;
  bottom: 90px;
  right: 20px;
  border-radius: 10px;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
}

@media (max-width: 768px) {
  .chat-widget.open .cs-main-container {
    width: calc(100vw - 40px);
    height: 500px;
  }
}
```

### Step 3: Integrate with Docusaurus

Edit `src/theme/Layout/index.tsx` (create if doesn't exist):

```tsx
import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatWidget />
    </>
  );
}
```

### Step 4: Configure Environment Variables

Create `.env` in repository root:

```bash
# Frontend configuration
REACT_APP_CHAT_API_URL=http://localhost:8000/v1

# For production (GitHub Pages):
# REACT_APP_CHAT_API_URL=https://your-railway-app.up.railway.app/v1
```

Update `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config
  
  customFields: {
    chatApiUrl: process.env.REACT_APP_CHAT_API_URL || 'http://localhost:8000/v1'
  },
  
  // ... rest of config
};
```

### Step 5: Test Frontend Locally

```bash
# Start Docusaurus development server
npm start

# Open browser to http://localhost:3000
# Click the chat button in bottom-right corner
# Ask a question (e.g., "What is ROS2?")
```

---

## Part 3: Deployment (Production)

### Backend Deployment (Railway)

#### Step 1: Create Railway Account
1. Visit https://railway.app/
2. Sign up with GitHub
3. Free tier: 500 hours/month, $5 credit

#### Step 2: Deploy Backend

```bash
cd backend

# Create railway.toml
cat > railway.toml << EOF
[build]
builder = "DOCKERFILE"
dockerfilePath = "Dockerfile"

[deploy]
numReplicas = 1
restartPolicyType = "ON_FAILURE"
EOF

# Create Dockerfile (if not exists)
cat > Dockerfile << EOF
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "4"]
EOF

# Install Railway CLI
npm install -g @railway/cli

# Login and deploy
railway login
railway init
railway up
```

#### Step 3: Add Environment Variables in Railway Dashboard
1. Go to your Railway project dashboard
2. Navigate to "Variables" tab
3. Add all environment variables from `.env` (GEMINI_API_KEY, QDRANT_URL, etc.)
4. Railway will auto-restart the service

#### Step 4: Get Backend URL
```bash
# Copy the generated URL (e.g., https://your-app.up.railway.app)
railway domain
```

### Frontend Deployment (GitHub Pages)

#### Step 1: Update Production Config

Edit `docusaurus.config.js`:

```javascript
const config = {
  url: 'https://your-username.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',
  
  customFields: {
    chatApiUrl: 'https://your-railway-app.up.railway.app/v1'
  },
  
  // ... rest of config
};
```

Update `.env.production`:

```bash
REACT_APP_CHAT_API_URL=https://your-railway-app.up.railway.app/v1
```

#### Step 2: Build and Deploy

```bash
# Build static site
npm run build

# Test production build locally
npm run serve

# Deploy to GitHub Pages
npm run deploy

# Or use GitHub Actions (recommended)
```

#### Step 3: Configure CORS on Backend

Update backend `.env` on Railway:

```bash
CORS_ORIGINS=https://your-username.github.io,http://localhost:3000
```

---

## Verification Checklist

### Backend Health Checks
- [ ] `/v1/health` returns `{"status": "healthy"}`
- [ ] Gemini API dependency is `healthy`
- [ ] Qdrant dependency is `healthy`
- [ ] Postgres dependency is `healthy`
- [ ] Can ask a test question via `/v1/chat/ask`
- [ ] Response includes citations with valid links

### Frontend Checks
- [ ] Chat widget button appears in bottom-right
- [ ] Clicking button opens chat interface
- [ ] Can type and send messages
- [ ] Responses appear within 5 seconds
- [ ] Citations are clickable and navigate to correct textbook section
- [ ] Chat persists across page navigation (session maintained)

### Integration Checks
- [ ] Selected text mode: Can highlight text and ask scoped questions
- [ ] Conversation history: Can retrieve past messages
- [ ] Rate limiting: Blocked after 10 requests/minute
- [ ] Error handling: Graceful error messages on API failures
- [ ] Mobile responsive: Chat works on phones/tablets

---

## Troubleshooting

### Issue: "Gemini API rate limit exceeded"

**Symptoms**: Error message "Rate limit exceeded, retry after X seconds"

**Solutions:**
1. Wait for rate limit to reset (1 minute for free tier)
2. Reduce concurrent users during testing
3. Implement request queuing in backend

### Issue: "Qdrant connection timeout"

**Symptoms**: Slow response times, health check shows Qdrant unhealthy

**Solutions:**
1. Check Qdrant Cloud cluster status (https://cloud.qdrant.io/)
2. Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
3. Test connection manually:
   ```bash
   curl -X GET "https://your-cluster.qdrant.io/collections" \
     -H "api-key: your_key"
   ```

### Issue: "Neon database connection failed"

**Symptoms**: Backend crashes on startup, health check fails

**Solutions:**
1. Verify `NEON_DATABASE_URL` is correct (check for typos)
2. Ensure database is not suspended (Neon auto-suspends after inactivity)
3. Test connection:
   ```bash
   psql "postgresql://user:pass@host/dbname?sslmode=require" -c "SELECT 1;"
   ```

### Issue: "CORS policy blocks frontend requests"

**Symptoms**: Browser console error: "CORS policy: No 'Access-Control-Allow-Origin' header"

**Solutions:**
1. Add frontend URL to backend `CORS_ORIGINS` environment variable
2. Restart backend server after updating `.env`
3. Clear browser cache and reload

### Issue: "Textbook indexing fails"

**Symptoms**: Script errors, incomplete indexing

**Solutions:**
1. Check file paths: Ensure `docs/` directory exists
2. Use fallback embeddings if Google API fails:
   ```bash
   python scripts/index_textbook.py --use-local-embeddings
   ```
3. Index in smaller batches:
   ```bash
   python scripts/index_textbook.py --batch-size 5
   ```

---

## Maintenance

### Re-indexing Textbook (After Content Updates)

```bash
# Backup existing embeddings
python scripts/backup_qdrant.py

# Re-index updated chapters only
python scripts/index_textbook.py --source-dir docs/week-13 --overwrite

# Full re-index (if major changes)
python scripts/index_textbook.py --source-dir docs --overwrite --batch-size 10
```

### Database Cleanup (Remove Old Conversations)

```bash
# Delete conversations older than 30 days
python scripts/cleanup_db.py --days 30

# Vacuum database to reclaim space
python scripts/cleanup_db.py --vacuum
```

### Monitoring

```bash
# View backend logs (Railway)
railway logs

# Check API metrics
curl http://localhost:8000/v1/health

# Monitor Qdrant storage usage
# Visit Qdrant Cloud dashboard: https://cloud.qdrant.io/

# Monitor Neon Postgres usage
# Visit Neon dashboard: https://neon.tech/
```

---

## Next Steps

1. **Phase 2: Create Tasks** - Run `/sp.tasks` to generate implementation tasks
2. **Implement Backend** - Build FastAPI endpoints, services, and models
3. **Implement Frontend** - Integrate chat UI with Docusaurus
4. **Testing** - Write unit, integration, and contract tests
5. **Deployment** - Deploy to Railway and GitHub Pages
6. **Documentation** - Update README and API docs

---

## Support Resources

- **OpenAPI Spec**: `specs/002-rag-chatbot/contracts/openapi.yaml`
- **Data Model**: `specs/002-rag-chatbot/data-model.md`
- **Research**: `specs/002-rag-chatbot/research.md`
- **Google Gemini Docs**: https://ai.google.dev/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Docusaurus Docs**: https://docusaurus.io/docs

---

**Estimated Total Time**: 45-60 minutes (excluding textbook indexing)
