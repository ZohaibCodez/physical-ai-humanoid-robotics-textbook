# Research Document: RAG Chatbot Implementation

**Date**: 2025-12-03  
**Feature**: 002-rag-chatbot  
**Status**: Completed

## Overview

This document consolidates research findings for implementing the RAG (Retrieval-Augmented Generation) chatbot using free AI models and services. All technical unknowns have been resolved.

---

## 1. Google Gemini API (Free Tier)

### Decision
Use **Gemini 1.5 Flash** model via `google-generativeai` Python SDK for conversational AI agent.

### Rationale
- **Free tier**: 15 requests/minute (RPM), 1 million tokens/minute (TPM), 1500 requests/day (RPD)
- **Performance**: Fast inference (~1-2s response time), suitable for real-time chat
- **Context window**: 1M tokens input context (far exceeds our needs for textbook chunks)
- **Multimodal**: Supports text input/output (images not needed for this use case)
- **API stability**: Production-ready, officially supported by Google

### Alternatives Considered
- **Gemini 1.5 Pro**: Better quality but lower free tier limits (2 RPM vs 15 RPM for Flash)
- **OpenAI GPT-3.5/4**: Not free; ChatGPT API requires paid subscription
- **Open-source LLMs (Llama, Mistral)**: Require local hosting, GPU resources, complex setup

### Implementation Details
```python
import google.generativeai as genai

genai.configure(api_key=os.environ["GEMINI_API_KEY"])
model = genai.GenerativeModel('gemini-1.5-flash')

# Generate response with context
response = model.generate_content([
    "Context: " + retrieved_chunks,
    "Question: " + user_question
])
```

### Rate Limiting Strategy
- Implement token bucket algorithm: 15 requests/minute per user
- Queue requests during burst traffic
- Fallback message: "High demand, please retry in 1 minute"

### References
- Gemini API docs: https://ai.google.dev/docs
- Pricing: https://ai.google.dev/pricing

---

## 2. Google Text Embeddings (Free Tier)

### Decision
Use **models/text-embedding-004** via `google-generativeai` SDK for generating 768-dimensional embeddings.

### Rationale
- **Free tier**: 1500 requests/day (sufficient for ~500 textbook chunks + user queries)
- **Quality**: State-of-the-art retrieval performance (outperforms OpenAI Ada on MTEB benchmarks)
- **Dimensionality**: 768 dimensions (balanced between quality and storage efficiency)
- **Task type support**: `retrieval_document` for chunks, `retrieval_query` for questions
- **Batch processing**: Can embed multiple texts in single API call

### Alternatives Considered
- **Sentence Transformers (all-MiniLM-L6-v2)**: Local, free, but lower quality (384 dims)
  - Will use as **fallback** when Google API quota exhausted
- **OpenAI text-embedding-3-small**: Not free ($0.02/1M tokens)
- **Cohere Embed**: Free tier exists but lower rate limits (100 requests/month)

### Implementation Details
```python
import google.generativeai as genai

# For textbook chunks
result = genai.embed_content(
    model="models/text-embedding-004",
    content=chunk_text,
    task_type="retrieval_document"
)
embedding = result['embedding']  # 768-dim vector

# For user questions
result = genai.embed_content(
    model="models/text-embedding-004",
    content=user_question,
    task_type="retrieval_query"
)
query_embedding = result['embedding']
```

### Fallback: Sentence Transformers
```python
from sentence_transformers import SentenceTransformer

model = SentenceTransformer('all-MiniLM-L6-v2')
embedding = model.encode(text)  # 384-dim vector
```

**Note**: Qdrant collection must support both 768-dim (primary) and 384-dim (fallback) vectors using named vectors feature.

### References
- Embedding guide: https://ai.google.dev/docs/embeddings_guide
- MTEB benchmark: https://huggingface.co/spaces/mteb/leaderboard

---

## 3. Qdrant Vector Database (Free Tier)

### Decision
Use **Qdrant Cloud** free tier for vector storage and semantic search.

### Rationale
- **Free tier**: 1GB storage, 1M vectors, unlimited API calls
- **Capacity**: Can store ~1000 chunks × 768 dims × 4 bytes ≈ 3MB (well under 1GB limit)
- **Performance**: <100ms search latency for top-k retrieval
- **Features**: Filtering by metadata (chapter, section), payload storage (text, citations)
- **Python client**: Official `qdrant-client` SDK with async support

### Alternatives Considered
- **Pinecone**: Free tier limited to 1 index, 100K vectors (sufficient but less flexible)
- **Weaviate**: Self-hosted only (no free cloud tier); requires infrastructure
- **ChromaDB**: Local only, no cloud option for persistent storage
- **FAISS**: In-memory only, no persistence without custom storage layer

### Implementation Details
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)

# Create collection with named vectors (primary + fallback)
client.create_collection(
    collection_name="textbook_chunks",
    vectors_config={
        "google": VectorParams(size=768, distance=Distance.COSINE),
        "local": VectorParams(size=384, distance=Distance.COSINE)
    }
)

# Search with filters
results = client.search(
    collection_name="textbook_chunks",
    query_vector=("google", query_embedding),
    limit=5,
    query_filter={"must": [{"key": "chapter", "match": {"value": 3}}]}
)
```

### Schema Design
```python
payload_schema = {
    "chunk_id": str,           # Unique identifier
    "text": str,               # Original chunk text
    "chapter": int,            # Chapter number
    "section": str,            # Section title
    "page": int,               # Page number (if applicable)
    "tokens": int,             # Token count
    "metadata": {
        "file_path": str,      # Source markdown file
        "heading": str         # Section heading
    }
}
```

### References
- Qdrant docs: https://qdrant.tech/documentation/
- Cloud pricing: https://qdrant.tech/pricing/

---

## 4. Neon Postgres (Free Tier)

### Decision
Use **Neon Serverless Postgres** free tier for conversation history and metadata storage.

### Rationale
- **Free tier**: 0.5GB storage, 3GB data transfer/month, auto-suspend after inactivity
- **Serverless**: No idle costs, scales to zero, instant cold starts (<1s)
- **PostgreSQL**: Standard SQL, JSON support, full ACID compliance
- **Python client**: `psycopg2` compatible with connection pooling

### Alternatives Considered
- **Supabase**: Similar free tier but requires account linking with external auth
- **PlanetScale**: MySQL (not PostgreSQL), limited free tier (1GB storage, 1 billion reads/month)
- **Railway Postgres**: Free tier limited to $5 credit/month (runs out quickly)

### Schema Design

#### Conversations Table
```sql
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) UNIQUE NOT NULL,
    user_ip VARCHAR(45),
    context_mode VARCHAR(20) CHECK (context_mode IN ('full', 'selected')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_session_id ON conversations(session_id);
CREATE INDEX idx_created_at ON conversations(created_at);
```

#### Questions Table
```sql
CREATE TABLE questions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
    question_text TEXT NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    selected_context JSONB,  -- For selected-text mode: {text, range, section}
    embedding_vector BYTEA    -- Optional: store query embedding for analytics
);

CREATE INDEX idx_conversation_id ON questions(conversation_id);
```

#### Answers Table
```sql
CREATE TABLE answers (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    question_id UUID REFERENCES questions(id) ON DELETE CASCADE,
    answer_text TEXT NOT NULL,
    citations JSONB NOT NULL,  -- [{chunk_id, chapter, section, page, relevance_score}]
    confidence_score FLOAT,
    token_count INT,
    generation_time_ms INT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_question_id ON answers(question_id);
```

### Implementation Details
```python
import psycopg2
from psycopg2.extras import RealDictCursor

conn = psycopg2.connect(os.environ["NEON_DATABASE_URL"])
cursor = conn.cursor(cursor_factory=RealDictCursor)

# Insert conversation
cursor.execute("""
    INSERT INTO conversations (session_id, user_ip, context_mode)
    VALUES (%s, %s, %s) RETURNING id
""", (session_id, user_ip, "full"))

conversation_id = cursor.fetchone()["id"]
conn.commit()
```

### References
- Neon docs: https://neon.tech/docs/introduction
- psycopg2 docs: https://www.psycopg.org/docs/

---

## 5. FastAPI Backend Architecture

### Decision
Use **FastAPI 0.104+** with async/await patterns for high-performance API.

### Rationale
- **Performance**: Async I/O enables concurrent request handling (100+ simultaneous users)
- **Type safety**: Pydantic v2 models for request/response validation
- **OpenAPI**: Auto-generated API documentation at `/docs`
- **Dependency injection**: Clean separation of services (Gemini, Qdrant, Postgres)

### API Endpoints Design

#### POST /chat/ask
**Request**:
```json
{
  "session_id": "uuid-string",
  "question": "What is inverse kinematics?",
  "context_mode": "full",  // or "selected"
  "selected_text": "optional-highlighted-text",
  "selected_metadata": {
    "chapter": 11,
    "section": "Humanoid Kinematics"
  }
}
```

**Response**:
```json
{
  "answer": "Inverse kinematics is...",
  "citations": [
    {
      "chunk_id": "uuid",
      "chapter": 11,
      "section": "Humanoid Kinematics",
      "page": 245,
      "relevance_score": 0.92,
      "link": "/week-11-12/humanoid-kinematics#inverse-kinematics"
    }
  ],
  "confidence": 0.87,
  "processing_time_ms": 1450
}
```

#### GET /chat/history/{session_id}
**Response**:
```json
{
  "session_id": "uuid",
  "context_mode": "full",
  "messages": [
    {
      "role": "user",
      "content": "What is forward kinematics?",
      "timestamp": "2025-12-03T10:30:00Z"
    },
    {
      "role": "assistant",
      "content": "Forward kinematics is...",
      "citations": [...],
      "timestamp": "2025-12-03T10:30:02Z"
    }
  ]
}
```

### Middleware & Dependencies
- **CORS**: Allow requests from Docusaurus frontend origin
- **Rate limiting**: Redis-based or in-memory token bucket (10 req/min per IP)
- **Logging**: Structured JSON logs with request IDs for tracing
- **Error handling**: Global exception handlers for Gemini API errors, Qdrant timeouts

### References
- FastAPI docs: https://fastapi.tiangolo.com/
- Pydantic v2: https://docs.pydantic.dev/latest/

---

## 6. ChatKit UI (OpenAI SDK)

### Decision
Use **OpenAI's ChatKit UI SDK** (open-source) for frontend chat interface.

### Rationale
- **Pre-built components**: Message list, input box, typing indicators
- **Customizable**: React components with TypeScript support
- **Backend-agnostic**: Works with any chat API (not locked to OpenAI)
- **Accessibility**: WCAG 2.1 AA compliant out-of-box

### Clarification: ChatKit UI Implementation
After research, **"ChatKit UI" is not an official OpenAI product**. The user likely refers to one of:
1. **react-chatbotkit-ui**: Popular open-source chat UI library
2. **Custom React chat component**: Built with Material-UI or Ant Design

**Recommended approach**: Build custom React chat component using:
- **@chatscope/chat-ui-kit-react**: Feature-rich, accessible chat UI library
- **react-markdown**: For rendering formatted responses
- **react-syntax-highlighter**: For code blocks in answers

### Implementation Details
```tsx
import { MainContainer, ChatContainer, MessageList, Message, MessageInput } from '@chatscope/chat-ui-kit-react';

function ChatInterface() {
  const [messages, setMessages] = useState<Message[]>([]);
  
  const handleSend = async (text: string) => {
    const response = await fetch('https://api.example.com/chat/ask', {
      method: 'POST',
      body: JSON.stringify({ question: text, session_id: sessionId })
    });
    const data = await response.json();
    
    setMessages([
      ...messages,
      { role: 'user', content: text },
      { 
        role: 'assistant', 
        content: data.answer,
        citations: data.citations 
      }
    ]);
  };
  
  return (
    <MainContainer>
      <ChatContainer>
        <MessageList>
          {messages.map((msg, i) => (
            <Message key={i} model={msg} />
          ))}
        </MessageList>
        <MessageInput onSend={handleSend} />
      </ChatContainer>
    </MainContainer>
  );
}
```

### References
- @chatscope/chat-ui-kit-react: https://chatscope.io/storybook/react/
- react-markdown: https://github.com/remarkjs/react-markdown

---

## 7. Text Selection & Context Scoping

### Decision
Use **DOM Range API** + **React state** for capturing highlighted text and metadata.

### Rationale
- **Native browser API**: `window.getSelection()` works across all modern browsers
- **No dependencies**: No additional libraries needed
- **Metadata extraction**: Can traverse DOM to find chapter/section from parent elements

### Implementation Details
```typescript
function useTextSelection() {
  const [selectedText, setSelectedText] = useState<string>("");
  const [metadata, setMetadata] = useState<any>(null);
  
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      
      if (text && text.length > 50) {  // Minimum 50 chars
        // Find parent section element
        const range = selection.getRangeAt(0);
        const container = range.commonAncestorContainer.parentElement;
        const section = container?.closest('[data-chapter]');
        
        setSelectedText(text);
        setMetadata({
          chapter: section?.getAttribute('data-chapter'),
          section: section?.getAttribute('data-section'),
          startOffset: range.startOffset,
          endOffset: range.endOffset
        });
      }
    };
    
    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);
  
  return { selectedText, metadata };
}
```

### Docusaurus Integration
Modify Docusaurus markdown pages to include metadata attributes:
```jsx
<section data-chapter="11" data-section="Humanoid Kinematics">
  {children}
</section>
```

### References
- Selection API: https://developer.mozilla.org/en-US/docs/Web/API/Selection
- Range API: https://developer.mozilla.org/en-US/docs/Web/API/Range

---

## 8. Textbook Indexing Strategy

### Decision
Use **recursive chunking** with 300-500 tokens per chunk, 50-token overlap.

### Rationale
- **Chunk size**: 300-500 tokens balances context completeness vs retrieval precision
- **Overlap**: 50-token overlap prevents information loss at chunk boundaries
- **Hierarchy**: Preserve markdown headings for citation resolution

### Chunking Algorithm
```python
def chunk_markdown(file_path: str, max_tokens: int = 400, overlap: int = 50):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Split by headings first
    sections = re.split(r'^#{1,6}\s+', content, flags=re.MULTILINE)
    
    chunks = []
    for section in sections:
        tokens = tokenize(section)  # Use tiktoken
        
        if len(tokens) <= max_tokens:
            chunks.append(section)
        else:
            # Split long sections
            for i in range(0, len(tokens), max_tokens - overlap):
                chunk_tokens = tokens[i:i + max_tokens]
                chunks.append(detokenize(chunk_tokens))
    
    return chunks
```

### Metadata Extraction
```python
def extract_metadata(file_path: str, chunk_text: str):
    # Parse file path: docs/week-11-12/humanoid-kinematics.md
    match = re.match(r'docs/week-(\d+)-(\d+)/(.+)\.md', file_path)
    week_start, week_end, topic = match.groups()
    
    # Extract heading from chunk
    heading = re.search(r'^#{1,6}\s+(.+)$', chunk_text, re.MULTILINE)
    
    return {
        "file_path": file_path,
        "week_range": f"{week_start}-{week_end}",
        "topic": topic.replace('-', ' ').title(),
        "heading": heading.group(1) if heading else None
    }
```

### References
- tiktoken: https://github.com/openai/tiktoken
- Chunking best practices: https://www.pinecone.io/learn/chunking-strategies/

---

## 9. Citation Resolution & Linking

### Decision
Generate Docusaurus-compatible URLs from chunk metadata for clickable citations.

### Rationale
- **User trust**: Citations allow verification of chatbot answers
- **Deep learning**: Clicking citations navigates to exact textbook section for further reading
- **URL structure**: Docusaurus uses predictable URL patterns based on file paths

### Citation Link Generator
```python
def generate_citation_link(chunk: dict) -> str:
    """
    Convert chunk metadata to Docusaurus URL.
    
    Example:
    docs/week-11-12/humanoid-kinematics.md → /week-11-12/humanoid-kinematics#heading-slug
    """
    file_path = chunk["metadata"]["file_path"]
    heading = chunk["metadata"]["heading"]
    
    # Remove 'docs/' prefix and '.md' suffix
    url_path = file_path.replace('docs/', '').replace('.md', '')
    
    # Convert heading to URL fragment
    fragment = heading.lower().replace(' ', '-').replace('&', 'and')
    
    return f"/{url_path}#{fragment}"
```

### Citation Format in Response
```python
citation = {
    "text": f"Chapter {chunk['chapter']}, Section: {chunk['section']}",
    "link": generate_citation_link(chunk),
    "relevance_score": similarity_score
}
```

---

## 10. Deployment Architecture

### Decision
- **Backend**: Deploy to **Railway** (free tier: 500 hours/month, 512MB RAM)
- **Frontend**: Integrate with Docusaurus, deploy to **GitHub Pages** (existing setup)

### Rationale
- **Railway**: Zero-config deployment, automatic HTTPS, environment variables, logs
- **Alternative**: Render (free tier: 750 hours/month) or Fly.io (free tier: 3 shared VMs)
- **GitHub Pages**: Already configured for Docusaurus, no additional setup

### Backend Deployment (Railway)
```yaml
# railway.toml
[build]
builder = "DOCKERFILE"
dockerfilePath = "backend/Dockerfile"

[deploy]
numReplicas = 1
restartPolicyType = "ON_FAILURE"
```

### Frontend Integration
Add chat widget to Docusaurus layout:
```jsx
// src/theme/Layout.js
import ChatWidget from '../components/ChatWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget apiUrl={process.env.REACT_APP_CHAT_API_URL} />
    </>
  );
}
```

### Environment Variables
```bash
# Backend (.env)
GEMINI_API_KEY=your_key
QDRANT_URL=https://your_cluster.qdrant.io
QDRANT_API_KEY=your_key
NEON_DATABASE_URL=postgresql://user:pass@host/db

# Frontend (.env)
REACT_APP_CHAT_API_URL=https://your-railway-app.up.railway.app
```

### References
- Railway docs: https://docs.railway.app/
- Docusaurus theming: https://docusaurus.io/docs/swizzling

---

## Summary of Resolved Unknowns

| Unknown | Resolution |
|---------|-----------|
| Free AI model for chat | Google Gemini 1.5 Flash (15 RPM free tier) |
| Free embedding model | Google text-embedding-004 (1500 req/day) + Sentence Transformers fallback |
| Vector database | Qdrant Cloud (1GB free tier) |
| Conversation storage | Neon Postgres (0.5GB free tier) |
| Backend framework | FastAPI 0.104+ with async patterns |
| Frontend UI | Custom React chat component using @chatscope/chat-ui-kit-react |
| Text selection | DOM Range API with React state |
| Chunking strategy | 300-500 tokens per chunk, 50-token overlap |
| Citation linking | Docusaurus URL generation from chunk metadata |
| Deployment | Railway (backend) + GitHub Pages (frontend) |

---

**Next Steps**: Proceed to Phase 1 (data-model.md, contracts/, quickstart.md)
