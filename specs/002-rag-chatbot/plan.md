# Implementation Plan: Integrated RAG Chatbot

**Feature**: 002-rag-chatbot
**Created**: 2025-12-05
**Status**: In Progress

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         FRONTEND (Docusaurus)                        │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  ChatBot Widget  │  │ TextSelection    │  │  Chat Context    │  │
│  │  (Floating UI)   │  │ Handler          │  │  Provider        │  │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘  │
│           │                     │                     │            │
│           └─────────────────────┴─────────────────────┘            │
│                                 │                                   │
└─────────────────────────────────┼───────────────────────────────────┘
                                  │ HTTPS REST API
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         BACKEND (FastAPI)                           │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐  │
│  │  /api/chat       │  │  /api/embed      │  │  /api/health     │  │
│  │  Chat endpoint   │  │  Ingestion       │  │  Status check    │  │
│  └────────┬─────────┘  └────────┬─────────┘  └──────────────────┘  │
│           │                     │                                   │
│  ┌────────┴─────────────────────┴────────┐                         │
│  │           RAG Service Layer           │                         │
│  │  ┌─────────────┐  ┌─────────────────┐ │                         │
│  │  │ Retriever   │  │ Generator       │ │                         │
│  │  │ (Qdrant)    │  │ (OpenAI GPT)    │ │                         │
│  │  └──────┬──────┘  └───────┬─────────┘ │                         │
│  └─────────┼─────────────────┼───────────┘                         │
│            │                 │                                      │
└────────────┼─────────────────┼──────────────────────────────────────┘
             │                 │
    ┌────────┴────────┐  ┌─────┴──────┐
    ▼                 ▼  ▼            ▼
┌────────────┐  ┌────────────┐  ┌────────────┐
│  Qdrant    │  │   Neon     │  │  OpenAI    │
│  Cloud     │  │  Postgres  │  │   API      │
│ (Vectors)  │  │  (History) │  │ (LLM+Embed)│
└────────────┘  └────────────┘  └────────────┘
```

## Directory Structure

```
my-research-paper/
├── api/                              # FastAPI backend
│   ├── main.py                       # FastAPI app entry point
│   ├── requirements.txt              # Python dependencies
│   ├── .env.example                  # Environment template
│   ├── config.py                     # Configuration management
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── chat.py                   # Chat endpoint
│   │   └── ingest.py                 # Embedding ingestion endpoint
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py             # OpenAI embedding service
│   │   ├── retriever.py              # Qdrant retrieval service
│   │   ├── generator.py              # OpenAI generation service
│   │   └── rag.py                    # RAG orchestration
│   ├── models/
│   │   ├── __init__.py
│   │   ├── schemas.py                # Pydantic models
│   │   └── database.py               # Neon Postgres models
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── chunker.py                # Document chunking logic
│   │   └── markdown_loader.py        # Load docs from filesystem
│   └── scripts/
│       └── ingest_docs.py            # One-time ingestion script
│
├── src/
│   ├── components/
│   │   └── ChatBot/
│   │       ├── index.tsx             # Main ChatBot component
│   │       ├── ChatBot.module.css    # Scoped styles
│   │       ├── ChatWindow.tsx        # Chat message window
│   │       ├── ChatInput.tsx         # Input field component
│   │       ├── Message.tsx           # Single message component
│   │       ├── TextSelectionHandler.tsx  # Selection detection
│   │       └── hooks/
│   │           ├── useChat.ts        # Chat API hook
│   │           └── useTextSelection.ts   # Selection hook
│   ├── context/
│   │   └── ChatContext.tsx           # Global chat state
│   └── theme/
│       └── Root.tsx                  # Theme wrapper for chatbot
│
└── docs/                             # (existing documentation)
```

## Phase 1: Backend Foundation

### 1.1 FastAPI Project Setup

**Files to create:**
- `api/main.py` - FastAPI application with CORS, middleware
- `api/requirements.txt` - Dependencies
- `api/.env.example` - Environment variable template
- `api/config.py` - Settings management with pydantic-settings

**Dependencies:**
```
fastapi>=0.109.0
uvicorn[standard]>=0.27.0
openai>=1.12.0
qdrant-client>=1.7.0
psycopg2-binary>=2.9.9
sqlalchemy>=2.0.25
python-dotenv>=1.0.0
pydantic-settings>=2.1.0
httpx>=0.26.0
```

### 1.2 Document Ingestion Pipeline

**Process:**
1. Load all `.md` files from `docs/` directory
2. Parse frontmatter and content
3. Chunk documents into ~400 token segments with overlap
4. Generate embeddings using `text-embedding-3-small`
5. Store in Qdrant with metadata (source path, title, module)

**Chunking Strategy:**
- Chunk size: 400 tokens (~1600 characters)
- Overlap: 50 tokens
- Preserve paragraph boundaries where possible
- Include heading hierarchy in metadata

### 1.3 Qdrant Cloud Setup

**Collection Configuration:**
```python
{
    "name": "book_content",
    "vectors": {
        "size": 1536,  # text-embedding-3-small dimensions
        "distance": "Cosine"
    },
    "payload_schema": {
        "source_path": "keyword",
        "title": "text",
        "module": "keyword",
        "heading": "text",
        "content": "text"
    }
}
```

### 1.4 Neon Postgres Schema

**Tables:**
```sql
-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    context JSONB,  -- Selected text, sources, etc.
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for efficient session lookups
CREATE INDEX idx_conversations_session ON conversations(session_id);
CREATE INDEX idx_messages_conversation ON messages(conversation_id);
```

## Phase 2: RAG Implementation

### 2.1 Retrieval Service

**Algorithm:**
1. Embed user query using `text-embedding-3-small`
2. Search Qdrant for top-k similar chunks (k=5)
3. If selected text provided, also search with selected text as query
4. Merge and deduplicate results
5. Re-rank by relevance score
6. Return top 3-5 chunks with metadata

### 2.2 Generation Service

**Prompt Template:**
```
You are a helpful assistant for the "Physical AI & Humanoid Robotics" book.
Answer questions based ONLY on the provided context from the book.
If the answer cannot be found in the context, say so clearly.
Always cite your sources using [Module X: Section Name] format.

Context:
{retrieved_chunks}

User's selected text (if any):
{selected_text}

Previous conversation:
{conversation_history}

User question: {question}

Instructions:
- Be concise but thorough
- Use technical terms accurately
- Reference specific sections when possible
- If unsure, acknowledge limitations
```

### 2.3 API Endpoints

**POST /api/chat**
```python
Request:
{
    "message": str,
    "session_id": str,
    "conversation_id": str | None,
    "selected_text": str | None
}

Response:
{
    "response": str,
    "sources": [
        {
            "path": str,
            "title": str,
            "module": str,
            "relevance": float
        }
    ],
    "conversation_id": str
}
```

**POST /api/ingest** (Admin only)
```python
Request: None (triggers full re-ingestion)

Response:
{
    "status": "success",
    "documents_processed": int,
    "chunks_created": int
}
```

**GET /api/health**
```python
Response:
{
    "status": "healthy",
    "qdrant": "connected",
    "postgres": "connected",
    "openai": "configured"
}
```

## Phase 3: Frontend Integration

### 3.1 ChatBot Component

**Features:**
- Floating action button (bottom-right)
- Expandable chat window
- Message history with timestamps
- Markdown rendering in responses
- Source links as clickable references
- Loading states and error handling
- Theme-aware styling (light/dark)

### 3.2 Text Selection Handler

**Implementation:**
1. Listen to `mouseup` and `selectionchange` events
2. On text selection, show floating tooltip near selection
3. Tooltip has "Ask about this" button
4. Click opens chat with selected text pre-filled as context
5. Store selection in ChatContext for API call

### 3.3 Docusaurus Integration

**Approach:** Create custom theme wrapper

```tsx
// src/theme/Root.tsx
import { ChatProvider } from '../context/ChatContext';
import ChatBot from '../components/ChatBot';

export default function Root({ children }) {
  return (
    <ChatProvider>
      {children}
      <ChatBot />
    </ChatProvider>
  );
}
```

## Phase 4: Deployment

### 4.1 Backend Deployment Options

**Option A: Vercel Serverless (Recommended)**
- Create `vercel.json` in api/ folder
- Deploy as Python serverless functions
- Environment variables via Vercel dashboard

**Option B: Railway**
- Create `railway.json` or use auto-detect
- Provision Postgres directly on Railway
- Good for development/testing

**Option C: Render**
- Create `render.yaml` for service definition
- Free tier with some sleep limitations

### 4.2 Environment Variables

```env
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.cloud
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# App Config
CORS_ORIGINS=https://aliraza4278.github.io
ENVIRONMENT=production
```

### 4.3 CI/CD Updates

Add to `.github/workflows/deploy.yml`:
```yaml
# After frontend build, trigger backend deployment
- name: Deploy API
  run: |
    # Vercel CLI or Railway CLI deployment
```

## Risk Analysis

### Top 3 Risks

1. **OpenAI API Costs**
   - Risk: Unexpected API charges from heavy usage
   - Mitigation: Implement rate limiting, use smaller models, cache common queries
   - Blast Radius: Financial impact only, not functional

2. **Cold Start Latency**
   - Risk: First request after idle period takes 3-5 seconds
   - Mitigation: Implement loading state, consider always-on tier for production
   - Blast Radius: User experience degradation

3. **Embedding Quality**
   - Risk: Poor retrieval results due to chunking or embedding issues
   - Mitigation: Iterative testing, A/B testing chunk sizes, relevance feedback
   - Blast Radius: Core feature quality affected

## Implementation Order

1. **Week 1: Backend Core**
   - FastAPI project setup
   - Qdrant Cloud account and collection
   - Document ingestion script
   - Basic chat endpoint

2. **Week 2: Full RAG**
   - Neon Postgres setup
   - Conversation history
   - Complete RAG pipeline
   - API testing

3. **Week 3: Frontend**
   - ChatBot component
   - Text selection handler
   - Docusaurus integration
   - Styling and polish

4. **Week 4: Deployment & Testing**
   - Backend deployment
   - Environment configuration
   - End-to-end testing
   - Documentation

## Decision Log

| Decision | Options Considered | Rationale |
|----------|-------------------|-----------|
| Vector DB | Qdrant, Pinecone, Weaviate | Qdrant free tier is generous (1GB), good Python SDK |
| Embedding Model | OpenAI, Cohere, local | OpenAI text-embedding-3-small is cost-effective and high quality |
| Generation Model | GPT-4o-mini, GPT-4o, Claude | GPT-4o-mini balances cost and quality for documentation Q&A |
| Backend Framework | FastAPI, Flask, Express | FastAPI has async support, OpenAPI docs, Python ecosystem |
| Postgres Host | Neon, Supabase, Railway | Neon serverless scales to zero, generous free tier |
| Frontend Pattern | Floating widget, page, sidebar | Floating widget is non-intrusive, always accessible |
