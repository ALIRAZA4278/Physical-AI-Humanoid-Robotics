# Implementation Plan: Integrated RAG Chatbot

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`
**Status**: In Progress

---

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot into the Physical AI & Humanoid Robotics Docusaurus book. The system provides two modes: **RAG Mode** (default) for question-answering with vector retrieval from Qdrant, and **Selected Text Only Mode** where retrieval is bypassed and answers are grounded strictly in user-highlighted text. The architecture uses FastAPI backend, OpenAI Agents/ChatKit SDK, Qdrant Cloud for vectors, and Neon Serverless Postgres for metadata and audit logs.

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/React 18 (frontend)
**Primary Dependencies**: FastAPI, OpenAI SDK, qdrant-client, psycopg2, React
**Storage**: Qdrant Cloud Free Tier (vectors), Neon Serverless Postgres (metadata/logs)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Web (Docusaurus on GitHub Pages + serverless backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <1.5s RAG response, <1s selected-text response, 100 concurrent users
**Constraints**: Free tier limits (Qdrant 1GB, Neon 0.5GB), OpenAI API costs, cold start latency
**Scale/Scope**: ~50 documentation pages, ~500 chunks, single-region deployment

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status |
|-----------|-------------|--------|
| **I. Accuracy** | All answers grounded in retrieved/selected context | ✅ FR-033 to FR-035 enforce grounding |
| **II. Clarity** | Flesch-Kincaid grade 10-12, academic tone | ✅ Style rules defined in spec |
| **III. Reproducibility** | Pipelines documented, schemas specified | ✅ Data model + API contracts in spec |
| **IV. Rigor** | 15+ APA sources, evaluation benchmarks | ✅ Evaluation requirements in spec |
| **V. Grounding** | No hallucination, selection-only skips retrieval | ✅ FR-008 explicitly skips Qdrant |

**Gate Status**: PASSED

---

## Architecture Overview

### High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      FRONTEND LAYER (Docusaurus Book UI)                     │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌────────────────┐  ┌────────────────────┐  ┌─────────────────────────┐   │
│  │ ChatBot Widget │  │ TextSelection      │  │ Chat History Panel      │   │
│  │ (Floating UI)  │  │ Handler            │  │ + System Messages       │   │
│  │                │  │                    │  │                         │   │
│  │ • Open/Close   │  │ • mouseup listener │  │ • Message list          │   │
│  │ • Mode toggle  │  │ • Selection tooltip│  │ • Streaming display     │   │
│  │ • Input field  │  │ • "Ask about this" │  │ • Citations with hover  │   │
│  └───────┬────────┘  └─────────┬──────────┘  └────────────┬────────────┘   │
│          │                     │                          │                 │
│          └─────────────────────┼──────────────────────────┘                 │
│                                │                                            │
│  ┌─────────────────────────────┴────────────────────────────────────────┐  │
│  │                    ChatContext Provider (React)                       │  │
│  │  • Session management  • Mode state  • Message history  • API client  │  │
│  └───────────────────────────────┬──────────────────────────────────────┘  │
└──────────────────────────────────┼──────────────────────────────────────────┘
                                   │ HTTPS REST API (fetch + streaming)
                                   ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         BACKEND LAYER (FastAPI)                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                │
│  │ POST /api/chat │  │ POST /api/     │  │ GET /api/health│                │
│  │                │  │ embed-book     │  │                │                │
│  │ • RAG mode     │  │                │  │ • Qdrant check │                │
│  │ • Selected-only│  │ • Load markdown│  │ • Postgres     │                │
│  │ • Streaming    │  │ • Chunk + embed│  │ • OpenAI ping  │                │
│  └───────┬────────┘  └───────┬────────┘  └────────────────┘                │
│          │                   │                                              │
│  ┌───────┴───────────────────┴──────────────────────────────────────────┐  │
│  │                      RAG Service Layer                                │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │  │
│  │  │ Retriever       │  │ Generator       │  │ Context Assembler   │   │  │
│  │  │                 │  │                 │  │                     │   │  │
│  │  │ • Query embed   │  │ • OpenAI Agent  │  │ • Context window    │   │  │
│  │  │ • Qdrant search │  │ • System prompt │  │ • Token compression │   │  │
│  │  │ • Top-k filter  │  │ • Grounding     │  │ • Safety checks     │   │  │
│  │  └────────┬────────┘  └────────┬────────┘  └──────────┬──────────┘   │  │
│  └───────────┼────────────────────┼──────────────────────┼──────────────┘  │
└──────────────┼────────────────────┼──────────────────────┼──────────────────┘
               │                    │                      │
      ┌────────┴────────┐  ┌────────┴────────┐  ┌──────────┴──────────┐
      ▼                 ▼  ▼                 ▼  ▼                     ▼
┌────────────────┐  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│  Qdrant Cloud  │  │  Neon Postgres │  │  OpenAI API    │  │ Local FS/GitHub│
│  (Vectors)     │  │  (Metadata)    │  │  (LLM + Embed) │  │ (Book Source)  │
│                │  │                │  │                │  │                │
│ • book_sections│  │ • book_sections│  │ • GPT-4o-mini  │  │ • docs/*.md    │
│ • 1536 dims    │  │ • chat_logs    │  │ • text-embed-  │  │ • Markdown     │
│ • Cosine dist  │  │ • sessions     │  │   3-small      │  │                │
└────────────────┘  └────────────────┘  └────────────────┘  └────────────────┘
```

### Data Flow Diagrams

#### Flow A: Book Ingestion Pipeline

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   docs/*.md  │────▶│  Markdown    │────▶│   Chunker    │────▶│  Embeddings  │
│  (Source)    │     │   Loader     │     │  (300-800    │     │  Generator   │
└──────────────┘     └──────────────┘     │   tokens)    │     │  (OpenAI)    │
                                          └──────────────┘     └──────┬───────┘
                                                                      │
                     ┌──────────────────────────────────────────────┬─┘
                     │                                              │
                     ▼                                              ▼
              ┌──────────────┐                              ┌──────────────┐
              │    Qdrant    │                              │    Neon      │
              │  (Vectors +  │                              │  (Metadata)  │
              │   Payload)   │                              │              │
              └──────────────┘                              └──────────────┘
```

#### Flow B: RAG Mode Query

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│    User      │────▶│  POST /api/  │────▶│   Query      │────▶│   Qdrant     │
│   Question   │     │    chat      │     │  Embedding   │     │   Search     │
│  mode: "rag" │     │              │     │              │     │  (top-k=5)   │
└──────────────┘     └──────────────┘     └──────────────┘     └──────┬───────┘
                                                                      │
┌──────────────────────────────────────────────────────────────────── │
│                                                                     │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐        │
│  │   Response   │◀────│   OpenAI     │◀────│   Context    │◀───────┘
│  │  + Citations │     │   Agent      │     │   Assembly   │
│  │  + Sources   │     │  (Grounded)  │     │  (Retrieved  │
│  └──────────────┘     └──────────────┘     │   chunks)    │
│                                            └──────────────┘
│
└─────────▶ Log to Postgres chat_logs
```

#### Flow C: Selected Text Only Mode

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│    User      │────▶│  POST /api/  │────▶│   SKIP       │────▶│   Context    │
│   Question   │     │    chat      │     │   Qdrant     │     │  = Selected  │
│  + Selected  │     │              │     │  Retrieval   │     │   Text ONLY  │
│    Text      │     │              │     │   ✗          │     │              │
│ mode:        │     └──────────────┘     └──────────────┘     └──────┬───────┘
│ "selected_   │                                                      │
│  only"       │                                                      │
└──────────────┘                                                      │
                                                                      │
┌──────────────────────────────────────────────────────────────────── │
│                                                                     │
│  ┌──────────────┐     ┌──────────────┐                             │
│  │   Response   │◀────│   OpenAI     │◀────────────────────────────┘
│  │  "Based on   │     │   Agent      │
│  │   selected   │     │  (Grounded   │
│  │   text only" │     │   in selection│
│  └──────────────┘     │   ONLY)      │
│                       └──────────────┘
│
└─────────▶ Log to Postgres chat_logs (mode: 'selected_only')
```

---

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── spec.md              # Feature specification
├── plan.md              # This file (implementation plan)
├── research.md          # Research notes (RAG patterns, embedding strategies)
├── data-model.md        # Database schema details
├── quickstart.md        # Developer setup guide
├── contracts/           # API contracts
│   ├── chat-api.md      # POST /api/chat contract
│   ├── embed-api.md     # POST /api/embed-book contract
│   └── health-api.md    # GET /api/health contract
└── tasks.md             # Task list (/sp.tasks output)
```

### Source Code (repository root)

```text
my-research-paper/
├── api/                                    # FastAPI backend
│   ├── main.py                             # FastAPI app entry point + CORS
│   ├── requirements.txt                    # Python dependencies
│   ├── .env.example                        # Environment variable template
│   ├── config.py                           # Pydantic settings management
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── chat.py                         # POST /api/chat endpoint
│   │   ├── ingest.py                       # POST /api/embed-book endpoint
│   │   └── health.py                       # GET /api/health endpoint
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py                   # OpenAI embedding generation
│   │   ├── retriever.py                    # Qdrant vector search
│   │   ├── generator.py                    # OpenAI Agent/ChatKit generation
│   │   ├── rag.py                          # RAG orchestration service
│   │   └── chunker.py                      # Document chunking logic
│   ├── models/
│   │   ├── __init__.py
│   │   ├── schemas.py                      # Pydantic request/response models
│   │   └── database.py                     # SQLAlchemy models for Neon
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── markdown_loader.py              # Load docs from filesystem
│   │   └── token_counter.py                # Token counting utility
│   ├── prompts/
│   │   ├── rag_system.txt                  # System prompt for RAG mode
│   │   └── selected_only_system.txt        # System prompt for selected-only
│   └── scripts/
│       └── ingest_docs.py                  # One-time ingestion script
│
├── src/                                    # Docusaurus frontend
│   ├── components/
│   │   └── ChatBot/
│   │       ├── index.tsx                   # Main ChatBot component
│   │       ├── ChatBot.module.css          # Scoped styles
│   │       ├── ChatWindow.tsx              # Chat message window
│   │       ├── ChatInput.tsx               # Input field component
│   │       ├── Message.tsx                 # Single message component
│   │       ├── Citation.tsx                # Citation with hover preview
│   │       ├── ModeToggle.tsx              # RAG/Selected-only toggle
│   │       ├── TextSelectionHandler.tsx    # Selection detection + tooltip
│   │       └── hooks/
│   │           ├── useChat.ts              # Chat API hook with streaming
│   │           ├── useTextSelection.ts     # Selection detection hook
│   │           └── useChatHistory.ts       # Conversation history hook
│   ├── context/
│   │   └── ChatContext.tsx                 # Global chat state provider
│   └── theme/
│       └── Root.tsx                        # Theme wrapper for chatbot
│
├── tests/
│   ├── api/                                # Backend tests
│   │   ├── test_chat.py                    # Chat endpoint tests
│   │   ├── test_retriever.py               # Retrieval tests
│   │   └── test_generator.py               # Generation tests
│   └── frontend/                           # Frontend tests
│       └── ChatBot.test.tsx                # Component tests
│
└── docs/                                   # Existing book documentation
```

**Structure Decision**: Web application structure selected. Backend (api/) handles RAG logic, embeddings, and database operations. Frontend (src/components/ChatBot/) integrates with existing Docusaurus via theme wrapper.

---

## Phase 1: Research & Foundations

### 1.1 RAG Pattern Comparison

| Pattern | Pros | Cons | Decision |
|---------|------|------|----------|
| **Standard RAG** | Simple, proven | May miss context | Baseline |
| **Hybrid Search** | Better precision | More complex | Consider for Phase 2 |
| **Rerankers** | Improved relevance | Latency cost | Optional enhancement |

**Decision**: Start with Standard RAG (vector-only). Add hybrid/reranking if retrieval quality insufficient.

### 1.2 Embedding Strategy

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **Model** | text-embedding-3-small | Cost-effective, 1536 dims |
| **Chunk Size** | 300-800 tokens | Balance context vs precision |
| **Overlap** | 50-100 tokens | Preserve context across boundaries |
| **Metadata** | module, chapter, section_title, source_path, paragraph_index | Enable filtering + citations |

### 1.3 OpenAI Agents vs ChatKit vs Raw API

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **OpenAI Agents SDK** | Tool calling, structured | Newer, less examples | Primary choice |
| **ChatKit SDK** | Higher-level abstractions | Less control | Fallback |
| **Raw OpenAI API** | Full control | More boilerplate | If SDKs insufficient |

**Decision**: Use OpenAI Agents SDK for tool invocation (retrieval, SQL). Fall back to raw API for streaming if needed.

### 1.4 Qdrant + Neon Performance Assessment

| Service | Free Tier Limit | Our Usage | Headroom |
|---------|-----------------|-----------|----------|
| **Qdrant** | 1GB vectors | ~50MB (500 chunks × 6KB) | 20x |
| **Neon** | 0.5GB storage | ~10MB (metadata + logs) | 50x |
| **OpenAI** | Pay-per-use | ~$0.10/1K queries | Budget required |

---

## Phase 2: System Architecture & Scaffolding

### 2.1 FastAPI Project Setup

**Dependencies** (`api/requirements.txt`):
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
tiktoken>=0.5.0
python-multipart>=0.0.6
```

### 2.2 Qdrant Collection Configuration

```python
collection_config = {
    "name": "book_sections",
    "vectors_config": {
        "size": 1536,  # text-embedding-3-small dimensions
        "distance": "Cosine"
    },
    "payload_schema": {
        "module": {"type": "keyword", "indexed": True},
        "chapter": {"type": "keyword", "indexed": True},
        "section_title": {"type": "text"},
        "source_path": {"type": "keyword", "indexed": True},
        "paragraph_index": {"type": "integer"},
        "token_count": {"type": "integer"},
        "text": {"type": "text"}
    }
}
```

### 2.3 Neon Postgres Schema

```sql
-- Book sections table (mirrors Qdrant metadata for queries)
CREATE TABLE book_sections (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    qdrant_id UUID NOT NULL UNIQUE,
    module TEXT NOT NULL,
    chapter TEXT NOT NULL,
    section_title TEXT,
    source_path TEXT NOT NULL,
    paragraph_index INT NOT NULL,
    text TEXT NOT NULL,
    citation TEXT,
    token_count INT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Chat logs (audit trail)
CREATE TABLE chat_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID,
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    mode TEXT NOT NULL CHECK (mode IN ('rag', 'selected_only')),
    sources JSONB,
    context_used TEXT,
    latency_ms INT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_book_sections_module ON book_sections(module);
CREATE INDEX idx_book_sections_chapter ON book_sections(chapter);
CREATE INDEX idx_chat_logs_session ON chat_logs(session_id);
CREATE INDEX idx_chat_logs_created ON chat_logs(created_at);
```

### 2.4 Docusaurus Plugin Scaffold

```tsx
// src/theme/Root.tsx
import React from 'react';
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

---

## Phase 3: Ingestion Pipeline

### 3.1 Pipeline Steps

1. **Load Markdown** - Scan `docs/` for `.md` files
2. **Parse Structure** - Extract frontmatter, headings, paragraphs
3. **Chunk Documents** - Split into 300-800 token segments with overlap
4. **Generate Embeddings** - Batch process via OpenAI API
5. **Store in Qdrant** - Upsert vectors with payload
6. **Store in Postgres** - Insert metadata rows for audit

### 3.2 Chunking Strategy

```python
CHUNK_CONFIG = {
    "min_tokens": 300,
    "max_tokens": 800,
    "overlap_tokens": 50,
    "preserve_paragraphs": True,
    "include_heading_context": True
}
```

### 3.3 Idempotency

- Use deterministic UUIDs based on `source_path + paragraph_index`
- Upsert to Qdrant (overwrite existing)
- Use `ON CONFLICT` in Postgres
- Track `created_at` for versioning

---

## Phase 4: Retrieval & Reasoning Logic

### 4.1 Retriever Implementation

```python
async def retrieve(query: str, top_k: int = 5) -> List[Chunk]:
    # 1. Generate query embedding
    query_embedding = await embed_text(query)

    # 2. Search Qdrant
    results = await qdrant.search(
        collection_name="book_sections",
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True
    )

    # 3. Convert to Chunk objects
    return [Chunk.from_qdrant(r) for r in results]
```

### 4.2 Selection-Only Pipeline

```python
async def answer_selected_only(question: str, selected_text: str) -> Response:
    # NO QDRANT RETRIEVAL
    context = selected_text

    response = await generate_answer(
        question=question,
        context=context,
        system_prompt=SELECTED_ONLY_SYSTEM_PROMPT,
        mode="selected_only"
    )

    return Response(
        answer=response.text,
        citations=[],  # No citations in selected-only mode
        sources=[],
        context_used=selected_text,
        mode="selected_only"
    )
```

### 4.3 System Prompts

**RAG Mode System Prompt** (`prompts/rag_system.txt`):
```
You are a helpful assistant for the "Physical AI & Humanoid Robotics" book.

CRITICAL RULES:
1. Answer ONLY based on the provided context from the book.
2. If the context does not contain the answer, say: "The book does not contain enough information to answer this question."
3. NEVER fabricate or guess information not in the context.
4. Always cite sources using [Module X: Section Name] format.

Style:
- Academic tone appropriate for CS/robotics students
- Flesch-Kincaid grade 10-12 readability
- Clear, concise paragraphs
- Explain technical terms on first use

Context:
{context}

Question: {question}
```

**Selected Text Only System Prompt** (`prompts/selected_only_system.txt`):
```
You are a helpful assistant for the "Physical AI & Humanoid Robotics" book.

CRITICAL RULES:
1. Answer ONLY based on the SELECTED TEXT provided below.
2. Do NOT use any external knowledge.
3. If the selected text does not contain the answer, say: "The selected text does not contain enough information to answer this question."
4. NEVER fabricate or guess information.

Selected Text:
{selected_text}

Question: {question}
```

---

## Phase 5: Frontend Chat UI

### 5.1 Component Architecture

```
ChatBot/
├── index.tsx              # Container, mounts in Root.tsx
├── ChatWindow.tsx         # Scrollable message list
├── ChatInput.tsx          # Text input + send button + mode toggle
├── Message.tsx            # User/assistant message bubble
├── Citation.tsx           # Clickable citation with hover preview
├── ModeToggle.tsx         # RAG vs Selected-only switch
├── TextSelectionHandler.tsx  # Detects selection, shows tooltip
└── hooks/
    ├── useChat.ts         # API calls, streaming, state
    ├── useTextSelection.ts  # Selection detection
    └── useChatHistory.ts  # Local storage persistence
```

### 5.2 Key Features

| Feature | Implementation |
|---------|----------------|
| **Floating Widget** | Fixed position bottom-right, toggle open/close |
| **Streaming** | Server-Sent Events or chunked response |
| **Mode Toggle** | Switch between RAG and Selected-only |
| **Text Selection** | mouseup listener, floating tooltip |
| **Citations** | Hover preview, click to navigate |
| **Copy Answer** | Clipboard API with toast notification |
| **Theme Aware** | CSS variables from Docusaurus |

### 5.3 API Client with Streaming

```typescript
async function* streamChat(request: ChatRequest): AsyncGenerator<string> {
  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request),
  });

  const reader = response.body?.getReader();
  const decoder = new TextDecoder();

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    yield decoder.decode(value);
  }
}
```

---

## Phase 6: QA, Testing, and Validation

### 6.1 Test Categories

| Category | Focus | Tools |
|----------|-------|-------|
| **Unit** | Chunker, embeddings, token counter | pytest |
| **Integration** | Qdrant search, Postgres CRUD, OpenAI calls | pytest + mocks |
| **Contract** | API request/response validation | pytest + pydantic |
| **E2E** | Full RAG + selected-only flows | pytest + httpx |
| **Frontend** | Component rendering, interactions | Jest + RTL |

### 6.2 Quality Validation

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Retrieval Recall@5** | >80% | Manual evaluation of 20 queries |
| **Citation Accuracy** | 100% | All citations map to actual chunks |
| **Hallucination Rate** | 0% | Adversarial prompt testing |
| **RAG Latency p95** | <1.5s | Load testing |
| **Selected-only Latency p95** | <1s | Load testing |

### 6.3 Acceptance Test Cases

```
✓ User asks question → receives grounded answer with citations
✓ User asks about uncovered topic → receives graceful decline
✓ User selects text + asks → receives answer from selection only
✓ Empty selection + selected-only mode → prompts to select text
✓ Qdrant unavailable → graceful error message
✓ OpenAI unavailable → graceful error message
✓ Citation links → navigate to correct book section
✓ Streaming → text appears progressively
```

---

## Phase 7: Deployment

### 7.1 Backend Deployment (Hugging Face Spaces)

**Configuration** (already exists at `api/`):
- FastAPI app in `main.py`
- Environment variables via Secrets
- CORS configured for GitHub Pages domain

**Environment Variables**:
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.us-east-1.aws.cloud.qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
CORS_ORIGINS=https://aliraza4278.github.io
ENVIRONMENT=production
```

### 7.2 Frontend Deployment

- Embedded in Docusaurus build
- Deployed to GitHub Pages via existing workflow
- API URL configured via environment variable

### 7.3 CI/CD Updates

```yaml
# .github/workflows/deploy.yml additions
- name: Build with API URL
  run: npm run build
  env:
    REACT_APP_API_URL: ${{ secrets.API_URL }}
```

---

## Decisions Needing Documentation

### ADR Candidates

| Decision | Significance | ADR Status |
|----------|-------------|------------|
| GPT-4o-mini over GPT-4o | Cost/quality tradeoff | Pending |
| Qdrant over Pinecone | Free tier + Python SDK | Pending |
| Selected-only skips retrieval | Core feature differentiation | Pending |
| 300-800 token chunks | Balance precision/context | Pending |
| OpenAI Agents SDK | Tool calling capability | Pending |

📋 **Architectural decisions detected**: Multiple significant decisions need documentation. Run `/sp.adr` to create ADRs for these decisions.

---

## Risk Analysis

### Top 3 Risks

| Risk | Impact | Mitigation | Kill Switch |
|------|--------|------------|-------------|
| **OpenAI API costs** | Financial | Rate limiting, caching, smaller model | Disable chatbot |
| **Hallucination despite grounding** | Trust erosion | Strict prompts, adversarial testing | Fallback to "I don't know" |
| **Cold start latency** | UX degradation | Loading states, warm-up pings | N/A |

### Blast Radius Assessment

- **Qdrant failure**: Chat returns error, book still functional
- **Postgres failure**: No history persistence, chat still works
- **OpenAI failure**: Chat unavailable, book still functional
- **Frontend bug**: Chat broken, book still functional

---

## Complexity Tracking

> No constitution violations requiring justification.

---

## Implementation Order

| Phase | Duration | Deliverables |
|-------|----------|--------------|
| **Phase 1** | Research | RAG pattern decision, embedding config |
| **Phase 2** | Scaffolding | FastAPI structure, Qdrant collection, Postgres schema |
| **Phase 3** | Ingestion | Markdown → chunks → embeddings → storage |
| **Phase 4** | RAG Logic | Retriever, generator, both pipelines |
| **Phase 5** | Frontend | ChatBot widget, text selection, streaming |
| **Phase 6** | Testing | Unit, integration, acceptance tests |
| **Phase 7** | Deployment | HF Spaces backend, GitHub Pages frontend |

---

## Decision Log

| Decision | Options Considered | Rationale |
|----------|-------------------|-----------|
| Vector DB | Qdrant, Pinecone, Weaviate | Qdrant: generous free tier (1GB), excellent Python SDK, cosine distance |
| Embedding Model | OpenAI, Cohere, local | OpenAI text-embedding-3-small: cost-effective ($0.00002/1K tokens), 1536 dims |
| Generation Model | GPT-4o-mini, GPT-4o, Claude | GPT-4o-mini: best cost/quality balance for doc Q&A |
| Backend Framework | FastAPI, Flask, Express | FastAPI: async support, OpenAPI docs, Pydantic validation |
| Postgres Host | Neon, Supabase, Railway | Neon: serverless scales to zero, 0.5GB free tier |
| Frontend Pattern | Floating widget, sidebar, page | Floating widget: non-intrusive, always accessible |
| Chunking Size | 200-500, 300-800, 500-1000 | 300-800 tokens: balance context window vs retrieval precision |
| Selected-only Mode | Skip retrieval, filter retrieval | Skip retrieval entirely: explicit grounding guarantee |
