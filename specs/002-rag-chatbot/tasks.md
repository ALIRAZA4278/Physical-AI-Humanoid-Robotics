# Implementation Tasks: RAG Chatbot

**Feature**: 002-rag-chatbot
**Created**: 2025-12-05
**Status**: Complete

## Completed Tasks

### Phase 1: Backend Foundation

- [x] **TASK-001**: Create FastAPI project structure
  - Created `api/` directory with modular structure
  - Set up `main.py` with FastAPI app, CORS, lifespan
  - Created `config.py` with pydantic-settings

- [x] **TASK-002**: Define Pydantic schemas
  - `ChatRequest`, `ChatResponse` for chat endpoint
  - `Source`, `Message` for response data
  - `IngestResponse`, `HealthResponse` for admin endpoints

- [x] **TASK-003**: Set up SQLAlchemy models for Neon Postgres
  - `Conversation` model for session tracking
  - `MessageModel` for chat history
  - Database initialization and session management

### Phase 2: Document Processing

- [x] **TASK-004**: Build Markdown loader
  - Load all `.md` files from `docs/` directory
  - Parse frontmatter metadata
  - Clean content for embedding (remove code blocks, images)
  - Extract module and title information

- [x] **TASK-005**: Implement document chunker
  - Token-based chunking (~400 tokens per chunk)
  - Overlap for context continuity
  - Preserve paragraph boundaries
  - Generate unique chunk IDs

### Phase 3: RAG Services

- [x] **TASK-006**: Create embedding service
  - OpenAI `text-embedding-3-small` integration
  - Single and batch embedding methods
  - Error handling and logging

- [x] **TASK-007**: Build retriever service
  - Qdrant client configuration
  - Collection creation and management
  - Semantic search with relevance scores
  - Merge and deduplicate results for selected text

- [x] **TASK-008**: Implement generator service
  - OpenAI GPT-4o-mini integration
  - RAG prompt template with citations
  - Context assembly from chunks
  - Conversation history support

- [x] **TASK-009**: Create RAG orchestrator
  - Combine retrieval and generation
  - Conversation management
  - Message persistence
  - Health checks

### Phase 4: API Endpoints

- [x] **TASK-010**: Build chat router
  - `POST /api/chat` endpoint
  - Session and conversation handling
  - Selected text support
  - Error handling

- [x] **TASK-011**: Build ingest router
  - `POST /api/ingest` async endpoint
  - `POST /api/ingest/sync` synchronous endpoint
  - `GET /api/ingest/status` status check
  - Background task processing

- [x] **TASK-012**: Create ingestion script
  - Standalone CLI tool
  - Force re-ingestion option
  - Progress logging
  - Statistics output

### Phase 5: Frontend Components

- [x] **TASK-013**: Create chat API hook
  - `useChat` hook for message management
  - API integration with fetch
  - Loading and error states
  - Conversation loading

- [x] **TASK-014**: Create text selection hook
  - `useTextSelection` hook
  - Selection change detection
  - Position calculation for tooltip
  - Content area filtering

- [x] **TASK-015**: Build ChatContext provider
  - Global chat state management
  - Session ID generation
  - Selected text handling
  - API URL configuration

- [x] **TASK-016**: Build Message component
  - User and assistant message styles
  - Markdown-like formatting
  - Code block rendering
  - Source link display

- [x] **TASK-017**: Build ChatInput component
  - Auto-resizing textarea
  - Send button with loading state
  - Selected text banner
  - Enter key handling

- [x] **TASK-018**: Build ChatWindow component
  - Header with actions
  - Messages container with scroll
  - Welcome message
  - Loading indicator

- [x] **TASK-019**: Build TextSelectionHandler
  - Floating tooltip on selection
  - "Ask about this" button
  - Position tracking
  - Chat integration

- [x] **TASK-020**: Build main ChatBot component
  - Floating action button
  - Chat window toggle
  - Animation on mount
  - Escape key handling

- [x] **TASK-021**: Create CSS module styles
  - Light/dark mode support
  - Responsive design
  - Animation keyframes
  - Component-specific styles

### Phase 6: Integration

- [x] **TASK-022**: Create theme Root wrapper
  - ChatProvider integration
  - ChatBot rendering
  - API URL configuration

### Phase 7: Deployment Configuration

- [x] **TASK-023**: Create Vercel configuration
  - `vercel.json` for Python deployment
  - Route configuration

- [x] **TASK-024**: Create Render configuration
  - `render.yaml` blueprint
  - Environment variable template

- [x] **TASK-025**: Create setup documentation
  - Step-by-step guide
  - Architecture overview
  - Troubleshooting section

## File Summary

### Backend (`api/`)

| File | Purpose |
|------|---------|
| `main.py` | FastAPI application entry point |
| `config.py` | Settings management |
| `requirements.txt` | Python dependencies |
| `.env.example` | Environment template |
| `.gitignore` | Git ignore patterns |
| `vercel.json` | Vercel deployment config |
| `render.yaml` | Render deployment config |
| `routers/chat.py` | Chat API endpoints |
| `routers/ingest.py` | Ingestion API endpoints |
| `services/embeddings.py` | OpenAI embedding service |
| `services/retriever.py` | Qdrant retrieval service |
| `services/generator.py` | OpenAI generation service |
| `services/rag.py` | RAG orchestration |
| `models/schemas.py` | Pydantic schemas |
| `models/database.py` | SQLAlchemy models |
| `utils/markdown_loader.py` | Document loader |
| `utils/chunker.py` | Document chunker |
| `scripts/ingest_docs.py` | Ingestion CLI script |

### Frontend (`src/`)

| File | Purpose |
|------|---------|
| `components/ChatBot/index.tsx` | Main ChatBot component |
| `components/ChatBot/ChatWindow.tsx` | Chat window container |
| `components/ChatBot/ChatInput.tsx` | Input field component |
| `components/ChatBot/Message.tsx` | Message display component |
| `components/ChatBot/TextSelectionHandler.tsx` | Selection tooltip |
| `components/ChatBot/ChatBot.module.css` | Component styles |
| `components/ChatBot/hooks/useChat.ts` | Chat API hook |
| `components/ChatBot/hooks/useTextSelection.ts` | Selection hook |
| `context/ChatContext.tsx` | Global chat context |
| `theme/Root.tsx` | Docusaurus theme wrapper |

### Specs (`specs/002-rag-chatbot/`)

| File | Purpose |
|------|---------|
| `spec.md` | Feature specification |
| `plan.md` | Implementation plan |
| `tasks.md` | This task list |
| `README.md` | Setup & deployment guide |

## Next Steps (Manual)

1. Create Qdrant Cloud account and cluster
2. Create Neon Postgres database
3. Configure OpenAI API key
4. Run document ingestion
5. Deploy backend to Render/Vercel
6. Update frontend API URL
7. Deploy frontend via GitHub Pages
8. Test end-to-end flow
