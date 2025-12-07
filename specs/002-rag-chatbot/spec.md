# Feature Specification: Integrated RAG Chatbot

**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-05
**Updated**: 2025-12-07
**Status**: In Progress
**Input**: User description: "Build and embed a RAG chatbot using OpenAI Agents/ChatKit SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier"

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - RAG Mode Question Answering (Priority: P1)

A reader studying Physical AI & Humanoid Robotics wants to ask questions about the book's content without manually searching through modules. They click the chatbot icon, type their question (e.g., "What is a ROS 2 node?"), and receive an accurate answer with citations to relevant sections.

**Why this priority**: This is the core value proposition of the RAG chatbot—enabling natural language Q&A about the book content. All other features depend on this foundation.

**Independent Test**: User can ask "What is URDF?" and receive a contextually accurate response citing Module 1 content with exact section references.

**Acceptance Scenarios**:

1. **Given** a reader is on any page of the documentation site, **When** they click the chatbot button, **Then** a chat interface opens.
2. **Given** the chat interface is open, **When** the user types "What is a ROS 2 topic?" and presses Enter, **Then** they receive an accurate answer within 1.5 seconds referencing Module 1 content.
3. **Given** the user asks a question not covered in the book, **When** the chatbot cannot find relevant content, **Then** it responds: "The book does not contain enough information to answer this question."
4. **Given** an answer is generated, **When** displayed, **Then** citations include module, chapter, and section references.

---

### User Story 2 - Selected Text Only Mode (Priority: P1)

A reader is studying a specific passage about Nav2 navigation and doesn't understand a technical term. They select the text, toggle "Selected Text Only" mode, and receive an explanation derived ONLY from the selected passage—with vector retrieval disabled.

**Why this priority**: This is explicitly required functionality that differentiates the chatbot from generic RAG—strict grounding in user-selected text without any external retrieval.

**Independent Test**: User selects a paragraph about bipedal locomotion, enables "Selected Text Only" mode, asks "How does this work?", and receives an explanation derived ONLY from that text.

**Acceptance Scenarios**:

1. **Given** a reader selects text on any documentation page, **When** they release the mouse button, **Then** a tooltip/button appears offering "Ask about this".
2. **Given** "Selected Text Only" mode is active, **When** the user asks a question, **Then** the answer is derived ONLY from the selected text with no Qdrant retrieval performed.
3. **Given** "Selected Text Only" mode is active, **When** the response is generated, **Then** the UI clearly indicates "Answer based on selected text only".
4. **Given** selected text is insufficient to answer, **When** the user asks, **Then** the chatbot responds: "The selected text does not contain enough information to answer this question."

---

### User Story 3 - Citation and Source Navigation (Priority: P2)

A reader receives an answer about sensor simulation and wants to learn more. The response includes clickable links to the relevant sections that the answer was derived from, with hover preview capability.

**Why this priority**: Citations build trust, enable verification, and support deeper learning by directing users to source material.

**Independent Test**: Answer about Gazebo setup includes clickable link to `module-2-simulation/01-gazebo-setup` with hover preview.

**Acceptance Scenarios**:

1. **Given** the chatbot answers a question in RAG mode, **When** the answer is derived from specific documentation sections, **Then** clickable source links are included.
2. **Given** source links are provided, **When** the user hovers over a link, **Then** a preview of the cited content appears.
3. **Given** source links are provided, **When** the user clicks a link, **Then** they navigate to the relevant documentation page.
4. **Given** an answer draws from multiple sources, **When** displayed, **Then** all relevant sources are listed.

---

### User Story 4 - Conversation History (Priority: P2)

A returning reader wants to continue their previous conversation about NVIDIA Isaac Sim setup. They open the chatbot and see their previous conversation history preserved in the chat window.

**Why this priority**: Chat history enables multi-message context and reference. Requires Neon Postgres integration.

**Independent Test**: User has a conversation, refreshes page, and sees previous chat history preserved.

**Acceptance Scenarios**:

1. **Given** a user has had a conversation, **When** they close and reopen the chatbot, **Then** they see their conversation history.
2. **Given** conversation history exists, **When** the user asks a follow-up question, **Then** the chatbot maintains context from previous messages.
3. **Given** a user wants to start fresh, **When** they click "New Chat", **Then** a new conversation begins without previous context.

---

### User Story 5 - Streaming Responses (Priority: P3)

A user asks a complex question that requires a lengthy answer. Instead of waiting for the entire response, they see the answer stream in progressively word-by-word.

**Why this priority**: Streaming improves perceived performance and user experience for longer responses.

**Independent Test**: User asks "Explain the SLAM algorithm" and sees the response appear incrementally.

**Acceptance Scenarios**:

1. **Given** a question is asked, **When** the response is being generated, **Then** text appears progressively in the chat window.
2. **Given** streaming is in progress, **When** the user wants to stop, **Then** they can click a stop button.
3. **Given** streaming completes, **When** all text is displayed, **Then** citations appear at the end.

---

### User Story 6 - Copy Answer to Clipboard (Priority: P3)

A user receives a helpful explanation and wants to save it for their notes. They click a "Copy" button next to the answer.

**Why this priority**: Convenience feature for users taking notes or sharing information.

**Independent Test**: User clicks copy button and pastes answer into a text editor.

**Acceptance Scenarios**:

1. **Given** an answer is displayed, **When** the user clicks the copy button, **Then** the answer text is copied to clipboard.
2. **Given** copy succeeds, **When** complete, **Then** a confirmation toast appears briefly.

---

### Edge Cases

- **Empty selection**: User tries to use "Selected Text Only" mode with no text selected → Prompt user to select text first.
- **Very long selection**: User selects >5000 characters → Truncate with warning or reject with message.
- **Qdrant unavailable**: Vector search fails → Return graceful error: "Unable to search book content. Please try again later."
- **OpenAI API unavailable**: Generation fails → Return: "Unable to generate response. Please try again later."
- **Postgres unavailable**: Chat history fails to save → Log error, continue without persistence, warn user.
- **Ambiguous question**: Question could have multiple interpretations → Ask clarifying question or provide multiple interpretations.
- **Non-English query**: User asks in another language → Respond in English or decline gracefully.
- **Hallucination attempt**: Model tries to answer beyond context → System prompt enforces strict grounding.

---

## Requirements *(mandatory)*

### Functional Requirements

**Core Chatbot Capabilities**
- **FR-001**: System MUST answer questions related to any module or chapter of the book.
- **FR-002**: System MUST cite the exact section(s) of the book used to generate each answer.
- **FR-003**: System MUST support two operating modes: RAG Mode (default) and Selected Text Only Mode.

**RAG Mode Requirements**
- **FR-004**: In RAG mode, system MUST perform embedding lookup from Qdrant.
- **FR-005**: In RAG mode, system MUST use retrieved context to answer questions.
- **FR-006**: In RAG mode, system MUST include citations with line/section references.

**Selected Text Only Mode Requirements**
- **FR-007**: In Selected Text Only mode, system MUST accept a block of text the user highlights.
- **FR-008**: In Selected Text Only mode, vector retrieval MUST be skipped.
- **FR-009**: In Selected Text Only mode, answer MUST be limited strictly to the selected text.
- **FR-010**: In Selected Text Only mode, UI MUST clearly indicate answer is based only on selected text.

**Retrieval-Augmentation Requirements**
- **FR-011**: Embedding pipeline MUST use OpenAI embedding model (text-embedding-3-small or newer).
- **FR-012**: System MUST chunk the book into sections with metadata: module, chapter, path, paragraph index, original markdown.
- **FR-013**: System MUST store embeddings in Qdrant Cloud.
- **FR-014**: System MUST store metadata, citations, and audit trail in Neon Serverless Postgres.
- **FR-015**: Chunks MUST be 300-800 tokens with appropriate overlap.

**API Requirements (FastAPI)**
- **FR-016**: Backend MUST expose `POST /api/chat` endpoint accepting:
  ```json
  {
    "question": "string",
    "selected_text": "string (optional)",
    "mode": "rag | selected_only"
  }
  ```
- **FR-017**: Backend MUST return from `/api/chat`:
  ```json
  {
    "answer": "string",
    "citations": ["array of citation objects"],
    "sources": ["array of source paths"],
    "context_used": "string"
  }
  ```
- **FR-018**: Backend MUST expose `POST /api/embed-book` for ingestion pipeline.
- **FR-019**: Backend MUST expose `GET /api/health` confirming connectivity to Qdrant, Neon Postgres, OpenAI, and file system.
- **FR-020**: Backend MUST validate and sanitize all user inputs.
- **FR-021**: Backend MUST implement rate limiting to prevent abuse.

**Frontend Requirements (Docusaurus)**
- **FR-022**: Chat widget MUST be embedded in book sidebar or floating bottom-right.
- **FR-023**: UI MUST support chat history display.
- **FR-024**: UI MUST support citations with hover preview.
- **FR-025**: UI MUST support "Selected Text Only" toggle.
- **FR-026**: UI MUST support copy-to-clipboard for answers.
- **FR-027**: UI MUST support streaming responses.
- **FR-028**: Frontend MUST display floating chatbot button on all documentation pages.
- **FR-029**: Frontend MUST support Markdown rendering in chat messages.
- **FR-030**: Frontend MUST support light/dark mode matching Docusaurus theme.
- **FR-031**: Frontend MUST show loading states during API calls.
- **FR-032**: Frontend MUST handle and display error states gracefully.

**Grounding Rules**
- **FR-033**: All answers MUST be grounded in retrieved text or selected text.
- **FR-034**: System MUST never fabricate citations.
- **FR-035**: When context is insufficient, system MUST respond: "The book does not contain enough information to answer this question."

---

### Non-Functional Requirements

**Performance**
- **NFR-001**: Retrieval + model response MUST complete in <1.5 seconds average.
- **NFR-002**: Selected Text Only mode MUST respond in <1 second.
- **NFR-003**: All chunks MUST be <2,000 tokens.
- **NFR-004**: Total vector DB size MUST fit within Qdrant Free Tier (1GB).
- **NFR-005**: UI MUST remain responsive during API calls (non-blocking).

**Reliability**
- **NFR-006**: If retrieval fails, system MUST provide graceful fallback with apology and suggestion to narrow question.
- **NFR-007**: If Qdrant or Postgres unreachable, system MUST log error, return safe fallback message, and NOT hallucinate.
- **NFR-008**: System MUST retry failed requests with exponential backoff.
- **NFR-009**: Embedding ingestion MUST be idempotent (re-runnable without duplication).

**Security**
- **NFR-010**: No user data stored without explicit permission.
- **NFR-011**: No cross-site scripting from user-selected text.
- **NFR-012**: Postgres MUST use least-privilege roles.
- **NFR-013**: API keys MUST NOT be exposed in frontend code.
- **NFR-014**: All API endpoints MUST use HTTPS.
- **NFR-015**: User inputs MUST be sanitized to prevent injection attacks.
- **NFR-016**: CORS MUST be configured to allow only the documentation domain.

**Scalability**
- **NFR-017**: System MUST handle 100 concurrent users on free tier infrastructure.
- **NFR-018**: Embedding storage MUST accommodate future content growth (2x current size).

---

### Key Entities

- **Document Chunk**: A segment of documentation content (300-800 tokens) with metadata (module, chapter, section_title, source_path, paragraph_index, token_count, text).
- **Embedding**: Vector representation of a document chunk (1536 dimensions for OpenAI text-embedding-3-small).
- **Conversation**: A sequence of user/assistant messages with unique ID and timestamps.
- **Message**: A single chat message with role (user/assistant), content, mode, and optional context.
- **Query Context**: Selected text and/or conversation history used to augment retrieval.
- **Citation**: Reference to source section with module, chapter, path, and text excerpt.
- **Chat Log**: Audit record of question, answer, mode, sources, and timestamp.

---

## Architecture Specification

### High-Level Architecture

```
User → Docusaurus Chat Widget → FastAPI Backend →
  ├── OpenAI Agents/ChatKit SDK (Generation)
  ├── Qdrant Cloud (Vector Search)
  └── Neon Postgres (Metadata + Audit Logs)
```

### Data Pipelines

**Pipeline A: Book Ingestion**
1. Load Markdown files from `docs/`
2. Split into hierarchical structure: Module → Chapter → Paragraph
3. Chunk text into 300-800 token blocks with overlap
4. Generate embeddings using OpenAI text-embedding-3-small
5. Insert vectors into Qdrant collection `book_sections`
6. Insert metadata rows into Postgres `book_sections` table

**Pipeline B: Query Answering (RAG Mode)**
1. Receive question from user
2. Generate query embedding
3. Perform Qdrant similarity search (top-k retrieval)
4. Pass retrieved context to OpenAI Agent
5. Generate grounded answer with citations
6. Return answer + source metadata
7. Log interaction to Postgres `chat_logs`

**Pipeline C: Query Answering (Selected Text Only Mode)**
1. Receive question + selected text from user
2. Skip Qdrant retrieval entirely
3. Pass selected text as sole context to OpenAI Agent
4. Generate grounded answer from selected text only
5. Return answer with "selected text only" indicator
6. Log interaction to Postgres `chat_logs`

---

## Data Model & Schemas

### Qdrant Collection Schema

**Collection name**: `book_sections`

```json
{
  "id": "UUID",
  "embedding": "float[1536]",
  "payload": {
    "module": "string",
    "chapter": "string",
    "section_title": "string",
    "source_path": "string",
    "paragraph_index": "int",
    "token_count": "int",
    "text": "string"
  }
}
```

### Postgres Schema (Neon)

**Table: book_sections**

| Column | Type | Constraints |
|--------|------|-------------|
| id | UUID | PRIMARY KEY |
| module | TEXT | NOT NULL |
| chapter | TEXT | NOT NULL |
| section_title | TEXT | |
| source_path | TEXT | NOT NULL |
| paragraph_index | INT | NOT NULL |
| text | TEXT | NOT NULL |
| citation | TEXT | |
| token_count | INT | |
| created_at | TIMESTAMP | DEFAULT NOW() |

**Table: chat_logs**

| Column | Type | Constraints |
|--------|------|-------------|
| id | UUID | PRIMARY KEY |
| session_id | UUID | |
| question | TEXT | NOT NULL |
| answer | TEXT | NOT NULL |
| mode | TEXT | CHECK (mode IN ('rag', 'selected_only')) |
| sources | JSONB | |
| context_used | TEXT | |
| latency_ms | INT | |
| created_at | TIMESTAMP | DEFAULT NOW() |

---

## Behavioral Rules for the Chatbot

### Grounding Rules
- All answers MUST be grounded in retrieved text or selected text.
- Never fabricate citations.
- If context insufficient: "The book does not contain enough information to answer this question."
- In Selected Text Only mode: "The selected text does not contain enough information to answer this question."

### Style Rules
- Academic tone appropriate for CS/robotics students
- Flesch-Kincaid grade 10-12 readability
- Clear, concise paragraphs
- Technical terms explained when first introduced

---

## Technology Stack

**Backend**
- **Framework**: FastAPI (Python 3.11+)
- **AI/ML**: OpenAI Agents SDK or ChatKit SDK
- **Embeddings**: text-embedding-3-small (1536 dimensions)
- **Generation**: gpt-4o-mini or gpt-4o
- **Vector Database**: Qdrant Cloud Free Tier (1GB storage)
- **Relational Database**: Neon Serverless Postgres (Free tier)

**Frontend**
- **Framework**: React 18 (Docusaurus native)
- **State Management**: React hooks (useState, useEffect, useContext)
- **Styling**: CSS Modules (Docusaurus convention)
- **HTTP Client**: fetch API with streaming support

**Deployment**
- **Backend**: FastAPI on Vercel, Railway, Render, or similar
- **Frontend**: Integrated with existing GitHub Pages deployment

---

## Constraints

- **Cost**: Must use free tiers for Qdrant Cloud, Neon Postgres; OpenAI API has usage costs
- **Rate Limits**: OpenAI API rate limits apply; implement client-side throttling
- **Storage**: Qdrant free tier limited to 1GB; optimize chunk size accordingly
- **Latency**: Cold starts on serverless may add initial delay
- **CORS**: Backend must allow requests from GitHub Pages domain
- **Documentation Length**: 5,000-7,000 words for final report
- **Minimum Sources**: 15 APA-cited sources in documentation
- **Output Format**: PDF with embedded APA citations required

---

## Assumptions

- OpenAI API key will be provided and budgeted
- Qdrant Cloud and Neon Postgres free tier accounts are available
- Documentation content is stable enough to pre-generate embeddings
- Users have modern browsers with JavaScript enabled
- Backend deployed on platform with Python 3.11+ support

---

## Out of Scope

- User authentication/accounts (using anonymous sessions)
- Multi-language support (English only)
- Voice input/output
- Image understanding from documentation
- Custom model fine-tuning
- Real-time content updates (embeddings regenerated manually)
- Mobile-native app versions

---

## Evaluation Requirements

### Quality Checks
- Citation correctness: 100% must map to text in the book
- Retrieval accuracy: Top-3 chunks must contain answer source
- Zero hallucination under Selected Text Only mode
- Latency test: <1.5 seconds for RAG mode, <1 second for selected-only

### RAG Benchmarks
- Recall@3 measurement
- Precision of retrieved context
- Human evaluation of 20 random questions

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of user questions about documented topics receive relevant, accurate answers
- **SC-002**: Response latency p95 < 1.5 seconds on warm requests
- **SC-003**: Selected Text Only mode responds in <1 second
- **SC-004**: Text selection feature works on Chrome, Firefox, Safari, Edge
- **SC-005**: Chatbot UI renders correctly in both light and dark mode
- **SC-006**: System handles 100 concurrent chat sessions without degradation
- **SC-007**: All documentation files are successfully indexed with embeddings
- **SC-008**: Source citations are included in 100% of RAG responses
- **SC-009**: Citation correctness: 100% of citations map to actual book text
- **SC-010**: Zero hallucination in Selected Text Only mode
- **SC-011**: Conversation history persists across browser sessions
- **SC-012**: No API keys exposed in browser network requests or source code
- **SC-013**: Build and deployment succeed without manual intervention
- **SC-014**: Streaming responses display progressively in UI
- **SC-015**: Health endpoint confirms all service connectivity

---

## Acceptance Criteria

The project is complete when:

### Required
- [ ] Full ingestion pipeline works (markdown → embeddings → Qdrant + Postgres)
- [ ] Chatbot embedded into Docusaurus
- [ ] RAG mode answers are grounded and cited
- [ ] Selected Text Only mode works flawlessly (no retrieval, strict grounding)
- [ ] Backend stable on FastAPI
- [ ] Neon Postgres + Qdrant Cloud fully integrated
- [ ] Streaming responses functional
- [ ] Deployable using standard commands (uvicorn, GitHub Pages, etc.)

### Documentation Requirements
- [ ] 5,000-7,000 words documentation
- [ ] ≥15 APA-cited sources
- [ ] PDF export with citations
- [ ] Architecture diagram included
- [ ] Database schema documented

---

## Deliverables

1. **FastAPI backend codebase** (api/)
2. **Qdrant + Postgres schema migrations** (db/)
3. **OpenAI Agent configuration** (prompts + system instructions)
4. **Docusaurus frontend widget** (src/components/)
5. **Deployment instructions** (README.md + quickstart.md)
6. **Final academic PDF report** (docs/report.pdf)
