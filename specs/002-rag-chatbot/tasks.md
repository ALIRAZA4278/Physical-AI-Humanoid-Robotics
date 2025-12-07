# Tasks: Integrated RAG Chatbot

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (✅), spec.md (✅)
**Created**: 2025-12-05
**Updated**: 2025-12-07
**Status**: In Progress

**Tests**: OPTIONAL - Tests can be added in Phase 9 (Polish) if needed.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `api/` (FastAPI Python)
- **Frontend**: `src/components/ChatBot/`, `src/context/`, `src/theme/` (Docusaurus React)
- **Prompts**: `api/prompts/`
- **Tests**: `tests/api/`, `tests/frontend/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Verify existing FastAPI project structure in `api/` matches plan.md
- [x] T002 [P] Update `api/requirements.txt` with missing dependencies (qdrant-client, sqlalchemy, tiktoken)
- [x] T003 [P] Create `api/.env.example` with all required environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, CORS_ORIGINS)
- [x] T004 [P] Create `api/config.py` with Pydantic settings management
- [x] T005 [P] Create directory structure for `api/routers/`, `api/services/`, `api/models/`, `api/utils/`, `api/prompts/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & External Services

- [x] T006 Create Qdrant Cloud collection `book_sections` with 1536 dimensions + payload schema (auto-created by retriever.ensure_collection())
- [x] T007 Create Neon Postgres tables `book_sections` and `chat_logs` per spec schema in `api/models/database.py` (conversations/messages tables auto-created by init_db())
- [x] T008 [P] Create SQLAlchemy models for `book_sections` and `chat_logs` in `api/models/database.py`
- [x] T009 [P] Create Pydantic schemas for ChatRequest, ChatResponse, Citation in `api/models/schemas.py`

### Core Services

- [x] T010 [P] Implement token counter utility in `api/utils/token_counter.py` using tiktoken
- [x] T011 [P] Implement markdown loader utility in `api/utils/markdown_loader.py`
- [x] T012 Implement document chunker service in `api/services/chunker.py` (300-800 tokens, 50 overlap)
- [x] T013 Implement OpenAI embedding service in `api/services/embeddings.py` (text-embedding-3-small)
- [x] T014 Implement Qdrant retriever service in `api/services/retriever.py`

### System Prompts

- [x] T015 [P] Create RAG mode system prompt in `api/prompts/rag_system.txt`
- [x] T016 [P] Create selected-only mode system prompt in `api/prompts/selected_only_system.txt`

### API Infrastructure

- [x] T017 Implement health endpoint in `api/routers/health.py` (check Qdrant, Postgres, OpenAI)
- [x] T018 Configure CORS middleware in `api/main.py` for GitHub Pages domain
- [x] T019 Implement error handling middleware with graceful fallback messages in `api/main.py`

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - RAG Mode Question Answering (Priority: P1) 🎯 MVP

**Goal**: User can ask questions about book content and receive grounded answers with citations

**Independent Test**: User asks "What is URDF?" and receives contextually accurate response citing Module 1

### Ingestion Pipeline (Required for RAG)

- [x] T020 [US1] Implement ingestion script in `api/scripts/ingest_docs.py` (load → chunk → embed → store)
- [ ] T021 [US1] Run ingestion pipeline to populate Qdrant and Postgres with book content
- [ ] T022 [US1] Verify ingestion success: check Qdrant collection count and Postgres rows

### Backend Implementation

- [x] T023 [US1] Implement OpenAI generator service in `api/services/generator.py` (gpt-4o-mini with grounding)
- [x] T024 [US1] Implement RAG orchestration in `api/services/rag.py` (retrieve → assemble context → generate)
- [x] T025 [US1] Implement POST /api/chat endpoint in `api/routers/chat.py` (RAG mode)
- [x] T026 [US1] Add chat logging to Postgres in chat endpoint

### Frontend Implementation

- [x] T027 [P] [US1] Create ChatContext provider in `src/context/ChatContext.tsx`
- [x] T028 [P] [US1] Create useChat hook in `src/components/ChatBot/hooks/useChat.ts`
- [x] T029 [US1] Create main ChatBot component in `src/components/ChatBot/index.tsx` (floating widget)
- [x] T030 [US1] Create ChatWindow component in `src/components/ChatBot/ChatWindow.tsx`
- [x] T031 [US1] Create ChatInput component in `src/components/ChatBot/ChatInput.tsx`
- [x] T032 [US1] Create Message component in `src/components/ChatBot/Message.tsx`
- [x] T033 [US1] Create ChatBot styles in `src/components/ChatBot/ChatBot.module.css`
- [x] T034 [US1] Integrate ChatBot into Docusaurus theme in `src/theme/Root.tsx`

### Integration & Validation

- [ ] T035 [US1] End-to-end test: Ask "What is a ROS 2 node?" and verify grounded answer with citation
- [ ] T036 [US1] Verify graceful decline: Ask question not in book, confirm "book does not contain" response
- [ ] T037 [US1] Verify latency: Confirm RAG response < 1.5 seconds on warm request

**Checkpoint**: User Story 1 (RAG Mode) is fully functional and testable independently

---

## Phase 4: User Story 2 - Selected Text Only Mode (Priority: P1)

**Goal**: User can select text and get answers based ONLY on that selection (no retrieval)

**Independent Test**: Select paragraph about bipedal locomotion, ask "How does this work?", receive answer from selection only

### Backend Implementation

- [ ] T038 [US2] Implement selected-only answering path in `api/services/rag.py` (skip Qdrant retrieval)
- [ ] T039 [US2] Update POST /api/chat to handle mode="selected_only" in `api/routers/chat.py`
- [ ] T040 [US2] Add selected_text validation (max 5000 chars) and sanitization

### Frontend Implementation

- [x] T041 [P] [US2] Create useTextSelection hook in `src/components/ChatBot/hooks/useTextSelection.ts`
- [x] T042 [US2] Create TextSelectionHandler component in `src/components/ChatBot/TextSelectionHandler.tsx`
- [ ] T043 [US2] Create ModeToggle component in `src/components/ChatBot/ModeToggle.tsx`
- [ ] T044 [US2] Update ChatInput to show selected text context
- [ ] T045 [US2] Update Message component to indicate "Based on selected text only"

### Integration & Validation

- [ ] T046 [US2] Test: Select text, enable selected-only mode, ask question, verify no Qdrant call made
- [ ] T047 [US2] Test: Ask question with insufficient selected text, verify graceful decline message
- [ ] T048 [US2] Verify latency: Selected-only mode response < 1 second

**Checkpoint**: User Story 2 (Selected Text Only) is fully functional and testable independently

---

## Phase 5: User Story 3 - Citation and Source Navigation (Priority: P2)

**Goal**: Answers include clickable links to source sections with hover preview

**Independent Test**: Answer about Gazebo setup includes clickable link to `module-2-simulation/01-gazebo-setup`

### Backend Implementation

- [ ] T049 [US3] Enhance citation metadata in RAG response (include source_path, section_title, text excerpt)
- [ ] T050 [US3] Add citation URL generation based on Docusaurus routing in `api/services/rag.py`

### Frontend Implementation

- [ ] T051 [US3] Create Citation component in `src/components/ChatBot/Citation.tsx` (clickable + hover preview)
- [ ] T052 [US3] Update Message component to render Citation components
- [ ] T053 [US3] Add CSS for citation hover preview popup

### Integration & Validation

- [ ] T054 [US3] Test: Verify citation links navigate to correct documentation section
- [ ] T055 [US3] Test: Verify hover preview shows cited content
- [ ] T056 [US3] Test: Multiple sources answer shows all citations

**Checkpoint**: User Story 3 (Citations) is fully functional and testable independently

---

## Phase 6: User Story 4 - Conversation History (Priority: P2)

**Goal**: Chat history persists across page refreshes and browser sessions

**Independent Test**: User has conversation, refreshes page, sees previous chat history

### Backend Implementation

- [ ] T057 [US4] Add session_id handling to chat endpoint in `api/routers/chat.py`
- [ ] T058 [US4] Implement conversation retrieval endpoint GET /api/conversations in `api/routers/chat.py`

### Frontend Implementation

- [ ] T059 [P] [US4] Create useChatHistory hook in `src/components/ChatBot/hooks/useChatHistory.ts`
- [ ] T060 [US4] Update ChatContext to persist session_id to localStorage
- [ ] T061 [US4] Update ChatWindow to load history on mount
- [ ] T062 [US4] Add "New Chat" button to clear conversation and start fresh

### Integration & Validation

- [ ] T063 [US4] Test: Have conversation, refresh page, verify history preserved
- [ ] T064 [US4] Test: Click "New Chat", verify new conversation starts without previous context
- [ ] T065 [US4] Test: Follow-up question uses previous message context

**Checkpoint**: User Story 4 (Conversation History) is fully functional and testable independently

---

## Phase 7: User Story 5 - Streaming Responses (Priority: P3)

**Goal**: Answers stream progressively word-by-word instead of appearing all at once

**Independent Test**: Ask "Explain the SLAM algorithm" and see response appear incrementally

### Backend Implementation

- [ ] T066 [US5] Update generator service to yield streaming chunks in `api/services/generator.py`
- [ ] T067 [US5] Update chat endpoint to return StreamingResponse in `api/routers/chat.py`

### Frontend Implementation

- [ ] T068 [US5] Update useChat hook to handle streaming response via ReadableStream
- [ ] T069 [US5] Update Message component to display partial streaming content
- [ ] T070 [US5] Add "Stop" button to abort streaming in progress
- [ ] T071 [US5] Update citations to appear after streaming completes

### Integration & Validation

- [ ] T072 [US5] Test: Ask complex question, verify text appears progressively
- [ ] T073 [US5] Test: Click stop button, verify streaming halts
- [ ] T074 [US5] Test: Citations appear after full response received

**Checkpoint**: User Story 5 (Streaming) is fully functional and testable independently

---

## Phase 8: User Story 6 - Copy Answer to Clipboard (Priority: P3)

**Goal**: User can copy chatbot answers to clipboard with one click

**Independent Test**: Click copy button, paste into text editor, verify answer text copied

### Frontend Implementation

- [ ] T075 [P] [US6] Add copy button to Message component in `src/components/ChatBot/Message.tsx`
- [ ] T076 [US6] Implement clipboard copy using navigator.clipboard API
- [ ] T077 [US6] Add toast notification for copy success/failure
- [ ] T078 [US6] Add CSS for copy button and toast notification

### Integration & Validation

- [ ] T079 [US6] Test: Click copy, paste into text editor, verify correct content
- [ ] T080 [US6] Test: Verify toast appears briefly after copy

**Checkpoint**: User Story 6 (Copy) is fully functional and testable independently

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### UI Polish

- [ ] T081 [P] Add light/dark mode support matching Docusaurus theme in `src/components/ChatBot/ChatBot.module.css`
- [ ] T082 [P] Add loading states during API calls (spinner, disabled input)
- [ ] T083 [P] Add error state UI with retry option
- [ ] T084 Add Markdown rendering support in Message component (code blocks, lists, links)
- [ ] T085 Responsive design for mobile viewport

### Backend Polish

- [ ] T086 [P] Add rate limiting middleware to prevent API abuse in `api/main.py`
- [ ] T087 [P] Add request logging for debugging in `api/main.py`
- [ ] T088 Add retry logic with exponential backoff for external service calls

### Security

- [ ] T089 Verify no API keys exposed in frontend (check network requests, source code)
- [ ] T090 Verify CORS configuration only allows documentation domain
- [ ] T091 Add input sanitization for XSS prevention on selected text

### Documentation

- [ ] T092 [P] Update README.md with chatbot feature documentation
- [ ] T093 [P] Create quickstart guide for local development in `specs/002-rag-chatbot/quickstart.md`
- [ ] T094 Document API endpoints in `specs/002-rag-chatbot/contracts/`

### Final Validation

- [ ] T095 Run full acceptance criteria checklist from spec.md
- [ ] T096 Verify all 15 success criteria (SC-001 to SC-015) are met
- [ ] T097 Cross-browser testing: Chrome, Firefox, Safari, Edge

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - ✅ COMPLETE
- **Foundational (Phase 2)**: Depends on Setup completion - IN PROGRESS
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - US1 and US2 are both P1 priority but US2 depends on US1 backend
  - US3 enhances US1 (citations)
  - US4-US6 are independent enhancements
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

```
Phase 2 (Foundational) - IN PROGRESS
        │
        ├──────────────────────────────────────────┐
        │                                          │
        ▼                                          ▼
Phase 3: US1 (RAG Mode) ──────▶ Phase 4: US2 (Selected Text)
        │                                          │
        │                                          │
        ├──────────────────────────────────────────┤
        │                                          │
        ▼                                          ▼
Phase 5: US3 (Citations)         Phase 6: US4 (History)
        │                                          │
        │                                          │
        └──────────────────────────────────────────┤
                                                   │
                        ┌──────────────────────────┤
                        │                          │
                        ▼                          ▼
              Phase 7: US5 (Streaming)   Phase 8: US6 (Copy)
                        │                          │
                        └──────────────────────────┘
                                  │
                                  ▼
                        Phase 9: Polish
```

### Parallel Opportunities Per User Story

**Phase 2 (Foundational)**:
- T006, T007 can run in parallel (different external services)
- T008, T009 can run in parallel (different files) ✅ DONE
- T010, T011 can run in parallel (different utilities)
- T015, T016 can run in parallel (different prompt files)

**Phase 3 (US1)**:
- T027, T028 can run in parallel (different frontend files) ✅ DONE
- Backend tasks mostly sequential but several complete ✅
- Frontend largely complete ✅

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. ✅ Complete Phase 1: Setup
2. 🔄 Complete Phase 2: Foundational (IN PROGRESS)
3. 🔄 Complete Phase 3: User Story 1 (RAG Mode) - PARTIALLY COMPLETE
4. **STOP and VALIDATE**: Test RAG mode independently
5. Deploy/demo if ready - This is your MVP!

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (RAG Mode) → Test → Deploy (MVP!)
3. Add US2 (Selected Text) → Test → Deploy
4. Add US3 (Citations) → Test → Deploy
5. Add US4 (History) → Test → Deploy
6. Add US5 (Streaming) → Test → Deploy
7. Add US6 (Copy) → Test → Deploy
8. Polish → Final release

### Suggested MVP Scope

**MVP = Phase 1 + Phase 2 + Phase 3 (User Story 1)**

This delivers:
- ✅ Working chatbot embedded in book
- ✅ RAG-based question answering
- ✅ Grounded answers with citations
- ✅ Graceful error handling

---

## Progress Summary

| Phase | Total | Complete | Remaining |
|-------|-------|----------|-----------|
| Phase 1: Setup | 5 | 5 | 0 |
| Phase 2: Foundational | 14 | 14 | 0 ✅ |
| Phase 3: US1 RAG Mode | 18 | 11 | 7 |
| Phase 4: US2 Selected Text | 11 | 2 | 9 |
| Phase 5: US3 Citations | 8 | 0 | 8 |
| Phase 6: US4 History | 9 | 0 | 9 |
| Phase 7: US5 Streaming | 9 | 0 | 9 |
| Phase 8: US6 Copy | 6 | 0 | 6 |
| Phase 9: Polish | 17 | 0 | 17 |
| **Total** | **97** | **32** | **65** |

### Independent Test Criteria

| Story | Independent Test | Status |
|-------|------------------|--------|
| US1 | Ask "What is URDF?" → Grounded answer + citation | 🔄 Pending |
| US2 | Select text + ask → Answer from selection only | ⏳ Not started |
| US3 | Citation link → Navigates to correct section | ⏳ Not started |
| US4 | Refresh page → History preserved | ⏳ Not started |
| US5 | Ask complex question → Text streams progressively | ⏳ Not started |
| US6 | Click copy → Answer in clipboard | ⏳ Not started |

---

## Next Actions (Priority Order)

**Phase 2 Complete** ✅ - Foundation ready for user story implementation

1. **T021**: Run ingestion pipeline to populate Qdrant with book content
2. **T022**: Verify ingestion success (check Qdrant collection count)
3. **T035**: End-to-end test with RAG query
4. **T036**: Test graceful decline for out-of-scope questions
5. **T037**: Verify latency requirements (<1.5s)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- MVP scope: Phases 1-3 (Setup + Foundational + US1)
