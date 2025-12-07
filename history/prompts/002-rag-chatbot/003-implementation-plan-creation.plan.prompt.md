---
id: 003
title: Implementation Plan Creation
stage: plan
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 002-rag-chatbot
branch: main
user: system
command: /sp.plan
labels: ["plan", "architecture", "rag-chatbot", "data-flow", "testing-strategy"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot/plan.md
tests:
  - null
---

## Prompt

/sp.plan

Project: Integrated RAG Chatbot Development for AI/Spec-Driven Book

Create: detailed architecture sketch, subsystem structure, data flow, research approach, quality validation, and testing strategy.
Follow the research-concurrent approach (research while writing, not upfront).

1. Architecture Sketch
   - Frontend Layer (Docusaurus Book UI): ChatBot widget, TextSelection handler, Chat history panel
   - Backend Layer (FastAPI): /ask, /ask_selected, /embed, /health endpoints
   - AI Runtime Layer: OpenAI Agents/ChatKit SDK with retrieval + SQL tools
   - Storage Layer: Qdrant Cloud (vectors), Neon Postgres (metadata/logs)

2. Section Structure (7 Phases):
   - Phase 1: Research & Foundations
   - Phase 2: System Architecture & Scaffolding
   - Phase 3: Ingestion Pipeline
   - Phase 4: Retrieval & Reasoning Logic
   - Phase 5: Frontend Chat UI
   - Phase 6: QA, Testing, and Validation
   - Phase 7: Deployment

3. Decisions Needing Documentation:
   - AI Model Choices (GPT model, Agents vs ChatKit)
   - Chunking Strategy (size, overlap, metadata)
   - Retrieval Strategy (vector-only vs hybrid)
   - Storage Design (Qdrant for vectors, Neon for logs)
   - Frontend Embedding Method (React component vs plugin)
   - Security Strategy (API keys, CORS)

4. Quality Validation Strategy:
   - Retrieval accuracy (Recall@5, MRR)
   - Answer grounding (citations required)
   - Zero hallucination tests
   - Performance (<1.5s RAG, <1s selected-only)
   - Citation rules and safety checks

5. Testing Strategy:
   - Unit tests (chunker, embeddings, search)
   - Integration tests (E2E RAG, selection-based)
   - Acceptance tests (grounding, citations, deployment)

## Response snapshot

Created comprehensive implementation plan for the Integrated RAG Chatbot with:

**Architecture:**
- 4-layer architecture: Frontend (Docusaurus) → Backend (FastAPI) → AI Runtime (OpenAI) → Storage (Qdrant + Neon)
- 3 data flow diagrams: Ingestion Pipeline, RAG Mode Query, Selected Text Only Mode

**7 Implementation Phases:**
1. Research & Foundations - RAG patterns, embedding strategy, SDK comparison
2. System Architecture - FastAPI setup, Qdrant collection config, Postgres schema
3. Ingestion Pipeline - Markdown → chunks → embeddings → storage (idempotent)
4. Retrieval & Reasoning - Retriever implementation, selected-only pipeline, system prompts
5. Frontend Chat UI - Component architecture, streaming API client, text selection
6. QA & Testing - Unit/integration/contract/E2E tests, quality metrics
7. Deployment - HF Spaces backend, GitHub Pages frontend, CI/CD

**Key Technical Decisions:**
- Vector DB: Qdrant (1GB free tier, cosine distance)
- Embeddings: text-embedding-3-small (1536 dims, $0.00002/1K tokens)
- Generation: GPT-4o-mini (cost/quality balance)
- Chunking: 300-800 tokens with 50-100 overlap
- Selected-only: Skip retrieval entirely (explicit grounding guarantee)

**Constitution Compliance:**
- ✅ Accuracy: FR-033 to FR-035 enforce grounding
- ✅ Clarity: Flesch-Kincaid grade 10-12
- ✅ Reproducibility: Schemas and pipelines documented
- ✅ Rigor: 15+ APA sources, evaluation benchmarks
- ✅ Grounding: FR-008 explicitly skips Qdrant in selected-only mode

**Risk Analysis:**
1. OpenAI API costs → Rate limiting, caching, smaller model
2. Hallucination → Strict prompts, adversarial testing
3. Cold start latency → Loading states, warm-up pings

**ADR Candidates Identified:**
- GPT-4o-mini over GPT-4o
- Qdrant over Pinecone
- Selected-only skips retrieval
- 300-800 token chunks
- OpenAI Agents SDK

## Outcome

- ✅ Impact: Complete implementation plan with architecture diagrams, data flows, and 7 phases
- 🧪 Tests: Testing strategy defined (unit, integration, contract, E2E, frontend)
- 📁 Files: 1 file updated (specs/002-rag-chatbot/plan.md)
- 🔁 Next prompts: /sp.tasks to generate task list, /sp.adr to document architectural decisions
- 🧠 Reflection: User provided structured requirements; integrated all elements into cohesive plan with ASCII diagrams

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - planning document
- Prompt variant (if applicable): Standard /sp.plan with detailed architecture requirements
- Next experiment (smallest change to try): Validate plan completeness during task generation
