---
id: 002
title: RAG Chatbot Constitution Amendment
stage: constitution
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: system
command: /sp.constitution
labels: ["constitution", "governance", "rag-chatbot", "major-amendment", "v2.0.0"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

/sp.constitution

Project: Integrated RAG Chatbot for the Physical AI & Humanoid Robotics Book
Objective: Build and embed an interactive Retrieval-Augmented Generation (RAG) chatbot directly into the published Docusaurus book. The chatbot must answer questions about the book's content and optionally answer questions based only on the user-selected portion of text.
This system must use OpenAI Agents / ChatKit SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier.

Core Principles

Accuracy through primary source verification
All technical statements (ROS 2, Isaac, Whisper, VSLAM, etc.) must be validated against official documentation.

Clarity for an academic audience
Target reader: students with computer science + robotics background.

Reproducibility
Every pipeline (embedding, ingestion, vector search, API, frontend chat widget) must be reconstructable via instructions.

Rigor

Use official docs, academic papers, and industry-grade technical sources.

Peer-reviewed robotics + AI papers preferred for theoretical sections.

Key Standards

All factual claims must be traceable and cited.

APA citation style across documentation and the book.

At least 50% peer-reviewed source material for the theory.

0% tolerance for plagiarism; rewrite or cite all reproduced content.

Writing should target Flesch-Kincaid grade 10–12 readability.

Technical Standards for the RAG System

Embedding + search:

Use Qdrant Cloud Free Tier for vector storage.

Dimensions, distance metric, and payload schema must be documented.

Metadata + structured knowledge:

Use Neon Serverless Postgres to store book sections, citations, timestamps, and retrieval logs.

Must be normalized and reproducible.

Models:

Use OpenAI's GPT models through the Agents or ChatKit SDK.

Enforce grounding rules: No hallucination; all answers grounded in retrieved context.

User-selected text mode:

Chatbot must accept a block of selected text and answer only from that text.

Retrieval must be disabled when in "selection-only" mode.

Frontend Integration:

Embed chatbot inside Docusaurus book (React-based widget).

Support streaming responses and citations in the UI.

Constraints

Word count for documentation: 5,000–7,000 words.

Minimum 15 sources.

Deliverable must include a PDF with embedded APA citations.

Must run on common environments:

Local development (FastAPI + Postgres + Qdrant cloud).

Production deployment on GitHub Pages + external API.

Success Criteria

Every claim in documentation verified against sources.

Zero plagiarism upon automated check.

System passes:

Factual accuracy review

RAG evaluation (retrieval correctness, hallucination suppression)

Performance validation (latency < 1.5s for retrieval + response under typical load).

Chatbot demonstrates:

High-quality answers grounded in the book.

Correct functioning of user-selected text only mode.

Seamless integration inside the published book UI.

Output

This constitution governs all planning, architecture, code, testing, and writing for the Integrated RAG Chatbot Project.

## Response snapshot

Executed MAJOR constitution amendment (v1.0.0 → v2.0.0) to redefine the project scope from "AI-Driven Book Creation" to "Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book".

Key changes:

1. **5 Redefined Core Principles**:
   - Accuracy Through Primary Source Verification (academic rigor for robotics/AI claims)
   - Clarity for Academic Audience (CS/robotics students, grade 10-12 readability)
   - Reproducibility (all pipelines must be reconstructable)
   - Rigor (50% peer-reviewed sources, industry-grade standards)
   - Grounding (no hallucination, citation requirements, selection-only mode)

2. **New Technical Standards**:
   - Qdrant Cloud Free Tier for vector storage
   - Neon Serverless Postgres for metadata/structured knowledge
   - OpenAI Agents/ChatKit SDK for LLM integration
   - FastAPI backend
   - React widget in Docusaurus frontend

3. **User-Selected Text Mode**: Explicit requirement for selection-only answering with disabled retrieval

4. **New Success Criteria**:
   - RAG evaluation (retrieval correctness, hallucination suppression)
   - Latency < 1.5s
   - Frontend integration quality
   - APA citation compliance

5. **Updated Constraints**: 5,000-7,000 words, 15 minimum sources, PDF deliverable with citations

Templates verified compatible (plan-template, spec-template, tasks-template all use generic Constitution Check references).

## Outcome

- ✅ Impact: Constitution v2.0.0 ratified - complete project scope redefinition for RAG chatbot development
- 🧪 Tests: N/A - governance document
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: /sp.specify to create RAG chatbot feature spec
- 🧠 Reflection: Major version bump justified due to fundamental principle redefinition; removed 4 principles (Modularity, Maintainability, Practicality, Version Awareness) and added 2 new ones (Rigor, Grounding)

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - constitution amendment
- Prompt variant (if applicable): Standard /sp.constitution with comprehensive user input
- Next experiment (smallest change to try): Validate new principles during first RAG feature spec/plan cycle
