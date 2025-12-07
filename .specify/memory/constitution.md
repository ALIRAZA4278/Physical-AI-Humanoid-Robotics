<!--
SYNC IMPACT REPORT
==================
Version change: 1.0.0 → 2.0.0
Bump rationale: MAJOR - Complete project scope redefinition from "AI-Driven Book Creation"
                to "Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book".
                All principles, technical standards, and success criteria fundamentally changed.

Modified principles:
- Accuracy → Accuracy (redefined: from technical docs validation to primary source verification
  with academic rigor)
- Clarity → Clarity (redefined: from intermediate developers to CS/robotics academic audience)
- Consistency → Reproducibility (NEW focus: pipeline reproducibility for RAG system)
- Modularity → removed (not applicable to RAG chatbot)
- Maintainability → removed (subsumed by technical standards)
- Practicality → removed (subsumed by grounding requirements)
- Version Awareness → removed (subsumed by technical standards)
- NEW: Rigor (academic/industry source standards)
- NEW: Grounding (hallucination prevention)

Added sections:
- Technical Standards for RAG System (Embedding, Metadata, Models, Selection Mode, Frontend)
- Infrastructure Requirements (Qdrant Cloud, Neon Postgres, OpenAI Agents SDK, FastAPI)
- RAG-Specific Success Criteria (retrieval correctness, hallucination suppression, latency)

Removed sections:
- Docusaurus-specific guidance
- GitHub Pages deployment focus
- Spec-Kit Plus workflow compliance
- Chapter structure requirements

Templates consistency check:
- .specify/templates/plan-template.md ✅ Compatible (Constitution Check section references
  constitution file generically)
- .specify/templates/spec-template.md ✅ Compatible (Requirements and Success Criteria
  structure is reusable)
- .specify/templates/tasks-template.md ✅ Compatible (Phase structure supports RAG system
  development workflows)

Follow-up TODOs: None
-->

# Integrated RAG Chatbot Constitution

## Project Overview

**Project**: Integrated RAG Chatbot for the Physical AI & Humanoid Robotics Book
**Objective**: Build and embed an interactive Retrieval-Augmented Generation (RAG) chatbot
directly into the published Docusaurus book. The chatbot MUST answer questions about the
book's content and optionally answer questions based only on user-selected text portions.

**Technology Stack**:
- **Backend**: FastAPI
- **Vector Store**: Qdrant Cloud Free Tier
- **Relational Database**: Neon Serverless Postgres
- **AI/LLM**: OpenAI Agents / ChatKit SDK with GPT models
- **Frontend**: React-based widget embedded in Docusaurus

## Core Principles

### I. Accuracy Through Primary Source Verification

All technical statements MUST be validated against official documentation and authoritative sources.

- Technical claims about ROS 2, NVIDIA Isaac, OpenAI Whisper, VSLAM, and other robotics/AI
  technologies MUST be verified against official documentation
- Code samples MUST be tested and validated before inclusion
- API contracts MUST match actual implementation behavior
- No AI hallucinations permitted—all answers MUST be grounded in retrieved context
- When documentation conflicts exist, prefer the most recent official release documentation

**Rationale**: A RAG system teaching robotics concepts becomes harmful if it provides
inaccurate technical information. Primary source verification is non-negotiable for
academic credibility.

### II. Clarity for Academic Audience

Writing and responses MUST be understandable for students with computer science and
robotics backgrounds.

- Target reader: Students with CS + robotics foundational knowledge
- Definitions MUST precede advanced concepts
- Technical explanations MUST follow logical progression
- Domain jargon MUST be explained on first use or be retrievable from the book context
- Complex concepts MUST include examples when retrieved context supports it
- Writing SHOULD target Flesch-Kincaid grade 10–12 readability

**Rationale**: The target audience has foundational knowledge but may lack familiarity
with specific robotics technologies. Meeting them at their level maximizes comprehension.

### III. Reproducibility

Every pipeline component MUST be reconstructable via documented instructions.

- Embedding generation pipeline MUST be fully documented
- Ingestion workflows MUST be reproducible from source book content
- Vector search configuration MUST be explicitly specified (dimensions, distance metric,
  payload schema)
- API endpoints MUST be documented with request/response contracts
- Frontend chat widget integration MUST have step-by-step setup instructions
- Environment setup MUST be documented for both local development and production

**Rationale**: Academic and production systems require reproducibility for peer review,
debugging, and maintenance.

### IV. Rigor

All content and system behavior MUST meet academic and industry-grade standards.

- Official documentation MUST be the primary source for technical claims
- Peer-reviewed robotics and AI papers SHOULD be preferred for theoretical sections
- At least 50% of theoretical content sources MUST be peer-reviewed material
- Industry-grade technical sources MAY supplement academic sources
- All factual claims MUST be traceable and citable

**Rationale**: A system embedded in an academic book MUST uphold academic integrity
and source quality standards.

### V. Grounding

All chatbot answers MUST be grounded in retrieved context with no hallucination.

- Answers MUST be derived exclusively from retrieved book content
- When in "selection-only" mode, retrieval MUST be disabled and answers MUST use only
  the user-selected text
- Citations MUST be provided in responses to indicate source sections
- System MUST decline to answer when retrieved context is insufficient
- Confidence indicators SHOULD be provided when appropriate

**Rationale**: RAG systems fail when they hallucinate beyond their knowledge base.
Strict grounding prevents misinformation.

## Key Standards

### Writing Style

- Technical-academic tone maintained throughout documentation
- Step-by-step explanations with numbered procedures for setup guides
- All code samples MUST be tested before inclusion
- Active voice preferred; passive voice only when emphasizing the action target
- APA citation style MUST be used across documentation and the book
- 0% tolerance for plagiarism; rewrite or properly cite all reproduced content

### Technical Requirements

#### Embedding & Search (Qdrant Cloud Free Tier)

- Vector dimensions MUST be documented (e.g., 1536 for OpenAI text-embedding-ada-002)
- Distance metric MUST be specified (cosine, dot product, or Euclidean)
- Payload schema MUST be documented for filtering and metadata storage
- Collection configuration MUST be reproducible

#### Metadata & Structured Knowledge (Neon Serverless Postgres)

- Book sections, citations, timestamps, and retrieval logs MUST be stored
- Schema MUST be normalized and documented
- Migration scripts MUST be provided for reproducibility
- Retrieval logs SHOULD support debugging and evaluation

#### AI Models (OpenAI Agents / ChatKit SDK)

- GPT models MUST be accessed through official OpenAI Agents or ChatKit SDK
- Model versions MUST be explicitly specified
- Grounding rules MUST be enforced in system prompts
- Token limits and context windows MUST be documented

#### User-Selected Text Mode

- Chatbot MUST accept a block of selected text as context
- When in selection-only mode, vector retrieval MUST be disabled
- Answers MUST be derived only from the selected text
- Mode switching MUST be clearly indicated in the UI

#### Frontend Integration (Docusaurus/React)

- Chat widget MUST be embedded inside the Docusaurus book
- Streaming responses MUST be supported
- Citations MUST be displayed in the UI
- Text selection interaction MUST be intuitive
- Widget MUST be responsive and accessible

### Source Standards

- Official documentation preferred (OpenAI, Qdrant, Neon, FastAPI, ROS 2, NVIDIA Isaac)
- Peer-reviewed papers preferred for theoretical robotics/AI content
- Minimum 15 sources required for documentation
- All claims MUST be linked to authoritative sources
- Community resources MAY be referenced for troubleshooting

### Citation Format

- APA citation style MUST be used in documentation and the book
- Example: Author, A. A. (Year). Title of work. Publisher. https://doi.org/xxxxx
- All external sources MUST have complete citation information
- Inline citations MUST reference the full citation in a references section

### Code Requirements

- All code MUST be functional and validated
- Use fenced code blocks with language tags (e.g., ```python, ```typescript, ```sql)
- Include file paths as comments when showing file contents
- API endpoints MUST include request/response examples
- Error handling MUST be demonstrated for critical paths

## Constraints

- **Documentation Length**: 5,000–7,000 words
- **Minimum Sources**: 15 authoritative sources
- **Output Format**: PDF with embedded APA citations (deliverable), plus Markdown for docs
- **Performance**: Latency < 1.5s for retrieval + response under typical load
- **Deployment Environments**:
  - Local development: FastAPI + Postgres + Qdrant Cloud
  - Production: GitHub Pages (frontend) + external API (backend)
- **No AI hallucinations**: All responses MUST be grounded in retrieved context
- **No plagiarism**: Content MUST be original or properly cited
- **Zero tolerance**: Plagiarism check MUST pass automated verification

## Success Criteria

### Technical Accuracy

- Every documentation claim verified against authoritative sources
- Zero plagiarism upon automated check
- All code examples execute correctly
- API contracts match implementation behavior
- All external links resolve to valid pages

### RAG System Quality

- **Retrieval Correctness**: Relevant passages retrieved for user queries
- **Hallucination Suppression**: No answers generated outside retrieved context
- **Selection Mode**: Correctly answers from user-selected text only when active
- **Citation Accuracy**: Sources correctly attributed in responses
- **Latency**: < 1.5s for retrieval + response under typical load

### Frontend Integration

- Chat widget renders correctly in Docusaurus
- Streaming responses display progressively
- Citations are clickable and navigate to source sections
- Text selection mode works intuitively
- Widget is responsive across device sizes

### Documentation Quality

- Clear setup and deployment instructions
- Reproducible pipelines with documented steps
- Logical flow from prerequisites to advanced topics
- Internal consistency across all documentation

### Reader/User Outcome

A user interacting with the chatbot MUST be able to:
1. Ask questions about book content and receive grounded answers
2. Select text and ask questions about that specific selection
3. See citations linking answers to source sections
4. Experience response latency under 1.5 seconds
5. Trust that answers are not hallucinated

## Governance

### Amendment Procedure

1. Proposed changes MUST be documented with rationale
2. Changes to Core Principles require explicit justification
3. All amendments MUST update the version number
4. Amendment history MUST be preserved in sync impact reports

### Versioning Policy

- **MAJOR**: Backward incompatible principle removals or redefinitions
- **MINOR**: New principle/section added or materially expanded guidance
- **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements

### Compliance Review

- All PRs/reviews MUST verify compliance with this constitution
- Technical claims MUST cite sources per Source Standards
- Code samples MUST be tested per Technical Requirements
- RAG responses MUST be evaluated for grounding compliance

### Conflict Resolution

- This constitution supersedes all other practices
- When principles conflict, prioritize in order: Accuracy > Grounding > Rigor > Clarity > others
- Ambiguous cases SHOULD be escalated to the project maintainer

**Version**: 2.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-07
