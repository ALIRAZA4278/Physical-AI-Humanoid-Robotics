<!--
SYNC IMPACT REPORT
==================
Version change: 0.0.0 → 1.0.0
Bump rationale: Initial constitution ratification (MAJOR) - establishing all principles and governance

Modified principles: N/A (new document)

Added sections:
- Core Principles (7 principles: Accuracy, Clarity, Consistency, Modularity, Maintainability, Practicality, Version Awareness)
- Key Standards (Writing Style, Technical Requirements, Source Standards, Citation Format, Code Requirements)
- Constraints
- Success Criteria
- Governance

Removed sections: None

Templates consistency check:
- .specify/templates/plan-template.md ✅ Compatible (Constitution Check section references constitution file)
- .specify/templates/spec-template.md ✅ Compatible (Requirements and Success Criteria align with constitution)
- .specify/templates/tasks-template.md ✅ Compatible (Phase structure supports book chapter workflows)

Follow-up TODOs: None
-->

# AI-Driven Book Creation Constitution

## Core Principles

### I. Accuracy

All technical explanations MUST be validated using official documentation (Docusaurus, GitHub Pages, Spec-Kit Plus, Claude Code, Node.js, etc.).

- Technical claims MUST be verifiable against official sources
- Code samples MUST be tested before inclusion
- Commands MUST be validated on stated tool versions
- No AI hallucinations permitted—all claims MUST be source-verified
- When documentation conflicts exist, prefer the most recent official release documentation

**Rationale**: Books teaching technical workflows become worthless if instructions don't work. Accuracy is non-negotiable for developer trust.

### II. Clarity

Writing MUST be understandable for intermediate developers (1–3 years experience).

- Definitions MUST precede advanced concepts
- Step-by-step explanations MUST follow logical progression
- Jargon MUST be explained on first use
- Complex concepts MUST include examples
- Each chapter MUST have clear learning objectives stated upfront

**Rationale**: The target audience has foundational knowledge but may lack familiarity with specific tools. Meeting them where they are maximizes comprehension.

### III. Consistency

All chapters MUST follow a unified structure, tone, and formatting style.

- Technical-tutorial tone throughout
- Consistent heading hierarchy across chapters
- Uniform code block formatting with language tags
- Standardized callout styles (warnings, tips, notes)
- Cross-references MUST use consistent link format

**Rationale**: Inconsistent documentation creates cognitive load and undermines professional credibility.

### IV. Modularity

Content MUST be structured in small, reusable sections suitable for Docusaurus.

- Each section SHOULD be independently navigable
- Topics MUST avoid circular dependencies where possible
- Content MUST support sidebar navigation patterns
- Sections SHOULD be suitable for re-ordering without breaking comprehension
- Each chapter MUST work as a standalone reference after initial setup

**Rationale**: Docusaurus is designed for modular documentation. Forcing long-form narrative into it creates poor user experience.

### V. Maintainability

All content MUST be easy to update as Docusaurus or tools evolve.

- Version numbers MUST be explicitly stated for all tools
- Installation commands MUST specify version constraints
- Deprecated features MUST NOT be taught as primary approaches
- Content SHOULD anticipate common version migration pain points
- Update procedures SHOULD be documented alongside initial setup

**Rationale**: Technical documentation has a shelf life. Designing for updates extends the book's useful lifespan.

### VI. Practicality

Prioritize actionable, hands-on guidance with working code examples.

- Every concept MUST include a working example
- Examples MUST be copy-paste ready (no pseudo-code without explanation)
- Build & deploy instructions MUST be complete end-to-end
- GitHub repo file structure MUST be shown whenever relevant
- Reader MUST be able to follow along and produce working results

**Rationale**: Developers learn by doing. Theoretical explanations without practical application fail to teach effectively.

### VII. Version Awareness

Always mention the version of every tool or framework used.

- Node.js version MUST be stated
- Docusaurus version MUST be stated
- npm/yarn version SHOULD be stated
- All dependency versions MUST be documented
- Version compatibility notes MUST be included for known conflicts

**Rationale**: Version mismatches are the #1 cause of "it doesn't work" issues. Explicit versioning enables debugging.

## Key Standards

### Writing Style
- Technical-tutorial tone maintained throughout
- Step-by-step explanations with numbered procedures
- Definitions MUST precede advanced concepts
- All code samples MUST be tested before inclusion
- Active voice preferred; passive voice only when emphasizing the action target

### Technical Requirements
- Commands MUST be tested on the latest stable Node.js and relevant Docusaurus version
- Include required GitHub repo file structure whenever relevant
- Provide complete build & deploy instructions using GitHub Pages
- For Spec-Kit Plus workflows, follow official schema and command patterns
- All CLI commands MUST show expected output where helpful

### Source Standards
- Prefer official documentation (Docusaurus, GitHub Pages, Spec-Kit Plus, Git)
- When external sources are used, prioritize reputable technical blogs or docs
- All claims about tools MUST be linked to official sources
- Community resources (Stack Overflow, GitHub issues) MAY be referenced for troubleshooting

### Citation Format
- Use Markdown links only (not APA or academic formats)
- Example: `[Docusaurus Deployment Docs](https://docusaurus.io/docs/deployment)`
- Links SHOULD use descriptive anchor text, not "click here"
- Official docs MUST be linked; third-party sources SHOULD be linked

### Code Requirements
- All code MUST be functional and validated
- Use fenced code blocks with language tags (e.g., ```bash, ```js, ```json)
- Avoid pseudo-code unless clearly explained as such
- Include file paths as comments when showing file contents
- Show both command and expected output where clarity benefits

## Constraints

- **Book Length**: 30–80 pages (flexible by chapter count)
- **Output Format**: Markdown files (`.md`/`.mdx`) in Docusaurus-friendly structure
- **Deployment**: MUST build and deploy to GitHub Pages without errors
- **Tooling**: MUST support Spec-Kit Plus workflows and Claude Code
- **No AI hallucinations**: All technical claims MUST be source-verified
- **No plagiarism**: Content MUST be original or properly referenced
- **No broken links**: All documentation links MUST be validated before publication

## Success Criteria

### Technical Accuracy
- Docusaurus builds successfully (`npm run build`)
- GitHub Pages deployment completes without CI/CD errors
- All code examples work exactly as shown
- All external links resolve to valid pages

### Documentation Quality
- Clear navigation and chapter structure
- Internal consistency across the book
- No ambiguous, misleading, or unverified statements
- Logical flow from prerequisites to advanced topics

### Spec-Kit Plus Compliance
- Book generation workflows run successfully
- Constitutional rules are followed throughout
- No contradictions between chapters
- PHRs created for all significant development sessions

### Reader Outcome
A reader with basic web development skills MUST be able to:
1. Create a Docusaurus project
2. Write chapters using Spec-Kit Plus
3. Use Claude Code to generate structured content
4. Deploy the book to GitHub Pages
5. Maintain and update the documentation easily

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
- Complexity MUST be justified against the Modularity principle
- Technical claims MUST cite sources per Source Standards
- Code samples MUST be tested per Technical Requirements

### Conflict Resolution
- This constitution supersedes all other practices
- When principles conflict, prioritize in order: Accuracy > Practicality > Clarity > others
- Ambiguous cases SHOULD be escalated to the project maintainer

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
