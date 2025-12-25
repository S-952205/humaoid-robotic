# Implementation Plan: Floating ChatKit & Neon Persistence

**Branch**: `1-chatkit-neon-history` | **Date**: 2025-12-19 | **Spec**: [link](../specs/1-chatkit-neon-history/spec.md)
**Input**: Feature specification from `/specs/1-chatkit-neon-history/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a floating chat widget using OpenAI ChatKit with chat history persistence to Neon Postgres database. The solution will include a React-based floating bubble component positioned at the bottom-right of Docusaurus pages, with backend API endpoints to store and retrieve chat history while maintaining integration with existing Qdrant RAG functionality.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend components
**Primary Dependencies**: FastAPI, SQLModel, @openai/chatkit, Neon Postgres, Qdrant
**Storage**: Neon Postgres (for chat history), Qdrant (for RAG)
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Web (Docusaurus site with FastAPI backend)
**Project Type**: Web (frontend + backend)
**Performance Goals**: <5 second response time for chat messages
**Constraints**: <200ms p95 for database operations, maintain existing Qdrant data without re-ingestion
**Scale/Scope**: Single user sessions with persistent history

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Beginner-friendly English: Implementation will include clear documentation and comments
- ✅ Simulation-first: N/A for this feature
- ✅ Practical Code Examples and Diagrams: Will provide implementation examples with documentation
- ✅ Accurate, Verified Technical Content: Code will be tested and verified
- ✅ Docusaurus-compatible Markdown: Frontend will be compatible with Docusaurus
- ✅ Clear Chapter Structure: Implementation will follow clear, structured approach
- ✅ No hardware assembly required: N/A for this feature
- ✅ No advanced mathematical concepts: N/A for this feature
- ✅ All robotics examples use ROS 2: N/A for this feature
- ✅ Deployable on GitHub Pages: Solution will be compatible with GitHub Pages deployment

## Project Structure

### Documentation (this feature)

```text
specs/1-chatkit-neon-history/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   └── chat_history.py          # SQLModel for chat history
│   ├── services/
│   │   └── chat_service.py          # Chat history management
│   └── api/
│       └── chat.py                  # Updated chat endpoints
└── tests/
    └── unit/

frontend/
├── src/
│   ├── components/
│   │   └── ChatWidget/              # Floating chat widget component
│   └── services/
│       └── chatkit_service.js       # ChatKit integration
└── tests/
    └── unit/
```

**Structure Decision**: Selected web application structure with separate backend and frontend directories to maintain clear separation of concerns between the FastAPI backend and Docusaurus frontend.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |