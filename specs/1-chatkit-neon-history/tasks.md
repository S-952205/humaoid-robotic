---
description: "Task list for ChatKit & Neon Integration feature implementation"
---

# Tasks: ChatKit & Neon Integration

**Input**: Design documents from `/specs/1-chatkit-neon-history/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure in backend/src/
- [x] T002 Create frontend project structure in src/components/ChatWidget/
- [x] T003 [P] Install backend dependencies: FastAPI, SQLModel, psycopg2-binary
- [x] T004 [P] Install frontend dependencies: @openai/chatkit
- [x] T005 [P] Configure environment variables in .env file

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Setup SQLModel database configuration with Neon Postgres in backend/src/database.py
- [x] T007 Create ChatHistory model in backend/src/models/chat_history.py
- [x] T008 [P] Create database engine and session dependencies in backend/src/database.py
- [x] T009 [P] Create database initialization function in backend/src/database.py
- [x] T010 Setup database migration framework in backend/src/database.py
- [x] T011 Configure existing Qdrant integration to remain intact in backend/src/services/rag_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Chat Widget (Priority: P1) 🎯 MVP

**Goal**: Display a floating chat bubble in the bottom-right corner that opens a chat interface when clicked, allowing users to send and receive messages.

**Independent Test**: The floating chat bubble should be visible on all pages of the Docusaurus site, and clicking it should open the chat interface with the ability to send and receive messages.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T012 [P] [US1] Contract test for GET /chat endpoint in backend/tests/contract/test_chat.py
- [ ] T013 [P] [US1] Contract test for POST /chat endpoint in backend/tests/contract/test_chat.py

### Implementation for User Story 1

- [x] T014 [P] [US1] Create ChatWidget component in src/components/ChatWidget/ChatWidget.jsx
- [x] T015 [P] [US1] Add CSS styling for floating position at bottom-right in src/components/ChatWidget/ChatWidget.css
- [x] T016 [US1] Implement basic chat functionality using @openai/chatkit in src/components/ChatWidget/ChatWidget.jsx
- [x] T017 [US1] Update backend POST /chat endpoint to handle basic chat requests in backend/src/api/chat.py
- [x] T018 [US1] Add session_id parameter to chat endpoint in backend/src/api/chat.py
- [x] T019 [US1] Add frontend integration to pass session_id to backend in src/components/ChatWidget/ChatWidget.jsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Persistent Chat History (Priority: P2)

**Goal**: Store chat messages in Neon Postgres database and retrieve previous chat history when initializing a session, maintaining persistence across browser sessions using localStorage.

**Independent Test**: After closing and reopening the browser, the chat history from previous sessions should still be visible in the chat interface.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T020 [P] [US2] Contract test for POST /chat/history endpoint in backend/tests/contract/test_history.py
- [ ] T021 [P] [US2] Contract test for GET /chat/history/{session_id} endpoint in backend/tests/contract/test_history.py

### Implementation for User Story 2

- [x] T022 [P] [US2] Create ChatHistory service in backend/src/services/chat_history_service.py
- [x] T023 [P] [US2] Implement POST /chat/history endpoint in backend/src/api/chat.py
- [x] T024 [P] [US2] Implement GET /chat/history/{session_id} endpoint in backend/src/api/chat.py
- [x] T025 [US2] Update chat endpoint to save user messages to Neon in backend/src/api/chat.py
- [x] T026 [US2] Update chat endpoint to save assistant responses to Neon in backend/src/api/chat.py
- [x] T027 [US2] Update chat endpoint to retrieve history before processing new messages in backend/src/api/chat.py
- [x] T028 [US2] Implement localStorage session_id management in src/components/ChatWidget/ChatWidget.jsx
- [x] T029 [US2] Fetch chat history from backend when ChatWidget initializes in src/components/ChatWidget/ChatWidget.jsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - RAG-Powered Responses (Priority: P3)

**Goal**: Integrate existing Qdrant RAG functionality with the chat history, ensuring that responses are based on both the chat history and relevant documentation from Qdrant.

**Independent Test**: When asking questions related to the site's content, the AI should provide answers based on the existing documentation stored in Qdrant, while maintaining the chat history context.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T030 [P] [US3] Contract test for enhanced POST /chat endpoint with history in backend/tests/contract/test_rag.py

### Implementation for User Story 3

- [x] T031 [P] [US3] Update chat service to include history in RAG context in backend/src/services/chat_service.py
- [x] T032 [US3] Modify RAG search to include chat history in query context in backend/src/services/rag_service.py
- [x] T033 [US3] Update response formatting to include RAG sources when applicable in backend/src/api/chat.py
- [x] T034 [US3] Add error handling for when Qdrant is unavailable while maintaining chat functionality in backend/src/services/rag_service.py
- [x] T035 [US3] Ensure existing Qdrant data is not re-ingested in backend/src/services/rag_service.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T036 [P] Add comprehensive error handling in src/components/ChatWidget/ChatWidget.jsx
- [x] T037 [P] Add loading states and UI feedback in src/components/ChatWidget/ChatWidget.jsx
- [X] T038 Add validation for message content and session_id in backend/src/api/chat.py
- [X] T039 [P] Add logging for chat operations in backend/src/services/chat_service.py
- [X] T040 Add graceful degradation when Neon database is unavailable in backend/src/services/chat_history_service.py
- [X] T041 [P] Documentation updates in docs/
- [X] T042 Code cleanup and refactoring
- [X] T043 Performance optimization for database queries in backend/src/services/chat_history_service.py
- [X] T044 [P] Additional unit tests in backend/tests/unit/
- [X] T045 Security hardening for API endpoints in backend/src/api/chat.py
- [X] T046 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on basic chat functionality (US1) for integration
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on chat history functionality (US2) for context integration

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for GET /chat endpoint in backend/tests/contract/test_chat.py"
Task: "Contract test for POST /chat endpoint in backend/tests/contract/test_chat.py"

# Launch all models for User Story 1 together:
Task: "Create ChatWidget component in frontend/src/components/ChatWidget/ChatWidget.jsx"
Task: "Add CSS styling for floating position at bottom-right in frontend/src/components/ChatWidget/ChatWidget.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence