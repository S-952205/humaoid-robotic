# Feature Specification: ChatKit UI & Neon History

**Feature Branch**: `1-chatkit-neon-history`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "ChatKit UI & Neon History

**Goal:** Add a professional bottom-right floating chat widget using OpenAI ChatKit and save history to Neon Postgres.

**Requirements:**

- **Frontend:** Install `@openai/chatkit` and implement a floating chat bubble in the **bottom-right corner** of the Docusaurus site.

- **Session:** Use `session_id` from `localStorage` to keep chat history persistent.

- **Backend:** Setup `SQLModel` with Neon Postgres to save/retrieve user and assistant messages.

- **RAG:** Continue using the existing Qdrant collection.

**Constraints:**

- No data re-ingestion (Keep existing Qdrant data).

- All keys must stay in `.env`.

- No user login/auth required.

docs of chatkit if you need to search: https://platform.openai.com/docs/guides/chatkit"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Access Chat Widget (Priority: P1)

As a visitor to the Docusaurus site, I want to access a floating chat widget in the bottom-right corner so that I can interact with the AI assistant without leaving the current page.

**Why this priority**: This is the foundational functionality that enables all other interactions. Without a visible and accessible chat interface, users cannot engage with the AI assistant.

**Independent Test**: The floating chat bubble should be visible on all pages of the Docusaurus site, and clicking it should open the chat interface with the ability to send and receive messages.

**Acceptance Scenarios**:

1. **Given** I am on any page of the Docusaurus site, **When** I see the floating chat bubble in the bottom-right corner, **Then** I can click it to open the chat interface
2. **Given** I have opened the chat interface, **When** I type a message and submit it, **Then** I should receive a response from the AI assistant

---

### User Story 2 - Persistent Chat History (Priority: P2)

As a returning visitor, I want my chat history to persist between sessions so that I can continue conversations where I left off.

**Why this priority**: This enhances user experience by maintaining continuity across visits, making the chat feel more personalized and useful.

**Independent Test**: After closing and reopening the browser, the chat history from previous sessions should still be visible in the chat interface.

**Acceptance Scenarios**:

1. **Given** I have had previous conversations, **When** I revisit the site after closing the browser, **Then** I should see my previous chat history
2. **Given** I am using a new browser or incognito mode, **When** I visit the site, **Then** I should start with a clean chat history

---

### User Story 3 - RAG-Powered Responses (Priority: P3)

As a user seeking information about the site's content, I want the AI assistant to provide accurate responses based on the site's documentation so that I can find relevant information quickly.

**Why this priority**: This adds value by leveraging existing content to provide more accurate and relevant responses to user queries.

**Independent Test**: When I ask questions related to the site's content, the AI should provide answers based on the existing documentation stored in Qdrant.

**Acceptance Scenarios**:

1. **Given** I ask a question about the site's content, **When** I submit the query to the AI, **Then** the response should be based on the site's documentation
2. **Given** I ask a question not covered by the site's documentation, **When** I submit the query, **Then** the AI should acknowledge the limitation and provide a general response

---

### Edge Cases

- What happens when the Neon database is temporarily unavailable?
- How does the system handle malformed or excessively long user messages?
- What occurs when the Qdrant vector database is unreachable during a query?
- How does the system behave when localStorage is disabled or full?
- What happens when the OpenAI ChatKit service is down?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a floating chat bubble in the bottom-right corner of all Docusaurus pages
- **FR-002**: System MUST initialize the OpenAI ChatKit interface when the chat bubble is clicked
- **FR-003**: System MUST store chat session ID in localStorage for persistence
- **FR-004**: System MUST save user and assistant messages to Neon Postgres database
- **FR-005**: System MUST retrieve chat history from Neon Postgres when initializing a session
- **FR-006**: System MUST integrate with existing Qdrant collection for RAG functionality
- **FR-007**: System MUST use environment variables for all API keys and connection strings
- **FR-008**: System MUST handle network errors gracefully without crashing the interface
- **FR-009**: System MUST maintain existing Qdrant data without re-ingestion

### Key Entities *(include if feature involves data)*

- **ChatSession**: Represents a user's chat session with a unique identifier stored in localStorage
- **ChatMessage**: Contains user or assistant messages with timestamp and session association
- **RAGContext**: Represents the connection to Qdrant for retrieving relevant documentation during chat

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can access the chat widget within 3 seconds of page load on 95% of page visits
- **SC-002**: Chat history persists across browser sessions with 99% reliability
- **SC-003**: 90% of user queries receive relevant responses based on site documentation
- **SC-004**: Chat interface responds to user messages within 5 seconds under normal load conditions
- **SC-005**: Zero data loss occurs when saving chat history to Neon Postgres during normal operation