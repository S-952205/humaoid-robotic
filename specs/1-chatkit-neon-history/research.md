# Research: Floating ChatKit & Neon Persistence

## Overview
This document addresses technical unknowns and research findings for implementing the ChatKit floating widget with Neon Postgres persistence.

## Technical Decisions

### 1. OpenAI ChatKit Integration
- **Decision**: Use OpenAI's ChatKit components with a custom floating UI wrapper
- **Rationale**: ChatKit provides pre-built UI components and chat functionality, but requires custom positioning for the floating bubble
- **Alternatives considered**:
  - Building from scratch with OpenAI API
  - Using other chat widget libraries
- **Chosen approach**: Use ChatKit with a React component that handles floating positioning

### 2. Session Management
- **Decision**: Use UUID stored in localStorage for session identification
- **Rationale**: Simple implementation without requiring authentication, meets requirement for persistence across sessions
- **Alternatives considered**:
  - Server-side session management
  - JWT tokens
- **Chosen approach**: Client-side UUID in localStorage (as specified in requirements)

### 3. Database Schema for Chat History
- **Decision**: Create a ChatHistory table with fields for session_id, message_role, message_content, timestamp
- **Rationale**: Simple structure that supports the requirement to save user and assistant messages
- **Schema**:
  - id (primary key)
  - session_id (indexed for fast lookups)
  - role (user/assistant)
  - content (message text)
  - timestamp
  - created_at

### 4. Backend Framework Integration
- **Decision**: Integrate with existing FastAPI backend to maintain consistency
- **Rationale**: The project already uses FastAPI, so extending it is the most efficient approach
- **Implementation**: Add new endpoints to handle chat history storage and retrieval

### 5. Frontend Integration with Docusaurus
- **Decision**: Create a React component that can be injected into Docusaurus pages
- **Rationale**: Docusaurus supports React components, allowing for seamless integration
- **Approach**: Create a floating ChatWidget component that can be added via Docusaurus theme customization

### 6. Message History Depth
- **Decision**: Fetch last 20 messages by default, configurable limit
- **Rationale**: Balances context retention with performance considerations
- **Alternative**: Unlimited history could impact performance

### 7. Error Handling Strategy
- **Decision**: Graceful degradation when Neon database is unavailable
- **Rationale**: Chat functionality should continue even if history persistence fails temporarily
- **Implementation**: Continue chat operation but log errors and attempt to sync when available

## Architecture Pattern

### Frontend Architecture
- Floating ChatWidget component using React
- Positioned at bottom-right using CSS positioning
- Manages session_id in localStorage
- Communicates with backend via API calls
- Integrates with OpenAI ChatKit components

### Backend Architecture
- FastAPI endpoints for chat history management
- SQLModel for database abstraction
- Neon Postgres for data persistence
- Integration with existing Qdrant RAG functionality
- Maintains existing chat endpoint with enhanced history features

## Implementation Considerations

### Security
- API keys stored in environment variables
- No sensitive user data stored in chat history
- Input validation for message content

### Performance
- Indexed database queries for session lookups
- Pagination for large chat histories
- Caching strategies for frequently accessed data

### Compatibility
- Works with existing Qdrant integration
- Maintains current chat functionality
- Compatible with Docusaurus deployment to GitHub Pages