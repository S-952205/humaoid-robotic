# ChatKit & Neon Integration Documentation

## Overview
This documentation covers the implementation of the floating chat widget with Neon Postgres persistence for the Docusaurus-based humanoid robot book.

## Architecture

### Frontend
- **ChatWidget Component**: React component that provides a floating chat interface
- **Positioning**: Fixed to bottom-right corner of the screen
- **Persistence**: Uses localStorage for session ID management
- **API Communication**: Makes requests to backend chat endpoints

### Backend
- **API Endpoints**: `/chat/` prefix for all chat-related operations
- **Database**: Neon Postgres for chat history persistence
- **RAG Integration**: Maintains compatibility with existing Qdrant functionality
- **Session Management**: Uses UUID-based session IDs for conversation tracking

## API Endpoints

### POST /chat/
- Handles basic chat requests with session_id
- Maintains compatibility with existing Qdrant RAG functionality
- Includes chat history persistence

### POST /chat/query
- Query endpoint for RAG functionality with session support
- Maintains compatibility with existing Qdrant integration
- Returns RAG-enhanced responses with context

### POST /chat/history
- Save a chat message to the history database
- Validates message content and session_id

### GET /chat/history/{`session_id`}
- Retrieve chat history for a specific session
- Supports pagination with limit and offset parameters

## Session Management

### Session ID Format
- UUID v4 format (e.g., "550e8400-e29b-41d4-a716-446655440000")
- Stored in localStorage for persistence across browser sessions
- Validated for security (5-100 characters, alphanumeric with hyphens/underscores only)

### Message Structure
- **Role**: "user" or "assistant"
- **Content**: Text content of the message (max 10000 characters)
- **Timestamp**: UTC timestamp of when the message was created

## Security Features

### Input Validation
- Query length limited to 10,000 characters
- Session ID format validated with regex
- Content sanitization to prevent basic injection attempts
- Role validation to ensure only "user" or "assistant" values

### Error Handling
- Comprehensive logging for debugging
- Graceful degradation when Neon database is unavailable
- Proper HTTP status codes for different error conditions

## Performance Optimizations

### Database
- Indexed fields for efficient querying:
  - session_id
  - role
  - timestamp
  - created_at
- COUNT queries instead of fetching all records for counting operations

### Frontend
- Loading states to provide user feedback
- Error handling for network requests
- Session persistence using localStorage

## Configuration

### Environment Variables
- `NEON_DATABASE_URL`: Connection string for Neon Postgres database
- `GEMINI_API_KEY`: API key for Gemini model (fallback to Qwen if not available)
- `QWEN_API_KEY`: API key for Qwen model (fallback if Gemini not available)

### Fallback Model System
The system prioritizes models in this order:
1. Gemini (if GEMINI_API_KEY is set)
2. Qwen (if QWEN_API_KEY is set and Gemini is not configured)
3. Generic OpenAI-compatible model for testing

## Troubleshooting

### Common Issues
1. **Database Connection Errors**: Check NEON_DATABASE_URL configuration
2. **CORS Issues**: Verify backend CORS settings for your frontend origin
3. **Model API Errors**: Ensure proper API keys are configured for fallback models

### Logging
- All chat operations are logged with session information
- Error conditions are logged with full context
- Performance metrics are available via the /metrics endpoint