# API Contracts: Floating ChatKit & Neon Persistence

## Overview
This document defines the API contracts for the chat history persistence feature, including request/response formats and integration points with existing functionality.

## 1. Chat History Endpoints

### 1.1 Save Chat Message
**Endpoint**: `POST /chat/history`

**Description**: Saves a user or assistant message to the database with session context.

**Request**:
```json
{
  "session_id": "string (UUID)",
  "role": "string (user|assistant)",
  "content": "string (message content)",
  "timestamp": "string (ISO 8601 datetime)"
}
```

**Response**:
```json
{
  "success": "boolean",
  "message_id": "string (UUID of saved message)",
  "timestamp": "string (ISO 8601 datetime)"
}
```

**Error Responses**:
- `400`: Invalid request format
- `500`: Database error

### 1.2 Get Chat History
**Endpoint**: `GET /chat/history/{session_id}`

**Description**: Retrieves chat history for a specific session.

**Parameters**:
- `session_id`: Path parameter (UUID)
- `limit`: Query parameter (integer, default: 20)
- `offset`: Query parameter (integer, default: 0)

**Response**:
```json
{
  "session_id": "string (UUID)",
  "messages": [
    {
      "id": "string (UUID)",
      "role": "string (user|assistant)",
      "content": "string (message content)",
      "timestamp": "string (ISO 8601 datetime)"
    }
  ],
  "total_count": "integer"
}
```

**Error Responses**:
- `404`: Session not found
- `500`: Database error

## 2. Enhanced Chat Endpoint

### 2.1 Chat with History Integration
**Endpoint**: `POST /chat`

**Description**: Enhanced chat endpoint that retrieves history before processing new messages and saves new exchanges to history.

**Request**:
```json
{
  "session_id": "string (UUID)",
  "message": "string (user message content)",
  "history_depth": "integer (optional, default: 20)"
}
```

**Response**:
```json
{
  "response": "string (assistant response)",
  "session_id": "string (UUID)",
  "timestamp": "string (ISO 8601 datetime)",
  "sources": "array (RAG sources, if applicable)"
}
```

**Error Responses**:
- `400`: Invalid request format
- `500`: Processing error

## 3. Frontend Integration Contracts

### 3.1 ChatWidget Component Interface
**Props**:
```typescript
interface ChatWidgetProps {
  openAIApiKey: string;
  backendUrl: string;
  initialOpen?: boolean;
  position?: 'bottom-right' | 'bottom-left';
}
```

**Methods**:
- `getSessionId()`: Returns current session ID
- `setSessionId(id: string)`: Sets session ID
- `clearHistory()`: Clears current chat history

### 3.2 Session Management
**LocalStorage Keys**:
- `chat_session_id`: Stores the current session UUID
- `chat_widget_open`: Boolean indicating if widget is open

## 4. Data Models

### 4.1 ChatMessage
```json
{
  "id": "string (UUID)",
  "session_id": "string (UUID)",
  "role": "string (user|assistant)",
  "content": "string (message content)",
  "timestamp": "string (ISO 8601 datetime)",
  "created_at": "string (ISO 8601 datetime)"
}
```

## 5. Integration Points

### 5.1 With Existing Qdrant Integration
The new chat endpoint maintains compatibility with existing Qdrant RAG functionality by:
1. Retrieving chat history before RAG search
2. Including history in context for more relevant responses
3. Preserving existing Qdrant search parameters

### 5.2 With Docusaurus Frontend
The ChatWidget component integrates with Docusaurus by:
1. Using React component injection
2. Following Docusaurus styling conventions
3. Maintaining responsive design principles