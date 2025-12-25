# Data Model: Floating ChatKit & Neon Persistence

## Overview
This document defines the data models for the chat history persistence feature using SQLModel with Neon Postgres.

## Entity: ChatHistory

### Fields
- **id** (UUID, Primary Key)
  - Type: UUID (auto-generated)
  - Required: Yes
  - Description: Unique identifier for each chat message

- **session_id** (UUID)
  - Type: UUID
  - Required: Yes
  - Indexed: Yes
  - Description: Identifier linking messages to a specific chat session

- **role** (String)
  - Type: String (enum: "user", "assistant")
  - Required: Yes
  - Description: Indicates whether the message is from user or assistant

- **content** (Text)
  - Type: Text
  - Required: Yes
  - Description: The actual message content

- **timestamp** (DateTime)
  - Type: DateTime
  - Required: Yes
  - Description: Time when the message was created

- **created_at** (DateTime)
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: Record creation time

### Relationships
- None (standalone entity)

### Validation Rules
- session_id must be a valid UUID
- role must be either "user" or "assistant"
- content must not be empty
- timestamp must be in the past or present

## Entity: ChatSession (Optional Enhancement)

### Fields
- **id** (UUID, Primary Key)
  - Type: UUID (auto-generated)
  - Required: Yes
  - Description: Unique identifier for the chat session

- **session_id** (UUID)
  - Type: UUID
  - Required: Yes
  - Indexed: Yes
  - Description: Client-side session identifier

- **created_at** (DateTime)
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: Session creation time

- **updated_at** (DateTime)
  - Type: DateTime
  - Required: Yes
  - Default: Current timestamp
  - Description: Last activity timestamp

### Relationships
- One ChatSession to Many ChatHistory records (via session_id foreign key)

### Validation Rules
- session_id must be a valid UUID
- updated_at must be >= created_at

## Database Schema (SQLModel)

```python
from sqlmodel import SQLModel, Field, create_engine, Session
from typing import Optional, List
from datetime import datetime
import uuid

class ChatHistory(SQLModel, table=True):
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    session_id: uuid.UUID = Field(index=True)
    role: str = Field(regex="^(user|assistant)$")  # Using regex for enum-like validation
    content: str
    timestamp: datetime
    created_at: datetime = Field(default_factory=datetime.utcnow)

class ChatSession(SQLModel, table=True):
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    session_id: uuid.UUID = Field(index=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
```

## State Transitions

### ChatHistory
- Immutable once created (no state transitions)

### ChatSession
- Created when a new chat session starts
- Updated when new messages are added to the session
- Potentially archived after inactivity period (future enhancement)

## Indexing Strategy

### Required Indexes
1. **session_id** on ChatHistory table - for efficient session-based queries
2. **created_at** on ChatHistory table - for chronological ordering
3. **session_id** on ChatSession table - for fast session lookups

### Query Patterns
1. Retrieve all messages for a session_id (with pagination)
2. Retrieve last N messages for a session_id
3. Create new chat history record
4. Update session timestamp when new messages are added