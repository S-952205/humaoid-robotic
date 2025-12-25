from sqlmodel import SQLModel, Field, create_engine, Session
from typing import Optional, List
from datetime import datetime
import uuid
from enum import Enum


class MessageRole(str, Enum):
    user = "user"
    assistant = "assistant"


class ChatHistory(SQLModel, table=True):
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    session_id: str = Field(index=True)  # Changed from uuid.UUID to str for flexibility
    role: str = Field(regex="^(user|assistant)$", index=True)  # Using regex for enum-like validation, added index for performance
    content: str
    timestamp: datetime = Field(index=True)  # Added index for performance on timestamp queries
    created_at: datetime = Field(default_factory=datetime.utcnow, index=True)  # Added index for performance


class ChatSession(SQLModel, table=True):
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    session_id: str = Field(index=True)  # Changed from uuid.UUID to str for flexibility
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)