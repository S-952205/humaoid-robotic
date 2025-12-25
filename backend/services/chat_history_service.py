"""
Service to handle chat history operations with Neon Postgres database.
This service manages storing and retrieving chat messages while maintaining
compatibility with existing Qdrant RAG functionality.
"""
from typing import List, Dict
from sqlmodel import Session, select
from sqlalchemy import func
from models.chat_history import ChatHistory, ChatSession
from datetime import datetime
import uuid
import logging

logger = logging.getLogger(__name__)

class ChatHistoryService:
    """Service class to handle chat history operations with Neon Postgres."""

    def __init__(self, session: Session):
        """Initialize the chat history service with a database session."""
        self.session = session

    def save_message(self, session_id: str, role: str, content: str) -> ChatHistory:
        """Save a chat message to the database."""
        logger.info(f"Saving message to history for session {session_id}, role: {role}")
        try:
            # Create a new chat history record with string session_id
            chat_message = ChatHistory(
                session_id=session_id,
                role=role,
                content=content,
                timestamp=datetime.utcnow()
            )

            # Add to session and commit
            self.session.add(chat_message)
            self.session.commit()
            self.session.refresh(chat_message)

            logger.info(f"Successfully saved message to history with ID {chat_message.id}")
            return chat_message
        except Exception as e:
            logger.error(f"Error saving message to history: {e}")
            self.session.rollback()
            # Graceful degradation: log the error but don't crash the application
            # In case of database issues, we can still continue with chat functionality
            logger.warning(f"Database error during message save for session {session_id}, continuing without message persistence")
            # Return a dummy ChatHistory-like object to maintain compatibility
            class DummyMessage:
                def __init__(self, session_id, role, content):
                    self.id = str(uuid.uuid4())
                    self.session_id = session_id
                    self.role = role
                    self.content = content
                    self.timestamp = datetime.utcnow()
            return DummyMessage(session_id, role, content)

    def get_history(self, session_id: str, limit: int = 20, offset: int = 0) -> List[ChatHistory]:
        """Retrieve chat history for a specific session."""
        logger.info(f"Retrieving chat history for session {session_id}, limit: {limit}, offset: {offset}")
        try:
            # Query for messages in the session, ordered by timestamp
            statement = (
                select(ChatHistory)
                .where(ChatHistory.session_id == session_id)
                .order_by(ChatHistory.timestamp)
                .offset(offset)
                .limit(limit)
            )

            results = self.session.exec(statement).all()
            logger.info(f"Retrieved {len(results)} messages from history for session {session_id}")
            return results
        except Exception as e:
            logger.error(f"Error retrieving chat history: {e}")
            # Graceful degradation: return empty list instead of crashing
            logger.warning(f"Database error during history retrieval for session {session_id}, returning empty history")
            return []

    def get_session_messages_count(self, session_id: str) -> int:
        """Get the total count of messages in a session."""
        logger.info(f"Getting message count for session {session_id}")
        try:
            # Use COUNT query for better performance instead of fetching all records
            from sqlalchemy import func
            statement = select(func.count(ChatHistory.id)).where(ChatHistory.session_id == session_id)
            count = self.session.exec(statement).one()
            logger.info(f"Session {session_id} has {count} messages")
            return count
        except Exception as e:
            logger.error(f"Error counting session messages: {e}")
            # Graceful degradation: return 0 instead of crashing
            logger.warning(f"Database error during message count for session {session_id}, returning 0")
            return 0

    def create_session(self, session_id: str) -> ChatSession:
        """Create a new chat session if it doesn't exist."""
        logger.info(f"Creating/updating session {session_id}")
        try:
            # Check if session already exists
            existing_session = self.session.exec(
                select(ChatSession).where(ChatSession.session_id == session_id)
            ).first()

            if existing_session:
                # Update the timestamp of existing session
                existing_session.updated_at = datetime.utcnow()
                self.session.add(existing_session)
                self.session.commit()
                logger.info(f"Updated existing session {session_id}")
                return existing_session

            # Create new session
            chat_session = ChatSession(
                session_id=session_id,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )

            self.session.add(chat_session)
            self.session.commit()
            self.session.refresh(chat_session)
            logger.info(f"Created new session {session_id}")

            return chat_session
        except Exception as e:
            logger.error(f"Error creating session: {e}")
            self.session.rollback()
            # Graceful degradation: log the error but don't crash the application
            # In case of database issues, we can still continue with chat functionality
            logger.warning(f"Database error during session creation for {session_id}, continuing without session persistence")
            # Return a dummy ChatSession-like object to maintain compatibility
            # In a real implementation, we'd handle this differently
            class DummySession:
                def __init__(self, session_id):
                    self.session_id = session_id
                    self.created_at = datetime.utcnow()
                    self.updated_at = datetime.utcnow()
            return DummySession(session_id)

    def format_history_for_response(self, history: List[ChatHistory]) -> List[Dict]:
        """Format chat history for API response."""
        try:
            formatted_history = []
            for msg in history:
                # Validate message object has required attributes
                if not hasattr(msg, 'id') or not hasattr(msg, 'role') or not hasattr(msg, 'content') or not hasattr(msg, 'timestamp'):
                    logger.warning(f"Skipping invalid message object in history formatting: {msg}")
                    continue
                formatted_history.append({
                    "id": str(msg.id),
                    "role": msg.role,
                    "content": msg.content,
                    "timestamp": msg.timestamp.isoformat()
                })
            logger.info(f"Formatted {len(formatted_history)} messages for response")
            return formatted_history
        except Exception as e:
            logger.error(f"Error formatting history for response: {e}")
            # Return empty list as fallback
            return []