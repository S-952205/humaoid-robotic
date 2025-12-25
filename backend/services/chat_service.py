"""
Service to handle chat operations with integrated RAG and history context.
This service coordinates between chat history, RAG functionality, and response generation.
"""
from typing import List, Dict, Optional
from sqlmodel import Session
from services.chat_history_service import ChatHistoryService
from services.rag_service import RagService
from models.chat_history import MessageRole
from agent import answer_query
import logging

logger = logging.getLogger(__name__)

class ChatService:
    """Service class to handle chat operations with RAG and history integration."""

    def __init__(self, session: Session):
        """Initialize the chat service with database session and other services."""
        self.session = session
        self.history_service = ChatHistoryService(session)
        self.rag_service = RagService()

    def create_session(self, session_id: str):
        """Create or update a chat session."""
        return self.history_service.create_session(session_id)

    def save_message(self, session_id: str, role: str, content: str):
        """Save a message to the chat history."""
        return self.history_service.save_message(session_id, role, content)

    def get_conversation_context(self, session_id: str, limit: int = 10) -> List[Dict]:
        """Get recent conversation history to provide context for RAG."""
        history = self.history_service.get_history(session_id, limit=limit)
        return self.history_service.format_history_for_response(history)

    def process_query_with_history(self, query: str, session_id: str, top_k: int = 5):
        """
        Process a query by incorporating conversation history with RAG search.
        This method maintains existing Qdrant functionality while adding history context.
        """
        try:
            logger.info(f"Processing query for session {session_id}")

            # Create or update session
            self.history_service.create_session(session_id)

            # Get recent conversation history
            conversation_history = self.get_conversation_context(session_id, limit=10)
            logger.info(f"Retrieved {len(conversation_history)} messages from history for session {session_id}")

            # Save user message to history
            user_message = self.history_service.save_message(
                session_id=session_id,
                role="user",
                content=query
            )
            logger.info(f"Saved user message to history: {user_message.id}")

            # Use the existing answer_query function which handles RAG
            # The session_id parameter is already used by the existing system for context
            response = answer_query(
                query=query,
                top_k=top_k,
                session_id=session_id
            )

            # Save assistant response to history
            assistant_message = self.history_service.save_message(
                session_id=session_id,
                role="assistant",
                content=response
            )
            logger.info(f"Saved assistant response to history: {assistant_message.id}")

            result = {
                "response": response,
                "session_id": session_id,
                "history": conversation_history,
                "sources": []  # Sources are handled internally by the existing system
            }

            logger.info(f"Successfully processed query for session {session_id}")
            return result
        except Exception as e:
            logger.error(f"Error in chat service: {e}")
            raise

    def ensure_existing_qdrant_intact(self):
        """Method to ensure existing Qdrant data and functionality remains intact."""
        try:
            # Verify the RAG service can access existing Qdrant data
            self.rag_service.ensure_existing_data_intact()
            logger.info("Existing Qdrant integration verified and preserved")
        except Exception as e:
            logger.error(f"Error verifying existing Qdrant data: {e}")
            raise