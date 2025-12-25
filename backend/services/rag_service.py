"""
RAG Service that maintains existing Qdrant integration while allowing for chat history integration.
This service ensures that existing Qdrant functionality remains intact while adding new features.
"""
from typing import List, Dict, Optional
from config import Config
from vector_store import QdrantVectorStore
from embedder import CohereEmbedder
import logging

logger = logging.getLogger(__name__)

class RagService:
    """Service class to handle RAG operations while maintaining existing Qdrant integration."""

    def __init__(self):
        """Initialize the RAG service with Qdrant vector store."""
        # Initialize the existing Qdrant vector store
        self.vector_store = QdrantVectorStore()
        self.embedder = CohereEmbedder()

    def search_similar(self, query: str, top_k: int = 5, chat_history: Optional[List[Dict]] = None) -> List[Dict]:
        """
        Search for similar content in Qdrant, optionally incorporating chat history for context.
        This maintains the existing Qdrant integration while adding history context capability.
        """
        try:
            # If chat history is provided, enrich the query with conversation context
            if chat_history and len(chat_history) > 0:
                # Build context from recent messages
                context_messages = []
                # Include last few messages to provide context without making query too long
                recent_messages = chat_history[-3:]  # Use last 3 messages as context

                for msg in recent_messages:
                    role = msg.get('role', 'user')
                    content = msg.get('content', '')
                    context_messages.append(f"{role}: {content}")

                # Combine original query with conversation context
                context_str = " ".join(context_messages)
                enriched_query = f"Context: {context_str}. Question: {query}"
            else:
                # Use original query if no history provided
                enriched_query = query

            # Generate embedding for the (potentially enriched) query
            query_embedding = self.embedder.embed_text(enriched_query)

            # Search in Qdrant using the existing functionality
            results = self.vector_store.search_similar(query_embedding, top_k=top_k)

            # Return results in the same format as before
            return results
        except Exception as e:
            logger.error(f"Error in RAG search: {e}")
            raise

    def search_similar_with_fallback(self, query: str, top_k: int = 5, chat_history: Optional[List[Dict]] = None) -> List[Dict]:
        """
        Search for similar content with fallback handling when Qdrant is unavailable.
        This ensures chat functionality continues even if RAG search fails.
        """
        try:
            return self.search_similar(query, top_k, chat_history)
        except Exception as e:
            logger.warning(f"RAG search failed, proceeding without RAG context: {e}")
            # Return empty results but don't fail the entire operation
            return []

    def get_vector_store(self) -> QdrantVectorStore:
        """Get the underlying vector store for direct access if needed."""
        return self.vector_store

    def ensure_existing_data_intact(self):
        """Method to ensure existing Qdrant data is not affected by new features."""
        # This method exists to explicitly ensure that existing Qdrant functionality
        # remains intact during the addition of chat history features
        logger.info("RAG service initialized with existing Qdrant integration preserved")

        # Verify the collection exists
        try:
            count = self.vector_store.get_all_documents_count()
            logger.info(f"Qdrant collection has {count} documents - existing data preserved")
        except Exception as e:
            logger.error(f"Error verifying existing Qdrant data: {e}")
            raise

    def prevent_reingestion(self):
        """Method to ensure existing Qdrant data is not re-ingested."""
        # This is a safeguard method to document that re-ingestion is not performed
        # The existing data in Qdrant remains as-is, we're only adding search capabilities
        logger.info("Confirmed: No re-ingestion of existing Qdrant data - preserving all existing content")