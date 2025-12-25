from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from config import Config
import logging

logger = logging.getLogger(__name__)

class QdrantVectorStore:
    """Handles vector storage and retrieval using Qdrant"""

    def __init__(self):
        # Initialize Qdrant client
        # The cluster_id parameter might not be supported in newer versions of qdrant-client
        # Instead, we'll use the URL and API key directly
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
        )
        self.collection_name = Config.COLLECTION_NAME

    def create_collection(self):
        """Create the collection if it doesn't exist"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with appropriate vector size (Cohere embeddings are 768-dim)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=768,  # Cohere multilingual-22-12 model produces 768-dim vectors
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error creating collection: {e}")
            raise

    def store_embeddings(self, chunks: List[Dict], embeddings: List[List[float]]):
        """Store chunks and their embeddings in Qdrant"""
        try:
            # Prepare points for insertion
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = models.PointStruct(
                    id=i,
                    vector=embedding,
                    payload={
                        "content": chunk["content"],
                        "title": chunk["title"],
                        "url": chunk["url"],
                        "source": chunk.get("source", "docusaurus"),
                        "chunk_index": chunk.get("chunk_index", 0)
                    }
                )
                points.append(point)

            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Stored {len(points)} embeddings in Qdrant")
        except Exception as e:
            logger.error(f"Error storing embeddings: {e}")
            raise

    def search_similar(self, query_embedding: List[float], top_k: int = 5) -> List[Dict]:
        """Search for similar content based on embedding"""
        try:
            # Verify that the client is properly initialized and accessible
            if not hasattr(self.client, 'search') and not hasattr(self.client, 'search_points'):
                # Try to ping the Qdrant server to verify connection
                try:
                    self.client.get_collections()
                except Exception as conn_error:
                    logger.error(f"Cannot connect to Qdrant server: {conn_error}")
                    raise AttributeError("Qdrant client is not properly connected or initialized")

                # If connection works but methods don't exist, raise specific error
                raise AttributeError("Qdrant client does not have 'search' or 'search_points' method - check client version")

            # Check which search method is available (different versions of qdrant-client have different method names)
            if hasattr(self.client, 'search'):
                # Newer versions use 'search' method
                results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k,
                    with_payload=True
                )
            elif hasattr(self.client, 'search_points'):
                # Older versions might use 'search_points' method
                results = self.client.search_points(
                    collection_name=self.collection_name,
                    vector=query_embedding,
                    limit=top_k,
                    with_payload=True
                )
            else:
                # This should not happen due to the earlier check, but just in case
                raise AttributeError("Qdrant client does not have 'search' or 'search_points' method")

            # Format results
            formatted_results = []
            for result in results:
                # Handle different result formats based on Qdrant client version
                if hasattr(result, 'payload'):
                    content = result.payload.get("content", "")
                    title = result.payload.get("title", "")
                    url = result.payload.get("url", "")
                elif hasattr(result, 'dict') and 'payload' in result.dict:
                    content = result.dict['payload'].get("content", "")
                    title = result.dict['payload'].get("title", "")
                    url = result.dict['payload'].get("url", "")
                else:
                    # Fallback to accessing as dictionary directly
                    content = result.get("content", "") if hasattr(result, 'get') else ""
                    title = result.get("title", "") if hasattr(result, 'get') else ""
                    url = result.get("url", "") if hasattr(result, 'get') else ""

                # Extract score
                if hasattr(result, 'score'):
                    score = result.score
                elif hasattr(result, 'dict') and 'score' in result.dict:
                    score = result.dict['score']
                else:
                    score = getattr(result, 'dict', {}).get("score", 0.0)

                formatted_results.append({
                    "content": content,
                    "title": title,
                    "url": url,
                    "score": score
                })

            return formatted_results
        except Exception as e:
            logger.error(f"Error searching similar content: {e}")
            # Re-raise the exception to be handled upstream
            raise

    def get_all_documents_count(self) -> int:
        """Get the total number of documents in the collection"""
        try:
            count = self.client.count(
                collection_name=self.collection_name
            )
            return count.count
        except Exception as e:
            logger.error(f"Error getting document count: {e}")
            return 0

    def clear_collection(self):
        """Clear all vectors from the collection (useful for re-indexing)"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=models.Filter()
                )
            )
            logger.info(f"Cleared collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error clearing collection: {e}")
            raise