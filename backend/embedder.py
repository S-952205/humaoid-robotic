import cohere
from typing import List, Dict
from config import Config
import logging
import time

logger = logging.getLogger(__name__)

class CohereEmbedder:
    """Handles embedding generation using Cohere API"""

    def __init__(self):
        self.client = cohere.Client(Config.COHERE_API_KEY)
        self.model = "multilingual-22-12"  # Cohere's free tier model

    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts"""
        try:
            # Cohere API has limits, so we may need to batch requests
            # For now, we'll handle it in chunks of 96 (under the 96 limit)
            all_embeddings = []
            batch_size = 96  # Conservative batch size to stay under limits

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]

                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"  # Optimal for document search
                )

                all_embeddings.extend(response.embeddings)

                # Add a small delay to respect rate limits
                time.sleep(0.1)

            return all_embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    def embed_text(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        return self.create_embeddings([text])[0]

    def chunk_text(self, text: str, max_tokens: int = Config.CHUNK_SIZE,
                   overlap: int = Config.OVERLAP_SIZE) -> List[str]:
        """Split text into chunks of approximately max_tokens"""
        import tiktoken

        # Use gpt-3.5-turbo encoding as it's similar to what Cohere uses
        encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")
        tokens = encoding.encode(text)

        chunks = []
        start_idx = 0

        while start_idx < len(tokens):
            end_idx = start_idx + max_tokens

            # If we're at the end, just take the remaining tokens
            if end_idx > len(tokens):
                end_idx = len(tokens)

            chunk_tokens = tokens[start_idx:end_idx]
            chunk_text = encoding.decode(chunk_tokens)
            chunks.append(chunk_text)

            # Move start index by (max_tokens - overlap) to create overlap
            if end_idx >= len(tokens):
                break  # We've reached the end
            start_idx = end_idx - overlap

            # Handle case where remaining tokens are less than overlap
            if len(tokens) - start_idx < overlap:
                break

        # Filter out empty chunks or chunks with only whitespace
        chunks = [chunk.strip() for chunk in chunks if chunk.strip()]

        return chunks