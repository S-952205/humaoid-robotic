import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables"""

    # Docusaurus website URL
    DEPLOYED_BOOK_URL = os.getenv("DEPLOYED_BOOK_URL")
    if not DEPLOYED_BOOK_URL:
        print("WARNING: DEPLOYED_BOOK_URL environment variable is not set")
        DEPLOYED_BOOK_URL = "http://localhost:3000"  # Use localhost for testing

    # Cohere API configuration
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    if not COHERE_API_KEY:
        print("WARNING: COHERE_API_KEY environment variable is not set")
        COHERE_API_KEY = "cohere-dummy-api-key-for-testing"  # Use dummy key for testing

    # Qdrant configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    if not QDRANT_URL:
        print("WARNING: QDRANT_URL environment variable is not set")
        QDRANT_URL = "http://localhost:6333"  # Use localhost for testing

    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    if not QDRANT_API_KEY:
        print("WARNING: QDRANT_API_KEY environment variable is not set")
        QDRANT_API_KEY = "qdrant-dummy-api-key-for-testing"  # Use dummy key for testing

    # Qdrant cluster ID (for managed Qdrant clusters)
    QDRANT_CLUSTER_ID = os.getenv("QDRANT_CLUSTER_ID")

    # OpenRouter API configuration (replacing OpenAI/Gemini)
    OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY")
    if not OPENROUTER_API_KEY:
        print("WARNING: OPENROUTER_API_KEY environment variable is not set")
        OPENROUTER_API_KEY = "sk-or-v1-dummy-key-for-testing"  # Use dummy key for testing

    # Chunking configuration
    CHUNK_SIZE = 500  # Maximum number of tokens per chunk
    OVERLAP_SIZE = 50  # Number of tokens to overlap between chunks

    # Qdrant collection name
    COLLECTION_NAME = "docusaurus_content"