"""
Pytest configuration for the RAG Chatbot tests
"""
import os
import sys
from unittest.mock import patch

# Set up environment variables for testing
os.environ.setdefault('DEPLOYED_BOOK_URL', 'https://example.com')
os.environ.setdefault('COHERE_API_KEY', 'test-key')
os.environ.setdefault('QDRANT_URL', 'https://test-qdrant.com')
os.environ.setdefault('QDRANT_API_KEY', 'test-key')
os.environ.setdefault('GEMINI_API_KEY', 'test-key')

# Add backend directory to path so imports work
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Mock the Config class to use test values
from unittest.mock import Mock

def pytest_configure(config):
    """Configure pytest"""
    import config

    # Mock the Config class to avoid environment variable requirements during testing
    mock_config = Mock()
    mock_config.DEPLOYED_BOOK_URL = os.environ['DEPLOYED_BOOK_URL']
    mock_config.COHERE_API_KEY = os.environ['COHERE_API_KEY']
    mock_config.QDRANT_URL = os.environ['QDRANT_URL']
    mock_config.QDRANT_API_KEY = os.environ['QDRANT_API_KEY']
    mock_config.OPENAI_API_KEY = os.environ['OPENAI_API_KEY']
    mock_config.CHUNK_SIZE = 500
    mock_config.OVERLAP_SIZE = 50
    mock_config.COLLECTION_NAME = "test_collection"

    # Patch the config module
    config.Config = mock_config