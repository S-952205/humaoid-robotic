"""
Unit tests for embedder functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add backend directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_embedder_initialization():
    """Test that the embedder initializes correctly"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        from embedder import CohereEmbedder

        embedder = CohereEmbedder()
        assert embedder.model == "multilingual-22-12"

def test_embed_text():
    """Test embedding a single text"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        with patch('cohere.Client') as mock_cohere_client:
            # Mock the Cohere client and its embed method
            mock_client_instance = Mock()
            mock_client_instance.embed.return_value = Mock()
            mock_client_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]
            mock_cohere_client.return_value = mock_client_instance

            from embedder import CohereEmbedder
            embedder = CohereEmbedder()
            result = embedder.embed_text("test text")

            assert result == [0.1, 0.2, 0.3]
            mock_client_instance.embed.assert_called_once()

def test_create_embeddings():
    """Test creating embeddings for multiple texts"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        with patch('cohere.Client') as mock_cohere_client:
            # Mock the Cohere client and its embed method
            mock_client_instance = Mock()
            mock_client_instance.embed.return_value = Mock()
            mock_client_instance.embed.return_value.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
            mock_cohere_client.return_value = mock_client_instance

            from embedder import CohereEmbedder
            embedder = CohereEmbedder()
            result = embedder.create_embeddings(["text1", "text2"])

            assert result == [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
            mock_client_instance.embed.assert_called_once()

def test_chunk_text():
    """Test the text chunking functionality"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'embedder' in sys.modules:
            del sys.modules['embedder']

        from embedder import CohereEmbedder

        embedder = CohereEmbedder()

        # Test with a simple text
        text = "This is a test sentence. " * 20  # Create a longer text
        chunks = embedder.chunk_text(text, max_tokens=20, overlap=5)

        # Should have multiple chunks
        assert len(chunks) > 1

        # Each chunk should not be empty
        for chunk in chunks:
            assert len(chunk.strip()) > 0

def test_chunk_text_small():
    """Test chunking with text smaller than max tokens"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'embedder' in sys.modules:
            del sys.modules['embedder']

        from embedder import CohereEmbedder

        embedder = CohereEmbedder()

        text = "This is a short text."
        chunks = embedder.chunk_text(text, max_tokens=100, overlap=10)

        # Should have just one chunk
        assert len(chunks) == 1
        assert chunks[0] == text

if __name__ == "__main__":
    pytest.main()