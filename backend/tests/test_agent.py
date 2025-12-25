"""
Unit tests for agent functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add backend directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_retrieve_context_tool():
    """Test the context retrieval tool"""
    # Mock vector store for testing
    mock_vector_store = Mock()
    mock_vector_store.search_similar.return_value = [
        {
            "content": "Test content for the query",
            "title": "Test Title",
            "url": "https://example.com/test",
            "score": 0.9
        }
    ]

    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Mock the embedder module in sys.modules before importing agent
        mock_embedder_module = Mock()
        mock_embedder_class = Mock()
        mock_embedder_instance = Mock()
        mock_embedder_instance.embed_text.return_value = [0.1, 0.2, 0.3]
        mock_embedder_class.return_value = mock_embedder_instance
        mock_embedder_module.CohereEmbedder = mock_embedder_class

        with patch.dict('sys.modules', {'embedder': mock_embedder_module}):
            # Need to reload the module to handle the environment variables
            modules_to_reload = ['config', 'embedder', 'agent']
            for mod in modules_to_reload:
                if mod in sys.modules:
                    del sys.modules[mod]

            from agent import set_vector_store

            set_vector_store(mock_vector_store)

            # Import after patching
            from agent import retrieve_context_tool
            results = retrieve_context_tool("test query", top_k=5)

            assert len(results) == 1
            assert results[0]["content"] == "Test content for the query"

def test_gemini_api_config():
    """Test that Gemini API is configured properly"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'agent' in sys.modules:
            del sys.modules['agent']

        # Test that the agent module can be imported without errors
        from agent import answer_query
        # The function should exist
        assert callable(answer_query)

def test_conversation_history_management():
    """Test conversation history management functions"""
    session_id = "test_session"

    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'agent' in sys.modules:
            del sys.modules['agent']

        from agent import (
            get_conversation_history,
            add_to_conversation_history,
            conversation_histories
        )

        # Clear any existing history
        conversation_histories[session_id] = []

        # Add some messages
        add_to_conversation_history(session_id, "user", "Hello")
        add_to_conversation_history(session_id, "assistant", "Hi there!")

        # Get the history
        history = get_conversation_history(session_id)

        assert len(history) == 2
        assert history[0]["role"] == "user"
        assert history[0]["content"] == "Hello"
        assert history[1]["role"] == "assistant"
        assert history[1]["content"] == "Hi there!"

def test_answer_query():
    """Test the answer_query function"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Mock the embedder module in sys.modules before importing agent
        mock_embedder_module = Mock()
        mock_embedder_class = Mock()
        mock_embedder_instance = Mock()
        mock_embedder_instance.embed_text.return_value = [0.1, 0.2, 0.3]
        mock_embedder_class.return_value = mock_embedder_instance
        mock_embedder_module.CohereEmbedder = mock_embedder_class

        with patch.dict('sys.modules', {'embedder': mock_embedder_module}):
            # Need to reload the module to handle the environment variables
            modules_to_reload = ['config', 'embedder', 'agent']
            for mod in modules_to_reload:
                if mod in sys.modules:
                    del sys.modules[mod]

            # Mock vector store
            mock_vector_store = Mock()
            mock_vector_store.search_similar.return_value = [
                {
                    "content": "Test content for the query",
                    "title": "Test Title",
                    "url": "https://example.com/test",
                    "score": 0.9
                }
            ]

            from agent import set_vector_store

            # Set up the mock vector store
            set_vector_store(mock_vector_store)

            # Mock the Google Generative AI module since we're using Gemini now
            with patch('google.generativeai.GenerativeModel') as mock_model_class:
                mock_model_instance = Mock()

                # Create a mock response object that has a 'text' attribute
                mock_response = Mock()
                mock_response.text = "This is a test response from Gemini"
                mock_model_instance.generate_content.return_value = mock_response
                mock_model_class.return_value = mock_model_instance

                with patch('google.generativeai.configure') as mock_configure:
                    # Import after patching
                    from agent import answer_query
                    response = answer_query("test query", top_k=5, session_id="test_session")

                    # Should return a string response
                    assert isinstance(response, str)
                    assert len(response) > 0
                    # Verify that the Gemini API was called
                    mock_configure.assert_called_once()
                    mock_model_class.assert_called_once_with('gemini-pro')

if __name__ == "__main__":
    pytest.main()