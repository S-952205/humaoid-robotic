"""
Unit tests for crawler functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add backend directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_crawler_initialization():
    """Test that the crawler initializes correctly"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Import after setting environment variables
        from crawler import DocusaurusCrawler

        crawler_instance = DocusaurusCrawler()
        assert crawler_instance.base_url == "https://example.com"

def test_is_same_domain():
    """Test the domain checking functionality"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        from crawler import DocusaurusCrawler

        crawler_instance = DocusaurusCrawler()

        # Same domain should return True
        assert crawler_instance._is_same_domain("https://example.com/page") is True

        # Different domain should return False
        assert crawler_instance._is_same_domain("https://other.com/page") is False

def test_extract_page_content_success():
    """Test extracting content from a page successfully"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'crawler' in sys.modules:
            del sys.modules['crawler']

        from crawler import DocusaurusCrawler

        with patch('requests.Session.get') as mock_get:
            # Mock response
            mock_response = Mock()
            mock_response.content = b"""
            <html>
                <head><title>Test Page</title></head>
                <body>
                    <main>This is the main content</main>
                </body>
            </html>
            """
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            crawler_instance = DocusaurusCrawler()
            title, content = crawler_instance.extract_page_content("https://example.com/test")

            assert title == "Test Page"
            assert "This is the main content" in content

def test_extract_page_content_with_error():
    """Test handling of errors when extracting content"""
    with patch.dict(os.environ, {
        'DEPLOYED_BOOK_URL': 'https://example.com',
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test-qdrant.com',
        'QDRANT_API_KEY': 'test-key',
        'GEMINI_API_KEY': 'test-key'
    }):
        # Need to reload the module to handle the environment variables
        if 'crawler' in sys.modules:
            del sys.modules['crawler']

        from crawler import DocusaurusCrawler

        with patch('requests.Session.get') as mock_get:
            # Mock an exception
            mock_get.side_effect = Exception("Network error")

            crawler_instance = DocusaurusCrawler()
            title, content = crawler_instance.extract_page_content("https://example.com/test")

            assert title == ""
            assert content == ""

if __name__ == "__main__":
    pytest.main()