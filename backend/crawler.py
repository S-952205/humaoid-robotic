import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse, urlunparse
import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple
from config import Config
import logging

logger = logging.getLogger(__name__)

class DocusaurusCrawler:
    """Crawls a Docusaurus website to extract text content from pages"""

    def __init__(self):
        self.base_url = Config.DEPLOYED_BOOK_URL
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; RAGBot/1.0)'
        })

    def get_sitemap_urls(self) -> List[str]:
        """Parse sitemap.xml to get all page URLs"""
        sitemap_url = urljoin(self.base_url, 'sitemap.xml')

        try:
            response = self.session.get(sitemap_url)
            response.raise_for_status()

            # Parse the sitemap XML
            root = ET.fromstring(response.content)

            # Handle both regular sitemap and sitemap index
            urls = []
            if root.tag.endswith('sitemapindex'):
                # This is a sitemap index, get individual sitemaps
                for sitemap in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}sitemap/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
                    sitemap_loc = sitemap.text
                    urls.extend(self._parse_individual_sitemap(sitemap_loc))
            else:
                # This is a regular sitemap
                urls = self._parse_individual_sitemap(sitemap_url)

            return urls
        except requests.RequestException as e:
            logger.error(f"Failed to fetch sitemap: {e}")
            raise
        except ET.ParseError:
            logger.error("Failed to parse sitemap XML")
            raise

    def _parse_individual_sitemap(self, sitemap_url: str) -> List[str]:
        """Parse an individual sitemap file"""
        try:
            response = self.session.get(sitemap_url)
            response.raise_for_status()

            root = ET.fromstring(response.content)
            urls = []

            for url in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url/{http://www.sitemaps.org/schemas/sitemap/0.9}loc'):
                page_url = url.text
                # Only include URLs from the same domain
                if self._is_same_domain(page_url):
                    urls.append(page_url)

            return urls
        except Exception as e:
            logger.error(f"Failed to parse individual sitemap {sitemap_url}: {e}")
            return []

    def _is_same_domain(self, url: str) -> bool:
        """Check if URL is from the same domain as the base URL or a related domain"""
        base_domain = urlparse(self.base_url).netloc
        url_domain = urlparse(url).netloc

        # Allow same domain
        if base_domain == url_domain:
            return True

        # Special case: allow both the GitHub username domain and the custom domain for this specific site
        # This handles the common GitHub Pages scenario where sitemaps list canonical URLs
        allowed_domains = {
            's-952205.github.io',  # GitHub Pages default URL
            'syed-sufyan.github.io'  # Custom domain
        }

        return base_domain in allowed_domains and url_domain in allowed_domains

    def extract_page_content(self, url: str) -> Tuple[str, str]:
        """Extract title and text content from a single page"""
        try:
            # First, try the URL as provided
            response = self.session.get(url)
            response.raise_for_status()
        except requests.RequestException:
            # If the canonical URL fails, try replacing the domain with the configured base domain
            # This handles cases where sitemaps list canonical URLs that don't resolve
            parsed_url = urlparse(url)
            base_parsed = urlparse(self.base_url)
            fallback_url = urlunparse((
                base_parsed.scheme,
                base_parsed.netloc,
                parsed_url.path,
                parsed_url.params,
                parsed_url.query,
                parsed_url.fragment
            ))

            logger.info(f"Original URL failed, trying fallback: {url} -> {fallback_url}")
            response = self.session.get(fallback_url)
            response.raise_for_status()
            url = fallback_url  # Update the URL for potential use in the function

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or 'Untitled'

            # Extract main content - try common content containers
            content_selectors = [
                'main',  # Most common main content container
                '.main-wrapper',  # Common in Docusaurus
                '.container',  # Generic container
                '.content',  # Content class
                'article',  # Article tag
                '.markdown',  # Markdown content in Docusaurus
                '.theme-doc-markdown',  # Docusaurus specific
                '.doc-content',  # Docusaurus specific
                'body'  # Fallback to body
            ]

            text_content = ""
            for selector in content_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    # Get text but exclude navigation, headers, footers
                    excluded_selectors = [
                        'nav', '.nav', 'header', '.header',
                        'footer', '.footer', '.sidebar',
                        '.table-of-contents', '.pagination'
                    ]

                    for excluded in excluded_selectors:
                        for elem in content_element.select(excluded):
                            elem.decompose()

                    text_content = content_element.get_text(separator=' ', strip=True)
                    if text_content:
                        break

            # If no content found with selectors, get from body
            if not text_content:
                text_content = soup.get_text(separator=' ', strip=True)

            # Clean up excessive whitespace
            import re
            text_content = re.sub(r'\s+', ' ', text_content)

            return title, text_content
        except Exception as e:
            logger.error(f"Failed to extract content from {url}: {e}")
            return "", ""

    def crawl_website(self) -> List[Dict[str, str]]:
        """Crawl the website and extract content from all pages"""
        urls = self.get_sitemap_urls()
        logger.info(f"Found {len(urls)} URLs to crawl")

        pages_content = []
        for i, url in enumerate(urls, 1):
            logger.info(f"Crawling {i}/{len(urls)}: {url}")
            try:
                title, content = self.extract_page_content(url)
                if content:  # Only add if we got content
                    pages_content.append({
                        'url': url,
                        'title': title,
                        'content': content
                    })
                else:
                    logger.warning(f"No content extracted from {url}")
            except Exception as e:
                logger.error(f"Error crawling {url}: {e}")
                continue

        logger.info(f"Successfully crawled {len(pages_content)} pages")
        return pages_content