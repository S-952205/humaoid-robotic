from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict
import uvicorn
import os
from config import Config
from crawler import DocusaurusCrawler
from embedder import CohereEmbedder
from vector_store import QdrantVectorStore
from agent import set_vector_store, answer_query
import logging
import time
from fastapi import Request
import asyncio
from fastapi.middleware.cors import CORSMiddleware

# Import the new chat API
from api.chat import router as chat_router

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Simple in-memory cache for frequently accessed content
cache = {}
CACHE_TTL = 300  # 5 minutes in seconds

def get_from_cache(key: str):
    """Get value from cache if it exists and hasn't expired"""
    if key in cache:
        value, timestamp = cache[key]
        if time.time() - timestamp < CACHE_TTL:
            return value
        else:
            # Remove expired entry
            del cache[key]
    return None

def set_in_cache(key: str, value):
    """Set value in cache with timestamp"""
    cache[key] = (value, time.time())

# Performance monitoring
def log_execution_time(func_name: str):
    def decorator(func):
        async def wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                return result
            finally:
                execution_time = time.time() - start_time
                logger.info(f"{func_name} executed in {execution_time:.2f} seconds")
        return wrapper
    return decorator

# Initialize FastAPI app
app = FastAPI(
    title="Docusaurus RAG Chatbot API",
    description="Custom RAG Chatbot for Deployed Docusaurus Book",
    version="1.0.0"
)

# Add CORS middleware to allow requests from Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins during development
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, OPTIONS, etc.)
    allow_headers=["*"],  # Allow all headers
    # In production, you should specify exact origins instead of ["*"]
)

# Include the new chat API routes
app.include_router(chat_router, prefix="/chat", tags=["chat"])

# Monitoring and metrics
request_count = 0
error_count = 0

@app.middleware("http")
async def monitoring_middleware(request: Request, call_next):
    global request_count, error_count
    request_count += 1

    start_time = time.time()
    try:
        response = await call_next(request)
        return response
    except Exception as e:
        error_count += 1
        raise
    finally:
        process_time = time.time() - start_time
        response.headers["X-Process-Time"] = str(process_time)
        logger.info(f"Request {request.method} {request.url.path} took {process_time:.2f} seconds")

# Add endpoint for monitoring metrics
@app.get("/metrics")
async def get_metrics():
    """Get monitoring metrics for the application"""
    return {
        "request_count": request_count,
        "error_count": error_count,
        "error_rate": error_count / request_count if request_count > 0 else 0,
        "health": "healthy" if error_count / request_count <= 0.05 else "unhealthy" if request_count > 0 else "unknown"
    }

# Add monitoring and alerting endpoints
@app.get("/monitoring/status")
async def monitoring_status():
    """Get detailed monitoring status"""
    return {
        "status": "operational",
        "timestamp": time.time(),
        "metrics": {
            "request_count": request_count,
            "error_count": error_count,
            "uptime": "unknown"  # In a real system, this would track uptime
        }
    }

# Global instances
crawler = None
embedder = None
vector_store = None

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5
    session_id: str = "default"
    selected_text: str = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict]



@app.on_event("startup")
async def startup_event():
    """Initialize all components on startup"""
    global crawler, embedder, vector_store

    logger.info("Initializing RAG system components...")

    try:
        # Initialize components
        crawler = DocusaurusCrawler()
        embedder = CohereEmbedder()
        vector_store = QdrantVectorStore()

        # Create the collection if it doesn't exist
        vector_store.create_collection()

        # Set the vector store in the agent module
        set_vector_store(vector_store)

        logger.info("RAG system components initialized successfully")
    except Exception as e:
        logger.error(f"Error during startup: {e}")
        # In testing environments, we might not have all services available
        # So we'll log the error but not crash the app
        if os.getenv("TESTING", "false").lower() != "true":
            raise
        else:
            logger.warning("Running in test mode - skipping service initialization")
            # Initialize with None to avoid errors in tests
            crawler = None
            embedder = None
            vector_store = None

@app.get("/")
async def root():
    """Root endpoint to check if the service is running"""
    return {"message": "Docusaurus RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy"}

@app.get("/crawl")
async def crawl_website():
    """Crawl the Docusaurus website and index content"""
    start_time = time.time()
    try:
        if crawler is None or embedder is None or vector_store is None:
            raise HTTPException(status_code=503, detail="Service not initialized - check configuration and external dependencies")

        logger.info("Starting website crawl...")

        # Crawl the website
        pages_content = crawler.crawl_website()

        if not pages_content:
            raise HTTPException(status_code=404, detail="No content found on the website")

        # Process and chunk content
        all_chunks = []
        for page in pages_content:
            chunks = embedder.chunk_text(page['content'])
            for i, chunk in enumerate(chunks):
                chunk_data = {
                    'content': chunk,
                    'title': page['title'],
                    'url': page['url'],
                    'chunk_index': i
                }
                all_chunks.append(chunk_data)

        logger.info(f"Chunked content into {len(all_chunks)} chunks")

        if not all_chunks:
            raise HTTPException(status_code=404, detail="No valid content chunks created")

        # Extract text for embedding
        texts_to_embed = [chunk['content'] for chunk in all_chunks]

        # Generate embeddings
        logger.info("Generating embeddings...")
        embeddings = embedder.create_embeddings(texts_to_embed)

        # Store embeddings in vector store
        logger.info("Storing embeddings in vector store...")
        vector_store.store_embeddings(all_chunks, embeddings)

        return {
            "message": f"Successfully crawled and indexed {len(pages_content)} pages",
            "pages_processed": len(pages_content),
            "chunks_created": len(all_chunks),
            "document_count": vector_store.get_all_documents_count()
        }
    except Exception as e:
        logger.error(f"Error during crawling: {e}")
        raise HTTPException(status_code=500, detail=f"Error during crawling: {str(e)}")
    finally:
        execution_time = time.time() - start_time
        logger.info(f"Crawl operation completed in {execution_time:.2f} seconds")

@app.post("/query")
@log_execution_time("query_endpoint")
async def query_endpoint(request: QueryRequest):
    """Query the RAG system"""
    try:
        if vector_store is None:
            raise HTTPException(status_code=503, detail="Service not initialized - check configuration and external dependencies")

        # Create cache key based on query and session
        cache_key = f"query:{request.query}:{request.session_id}:{request.top_k}"

        # Try to get response from cache first
        cached_response = get_from_cache(cache_key)
        if cached_response:
            logger.info(f"Cache hit for query: {request.query[:50]}...")
            return QueryResponse(
                answer=cached_response,
                sources=[]  # Sources would need to be cached separately if needed
            )

        # Get the answer from the agent with timeout handling
        loop = asyncio.get_event_loop()
        try:
            # Run the synchronous answer_query function with timeout
            answer = await asyncio.wait_for(
                loop.run_in_executor(None, answer_query, request.query, request.top_k, request.session_id, request.selected_text),
                timeout=30.0  # 30 second timeout
            )
        except asyncio.TimeoutError:
            logger.error(f"Query timeout for: {request.query[:50]}...")
            raise HTTPException(status_code=408, detail="Query timed out")

        # Cache the response for future similar queries
        set_in_cache(cache_key, answer)

        # For now, we'll extract sources from the answer text
        # In a more sophisticated implementation, we'd return structured source data
        sources = []

        return QueryResponse(
            answer=answer,
            sources=sources
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error during query: {e}")
        raise HTTPException(status_code=500, detail=f"Error during query: {str(e)}")



@app.get("/stats")
async def get_stats():
    """Get statistics about the indexed content"""
    try:
        if vector_store is None:
            # Return default values when in test mode
            return {
                "indexed_documents": 0,
                "website_url": os.getenv("DEPLOYED_BOOK_URL", "https://example.com")
            }

        count = vector_store.get_all_documents_count()
        return {
            "indexed_documents": count,
            "website_url": Config.DEPLOYED_BOOK_URL
        }
    except Exception as e:
        logger.error(f"Error getting stats: {e}")
        raise HTTPException(status_code=500, detail=f"Error getting stats: {str(e)}")



if __name__ == "__main__":
    # This allows running the app directly with uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)