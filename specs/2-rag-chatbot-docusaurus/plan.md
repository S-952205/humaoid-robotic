# Plan: Custom RAG Chatbot for Deployed Docusaurus Book

## Architecture Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Docusaurus    │    │   Crawler &     │    │  Cohere Embed   │
│   Website       │───▶│   Chunker       │───▶│   Service       │
│   (sitemap.xml) │    │   (main.py)     │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │                          │
                              ▼                          ▼
                       ┌──────────────────┐    ┌─────────────────┐
                       │   FastAPI        │    │  Qdrant Vector  │
                       │   Server         │    │  Database       │
                       │                  │    │                 │
                       └──────────────────┘    └─────────────────┘
                              │                          │
                              ▼                          ▼
                       ┌──────────────────┐    ┌─────────────────┐
                       │   OpenAI Agent   │    │  Chatbot API    │
                       │   SDK            │◀───│  Interface      │
                       │   (agent.py)     │    │                 │
                       └──────────────────┘    └─────────────────┘
```

## Backend Folder Structure

```
backend/
├── .env.example
├── .env
├── pyproject.toml          # UV project file
├── main.py                 # Crawler, chunker, and FastAPI app
├── agent.py                # OpenAI Agent SDK implementation
├── crawler.py              # Sitemap crawling and content extraction
├── embedder.py             # Cohere embedding functions
├── vector_store.py         # Qdrant integration
├── config.py               # Configuration and .env loading
├── requirements.txt        # Dependencies (managed via pyproject.toml)
└── tests/
    ├── test_crawler.py
    ├── test_embedder.py
    ├── test_agent.py
    └── test_e2e.py
```

## Technology Stack Decisions

### 1. UV vs pip
- **Decision**: Use UV for package management
- **Rationale**: UV provides faster dependency resolution and installation compared to pip
- **Trade-offs**: Newer tool, may have less community support; but offers significant performance improvements

### 2. Sitemap.xml vs Full Crawl
- **Decision**: Start with sitemap.xml crawling, with option for full crawl
- **Rationale**: Sitemap.xml provides a structured list of pages, ensuring we crawl only relevant content
- **Trade-offs**: May miss content not listed in sitemap, but avoids crawling irrelevant pages

### 3. Cohere Free Model Selection
- **Decision**: Use Cohere's multilingual-22-12 embedding model (free tier)
- **Rationale**: Excellent performance for text similarity, free tier sufficient for development
- **Trade-offs**: Rate limits on free tier, but suitable for initial implementation

### 4. File Organization
- **Decision**: Separate modules for different concerns (crawler, embedder, vector store, agent)
- **Rationale**: Maintains clean separation of concerns and improves testability
- **Trade-offs**: More files to manage, but better maintainability

## Implementation Phases

### Phase 1: UV Setup
**Objective**: Initialize project with UV and set up basic structure

**Tasks**:
1. Initialize UV project: `uv init`
2. Add dependencies to pyproject.toml:
   - fastapi
   - uvicorn
   - cohere
   - qdrant-client
   - openai
   - beautifulsoup4
   - python-dotenv
   - requests
3. Create basic project structure
4. Set up .env configuration

**Acceptance Criteria**:
- UV project initialized successfully
- Dependencies installed and working
- Basic file structure in place
- .env configuration loaded properly

### Phase 2: Sitemap Crawling
**Objective**: Implement website crawling functionality to extract content

**Tasks**:
1. Implement sitemap.xml parser
2. Create web crawler to extract text from each URL
3. Implement content cleaning and normalization
4. Create content chunking algorithm
5. Add error handling for inaccessible URLs

**Acceptance Criteria**:
- Successfully parse sitemap.xml from DEPLOYED_BOOK_URL
- Extract text content from all listed pages
- Clean and normalize content (remove HTML tags, etc.)
- Chunk content into appropriate sizes (max 500 tokens)
- Handle errors gracefully

### Phase 3: Embedding Pipeline
**Objective**: Implement embedding generation and storage in Qdrant

**Tasks**:
1. Integrate with Cohere API for embeddings
2. Implement embedding generation for content chunks
3. Set up Qdrant client connection
4. Store embeddings in Qdrant with metadata
5. Implement retrieval function for semantic search

**Acceptance Criteria**:
- Generate embeddings for all content chunks
- Successfully store embeddings in Qdrant
- Retrieve relevant chunks based on semantic similarity
- Handle Cohere API rate limits
- Proper error handling for API failures

### Phase 4: Agent Integration
**Objective**: Implement OpenAI Agent SDK for chatbot functionality

**Tasks**:
1. Create OpenAI Agent with custom tools
2. Implement retrieve tool to query Qdrant
3. Create response generation with context
4. Add conversation history management
5. Implement citation of sources

**Acceptance Criteria**:
- Agent can retrieve relevant content from Qdrant
- Agent generates contextual responses using retrieved content
- Responses include citations to original sources
- Conversation history maintained properly
- Agent handles various query types effectively

### Phase 5: FastAPI Deployment
**Objective**: Create API endpoints and prepare for Vercel deployment

**Tasks**:
1. Create FastAPI endpoints for chat functionality
2. Implement API request/response validation
3. Add authentication if needed
4. Prepare for Vercel deployment
5. Add monitoring and logging

**Acceptance Criteria**:
- API endpoints available and functional
- Request/response validation implemented
- Deployable to Vercel
- Proper error handling and logging
- Health check endpoints available

## .env Structure

```
DEPLOYED_BOOK_URL=
COHERE_API_KEY=
QDRANT_URL=
QDRANT_API_KEY=
OPENAI_API_KEY=
```

## Testing Strategy

### Unit Tests
- Test crawler functionality with mock HTTP responses
- Test embedding generation with known inputs
- Test vector storage operations
- Test agent tools independently

### Integration Tests
- Test full pipeline: crawl → embed → store → retrieve
- Test API endpoints with various inputs
- Test error handling across components

### End-to-End Tests
- Test complete user flow: query → retrieval → response
- Validate response quality and accuracy
- Test deployment and scaling

## Risk Analysis

### High Risk
- Cohere API rate limits affecting performance
- Large website causing long initial indexing time

### Medium Risk
- Dynamic content not properly captured by crawler
- Qdrant connection stability issues

### Mitigation Strategies
- Implement caching and rate limiting
- Add incremental indexing capability
- Implement fallback mechanisms

## Deployment Strategy

1. Prepare Vercel configuration files
2. Set up environment variables in Vercel dashboard
3. Deploy backend API
4. Verify functionality post-deployment
5. Set up monitoring and alerting