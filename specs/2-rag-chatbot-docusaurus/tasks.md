# Tasks: Custom RAG Chatbot for Deployed Docusaurus Book

## Feature Overview
A custom Retrieval-Augmented Generation (RAG) chatbot that enables book readers to get content-specific answers from a deployed Docusaurus website. The system crawls the Docusaurus site, extracts text content, embeds it using Cohere's free embedding model, stores it in Qdrant Cloud vector database, and provides an OpenAI Agent SDK-powered chat interface.

## Dependencies
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- OpenAI API for chat functionality
- Target Docusaurus website accessibility

## Implementation Strategy
MVP approach focusing on core functionality: crawling → embedding → storage → retrieval → response generation. Each user story will be implemented as a complete, independently testable increment.

## Phase 1: Setup
**Objective**: Initialize project with UV and set up basic structure

- [X] T001 Initialize UV project in backend directory
- [X] T002 Create basic project directory structure per implementation plan
- [X] T003 [P] Add dependencies to pyproject.toml: fastapi, uvicorn, cohere, qdrant-client, openai, openai-agents
- [X] T004 [P] Add dependencies to pyproject.toml: beautifulsoup4, python-dotenv, requests, pydantic, tiktoken
- [X] T005 Create .env.example file with required environment variables
- [X] T006 Create requirements.txt file from pyproject.toml

## Phase 2: Foundational Components
**Objective**: Create foundational components that block all user stories

- [X] T007 Create config.py to handle environment variables and configuration
- [X] T008 [P] Create crawler.py module with basic sitemap parsing functionality
- [X] T009 [P] Create embedder.py module with Cohere integration
- [X] T010 [P] Create vector_store.py module with Qdrant integration
- [X] T011 Create agent.py module with basic OpenAI Agent SDK setup
- [X] T012 Create main.py with basic FastAPI app structure
- [X] T013 [P] Implement error handling and logging setup across modules

## Phase 3: [US1] Website Crawling and Content Extraction
**User Story**: As a book reader, I want to ask questions about the book content and receive accurate answers based on the deployed Docusaurus site

**Goal**: Implement website crawling functionality to extract content from Docusaurus site

**Independent Test Criteria**: System can crawl a Docusaurus website and extract text content from all pages listed in sitemap.xml

- [X] T014 [P] [US1] Implement sitemap.xml parser in crawler.py
- [X] T015 [P] [US1] Create web crawler to extract text from each URL in crawler.py
- [X] T016 [P] [US1] Implement content cleaning and normalization in crawler.py
- [X] T017 [P] [US1] Create content chunking algorithm in embedder.py
- [X] T018 [P] [US1] Add error handling for inaccessible URLs in crawler.py
- [X] T019 [P] [US1] Add logging to crawling process in crawler.py
- [X] T020 [US1] Create /crawl endpoint in main.py that triggers website crawling
- [X] T021 [US1] Implement crawl statistics reporting in main.py
- [X] T022 [US1] Test crawling functionality with sample Docusaurus site
- [X] T023 [US1] Validate that 95% of pages are successfully crawled

## Phase 4: [US2] Embedding Generation and Storage
**User Story**: As a book reader, I want to get answers that cite specific parts of the book content for verification

**Goal**: Implement embedding generation and storage in Qdrant for semantic search

**Independent Test Criteria**: System can generate embeddings for content chunks and store them in Qdrant with proper metadata

- [X] T024 [P] [US2] Integrate with Cohere API for embeddings in embedder.py
- [X] T025 [P] [US2] Implement embedding generation for content chunks in embedder.py
- [X] T026 [P] [US2] Set up Qdrant client connection in vector_store.py
- [X] T027 [P] [US2] Store embeddings in Qdrant with metadata in vector_store.py
- [X] T028 [P] [US2] Implement retrieval function for semantic search in vector_store.py
- [X] T029 [P] [US2] Add error handling for Cohere API failures in embedder.py
- [X] T030 [P] [US2] Add error handling for Qdrant connection issues in vector_store.py
- [X] T031 [US2] Implement rate limiting for Cohere API calls in embedder.py
- [X] T032 [US2] Create collection in Qdrant with appropriate vector size in vector_store.py
- [X] T033 [US2] Test embedding pipeline with sample content
- [X] T034 [US2] Validate embedding accuracy and retrieval quality

## Phase 5: [US3] Chat Interface and Response Generation
**User Story**: As a book reader, I want the chatbot to understand context in my questions and provide relevant responses

**Goal**: Implement OpenAI Agent SDK for chatbot functionality with context retrieval

**Independent Test Criteria**: System can receive user queries, retrieve relevant context, and generate contextual responses with citations

- [X] T035 [P] [US3] Create OpenAI Agent with custom tools in agent.py
- [X] T036 [P] [US3] Implement retrieve tool to query Qdrant in agent.py
- [X] T037 [P] [US3] Create response generation with context in agent.py
- [X] T038 [P] [US3] Implement citation of sources in agent.py
- [X] T039 [P] [US3] Add conversation history management in agent.py
- [X] T040 [P] [US3] Implement query processing with embedding generation in agent.py
- [X] T041 [P] [US3] Add error handling for OpenAI API calls in agent.py
- [X] T042 [US3] Create /query endpoint in main.py that connects to the agent
- [X] T043 [US3] Implement request/response validation for queries in main.py
- [X] T044 [US3] Add response formatting with source citations in main.py
- [X] T045 [US3] Test chat functionality with sample queries
- [X] T046 [US3] Validate response quality and citation accuracy

## Phase 6: [US4] API Integration and Performance
**User Story**: As a book reader, I want responses within 5 seconds and reliable service

**Goal**: Create complete API with performance optimization and reliability features

**Independent Test Criteria**: System responds to queries within 5 seconds with 99% uptime under normal load

- [X] T047 [P] [US4] Implement API request/response validation in main.py
- [X] T048 [P] [US4] Add performance monitoring to critical endpoints in main.py
- [X] T049 [P] [US4] Implement caching for frequently accessed content in main.py
- [ ] T050 [P] [US4] Add authentication if needed in main.py
- [X] T051 [P] [US4] Add comprehensive logging across all modules
- [X] T052 [P] [US4] Implement health check endpoints in main.py
- [X] T053 [US4] Add monitoring and alerting setup in main.py
- [X] T054 [US4] Implement timeout handling for external API calls
- [X] T055 [US4] Create statistics endpoint to monitor indexed content
- [X] T056 [US4] Performance test the complete pipeline
- [X] T057 [US4] Validate 5-second response time under normal load

## Phase 7: [US5] Deployment and Testing
**User Story**: As a user, I want the system to be reliably deployed and tested

**Goal**: Prepare for Vercel deployment with comprehensive testing

**Independent Test Criteria**: System deploys successfully to Vercel and passes all tests

- [X] T058 [P] [US5] Create Vercel configuration file (vercel.json)
- [X] T059 [P] [US5] Create comprehensive README.md with setup instructions
- [X] T060 [P] [US5] Write unit tests for crawler functionality in tests/test_crawler.py
- [X] T061 [P] [US5] Write unit tests for embedder functionality in tests/test_embedder.py
- [X] T062 [P] [US5] Write unit tests for agent functionality in tests/test_agent.py
- [X] T063 [P] [US5] Write end-to-end tests for complete pipeline in tests/test_e2e.py
- [X] T064 [US5] Run all tests and fix any failures
- [X] T065 [US5] Prepare for Vercel deployment with proper environment setup
- [X] T066 [US5] Deploy to Vercel and verify functionality
- [X] T067 [US5] Conduct final end-to-end testing on deployed system

## Phase 8: Polish & Cross-Cutting Concerns
**Objective**: Final improvements and cross-cutting concerns

- [X] T068 Add comprehensive error messages and user feedback
- [X] T069 Implement graceful degradation when external services are unavailable
- [X] T070 Add documentation for each module
- [X] T071 Review and optimize code for maintainability
- [X] T072 Final testing and validation of all user stories
- [X] T073 Prepare final deployment and documentation

## Dependencies
- User Story 1 (Crawling) must be completed before User Story 2 (Embedding) can begin
- User Story 2 (Embedding) must be completed before User Story 3 (Chat Interface) can begin
- User Stories 4 and 5 can be implemented in parallel after User Story 3 is complete

## Parallel Execution Examples
- Tasks T008-T011 (foundational modules) can be developed in parallel by different developers
- Tasks T014-T018 (crawling components) can be developed in parallel
- Tasks T024-T028 (embedding components) can be developed in parallel
- Tasks T035-T041 (agent components) can be developed in parallel
- Tasks T060-T062 (unit tests) can be developed in parallel