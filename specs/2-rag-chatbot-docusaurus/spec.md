# Specification: Custom RAG Chatbot for Deployed Docusaurus Book

## Overview
A custom Retrieval-Augmented Generation (RAG) chatbot that enables book readers to get content-specific answers from a deployed Docusaurus website. The system will crawl the Docusaurus site, extract text content, embed it using Cohere's free embedding model, store it in Qdrant Cloud vector database, and provide an OpenAI Agent SDK-powered chat interface.

## User Stories
- As a book reader, I want to ask questions about the book content and receive accurate answers based on the deployed Docusaurus site
- As a book reader, I want to get answers that cite specific parts of the book content for verification
- As a book reader, I want the chatbot to understand context in my questions and provide relevant responses

## Scope
### In Scope
- Crawling a deployed Docusaurus website to extract all page text content
- Processing and chunking the extracted content into manageable pieces
- Embedding content chunks using Cohere's free embedding model
- Storing embeddings in Qdrant Cloud vector database
- Building an OpenAI Agent SDK-powered chatbot interface
- Supporting selected text queries from the book content
- Configuring the system using environment variables for API keys
- Deploying the backend service to Vercel

### Out of Scope
- Building the frontend chat interface (focus is on backend API)
- Modifying the existing Docusaurus site
- Providing speech-to-text or text-to-speech capabilities
- Advanced user management or authentication

## Functional Requirements
1. **Website Crawling**: The system shall automatically crawl a configured Docusaurus website URL and extract all accessible page text content
2. **Content Processing**: The system shall process the crawled content by removing HTML tags, normalizing text, and splitting into appropriately sized chunks
3. **Embedding Generation**: The system shall generate vector embeddings for each content chunk using Cohere's free embedding model
4. **Vector Storage**: The system shall store the embeddings and associated metadata in Qdrant Cloud vector database
5. **Chat Interface**: The system shall provide a chat interface using OpenAI Agent SDK that can retrieve relevant content based on user queries
6. **Query Processing**: The system shall process user queries by generating embeddings and performing similarity searches in the vector database
7. **Response Generation**: The system shall generate contextually relevant responses based on retrieved content and user queries
8. **Configuration**: The system shall support configuration via environment variables for API keys and service endpoints

## Non-Functional Requirements
1. **Performance**: The system shall respond to user queries within 5 seconds under normal load conditions
2. **Scalability**: The system shall handle up to 100 concurrent users during peak usage
3. **Reliability**: The system shall maintain 99% uptime during business hours
4. **Security**: All API keys and sensitive information shall be stored securely using environment variables
5. **Maintainability**: The codebase shall follow clean code principles with appropriate documentation

## Success Criteria
- Book readers can ask questions about the book content and receive accurate, contextually relevant answers within 5 seconds
- The system successfully indexes 95% of the pages from the target Docusaurus website
- The chatbot provides answers with cited sources from the original book content
- The backend service deploys successfully to Vercel and remains stable during testing
- The system handles API key configurations securely without exposing them in code

## Key Entities
- **Docusaurus Website**: The deployed website containing book content to be indexed
- **Content Chunks**: Segments of text extracted from the website, processed and prepared for embedding
- **Vector Embeddings**: Numerical representations of content chunks suitable for similarity search
- **Qdrant Database**: Vector database storing embeddings and enabling semantic search
- **Chat Sessions**: Conversations between users and the chatbot
- **API Keys**: Credentials for Cohere, Qdrant, and OpenAI services

## Assumptions
- The target Docusaurus website is publicly accessible and has a sitemap.xml
- The website content is primarily text-based with minimal dynamic content
- Users have basic familiarity with chat interfaces
- Stable internet connection is available for API calls to external services

## Constraints
- Must use Cohere's free embedding model to avoid costs
- Must use Qdrant Cloud Free tier for vector storage
- Must use OpenAI Agent SDK for chatbot implementation
- Must support deployment to Vercel
- Must configure all API keys through environment variables
- Backend must be initialized with UV (possibly referring to a project structure convention)

## Dependencies
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- OpenAI API for chat functionality
- Target Docusaurus website accessibility