# RAG Chatbot for Docusaurus Book

A custom RAG (Retrieval-Augmented Generation) chatbot that enables book readers to get content-specific answers from a deployed Docusaurus website.

## Features

- Crawls Docusaurus websites to extract content
- Uses Cohere embeddings for semantic search
- Stores embeddings in Qdrant vector database
- Provides chat interface using Google Gemini
- FastAPI backend with Vercel deployment ready

## Prerequisites

- Python 3.9+
- UV package manager
- Access to Cohere API (free tier available)
- Qdrant Cloud account
- Google AI API key for Gemini

## Setup

1. Clone the repository and navigate to the backend directory
2. Install dependencies using UV:
   ```bash
   uv sync
   ```

3. Copy the environment file and add your API keys:
   ```bash
   cp .env.example .env
   ```
   Edit `.env` and add your API keys:
   - `DEPLOYED_BOOK_URL`: The URL of the Docusaurus website to index
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `GEMINI_API_KEY`: Your Google AI API key for Gemini
   - `QDRANT_CLUSTER_ID`: (Optional) Your Qdrant cluster ID if using managed cluster

## Usage

1. Start the development server:
   ```bash
   uv run python main.py
   ```

2. Index your Docusaurus website:
   - Make a GET request to `/crawl` to crawl and index the website content
   - This will extract all pages from the sitemap and store embeddings

3. Query the chatbot:
   - Send POST requests to `/query` with a JSON payload:
     ```json
     {
       "query": "Your question here",
       "top_k": 5
     }
     ```

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `GET /crawl` - Crawl and index the Docusaurus website
- `POST /query` - Query the RAG system
- `GET /stats` - Get indexing statistics

## Environment Variables

Create a `.env` file with the following variables:

```
DEPLOYED_BOOK_URL=
COHERE_API_KEY=
QDRANT_URL=
QDRANT_API_KEY=
QDRANT_CLUSTER_ID=
GEMINI_API_KEY=
```

## Deployment to Vercel

1. Install the Vercel CLI: `npm install -g vercel`
2. Run `vercel` in the backend directory
3. Follow the prompts to deploy

Make sure to set your environment variables in the Vercel dashboard.

## Architecture

The system consists of the following components:

- **Crawler**: Extracts content from the Docusaurus website using sitemap.xml
- **Embedder**: Generates embeddings using Cohere API
- **Vector Store**: Stores embeddings in Qdrant for fast similarity search
- **Agent**: Uses Google Gemini to generate responses based on retrieved context
- **API**: FastAPI interface for interacting with the system

## Testing

Run the tests using pytest:
```bash
uv run pytest
```

## License

[Specify your license here]