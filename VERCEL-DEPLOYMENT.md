# Vercel Single Deployment Guide for Humanoid Robot Book

This guide provides step-by-step instructions for deploying the Humanoid Robot Book application with both frontend and backend components to Vercel using a single URL.

## Architecture Overview

The application consists of:
- **Frontend**: Docusaurus static site (React-based)
- **Backend**: FastAPI server with RAG capabilities
- **Database**: Neon Postgres for chat history
- **Vector DB**: Qdrant for document embeddings

## Unified Deployment Architecture

The unified deployment uses a single vercel.json configuration that routes API requests to the FastAPI backend while serving the Docusaurus frontend for all other requests. This creates a single URL endpoint for the entire application.

## Updated vercel.json Configuration

```json
{
  "version": 2,
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distPath": "build"
      }
    },
    {
      "src": "backend/main.py",
      "use": "@vercel/python",
      "config": {
        "runtime": "python3.11"
      }
    }
  ],
  "routes": [
    {
      "src": "/chat/(.*)",
      "dest": "/backend/main.py"
    },
    {
      "src": "/crawl",
      "dest": "/backend/main.py"
    },
    {
      "src": "/query",
      "dest": "/backend/main.py"
    },
    {
      "src": "/health",
      "dest": "/backend/main.py"
    },
    {
      "src": "/metrics",
      "dest": "/backend/main.py"
    },
    {
      "src": "/monitoring/status",
      "dest": "/backend/main.py"
    },
    {
      "src": "/stats",
      "dest": "/backend/main.py"
    },
    {
      "src": "/(.*)",
      "dest": "/index.html"
    }
  ]
}
```

## Environment Variables

### Backend (FastAPI) - Required
- `DEPLOYED_BOOK_URL` - URL of the deployed Docusaurus site (e.g., `https://your-username.github.io/humaoid-robotic/`)
- `DATABASE_URL` - Neon Postgres database URL
- `COHERE_API_KEY` - Cohere API key for embeddings
- `QDRANT_URL` - Qdrant vector database URL
- `QDRANT_API_KEY` - Qdrant API key (if required)
- `OPENROUTER_API_KEY` - OpenRouter API key (if using OpenRouter)

### Frontend (Docusaurus) - Required
- `REACT_APP_BACKEND_URL` - URL of the deployed application (same as the frontend URL)

## Deployment Steps

### Step 1: Prepare Environment Variables

1. Set the following environment variables in your Vercel project settings:
   - `DEPLOYED_BOOK_URL` - The URL where your Docusaurus site will be deployed
   - `DATABASE_URL` - Your Neon Postgres database connection string
   - `COHERE_API_KEY` - Your Cohere API key
   - `QDRANT_URL` - Your Qdrant vector database URL
   - `QDRANT_API_KEY` - Your Qdrant API key
   - `OPENROUTER_API_KEY` - Your OpenRouter API key (if using OpenRouter)

2. For the frontend, you need to set `REACT_APP_BACKEND_URL` to the same URL as your deployment (since it's a unified deployment).

### Step 2: Deploy to Vercel

1. Make sure you have the Vercel CLI installed:
   ```bash
   npm i -g vercel
   ```

2. Deploy the project from the root directory:
   ```bash
   vercel --prod
   ```

3. During the deployment process, Vercel will:
   - Build the Docusaurus frontend using the `docusaurus build` command
   - Deploy the FastAPI backend as a Python serverless function
   - Set up routing based on the vercel.json configuration

### Step 3: Post-Deployment Configuration

1. After deployment, update your Docusaurus config if needed:
   - The `REACT_APP_BACKEND_URL` will be automatically available as the root URL of your deployment
   - The chat widget will connect to the backend using the same domain

2. Verify the deployment by checking:
   - The frontend loads at the root URL
   - The backend endpoints are accessible (e.g., `/health`, `/chat/query`)
   - The chat widget connects properly to the backend

## API Endpoints

After deployment, the following endpoints will be available under the same domain:
- `GET /` - Docusaurus frontend
- `GET /health` - Backend health check
- `POST /chat/query` - Query the RAG system
- `POST /chat` - Chat endpoint with history
- `POST /chat/history` - Save chat history
- `GET /chat/history/{session_id}` - Get chat history
- `GET /metrics` - Monitoring metrics
- `GET /monitoring/status` - Monitoring status
- `GET /stats` - Statistics about indexed content
- `GET /crawl` - Crawl and index website content (requires initial setup)

## Frontend Configuration

The Docusaurus frontend is configured to use the backend URL from environment variables:

```javascript
// In docusaurus.config.js
customFields: {
  backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
},
```

In the unified deployment, `REACT_APP_BACKEND_URL` will be set to the same domain as the frontend, so API calls will be made to the same origin.

## Troubleshooting

1. **API Calls Not Working**: Make sure your API routes in vercel.json are listed before the catch-all route for the frontend.

2. **CORS Issues**: With a unified deployment, CORS issues should be eliminated since both frontend and backend are on the same origin.

3. **Environment Variables**: Verify all required environment variables are set in the Vercel dashboard.

4. **Build Errors**: Make sure all dependencies are properly specified in both package.json and backend requirements.txt.

## Security Notes

- The unified deployment improves security by eliminating cross-origin requests
- API keys are stored as environment variables in Vercel and are not exposed to the client
- The backend is only accessible through the defined API routes

## Performance Considerations

- The Docusaurus frontend is served as static files for optimal performance
- The FastAPI backend runs as serverless functions, scaling automatically with demand
- API responses benefit from Vercel's global edge network

## Updating the Deployment

To update your deployment after making changes:
1. Push changes to your Git repository (if using Git integration)
2. Or redeploy using the CLI: `vercel --prod`

The updated vercel.json configuration ensures that both frontend and backend changes are properly handled in a single deployment.