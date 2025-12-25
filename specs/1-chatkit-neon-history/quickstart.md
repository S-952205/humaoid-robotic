# Quickstart: Floating ChatKit & Neon Persistence

## Prerequisites
- Python 3.11+
- Node.js 18+
- Docusaurus project set up
- FastAPI backend running
- Neon Postgres database instance
- OpenAI API key

## Setup Instructions

### 1. Environment Configuration
```bash
# Add to your .env file
DATABASE_URL="postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require"
OPENAI_API_KEY="your-openai-api-key"
QDRANT_URL="your-qdrant-url"
```

### 2. Backend Setup
```bash
# Install backend dependencies
pip install sqlmodel psycopg2-binary

# Set up database tables
python -c "from backend.src.models.chat_history import create_db_and_tables; create_db_and_tables()"
```

### 3. Frontend Setup
```bash
# Install ChatKit in your Docusaurus project
npm install @openai/chatkit

# Add the ChatWidget component to your Docusaurus layout
```

### 4. Database Migration
```bash
# Run the database initialization
python backend/src/models/chat_history.py
```

## Component Integration

### 1. Backend API Endpoints
The following endpoints will be available:
- `POST /chat/history` - Save a chat message
- `GET /chat/history/{session_id}` - Retrieve chat history for a session
- Enhanced `POST /chat` - With history integration

### 2. Frontend Integration
The ChatWidget component will:
- Appear as a floating bubble in the bottom-right corner
- Store session_id in localStorage
- Fetch chat history when opened
- Send messages to backend with session context

## Running the Application

### 1. Start Backend
```bash
cd backend
uvicorn src.api.main:app --reload --port 8000
```

### 2. Start Frontend
```bash
cd frontend  # or your Docusaurus directory
npm run start
```

### 3. Verify Installation
1. Visit your Docusaurus site
2. Look for the floating chat bubble in the bottom-right corner
3. Click to open the chat interface
4. Send a test message and verify it's stored in Neon Postgres

## Configuration Options

### 1. History Depth
Adjust the number of historical messages retrieved by modifying the `limit` parameter in the API call.

### 2. Styling
Customize the chat widget appearance by modifying the CSS in the ChatWidget component.

### 3. Session Management
Configure session persistence behavior in the localStorage settings.

## Troubleshooting

### Common Issues
1. **Database Connection**: Verify DATABASE_URL is correctly set in .env
2. **API Keys**: Ensure all required API keys are properly configured
3. **Component Loading**: Check browser console for any JavaScript errors

### Debugging Steps
1. Check backend logs for API errors
2. Verify database connectivity with `psql` or similar tool
3. Confirm Qdrant connection for RAG functionality
4. Review browser developer tools for frontend issues