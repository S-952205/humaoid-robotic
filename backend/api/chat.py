from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, validator
from typing import List, Dict, Optional
import logging
import asyncio
import re
from sqlmodel import Session

# Import from the existing backend structure
from database import get_session
from services.chat_history_service import ChatHistoryService
from agent import answer_query

logger = logging.getLogger(__name__)

router = APIRouter()

class ChatRequest(BaseModel):
    query: str
    session_id: str
    top_k: int = 5
    selected_text: str = None

    @validator('query')
    def validate_query(cls, v):
        if not v or not v.strip():
            raise ValueError('Query cannot be empty')
        if len(v) > 10000:
            raise ValueError('Query is too long (max 10000 characters)')
        # Basic sanitization to prevent simple injection attempts
        if re.search(r'[<>"\']', v):
            # Sanitize the input to prevent basic XSS attempts
            v = re.sub(r'[<>"\']', '', v)
        return v.strip()

    @validator('session_id')
    def validate_session_id(cls, v):
        if not v or len(v) < 5:
            raise ValueError('Valid session_id is required')
        if len(v) > 100:
            raise ValueError('Session ID is too long (max 100 characters)')
        # Validate session_id format to prevent injection
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError('Session ID contains invalid characters')
        return v

class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: Optional[List[Dict]] = []

class ChatHistoryRequest(BaseModel):
    session_id: str
    role: str  # "user" or "assistant"
    content: str

    @validator('session_id')
    def validate_session_id(cls, v):
        if not v or len(v) < 5:
            raise ValueError('Valid session_id is required')
        if len(v) > 100:
            raise ValueError('Session ID is too long (max 100 characters)')
        # Validate session_id format to prevent injection
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError('Session ID contains invalid characters')
        return v

    @validator('role')
    def validate_role(cls, v):
        if v not in ["user", "assistant"]:
            raise ValueError('Role must be "user" or "assistant"')
        return v

    @validator('content')
    def validate_content(cls, v):
        if not v or not v.strip():
            raise ValueError('Message content cannot be empty')
        if len(v) > 10000:
            raise ValueError('Message content is too long (max 10000 characters)')
        # Basic sanitization to prevent simple injection attempts
        if re.search(r'[<>"\']', v):
            # Sanitize the input to prevent basic XSS attempts
            v = re.sub(r'[<>"\']', '', v)
        return v.strip()

class ChatHistoryResponse(BaseModel):
    success: bool
    message_id: str
    timestamp: str

class GetHistoryResponse(BaseModel):
    session_id: str
    messages: List[Dict]
    total_count: int

@router.post("/", response_model=Dict)
async def chat_endpoint(request: ChatRequest, session: Session = Depends(get_session)):
    """
    Enhanced chat endpoint that handles basic chat requests with session_id.
    This endpoint maintains compatibility with existing Qdrant RAG functionality
    and now includes chat history persistence.
    """
    logger.info(f"Received chat request for session {request.session_id}")
    try:
        # Input validation is now handled by Pydantic models

        # Create chat history service instance
        history_service = ChatHistoryService(session)

        # Create the session (or update if it exists)
        history_service.create_session(request.session_id)

        # Retrieve chat history to provide context
        history = history_service.get_history(request.session_id, limit=10)  # Get last 10 messages
        history_messages = history_service.format_history_for_response(history)

        # Save user message to history
        history_service.save_message(
            session_id=request.session_id,
            role="user",
            content=request.query
        )

        # Use the existing answer_query function which already handles RAG
        # We'll pass the session_id to maintain conversation context and selected_text if available
        response = answer_query(
            query=request.query,
            top_k=request.top_k,
            session_id=request.session_id,
            selected_text=request.selected_text
        )

        # Save assistant response to history
        history_service.save_message(
            session_id=request.session_id,
            role="assistant",
            content=response
        )

        # Return response in the expected format
        return {
            "response": response,
            "session_id": request.session_id,
            "sources": [],  # Sources are handled internally by the existing system
            "history": history_messages  # Include history in response if needed by frontend
        }
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

# For backward compatibility with existing query endpoint
@router.post("/query", response_model=Dict)
async def query_endpoint(request: ChatRequest, session: Session = Depends(get_session)):
    """
    Query endpoint for RAG functionality with session support.
    Maintains compatibility with existing Qdrant integration.
    """
    logger.info(f"Received query request for session {request.session_id}")
    try:
        # Input validation is now handled by Pydantic models

        # Create chat history service instance
        history_service = ChatHistoryService(session)

        # Create the session (or update if it exists)
        history_service.create_session(request.session_id)

        # Retrieve chat history to provide context
        history = history_service.get_history(request.session_id, limit=10)  # Get last 10 messages
        history_messages = history_service.format_history_for_response(history)

        # Save user message to history
        history_service.save_message(
            session_id=request.session_id,
            role="user",
            content=request.query
        )

        # Get the answer from the agent with timeout handling (mimicking main.py)
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

        # Save assistant response to history
        history_service.save_message(
            session_id=request.session_id,
            role="assistant",
            content=answer
        )

        return {
            "answer": answer,
            "session_id": request.session_id,
            "sources": [],  # Sources are handled internally by the existing system
            "history": history_messages  # Include history in response if needed by frontend
        }
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error in query endpoint: {e}")
        error_str = str(e)
        # Check if it's a quota-related error to return a more specific status code
        if "quota" in error_str.lower() or "429" in error_str or "RESOURCE_EXHAUSTED" in error_str:
            raise HTTPException(status_code=429, detail=f"Rate limit exceeded: {str(e)}")
        else:
            raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.post("/history", response_model=ChatHistoryResponse)
async def save_chat_history(request: ChatHistoryRequest, session: Session = Depends(get_session)):
    """
    Save a chat message to the history database.
    """
    logger.info(f"Saving chat history for session {request.session_id}, role: {request.role}")
    try:
        # Input validation is now handled by Pydantic models

        # Create chat history service instance
        history_service = ChatHistoryService(session)

        # Create the session (or update if it exists)
        history_service.create_session(request.session_id)

        # Save the message
        saved_message = history_service.save_message(
            session_id=request.session_id,
            role=request.role,
            content=request.content
        )

        return {
            "success": True,
            "message_id": str(saved_message.id),
            "timestamp": saved_message.timestamp.isoformat()
        }
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error saving chat history: {e}")
        raise HTTPException(status_code=500, detail=f"Error saving chat history: {str(e)}")

@router.get("/history/{session_id}", response_model=GetHistoryResponse)
async def get_chat_history(
    session_id: str,
    limit: int = 20,
    offset: int = 0,
    session: Session = Depends(get_session)
):
    """
    Retrieve chat history for a specific session.
    """
    logger.info(f"Retrieving chat history for session {session_id}, limit: {limit}, offset: {offset}")
    try:
        # Validate session_id format to prevent injection
        if not re.match(r'^[a-zA-Z0-9_-]+$', session_id):
            raise HTTPException(status_code=400, detail="Invalid session_id format")

        # Validate session_id length
        if len(session_id) < 5 or len(session_id) > 100:
            raise HTTPException(status_code=400, detail="Valid session_id (5-100 characters) is required")

        # Validate limit and offset
        if limit <= 0 or limit > 100:
            raise HTTPException(status_code=400, detail="Limit must be between 1 and 100")

        if offset < 0:
            raise HTTPException(status_code=400, detail="Offset must be non-negative")

        # Create chat history service instance
        history_service = ChatHistoryService(session)

        # Get the history
        history = history_service.get_history(session_id, limit, offset)
        total_count = history_service.get_session_messages_count(session_id)

        # Format the response
        formatted_history = history_service.format_history_for_response(history)

        return {
            "session_id": session_id,
            "messages": formatted_history,
            "total_count": total_count
        }
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error retrieving chat history: {e}")
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")