"""
Simple test script to validate the ChatKit & Neon Integration implementation.
This script tests the core functionality of the implemented feature.
"""
import asyncio
import uuid
from sqlmodel import create_engine, Session, SQLModel
from api.chat import ChatRequest
from database import DATABASE_URL, engine
from services.chat_history_service import ChatHistoryService
from services.rag_service import RagService

def test_database_connection():
    """Test database connection and model creation."""
    print("Testing database connection...")
    try:
        # Test creating tables
        SQLModel.metadata.create_all(engine)
        print("[OK] Database tables created successfully")

        # Test session creation
        with Session(engine) as session:
            print("[OK] Database session created successfully")
        return True
    except Exception as e:
        print(f"[ERROR] Database connection failed: {e}")
        return False

def test_chat_history_service():
    """Test the chat history service functionality."""
    print("\nTesting chat history service...")
    try:
        with Session(engine) as session:
            history_service = ChatHistoryService(session)

            # Generate a test session ID
            test_session_id = str(uuid.uuid4())

            # Test creating a session
            chat_session = history_service.create_session(test_session_id)
            print(f"[OK] Session created: {chat_session.session_id}")

            # Test saving a message
            message = history_service.save_message(
                session_id=test_session_id,
                role="user",
                content="Test message for validation"
            )
            print(f"[OK] Message saved: {message.id}")

            # Test retrieving history
            history = history_service.get_history(test_session_id)
            print(f"[OK] Retrieved {len(history)} messages from history")

            print("[OK] Chat history service working correctly")
            return True
    except Exception as e:
        print(f"[ERROR] Chat history service failed: {e}")
        return False

def test_rag_service():
    """Test the RAG service functionality."""
    print("\nTesting RAG service...")
    try:
        rag_service = RagService()

        # Test that the service initializes correctly
        print("[OK] RAG service initialized successfully")

        # Verify existing data is intact
        rag_service.ensure_existing_data_intact()
        print("[OK] Existing Qdrant data integrity verified")

        # Test preventing reingestion
        rag_service.prevent_reingestion()
        print("[OK] Reingestion prevention verified")

        print("[OK] RAG service working correctly")
        return True
    except Exception as e:
        print(f"[ERROR] RAG service failed: {e}")
        return False

def run_validation():
    """Run all validation tests."""
    print("Starting ChatKit & Neon Integration validation...\n")

    results = []

    # Test database
    results.append(test_database_connection())

    # Test chat history
    results.append(test_chat_history_service())

    # Test RAG
    results.append(test_rag_service())

    # Summary
    print(f"\nValidation Summary:")
    print(f"Passed: {sum(results)}/{len(results)} tests")

    if all(results):
        print("[SUCCESS] All validations passed! Implementation is working correctly.")
        return True
    else:
        print("[FAILURE] Some validations failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    run_validation()