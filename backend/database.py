from sqlmodel import create_engine, Session, SQLModel
from sqlalchemy.pool import QueuePool
from typing import Generator
import os
from dotenv import load_dotenv

# Import models so that SQLModel knows about them
from models.chat_history import ChatHistory, ChatSession

# Load environment variables
load_dotenv()

# Get database URL from environment
DATABASE_URL = os.getenv("NEON_DATABASE_URL", "")

# Create engine with connection pooling
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections after 5 minutes
)

def get_session() -> Generator[Session, None, None]:
    """Get database session for dependency injection."""
    with Session(engine) as session:
        yield session

def create_db_and_tables():
    """Create database tables (basic migration framework)."""
    SQLModel.metadata.create_all(engine)

# Export for dependency injection
__all__ = ["engine", "get_session", "create_db_and_tables"]


def initialize_database():
    """Initialize database with required setup."""
    create_db_and_tables()