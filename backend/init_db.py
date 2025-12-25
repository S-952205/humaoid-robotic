"""
Script to initialize the database tables for the ChatKit & Neon History feature.
This creates the required tables in the Neon Postgres database.
"""
from database import create_db_and_tables, initialize_database

def main():
    print("Initializing database tables...")
    try:
        initialize_database()
        print("[SUCCESS] Database tables created successfully!")
        print("Tables created:")
        print("- chat_history")
        print("- chat_session")
        print("Database initialization completed.")
    except Exception as e:
        print(f"[ERROR] Error initializing database: {e}")
        raise

if __name__ == "__main__":
    main()