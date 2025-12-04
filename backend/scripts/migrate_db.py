"""
Database Migration Script

Creates Neon Postgres schema for RAG chatbot.
Run this script before starting the application.

Usage:
    python backend/scripts/migrate_db.py
"""

import asyncio
import asyncpg
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings
from app.utils.logger import app_logger as logger


async def create_schema():
    """Create database schema."""
    
    try:
        # Connect to database
        logger.info("Connecting to Neon Postgres...")
        conn = await asyncpg.connect(settings.neon_database_url)
        
        # Create conversations table
        logger.info("Creating conversations table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS conversations (
                id SERIAL PRIMARY KEY,
                session_id VARCHAR(100) NOT NULL UNIQUE,
                user_ip VARCHAR(45) NOT NULL,
                context_mode VARCHAR(20) DEFAULT 'full',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Create index on session_id
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_conversations_session_id 
            ON conversations(session_id)
        """)
        
        # Create questions table
        logger.info("Creating questions table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS questions (
                id SERIAL PRIMARY KEY,
                conversation_id INTEGER NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
                question_text TEXT NOT NULL,
                selected_context JSONB,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Create index on conversation_id
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_questions_conversation_id 
            ON questions(conversation_id)
        """)
        
        # Create answers table
        logger.info("Creating answers table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS answers (
                id SERIAL PRIMARY KEY,
                question_id INTEGER NOT NULL REFERENCES questions(id) ON DELETE CASCADE,
                answer_text TEXT NOT NULL,
                citations JSONB DEFAULT '[]',
                confidence_score FLOAT DEFAULT 0.0,
                processing_time_ms INTEGER DEFAULT 0,
                tokens_used INTEGER DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Create index on question_id
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_answers_question_id 
            ON answers(question_id)
        """)
        
        # Create index on created_at for cleanup queries
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_conversations_created_at 
            ON conversations(created_at)
        """)
        
        logger.info("✅ Database schema created successfully!")
        
        # Verify tables
        tables = await conn.fetch("""
            SELECT table_name 
            FROM information_schema.tables 
            WHERE table_schema = 'public' 
            AND table_type = 'BASE TABLE'
        """)
        
        logger.info(f"Created tables: {[t['table_name'] for t in tables]}")
        
        await conn.close()
        
    except Exception as e:
        logger.error(f"❌ Migration failed: {str(e)}")
        raise


if __name__ == "__main__":
    asyncio.run(create_schema())
