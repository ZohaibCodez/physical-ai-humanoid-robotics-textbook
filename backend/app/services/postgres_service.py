"""
Postgres Database Service

Manages conversation history using Neon Postgres with connection pooling.
"""

import asyncpg
from typing import Optional, List, Dict, Any
from datetime import datetime
from app.config import settings
from app.utils.logger import app_logger as logger
from app.utils.exceptions import DatabaseError


class PostgresService:
    """Service for managing Postgres database operations."""

    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None

    async def connect(self):
        """Initialize database connection pool."""
        try:
            self.pool = await asyncpg.create_pool(
                settings.neon_database_url, 
                min_size=2, 
                max_size=10, 
                command_timeout=30,
                timeout=10  # Connection timeout in seconds
            )
            logger.info("✅ Connected to Neon Postgres")
        except Exception as e:
            logger.error(f"❌ Failed to connect to Postgres: {str(e)}")
            raise DatabaseError(f"Connection failed: {str(e)}")

    async def disconnect(self):
        """Close database connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Neon Postgres")

    async def create_or_get_conversation(
        self, session_id: str, user_ip: str, context_mode: str = "full"
    ) -> int:
        """
        Create a new conversation or retrieve existing one.

        Args:
            session_id: Session identifier
            user_ip: User IP address
            context_mode: Context mode (full or selected)

        Returns:
            Conversation ID
        """
        try:
            async with self.pool.acquire() as conn:  # type: ignore
                # Try to get existing conversation
                row = await conn.fetchrow(
                    "SELECT id FROM conversations WHERE session_id = $1", session_id
                )

                if row:
                    # Update timestamp
                    await conn.execute(
                        """UPDATE conversations 
                        SET updated_at = $1, context_mode = $2 
                        WHERE id = $3""",
                        datetime.utcnow(),
                        context_mode,
                        row["id"],
                    )
                    return row["id"]

                # Create new conversation
                row = await conn.fetchrow(
                    """INSERT INTO conversations (session_id, user_ip, context_mode)
                    VALUES ($1, $2, $3)
                    RETURNING id""",
                    session_id,
                    user_ip,
                    context_mode,
                )

                logger.info(
                    f"Created conversation {row['id']} for session {session_id}"
                )
                return row["id"]

        except Exception as e:
            logger.error(f"Error creating/getting conversation: {str(e)}")
            raise DatabaseError(f"Failed to create conversation: {str(e)}")

    async def save_question(
        self,
        conversation_id: int,
        question_text: str,
        selected_context: Optional[Dict] = None,
    ) -> int:
        """
        Save a question to the database.

        Args:
            conversation_id: Conversation ID
            question_text: Question text
            selected_context: Optional selected text context

        Returns:
            Question ID
        """
        try:
            async with self.pool.acquire() as conn:  # type: ignore
                row = await conn.fetchrow(
                    """INSERT INTO questions (conversation_id, question_text, selected_context)
                    VALUES ($1, $2, $3)
                    RETURNING id""",
                    conversation_id,
                    question_text,
                    selected_context,
                )
                return row["id"]
        except Exception as e:
            logger.error(f"Error saving question: {str(e)}")
            raise DatabaseError(f"Failed to save question: {str(e)}")

    async def save_answer(
        self,
        question_id: int,
        answer_text: str,
        citations: List[Dict],
        confidence_score: float,
        processing_time_ms: int,
        tokens_used: int,
    ) -> int:
        """
        Save an answer to the database.

        Args:
            question_id: Question ID
            answer_text: Answer text
            citations: List of citation dictionaries
            confidence_score: Confidence score (0.0-1.0)
            processing_time_ms: Processing time in milliseconds
            tokens_used: Number of tokens used


        Returns:
            Answer ID
        """
        try:
            async with self.pool.acquire() as conn:  # type: ignore
                row = await conn.fetchrow(
                    """INSERT INTO answers 
                    (question_id, answer_text, citations, confidence_score, processing_time_ms, tokens_used)
                    VALUES ($1, $2, $3, $4, $5, $6)
                    RETURNING id""",
                    question_id,
                    answer_text,
                    citations,
                    confidence_score,
                    processing_time_ms,
                    tokens_used,
                )
                return row["id"]
        except Exception as e:
            logger.error(f"Error saving answer: {str(e)}")
            raise DatabaseError(f"Failed to save answer: {str(e)}")

    async def get_conversation_history(
        self, session_id: str, limit: int = 10
    ) -> List[Dict[str, Any]]:
        """
        Retrieve conversation history for a session.

        Args:
            session_id: Session identifier
            limit: Maximum number of turns to retrieve (default: 10)

        Returns:
            List of messages with role, content, and timestamp
        """
        try:
            async with self.pool.acquire() as conn:  # type: ignore
                # Get conversation ID
                conv_row = await conn.fetchrow(
                    "SELECT id FROM conversations WHERE session_id = $1", session_id
                )

                if not conv_row:
                    return []

                # Get last N question-answer pairs
                rows = await conn.fetch(
                    """
                    SELECT 
                        q.question_text,
                        q.created_at as question_time,
                        a.answer_text,
                        a.created_at as answer_time
                    FROM questions q
                    LEFT JOIN answers a ON a.question_id = q.id
                    WHERE q.conversation_id = $1
                    ORDER BY q.created_at DESC
                    LIMIT $2
                    """,
                    conv_row["id"],
                    limit,
                )

                # Format as messages
                messages = []
                for row in reversed(rows):
                    messages.append(
                        {
                            "role": "user",
                            "content": row["question_text"],
                            "timestamp": row["question_time"].isoformat(),
                        }
                    )
                    if row["answer_text"]:
                        messages.append(
                            {
                                "role": "assistant",
                                "content": row["answer_text"],
                                "timestamp": row["answer_time"].isoformat(),
                            }
                        )

                return messages

        except Exception as e:
            logger.error(f"Error retrieving conversation history: {str(e)}")
            raise DatabaseError(f"Failed to retrieve history: {str(e)}")

    async def cleanup_old_conversations(self, days_old: int = 30) -> int:
        """
        Delete conversations older than specified days.

        Args:
            days_old: Number of days to keep (default: 30)

        Returns:
            Number of conversations deleted
        """
        try:
            async with self.pool.acquire() as conn:  # type: ignore
                result = await conn.execute(
                    """DELETE FROM conversations 
                    WHERE created_at < NOW() - INTERVAL '$1 days'""",
                    days_old,
                )
                # Extract count from result string like "DELETE 5"
                count = int(result.split()[-1]) if result else 0
                logger.info(f"Cleaned up {count} old conversations")
                return count
        except Exception as e:
            logger.error(f"Error cleaning up conversations: {str(e)}")
            raise DatabaseError(f"Failed to cleanup: {str(e)}")


# Global service instance
postgres_service = PostgresService()
