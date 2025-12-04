"""
Conversation Data Models

Pydantic models for conversations, questions, and answers.
"""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field


class Conversation(BaseModel):
    """Conversation session model."""
    
    id: Optional[int] = None
    session_id: str = Field(..., min_length=1, max_length=100)
    user_ip: str = Field(..., max_length=45)
    context_mode: str = Field(default="full", pattern="^(full|selected)$")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123",
                "user_ip": "192.168.1.1",
                "context_mode": "full"
            }
        }


class Question(BaseModel):
    """Question model."""
    
    id: Optional[int] = None
    conversation_id: int
    question_text: str = Field(..., min_length=10, max_length=1000)
    selected_context: Optional[dict] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        json_schema_extra = {
            "example": {
                "conversation_id": 1,
                "question_text": "What are the key components of ROS2?",
                "selected_context": None
            }
        }


class Answer(BaseModel):
    """Answer model."""
    
    id: Optional[int] = None
    question_id: int
    answer_text: str = Field(..., max_length=5000)
    citations: list = Field(default_factory=list)
    confidence_score: float = Field(default=0.0, ge=0.0, le=1.0)
    processing_time_ms: int = Field(default=0, ge=0)
    tokens_used: int = Field(default=0, ge=0)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        json_schema_extra = {
            "example": {
                "question_id": 1,
                "answer_text": "ROS2 has several key components...",
                "citations": [
                    {
                        "text": "Chapter 3, Section: ROS2 Architecture",
                        "url": "/week-03-05/ros2-architecture#key-components",
                        "relevance_score": 0.95
                    }
                ],
                "confidence_score": 0.92,
                "processing_time_ms": 1250,
                "tokens_used": 450
            }
        }
