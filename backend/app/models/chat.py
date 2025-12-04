"""
API Request/Response Models

Pydantic schemas for chat API endpoints matching OpenAPI specification.
"""

from typing import Optional, List
from pydantic import BaseModel, Field
from app.models.textbook import Citation


class QuestionRequest(BaseModel):
    """Request model for asking questions."""
    
    session_id: str = Field(..., min_length=1, max_length=100, description="Session identifier")
    question_text: str = Field(..., min_length=10, max_length=1000, description="User question")
    context_mode: str = Field(default="full", pattern="^(full|selected)$", description="Search mode")
    selected_text: Optional[str] = Field(None, min_length=50, max_length=5000, description="Selected textbook content")
    selected_metadata: Optional[dict] = Field(None, description="Metadata for selected text")
    
    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123",
                "question_text": "What are the key components of ROS2?",
                "context_mode": "full",
                "selected_text": None,
                "selected_metadata": None
            }
        }


class AnswerResponse(BaseModel):
    """Response model for answers."""
    
    answer: str = Field(..., description="Generated answer (max 500 words)")
    citations: List[Citation] = Field(default_factory=list, description="1-5 citation references")
    confidence_score: float = Field(..., ge=0.0, le=1.0, description="Answer confidence")
    processing_time_ms: int = Field(..., ge=0, description="Processing time in milliseconds")
    context_used: str = Field(..., pattern="^(full|selected)$", description="Context mode used")
    tokens_used: int = Field(default=0, ge=0, description="Total tokens consumed")
    
    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS2 has several key components including nodes, topics, services, and actions...",
                "citations": [
                    {
                        "text": "Chapter 3, Section: ROS2 Architecture",
                        "url": "/week-03-05/ros2-architecture#key-components",
                        "relevance_score": 0.95
                    }
                ],
                "confidence_score": 0.92,
                "processing_time_ms": 1250,
                "context_used": "full",
                "tokens_used": 450
            }
        }


class HistoryResponse(BaseModel):
    """Response model for conversation history."""
    
    session_id: str
    messages: List[dict] = Field(default_factory=list, description="Ordered conversation messages")
    total_questions: int = Field(..., ge=0)
    context_mode: str
    
    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123",
                "messages": [
                    {
                        "role": "user",
                        "content": "What are the key components of ROS2?",
                        "timestamp": "2025-12-03T10:30:00Z"
                    },
                    {
                        "role": "assistant",
                        "content": "ROS2 has several key components...",
                        "timestamp": "2025-12-03T10:30:02Z"
                    }
                ],
                "total_questions": 1,
                "context_mode": "full"
            }
        }


class HealthResponse(BaseModel):
    """Response model for health check."""
    
    status: str = Field(..., pattern="^(healthy|degraded|unhealthy)$")
    services: dict = Field(default_factory=dict, description="Service health status")
    timestamp: str
    
    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "services": {
                    "gemini": "healthy",
                    "qdrant": "healthy",
                    "postgres": "healthy"
                },
                "timestamp": "2025-12-03T10:30:00Z"
            }
        }


class ErrorResponse(BaseModel):
    """Error response model."""
    
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    retry_after: Optional[int] = Field(None, description="Retry after seconds (for rate limits)")
    
    class Config:
        json_schema_extra = {
            "example": {
                "error": "Rate limit exceeded",
                "detail": "Maximum 10 requests per minute exceeded",
                "retry_after": 30
            }
        }
