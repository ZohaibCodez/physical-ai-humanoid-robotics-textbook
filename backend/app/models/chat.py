"""
API Request/Response Models

Pydantic schemas for chat API endpoints matching OpenAPI specification.
"""

from typing import Optional, List
import re
from pydantic import BaseModel, Field, field_validator
from app.models.textbook import Citation
from app.models.context import SelectedContext


class QuestionRequest(BaseModel):
    """Request model for asking questions with input validation and sanitization."""
    
    session_id: str = Field(..., min_length=1, max_length=100, description="Session identifier")
    question_text: str = Field(..., min_length=10, max_length=1000, description="User question")
    context_mode: str = Field(default="full", pattern="^(full|selected)$", description="Search mode")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Selected textbook content (deprecated, use selected_context)")
    selected_metadata: Optional[dict] = Field(None, description="Metadata for selected text (deprecated, use selected_context)")
    selected_context: Optional[SelectedContext] = Field(None, description="Structured selected context for scoped QA")
    
    @field_validator('session_id')
    @classmethod
    def validate_session_id(cls, v: str) -> str:
        """Validate and sanitize session ID format."""
        # Allow only alphanumeric, hyphens, underscores
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError("session_id must contain only alphanumeric characters, hyphens, and underscores")
        return v
    
    @field_validator('question_text')
    @classmethod
    def sanitize_question(cls, v: str) -> str:
        """Sanitize user input to prevent injection attacks."""
        # Trim whitespace
        v = v.strip()
        
        # Ensure minimum length after trimming
        if len(v) < 10:
            raise ValueError("question_text must be at least 10 characters after trimming")
        
        # Remove excessive whitespace
        v = re.sub(r'\s+', ' ', v)
        
        # Block suspicious patterns (SQL injection attempts)
        dangerous_patterns = [
            r'(--|;\s*DROP|;\s*DELETE|;\s*INSERT|;\s*UPDATE|;\s*EXEC)',
            r'(<script|javascript:|onerror=|onload=)',
            r'(\$\{|#\{)',  # Template injection
        ]
        
        for pattern in dangerous_patterns:
            if re.search(pattern, v, re.IGNORECASE):
                raise ValueError("question_text contains potentially dangerous content")
        
        return v
    
    @field_validator('selected_text')
    @classmethod
    def validate_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Validate selected text length and content."""
        if v is not None:
            v = v.strip()
            if len(v) > 5000:
                raise ValueError("selected_text must not exceed 5000 characters")
        return v
    
    @field_validator('selected_context')
    @classmethod
    def validate_selected_mode(cls, v: Optional[SelectedContext], info) -> Optional[SelectedContext]:
        """Ensure selected_context is provided when context_mode is 'selected'."""
        if 'context_mode' in info.data and info.data['context_mode'] == 'selected':
            if not v and not info.data.get('selected_text'):
                raise ValueError("selected_context or selected_text required when context_mode is 'selected'")
        return v
    
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
