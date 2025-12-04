"""
Custom Exception Classes

Defines application-specific exceptions for error handling.
"""

from typing import Optional


class RagChatbotException(Exception):
    """Base exception for RAG chatbot errors."""
    
    def __init__(self, message: str, details: Optional[dict] = None):
        self.message = message
        self.details = details or {}
        super().__init__(self.message)


class RateLimitError(RagChatbotException):
    """Raised when rate limit is exceeded."""
    
    def __init__(self, message: str = "Rate limit exceeded", retry_after: Optional[int] = None):
        super().__init__(message, {"retry_after": retry_after})
        self.retry_after = retry_after


class EmbeddingError(RagChatbotException):
    """Raised when embedding generation fails."""
    
    def __init__(self, message: str = "Failed to generate embeddings", provider: Optional[str] = None):
        super().__init__(message, {"provider": provider})
        self.provider = provider


class VectorStoreError(RagChatbotException):
    """Raised when vector store operations fail."""
    
    def __init__(self, message: str = "Vector store operation failed", operation: Optional[str] = None):
        super().__init__(message, {"operation": operation})
        self.operation = operation


class DatabaseError(RagChatbotException):
    """Raised when database operations fail."""
    
    def __init__(self, message: str = "Database operation failed", query: Optional[str] = None):
        super().__init__(message, {"query": query})
        self.query = query


class AgentError(RagChatbotException):
    """Raised when OpenAI Agents SDK operations fail."""
    
    def __init__(self, message: str = "Agent execution failed", agent_name: Optional[str] = None):
        super().__init__(message, {"agent_name": agent_name})
        self.agent_name = agent_name


class ValidationError(RagChatbotException):
    """Raised when input validation fails."""
    
    def __init__(self, message: str = "Validation failed", field: Optional[str] = None):
        super().__init__(message, {"field": field})
        self.field = field
