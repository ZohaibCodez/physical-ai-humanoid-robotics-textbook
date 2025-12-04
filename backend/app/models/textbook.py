"""
Textbook Data Models

Pydantic models for textbook chunks and citations.
"""

from typing import Optional, Dict, List
from pydantic import BaseModel, Field


class TextbookChunk(BaseModel):
    """Textbook chunk model for vector storage."""
    
    chunk_id: str = Field(..., min_length=1)
    text: str = Field(..., min_length=1)
    chapter: int = Field(..., ge=1, le=13)
    section: str
    file_path: str
    heading: Optional[str] = None
    embeddings: Dict[str, List[float]] = Field(default_factory=dict)
    metadata: Dict = Field(default_factory=dict)
    
    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "week-03-05_ros2-architecture_001",
                "text": "ROS2 (Robot Operating System 2) is a flexible framework...",
                "chapter": 3,
                "section": "ROS2 Architecture",
                "file_path": "docs/week-03-05/ros2-architecture.md",
                "heading": "Key Components",
                "metadata": {
                    "week_range": "3-5",
                    "token_count": 325
                }
            }
        }


class Citation(BaseModel):
    """Citation model for answer references."""
    
    text: str = Field(..., description="Citation display text")
    url: str = Field(..., description="URL to textbook section")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance to question")
    snippet: Optional[str] = Field(None, description="Text preview (100-200 chars)")
    
    class Config:
        json_schema_extra = {
            "example": {
                "text": "Chapter 3, Section: ROS2 Architecture",
                "url": "/week-03-05/ros2-architecture#key-components",
                "relevance_score": 0.95,
                "snippet": "ROS2 has several key components including nodes, topics, services..."
            }
        }


class SelectedContext(BaseModel):
    """Selected text context model."""
    
    text: str = Field(..., min_length=50, max_length=5000)
    start_offset: int = Field(..., ge=0)
    end_offset: int = Field(..., ge=0)
    chapter: int = Field(..., ge=1, le=13)
    section: str
    file_path: str
    
    class Config:
        json_schema_extra = {
            "example": {
                "text": "TF2 (Transform Framework 2) manages coordinate transformations...",
                "start_offset": 1234,
                "end_offset": 2456,
                "chapter": 3,
                "section": "TF2 Transformations",
                "file_path": "docs/week-03-05/tf2-transformations.md"
            }
        }
