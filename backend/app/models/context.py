"""
Selected Context Models

Data models for handling user-selected text context in RAG chatbot.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, Dict, Any


class TextRange(BaseModel):
    """Represents the position range of selected text in the document."""
    
    start_offset: int = Field(..., ge=0, description="Starting character offset in document")
    end_offset: int = Field(..., ge=0, description="Ending character offset in document")
    
    @field_validator('end_offset')
    @classmethod
    def validate_range(cls, v: int, info) -> int:
        """Ensure end_offset is greater than start_offset."""
        if 'start_offset' in info.data and v <= info.data['start_offset']:
            raise ValueError("end_offset must be greater than start_offset")
        return v


class ContextMetadata(BaseModel):
    """Metadata about the selected text's source location."""
    
    chapter: Optional[int] = Field(None, ge=1, le=13, description="Textbook chapter number (1-13)")
    section: Optional[str] = Field(None, max_length=200, description="Section name")
    file_path: Optional[str] = Field(None, max_length=500, description="Relative path to markdown file")
    heading: Optional[str] = Field(None, max_length=200, description="Heading text where selection is located")


class SelectedContext(BaseModel):
    """
    Represents user-selected text from the textbook for scoped question-answering.
    
    Used in User Story 2: Ask Question from Selected Text
    """
    
    text: str = Field(
        ...,
        min_length=50,
        max_length=5000,
        description="Selected text content from textbook"
    )
    range: Optional[TextRange] = Field(
        None,
        description="Position of selected text in source document"
    )
    metadata: Optional[ContextMetadata] = Field(
        None,
        description="Source location metadata for filtering and citations"
    )
    
    @field_validator('text')
    @classmethod
    def validate_text(cls, v: str) -> str:
        """Ensure selected text is not empty or only whitespace."""
        if not v or not v.strip():
            raise ValueError("Selected text cannot be empty or only whitespace")
        return v.strip()
    
    class Config:
        json_schema_extra = {
            "example": {
                "text": "TF2 (Transform2) is the ROS 2 library for managing coordinate frames...",
                "range": {
                    "start_offset": 120,
                    "end_offset": 450
                },
                "metadata": {
                    "chapter": 3,
                    "section": "TF2 Transformations",
                    "file_path": "docs/week-03-05/06-tf2-transformations.md",
                    "heading": "Introduction to TF2"
                }
            }
        }
