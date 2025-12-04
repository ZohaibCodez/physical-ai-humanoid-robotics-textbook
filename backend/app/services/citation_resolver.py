"""
Citation Resolver Service

Generates Docusaurus-compatible URLs from textbook chunk metadata.
"""

import re
from typing import Dict, Optional
from app.utils.logger import app_logger as logger


class CitationResolverService:
    """Service for resolving citations to Docusaurus URLs."""
    
    def __init__(self, base_url: str = ""):
        """
        Initialize citation resolver.
        
        Args:
            base_url: Base URL for the site (empty for relative URLs)
        """
        self.base_url = base_url.rstrip('/')
        logger.info("✅ Citation Resolver initialized")
    
    def slugify(self, text: str) -> str:
        """
        Convert text to URL-friendly slug.
        
        Args:
            text: Text to slugify
        
        Returns:
            Lowercase slug with hyphens
        """
        # Convert to lowercase and replace spaces/special chars with hyphens
        slug = re.sub(r'[^\w\s-]', '', text.lower())
        slug = re.sub(r'[-\s]+', '-', slug)
        return slug.strip('-')
    
    def generate_url(
        self,
        file_path: str,
        heading: Optional[str] = None,
        section: Optional[str] = None
    ) -> str:
        """
        Generate Docusaurus URL from chunk metadata.
        
        Args:
            file_path: File path (e.g., docs/week-03-05/ros2-architecture.md)
            heading: Heading text for anchor link
            section: Section name
        
        Returns:
            Docusaurus URL (e.g., /week-03-05/ros2-architecture#key-components)
        """
        try:
            # Extract path components
            # docs/week-03-05/ros2-architecture.md -> /week-03-05/ros2-architecture
            path_parts = file_path.replace('\\', '/').split('/')
            
            # Find week directory and filename
            week_dir = None
            filename = None
            
            for i, part in enumerate(path_parts):
                if part.startswith('week-'):
                    week_dir = part
                    if i + 1 < len(path_parts):
                        filename = path_parts[i + 1].replace('.md', '')
                    break
            
            if not week_dir or not filename:
                logger.warning(f"Could not parse file path: {file_path}")
                return "/"
            
            # Build base URL
            url = f"{self.base_url}/{week_dir}/{filename}"
            
            # Add heading anchor if provided
            if heading:
                anchor = self.slugify(heading)
                url += f"#{anchor}"
            
            return url
            
        except Exception as e:
            logger.error(f"Failed to generate URL for {file_path}: {str(e)}")
            return "/"
    
    def format_citation_text(
        self,
        chapter: int,
        section: str,
        heading: Optional[str] = None
    ) -> str:
        """
        Format citation display text.
        
        Args:
            chapter: Chapter number
            section: Section name
            heading: Optional heading name
        
        Returns:
            Formatted citation text (e.g., "Week 3-5, Section: ROS2 Architecture")
        """
        if chapter <= 2:
            week_range = "1-2"
        elif chapter <= 5:
            week_range = "3-5"
        elif chapter <= 7:
            week_range = "6-7"
        elif chapter <= 10:
            week_range = "8-10"
        elif chapter <= 12:
            week_range = "11-12"
        else:
            week_range = "13"
        
        text = f"Week {week_range}, Section: {section}"
        
        if heading:
            text += f" - {heading}"
        
        return text
    
    def create_citation(
        self,
        chunk: Dict,
        relevance_score: float,
        snippet_length: int = 150
    ) -> Dict:
        """
        Create a complete citation object from chunk data.
        
        Args:
            chunk: Chunk dictionary with metadata
            relevance_score: Relevance score (0.0-1.0)
            snippet_length: Length of text snippet
        
        Returns:
            Citation dictionary
        """
        url = self.generate_url(
            file_path=chunk.get("file_path", ""),
            heading=chunk.get("heading"),
            section=chunk.get("section")
        )
        
        text = self.format_citation_text(
            chapter=chunk.get("chapter", 1),
            section=chunk.get("section", "Unknown"),
            heading=chunk.get("heading")
        )
        
        # Create snippet
        snippet = chunk.get("text", "")[:snippet_length]
        if len(chunk.get("text", "")) > snippet_length:
            snippet += "..."
        
        return {
            "text": text,
            "url": url,
            "relevance_score": relevance_score,
            "snippet": snippet
        }


# Global service instance
citation_resolver = CitationResolverService()
