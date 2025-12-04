"""
Textbook Indexer Service

Processes textbook content into chunks and generates embeddings for vector storage.
"""

import os
import re
import hashlib
from pathlib import Path
from typing import List, Dict, Optional
import tiktoken
from app.models.textbook import TextbookChunk
from app.services.embeddings import google_embedding_service
from app.services.embeddings_local import local_embedding_service
from app.utils.logger import app_logger as logger
from app.utils.exceptions import EmbeddingError


class IndexerService:
    """Service for indexing textbook content."""
    
    def __init__(
        self,
        chunk_size: int = 400,
        overlap: int = 50,
        docs_dir: str = "docs"
    ):
        """
        Initialize indexer.
        
        Args:
            chunk_size: Target chunk size in tokens (default: 400)
            overlap: Token overlap between chunks (default: 50)
            docs_dir: Directory containing markdown files
        """
        self.chunk_size = chunk_size
        self.overlap = overlap
        
        # Resolve docs directory relative to repository root
        docs_path = Path(docs_dir)
        if not docs_path.is_absolute():
            # Get repository root (parent of backend directory)
            repo_root = Path(__file__).parent.parent.parent.parent
            docs_path = repo_root / docs_dir
        
        self.docs_dir = docs_path
        self.encoding = tiktoken.get_encoding("cl100k_base")
        logger.info(f"✅ Indexer initialized (chunk_size={chunk_size}, overlap={overlap})")
        logger.info(f"   Docs directory: {self.docs_dir}")
    
    def extract_metadata_from_path(self, file_path: Path) -> Dict:
        """
        Extract chapter, section, and week info from file path.
        
        Example: docs/week-03-05/ros2-architecture.md
        Returns: {chapter: 3, section: "ROS2 Architecture", week_range: "3-5"}
        """
        path_parts = file_path.parts
        
        # Extract week range (e.g., week-03-05)
        week_pattern = re.search(r'week-(\d+)-(\d+)', str(file_path))
        if week_pattern:
            start_week, end_week = week_pattern.groups()
            chapter = int(start_week)
            week_range = f"{start_week}-{end_week}"
        else:
            chapter = 1
            week_range = "unknown"
        
        # Extract section from filename
        filename = file_path.stem
        section = filename.replace('-', ' ').title()
        
        return {
            "chapter": chapter,
            "section": section,
            "week_range": week_range,
            "file_path": str(file_path)
        }
    
    def extract_headings(self, content: str) -> List[Dict]:
        """
        Extract markdown headings and their positions.
        
        Returns:
            List of {level, text, start_pos} dictionaries
        """
        headings = []
        for match in re.finditer(r'^(#{1,6})\s+(.+)$', content, re.MULTILINE):
            headings.append({
                "level": len(match.group(1)),
                "text": match.group(2).strip(),
                "start_pos": match.start()
            })
        return headings
    
    def chunk_text(self, text: str, metadata: Dict) -> List[Dict]:
        """
        Split text into overlapping chunks.
        
        Args:
            text: Text to chunk
            metadata: File metadata
        
        Returns:
            List of chunk dictionaries with text and metadata
        """
        # Tokenize text
        tokens = self.encoding.encode(text)
        chunks = []
        
        # Extract headings for context
        headings = self.extract_headings(text)
        
        # Create overlapping chunks
        start_idx = 0
        chunk_num = 0
        
        while start_idx < len(tokens):
            # Get chunk tokens
            end_idx = min(start_idx + self.chunk_size, len(tokens))
            chunk_tokens = tokens[start_idx:end_idx]
            
            # Decode back to text
            chunk_text = self.encoding.decode(chunk_tokens)
            
            # Find current heading context
            char_pos = len(self.encoding.decode(tokens[:start_idx]))
            current_heading = None
            for heading in reversed(headings):
                if heading["start_pos"] <= char_pos:
                    current_heading = heading["text"]
                    break
            
            # Generate chunk ID
            chunk_id = f"{metadata['week_range']}_{metadata['section'].replace(' ', '-').lower()}_{chunk_num:03d}"
            
            chunks.append({
                "chunk_id": chunk_id,
                "text": chunk_text.strip(),
                "heading": current_heading,
                "token_count": len(chunk_tokens),
                **metadata
            })
            
            chunk_num += 1
            
            # Move to next chunk with overlap
            start_idx = end_idx - self.overlap if end_idx < len(tokens) else end_idx
        
        return chunks
    
    async def process_file(self, file_path: Path) -> List[TextbookChunk]:
        """
        Process a single markdown file into chunks with embeddings.
        
        Args:
            file_path: Path to markdown file
        
        Returns:
            List of TextbookChunk objects with embeddings
        """
        try:
            # Read file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract metadata
            metadata = self.extract_metadata_from_path(file_path)
            
            # Create chunks
            chunk_dicts = self.chunk_text(content, metadata)
            
            logger.info(f"Processing {len(chunk_dicts)} chunks from {file_path.name}")
            
            # Generate embeddings for all chunks
            texts = [chunk["text"] for chunk in chunk_dicts]
            
            try:
                # Try Google embeddings first
                google_embeddings = await google_embedding_service.generate_embeddings_batch(
                    texts, task_type="RETRIEVAL_DOCUMENT"
                )
                logger.info(f"✅ Generated Google embeddings for {file_path.name}")
            except EmbeddingError as e:
                logger.warning(f"Google embeddings failed, using empty vectors: {str(e)}")
                google_embeddings = [[0.0] * 768] * len(texts)
            
            try:
                # Generate local embeddings as fallback
                local_embeddings = await local_embedding_service.generate_embeddings_batch(texts)
                logger.info(f"✅ Generated local embeddings for {file_path.name}")
            except EmbeddingError as e:
                logger.warning(f"Local embeddings failed, using empty vectors: {str(e)}")
                local_embeddings = [[0.0] * 384] * len(texts)
            
            # Create TextbookChunk objects
            chunks = []
            for chunk_dict, google_emb, local_emb in zip(chunk_dicts, google_embeddings, local_embeddings):
                chunk = TextbookChunk(
                    chunk_id=chunk_dict["chunk_id"],
                    text=chunk_dict["text"],
                    chapter=chunk_dict["chapter"],
                    section=chunk_dict["section"],
                    file_path=chunk_dict["file_path"],
                    heading=chunk_dict.get("heading"),
                    embeddings={
                        "google": google_emb,
                        "local": local_emb
                    },
                    metadata={
                        "week_range": chunk_dict["week_range"],
                        "token_count": chunk_dict["token_count"]
                    }
                )
                chunks.append(chunk)
            
            return chunks
            
        except Exception as e:
            logger.error(f"Failed to process file {file_path}: {str(e)}")
            raise
    
    async def index_directory(self, directory: Optional[Path] = None) -> List[TextbookChunk]:
        """
        Index all markdown files in a directory.
        
        Args:
            directory: Directory to index (default: self.docs_dir)
        
        Returns:
            List of all TextbookChunk objects
        """
        target_dir = directory or self.docs_dir
        
        if not target_dir.exists():
            raise FileNotFoundError(f"Directory not found: {target_dir}")
        
        # Find all markdown files
        md_files = list(target_dir.rglob("*.md"))
        logger.info(f"Found {len(md_files)} markdown files to index")
        
        all_chunks = []
        
        for file_path in md_files:
            # Skip certain files
            if any(skip in str(file_path) for skip in ["node_modules", ".docusaurus", "README"]):
                continue
            
            try:
                chunks = await self.process_file(file_path)
                all_chunks.extend(chunks)
                logger.info(f"✅ Indexed {file_path.name}: {len(chunks)} chunks")
            except Exception as e:
                logger.error(f"❌ Failed to index {file_path.name}: {str(e)}")
                continue
        
        logger.info(f"✅ Total chunks indexed: {len(all_chunks)}")
        return all_chunks


# Global service instance
indexer_service = IndexerService()
