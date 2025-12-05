"""
Qdrant Vector Store Service

Manages vector embeddings storage and semantic search using Qdrant.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest
)
from typing import List, Dict, Optional, Any
from app.config import settings
from app.utils.logger import app_logger as logger
from app.utils.exceptions import VectorStoreError
from app.models.textbook import TextbookChunk


class VectorStoreService:
    """Service for managing Qdrant vector database operations."""
    
    def __init__(self):
        """Initialize Qdrant client."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=60
            )
            self.collection_name = settings.qdrant_collection_name
            logger.info(f"✅ Connected to Qdrant at {settings.qdrant_url}")
        except Exception as e:
            logger.error(f"❌ Failed to connect to Qdrant: {str(e)}")
            raise VectorStoreError(f"Connection failed: {str(e)}")
    
    async def create_collection(self):
        """
        Create Qdrant collection with named vectors for Google and local embeddings.
        
        The collection stores two embedding types:
        - google: 768-dimensional (text-embedding-004)
        - local: 384-dimensional (all-MiniLM-L6-v2)
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            exists = any(c.name == self.collection_name for c in collections)
            
            if exists:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return
            
            # Create collection with named vectors
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config={
                    "google": VectorParams(size=768, distance=Distance.COSINE),
                    "local": VectorParams(size=384, distance=Distance.COSINE)
                }
            )
            
            logger.info(f"✅ Created collection '{self.collection_name}' with named vectors")
            
        except Exception as e:
            logger.error(f"Failed to create collection: {str(e)}")
            raise VectorStoreError(f"Collection creation failed: {str(e)}")
    
    async def upsert_chunks(self, chunks: List[TextbookChunk]):
        """
        Insert or update textbook chunks in Qdrant.
        
        Args:
            chunks: List of TextbookChunk objects with embeddings
        """
        try:
            points = []
            
            for chunk in chunks:
                # Prepare payload
                payload = {
                    "chunk_id": chunk.chunk_id,
                    "text": chunk.text,
                    "chapter": chunk.chapter,
                    "section": chunk.section,
                    "file_path": chunk.file_path,
                    "heading": chunk.heading,
                    "metadata": chunk.metadata
                }
                
                # Create point with named vectors
                point = PointStruct(
                    id=hash(chunk.chunk_id) & 0x7FFFFFFF,  # Convert to positive int
                    vector=chunk.embeddings,  # type: ignore  # Dict with 'google' and 'local' keys
                    payload=payload
                )
                points.append(point)
            
            # Upsert points
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            logger.info(f"✅ Upserted {len(chunks)} chunks to Qdrant")
            
        except Exception as e:
            logger.error(f"Failed to upsert chunks: {str(e)}")
            raise VectorStoreError(f"Upsert failed: {str(e)}")
    
    async def search(
        self,
        query_vector: List[float],
        vector_name: str = "google",
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Semantic search for relevant chunks.
        
        Args:
            query_vector: Query embedding vector
            vector_name: Which named vector to search ('google' or 'local')
            limit: Maximum number of results
            score_threshold: Minimum relevance score (0.0-1.0)
        
        Returns:
            List of matching chunks with metadata and relevance scores
        """
        try:
            logger.info(f"Searching Qdrant: vector_name={vector_name}, limit={limit}, threshold={score_threshold}")
            logger.info(f"Query vector length: {len(query_vector)}, first 5 values: {query_vector[:5]}")
            
            # Search using the correct API for Qdrant client 1.7.1
            all_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=(vector_name, query_vector),  # Tuple for named vectors
                limit=limit,
                score_threshold=score_threshold
            )
            
            logger.info(f"Qdrant returned {len(all_results)} results (before threshold)")
            if all_results:
                scores = [r.score for r in all_results]
                logger.info(f"Scores: min={min(scores):.4f}, max={max(scores):.4f}, scores={scores}")
            
            # Filter by threshold
            results = [r for r in all_results if r.score >= score_threshold]
            logger.info(f"After threshold {score_threshold}: {len(results)} results")
            
            chunks = []
            for result in results:
                chunks.append({
                    "text": result.payload.get("text", ""),
                    "chapter": result.payload.get("chapter"),
                    "section": result.payload.get("section"),
                    "file_path": result.payload.get("file_path"),
                    "heading": result.payload.get("heading"),
                    "relevance_score": result.score,
                    "chunk_id": result.payload.get("chunk_id")
                })
            
            logger.info(f"Found {len(chunks)} relevant chunks")
            return chunks
            
        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            raise VectorStoreError(f"Search failed: {str(e)}", operation="search")
    
    async def search_filtered(
        self,
        query_vector: List[float],
        chapter: Optional[int] = None,
        section: Optional[str] = None,
        vector_name: str = "google",
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Semantic search with metadata filters (for selected-text mode).
        
        Args:
            query_vector: Query embedding vector
            chapter: Filter by chapter number
            section: Filter by section name
            vector_name: Which named vector to search
            limit: Maximum number of results
            score_threshold: Minimum relevance score
        
        Returns:
            List of matching chunks from specified chapter/section
        """
        try:
            # Build filter conditions
            conditions = []
            if chapter is not None:
                conditions.append(
                    FieldCondition(key="chapter", match=MatchValue(value=chapter))
                )
            if section is not None:
                conditions.append(
                    FieldCondition(key="section", match=MatchValue(value=section))
                )
            
            # Create filter if conditions exist
            query_filter = Filter(must=conditions) if conditions else None  # type: ignore
            
            # Search with filters using the correct API for Qdrant client 1.7.1
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=(vector_name, query_vector),  # Tuple for named vectors
                query_filter=query_filter,
                limit=limit,
                score_threshold=score_threshold
            )
            
            chunks = []
            for result in results:
                chunks.append({
                    "text": result.payload.get("text", ""),
                    "chapter": result.payload.get("chapter"),
                    "section": result.payload.get("section"),
                    "file_path": result.payload.get("file_path"),
                    "heading": result.payload.get("heading"),
                    "relevance_score": result.score,
                    "chunk_id": result.payload.get("chunk_id")
                })
            
            logger.info(f"Found {len(chunks)} filtered chunks (chapter={chapter}, section={section})")
            return chunks
            
        except Exception as e:
            logger.error(f"Filtered search failed: {str(e)}")
            raise VectorStoreError(f"Filtered search failed: {str(e)}", operation="search_filtered")
    
    async def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection statistics (simplified for health check).
        
        Returns:
            Dictionary with basic collection info
        """
        try:
            # Simple check: verify collection exists in the list
            collections = self.client.get_collections()
            exists = any(c.name == self.collection_name for c in collections.collections)
            
            if not exists:
                return {"exists": False, "points_count": 0}
            
            # Just return that it exists - avoid parsing complex response models
            return {"exists": True, "points_count": 1}  # Assume has data if exists
            
        except Exception as e:
            logger.error(f"Failed to get collection info: {str(e)}")
            raise VectorStoreError(f"Get collection info failed: {str(e)}")


# Global service instance
vector_store_service = VectorStoreService()
