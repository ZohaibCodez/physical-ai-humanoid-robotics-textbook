"""
Local Embedding Service (Fallback)

Generates vector embeddings using Sentence Transformers (local model).
Used as fallback when Google embedding API is unavailable or rate limited.
"""

from sentence_transformers import SentenceTransformer
from typing import List
import asyncio
from app.utils.logger import app_logger as logger
from app.utils.exceptions import EmbeddingError


class LocalEmbeddingService:
    """Service for generating embeddings using local Sentence Transformers model."""
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize Sentence Transformers model.
        
        Args:
            model_name: HuggingFace model name (default: all-MiniLM-L6-v2)
        """
        try:
            logger.info(f"Loading local embedding model: {model_name}...")
            self.model = SentenceTransformer(model_name)
            self.model_name = model_name
            logger.info(f"✅ Initialized Local Embedding Service with {model_name}")
        except Exception as e:
            logger.error(f"❌ Failed to load local embedding model: {str(e)}")
            raise EmbeddingError(f"Local model initialization failed: {str(e)}", provider="local")
    
    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Input text to embed
        
        Returns:
            384-dimensional embedding vector
        
        Raises:
            EmbeddingError: If embedding generation fails
        """
        try:
            # Run in thread pool to avoid blocking event loop
            loop = asyncio.get_event_loop()
            embedding = await loop.run_in_executor(
                None,
                lambda: self.model.encode(text, convert_to_numpy=True).tolist()
            )
            
            logger.debug(f"Generated local embedding with dimension {len(embedding)}")
            return embedding
            
        except Exception as e:
            logger.error(f"Failed to generate local embedding: {str(e)}")
            raise EmbeddingError(f"Local embedding failed: {str(e)}", provider="local")
    
    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts (batch processing).
        
        Args:
            texts: List of texts to embed
        
        Returns:
            List of 384-dimensional embedding vectors
        """
        try:
            # Run batch encoding in thread pool
            loop = asyncio.get_event_loop()
            embeddings = await loop.run_in_executor(
                None,
                lambda: self.model.encode(texts, convert_to_numpy=True, batch_size=32).tolist()
            )
            
            logger.info(f"Generated {len(embeddings)} local embeddings")
            return embeddings
            
        except Exception as e:
            logger.error(f"Failed to generate batch embeddings: {str(e)}")
            raise EmbeddingError(f"Batch local embedding failed: {str(e)}", provider="local")
    
    def get_embedding_dimension(self) -> int:
        """Return the dimension of embeddings produced by this model."""
        return 384  # all-MiniLM-L6-v2 produces 384-dim vectors


# Global service instance
local_embedding_service = LocalEmbeddingService()
