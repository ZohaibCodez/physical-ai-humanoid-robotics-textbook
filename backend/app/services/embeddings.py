"""
Google Embedding Service

Generates vector embeddings using Google's text-embedding-004 model.
"""

import google.generativeai as genai
from typing import List, Dict
import asyncio
from app.config import settings
from app.utils.logger import app_logger as logger
from app.utils.exceptions import EmbeddingError


class GoogleEmbeddingService:
    """Service for generating embeddings using Google's embedding model."""
    
    def __init__(self):
        """Initialize Google GenAI client."""
        try:
            genai.configure(api_key=settings.google_api_key)
            self.model_name = "models/text-embedding-004"
            logger.info(f"✅ Initialized Google Embedding Service with {self.model_name}")
        except Exception as e:
            logger.error(f"❌ Failed to initialize Google Embedding Service: {str(e)}")
            raise EmbeddingError(f"Initialization failed: {str(e)}", provider="google")
    
    async def generate_embedding(self, text: str, task_type: str = "RETRIEVAL_DOCUMENT") -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Input text to embed
            task_type: Task type hint for the embedding model
                      - RETRIEVAL_DOCUMENT: For indexing documents
                      - RETRIEVAL_QUERY: For search queries
        
        Returns:
            768-dimensional embedding vector
        
        Raises:
            EmbeddingError: If embedding generation fails
        """
        try:
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                lambda: genai.embed_content(
                    model=self.model_name,
                    content=text,
                    task_type=task_type
                )
            )
            
            embedding = result['embedding']
            logger.debug(f"Generated embedding with dimension {len(embedding)}")
            return embedding
            
        except Exception as e:
            logger.error(f"Failed to generate Google embedding: {str(e)}")
            raise EmbeddingError(f"Google embedding failed: {str(e)}", provider="google")
    
    async def generate_embeddings_batch(
        self,
        texts: List[str],
        task_type: str = "RETRIEVAL_DOCUMENT"
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.
        
        Args:
            texts: List of texts to embed
            task_type: Task type hint for the embedding model
        
        Returns:
            List of 768-dimensional embedding vectors
        """
        embeddings = []
        
        # Process in batches to respect rate limits
        batch_size = 5
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            
            try:
                # Generate embeddings for batch
                tasks = [self.generate_embedding(text, task_type) for text in batch]
                batch_embeddings = await asyncio.gather(*tasks)
                embeddings.extend(batch_embeddings)
                
                # Small delay between batches to respect rate limits
                if i + batch_size < len(texts):
                    await asyncio.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Failed to generate batch embeddings: {str(e)}")
                raise EmbeddingError(f"Batch embedding failed: {str(e)}", provider="google")
        
        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings
    
    def get_embedding_dimension(self) -> int:
        """Return the dimension of embeddings produced by this model."""
        return 768


# Global service instance
google_embedding_service = GoogleEmbeddingService()
