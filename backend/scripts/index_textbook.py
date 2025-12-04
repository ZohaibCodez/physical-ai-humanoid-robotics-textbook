"""
Textbook Indexing Script

CLI script to index all textbook content into Qdrant vector database.

Usage:
    python backend/scripts/index_textbook.py
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.indexer import indexer_service
from app.services.vector_store import vector_store_service
from app.utils.logger import app_logger as logger


async def main():
    """Index textbook content and store in Qdrant."""
    
    try:
        logger.info("🚀 Starting textbook indexing...")
        
        # Create Qdrant collection
        logger.info("Creating Qdrant collection...")
        await vector_store_service.create_collection()
        
        # Index all markdown files
        logger.info("Indexing textbook content...")
        chunks = await indexer_service.index_directory()
        
        if not chunks:
            logger.warning("⚠️ No chunks were created. Check docs directory.")
            return
        
        # Store chunks in Qdrant
        logger.info(f"Storing {len(chunks)} chunks in Qdrant...")
        await vector_store_service.upsert_chunks(chunks)
        
        # Get collection info
        info = await vector_store_service.get_collection_info()
        logger.info(f"✅ Indexing complete!")
        logger.info(f"   Collection: {vector_store_service.collection_name}")
        logger.info(f"   Total points: {info.get('points_count', 0)}")
        logger.info(f"   Status: {info.get('status', 'unknown')}")
        
    except Exception as e:
        logger.error(f"❌ Indexing failed: {str(e)}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
