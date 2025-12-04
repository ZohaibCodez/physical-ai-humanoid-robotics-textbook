"""
Database Cleanup Script

Removes old conversations and performs maintenance tasks.

Usage:
    python backend/scripts/cleanup_db.py [--days 30]
"""

import asyncio
import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.postgres_service import postgres_service
from app.utils.logger import app_logger as logger


async def main(days_old: int = 30):
    """
    Cleanup old conversation data.
    
    Args:
        days_old: Delete conversations older than this many days
    """
    try:
        logger.info(f"🧹 Starting database cleanup (removing data older than {days_old} days)...")
        
        # Connect to database
        await postgres_service.connect()
        
        # Cleanup old conversations
        deleted_count = await postgres_service.cleanup_old_conversations(days_old)
        
        logger.info(f"✅ Cleanup complete!")
        logger.info(f"   Deleted conversations: {deleted_count}")
        
        # Disconnect
        await postgres_service.disconnect()
        
    except Exception as e:
        logger.error(f"❌ Cleanup failed: {str(e)}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Cleanup old conversation data")
    parser.add_argument(
        "--days",
        type=int,
        default=30,
        help="Delete conversations older than this many days (default: 30)"
    )
    
    args = parser.parse_args()
    asyncio.run(main(args.days))
