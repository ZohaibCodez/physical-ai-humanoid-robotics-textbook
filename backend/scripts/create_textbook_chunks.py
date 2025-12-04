"""
Create textbook_chunks table in Neon Postgres

This table stores the actual textbook content alongside Qdrant vectors.
"""

import asyncio
import asyncpg
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings


async def create_textbook_chunks_table():
    """Create the textbook_chunks table."""
    print("🔄 Connecting to Neon Postgres...")
    
    try:
        conn = await asyncpg.connect(settings.neon_database_url, timeout=15)
        print("✅ Connected to Neon Postgres")
        
        # Read SQL file
        sql_file = Path(__file__).parent / "create_textbook_chunks_table.sql"
        with open(sql_file, 'r', encoding='utf-8') as f:
            sql = f.read()
        
        print("🔄 Creating textbook_chunks table...")
        await conn.execute(sql)
        print("✅ textbook_chunks table created successfully!")
        
        # Verify table exists
        result = await conn.fetchrow(
            """
            SELECT COUNT(*) as count 
            FROM information_schema.tables 
            WHERE table_name = 'textbook_chunks'
            """
        )
        
        if result['count'] > 0:
            print("✅ Table verification passed")
            
            # Check if table has any data
            count = await conn.fetchval("SELECT COUNT(*) FROM textbook_chunks")
            print(f"📊 Current textbook_chunks count: {count}")
        else:
            print("❌ Table verification failed")
        
        await conn.close()
        print("✅ Migration complete!")
        
    except asyncpg.PostgresError as e:
        print(f"❌ PostgreSQL error: {str(e)}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print("Creating textbook_chunks table in Neon Postgres")
    print("=" * 60)
    asyncio.run(create_textbook_chunks_table())
