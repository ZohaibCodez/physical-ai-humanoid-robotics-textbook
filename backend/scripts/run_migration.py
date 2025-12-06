"""
Database Migration Script - Execute create_auth_tables.sql

Reads SQL migration file and executes against Neon Postgres database.
Loads NEON_DATABASE_URL from .env file.
"""

import asyncio
import os
import asyncpg
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


async def run_migration():
    """Execute authentication tables migration."""
    # Read SQL migration file
    migration_file = Path(__file__).parent / "create_auth_tables.sql"
    
    if not migration_file.exists():
        print(f"❌ Migration file not found: {migration_file}")
        return False
    
    with open(migration_file, "r") as f:
        migration_sql = f.read()
    
    # Get database URL from environment (.env file)
    database_url = os.getenv("NEON_DATABASE_URL")
    if not database_url:
        print("❌ NEON_DATABASE_URL environment variable not set in .env file")
        print("   Copy .env.example to .env and set your database URL")
        return False
    
    try:
        # Connect to database
        print("🔗 Connecting to Neon Postgres...")
        conn = await asyncpg.connect(database_url)
        
        # Execute migration
        print("📝 Executing migration: create_auth_tables.sql")
        await conn.execute(migration_sql)
        
        # Verify tables created
        print("✅ Verifying tables...")
        tables = await conn.fetch("""
            SELECT table_name 
            FROM information_schema.tables 
            WHERE table_schema = 'public' 
              AND table_name IN ('users', 'user_preferences')
            ORDER BY table_name
        """)
        
        if len(tables) == 2:
            print(f"✅ Migration successful! Tables created:")
            for table in tables:
                print(f"   - {table['table_name']}")
            result = True
        else:
            print(f"⚠️  Only {len(tables)} table(s) found. Expected 2.")
            result = False
        
        # Close connection
        await conn.close()
        return result
        
    except Exception as e:
        print(f"❌ Migration failed: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(run_migration())
    exit(0 if success else 1)
