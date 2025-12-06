"""
Verify Authentication Tables Migration

Checks if users and user_preferences tables exist with correct schema.
"""

import asyncio
import os
import asyncpg
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


async def verify_tables():
    """Verify authentication tables exist and have correct schema."""
    database_url = os.getenv("NEON_DATABASE_URL")
    if not database_url:
        print("❌ NEON_DATABASE_URL not set in .env file")
        return False
    
    try:
        print("🔗 Connecting to Neon Postgres...")
        conn = await asyncpg.connect(database_url)
        
        # Check tables exist
        print("\n📋 Checking tables...")
        tables = await conn.fetch("""
            SELECT table_name 
            FROM information_schema.tables 
            WHERE table_schema = 'public' 
              AND table_name IN ('users', 'user_preferences')
            ORDER BY table_name
        """)
        
        if len(tables) < 2:
            print(f"❌ Missing tables. Found: {[t['table_name'] for t in tables]}")
            return False
        
        print("✅ Tables exist:")
        for table in tables:
            print(f"   - {table['table_name']}")
        
        # Check users table columns
        print("\n📋 Checking 'users' table schema...")
        users_columns = await conn.fetch("""
            SELECT column_name, data_type, is_nullable
            FROM information_schema.columns
            WHERE table_schema = 'public' 
              AND table_name = 'users'
            ORDER BY ordinal_position
        """)
        
        print("   Columns:")
        for col in users_columns:
            nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
            print(f"   - {col['column_name']}: {col['data_type']} {nullable}")
        
        # Check user_preferences table columns
        print("\n📋 Checking 'user_preferences' table schema...")
        prefs_columns = await conn.fetch("""
            SELECT column_name, data_type, is_nullable
            FROM information_schema.columns
            WHERE table_schema = 'public' 
              AND table_name = 'user_preferences'
            ORDER BY ordinal_position
        """)
        
        print("   Columns:")
        for col in prefs_columns:
            nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
            print(f"   - {col['column_name']}: {col['data_type']} {nullable}")
        
        # Check constraints
        print("\n📋 Checking constraints...")
        constraints = await conn.fetch("""
            SELECT conname, contype
            FROM pg_constraint
            WHERE conrelid IN ('users'::regclass, 'user_preferences'::regclass)
            ORDER BY conname
        """)
        
        print("   Constraints:")
        for constraint in constraints:
            constraint_type = {
                'p': 'PRIMARY KEY',
                'f': 'FOREIGN KEY',
                'u': 'UNIQUE',
                'c': 'CHECK'
            }.get(constraint['contype'], constraint['contype'])
            print(f"   - {constraint['conname']}: {constraint_type}")
        
        # Check indexes
        print("\n📋 Checking indexes...")
        indexes = await conn.fetch("""
            SELECT indexname, tablename
            FROM pg_indexes
            WHERE schemaname = 'public'
              AND tablename IN ('users', 'user_preferences')
            ORDER BY tablename, indexname
        """)
        
        print("   Indexes:")
        for idx in indexes:
            print(f"   - {idx['indexname']} on {idx['tablename']}")
        
        # Check triggers
        print("\n📋 Checking triggers...")
        triggers = await conn.fetch("""
            SELECT trigger_name, event_manipulation, event_object_table
            FROM information_schema.triggers
            WHERE event_object_schema = 'public'
              AND event_object_table IN ('users', 'user_preferences')
            ORDER BY event_object_table, trigger_name
        """)
        
        print("   Triggers:")
        for trigger in triggers:
            print(f"   - {trigger['trigger_name']} on {trigger['event_object_table']} ({trigger['event_manipulation']})")
        
        await conn.close()
        print("\n✅ Migration verification complete!")
        return True
        
    except Exception as e:
        print(f"❌ Verification failed: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(verify_tables())
    exit(0 if success else 1)
