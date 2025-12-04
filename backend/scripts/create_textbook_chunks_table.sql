-- Create textbook_chunks table for storing actual textbook content
-- This complements Qdrant vector storage with structured relational data

CREATE TABLE IF NOT EXISTS textbook_chunks (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(255) UNIQUE NOT NULL,      -- "week-01-02-foundations-chunk-0"
    content TEXT NOT NULL,                       -- Actual text from markdown files
    chapter_name VARCHAR(255),                   -- "Weeks 1-2: Introduction to Physical AI"
    section_name VARCHAR(255),                   -- "Foundations of Physical AI"
    week_number INTEGER,                         -- 1, 2, 3, etc.
    file_path VARCHAR(500),                      -- "/week-01-02/01-foundations"
    chunk_index INTEGER,                         -- 0, 1, 2, 3... (position within document)
    token_count INTEGER,                         -- ~500-800 tokens per chunk
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for efficient querying
CREATE INDEX IF NOT EXISTS idx_textbook_chunks_chapter ON textbook_chunks(chapter_name);
CREATE INDEX IF NOT EXISTS idx_textbook_chunks_section ON textbook_chunks(section_name);
CREATE INDEX IF NOT EXISTS idx_textbook_chunks_week ON textbook_chunks(week_number);
CREATE INDEX IF NOT EXISTS idx_textbook_chunks_file_path ON textbook_chunks(file_path);
CREATE INDEX IF NOT EXISTS idx_textbook_chunks_chunk_id ON textbook_chunks(chunk_id);

-- Comments for documentation
COMMENT ON TABLE textbook_chunks IS 'Stores actual textbook content chunks for RAG retrieval';
COMMENT ON COLUMN textbook_chunks.chunk_id IS 'Unique identifier matching Qdrant point ID';
COMMENT ON COLUMN textbook_chunks.content IS 'Full text content of the chunk';
COMMENT ON COLUMN textbook_chunks.chunk_index IS 'Sequential position of chunk within source document';
COMMENT ON COLUMN textbook_chunks.token_count IS 'Number of tokens (for context window management)';
