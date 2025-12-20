-- Initial schema for RAG Chatbot
-- Feature: 002-rag-chatbot
-- Date: 2025-12-17

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Book chunks metadata (vectors stored in Qdrant)
CREATE TABLE IF NOT EXISTS book_chunks (
    id SERIAL PRIMARY KEY,
    chunk_id VARCHAR(500) UNIQUE NOT NULL,
    chapter VARCHAR(255) NOT NULL,
    section VARCHAR(255) NOT NULL,
    file_path VARCHAR(500) NOT NULL,
    word_count INTEGER NOT NULL CHECK (word_count > 0),
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_book_chunks_chapter ON book_chunks(chapter);
CREATE INDEX IF NOT EXISTS idx_book_chunks_file_path ON book_chunks(file_path);

-- Query logs for analytics
CREATE TABLE IF NOT EXISTS query_logs (
    id SERIAL PRIMARY KEY,
    session_id VARCHAR(255),
    query_text TEXT NOT NULL,
    response_summary TEXT,
    sources_used JSONB DEFAULT '[]',
    response_time_ms INTEGER NOT NULL CHECK (response_time_ms >= 0),
    context_type VARCHAR(50) DEFAULT 'full',
    selected_text TEXT,
    created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_query_logs_session ON query_logs(session_id);
CREATE INDEX IF NOT EXISTS idx_query_logs_created ON query_logs(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_query_logs_response_time ON query_logs(response_time_ms);

-- View for analytics dashboard
CREATE OR REPLACE VIEW query_analytics AS
SELECT
    DATE(created_at) as query_date,
    COUNT(*) as total_queries,
    AVG(response_time_ms) as avg_response_time,
    COUNT(CASE WHEN context_type = 'selection' THEN 1 END) as selection_queries,
    COUNT(CASE WHEN context_type = 'full' THEN 1 END) as full_queries
FROM query_logs
GROUP BY DATE(created_at)
ORDER BY query_date DESC;
