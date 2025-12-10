#!/usr/bin/env python3
"""
Document Ingestion Script for Physical AI RAG Chatbot

This script:
1. Reads all markdown files from website/docs/
2. Chunks content with 500 tokens, 50 token overlap
3. Embeds chunks using sentence-transformers/all-MiniLM-L6-v2
4. Uploads to Qdrant Cloud vector database

Usage:
    python scripts/ingest_docs.py

Environment variables required:
    - QDRANT_URL: Qdrant Cloud URL
    - QDRANT_API_KEY: Qdrant API key
"""

import os
import re
import glob
from pathlib import Path
from typing import List, Dict, Any
from dataclasses import dataclass

from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models as qmodels

# Load environment variables
load_dotenv()

# Configuration
DOCS_DIR = Path(__file__).parent.parent / "website" / "docs"
QDRANT_URL = os.getenv("QDRANT_URL", "")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
COLLECTION_NAME = "physical_ai_docs"
EMBEDDING_MODEL = "all-MiniLM-L6-v2"
CHUNK_SIZE = 500  # tokens
CHUNK_OVERLAP = 50  # tokens
VECTOR_SIZE = 384  # MiniLM output dimension


@dataclass
class DocumentChunk:
    """Represents a chunk of document content."""
    content: str
    source: str
    chapter: str
    module: str
    chunk_index: int
    metadata: Dict[str, Any]


def extract_frontmatter(content: str) -> tuple[Dict[str, str], str]:
    """Extract YAML frontmatter from markdown content."""
    frontmatter = {}
    body = content

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            fm_text = parts[1].strip()
            body = parts[2].strip()

            for line in fm_text.split("\n"):
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip()

    return frontmatter, body


def clean_markdown(content: str) -> str:
    """Clean markdown content for better embedding."""
    # Remove code blocks but keep content description
    content = re.sub(r'```[\w]*\n(.*?)```', r'[CODE BLOCK]', content, flags=re.DOTALL)

    # Remove inline code backticks
    content = re.sub(r'`([^`]+)`', r'\1', content)

    # Remove HTML tags
    content = re.sub(r'<[^>]+>', '', content)

    # Remove image references
    content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'[IMAGE: \1]', content)

    # Remove link URLs but keep text
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)

    # Normalize whitespace
    content = re.sub(r'\n{3,}', '\n\n', content)
    content = re.sub(r' {2,}', ' ', content)

    return content.strip()


def estimate_tokens(text: str) -> int:
    """Estimate token count (rough approximation: 4 chars = 1 token)."""
    return len(text) // 4


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
    """Split text into overlapping chunks based on token estimate."""
    # Split by paragraphs first
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = []
    current_size = 0

    for para in paragraphs:
        para_tokens = estimate_tokens(para)

        if current_size + para_tokens > chunk_size and current_chunk:
            # Save current chunk
            chunks.append('\n\n'.join(current_chunk))

            # Keep overlap
            overlap_text = current_chunk[-1] if current_chunk else ""
            overlap_tokens = estimate_tokens(overlap_text)

            if overlap_tokens < overlap:
                current_chunk = [overlap_text, para]
                current_size = overlap_tokens + para_tokens
            else:
                current_chunk = [para]
                current_size = para_tokens
        else:
            current_chunk.append(para)
            current_size += para_tokens

    # Don't forget the last chunk
    if current_chunk:
        chunks.append('\n\n'.join(current_chunk))

    return chunks


def parse_source_path(filepath: Path) -> Dict[str, str]:
    """Extract module and chapter info from file path."""
    parts = filepath.relative_to(DOCS_DIR).parts

    module = "general"
    chapter = filepath.stem

    if len(parts) >= 2:
        module = parts[0]
        chapter = parts[-1].replace('.md', '')

    return {
        "module": module,
        "chapter": chapter,
        "filename": filepath.name,
        "path": str(filepath.relative_to(DOCS_DIR))
    }


def process_documents() -> List[DocumentChunk]:
    """Process all markdown documents and create chunks."""
    chunks = []

    # Find all markdown files
    md_files = list(DOCS_DIR.glob("**/*.md")) + list(DOCS_DIR.glob("**/*.mdx"))

    print(f"Found {len(md_files)} markdown files")

    for filepath in md_files:
        print(f"Processing: {filepath.relative_to(DOCS_DIR)}")

        try:
            content = filepath.read_text(encoding='utf-8')
        except Exception as e:
            print(f"  Error reading file: {e}")
            continue

        # Extract frontmatter and body
        frontmatter, body = extract_frontmatter(content)

        # Clean content
        cleaned = clean_markdown(body)

        if not cleaned or len(cleaned) < 50:
            print(f"  Skipping: Content too short")
            continue

        # Parse source info
        source_info = parse_source_path(filepath)

        # Create chunks
        text_chunks = chunk_text(cleaned)
        print(f"  Created {len(text_chunks)} chunks")

        for i, chunk_text_content in enumerate(text_chunks):
            chunk = DocumentChunk(
                content=chunk_text_content,
                source=source_info["path"],
                chapter=source_info["chapter"],
                module=source_info["module"],
                chunk_index=i,
                metadata={
                    **source_info,
                    **frontmatter,
                    "chunk_index": i,
                    "total_chunks": len(text_chunks)
                }
            )
            chunks.append(chunk)

    return chunks


def create_embeddings(chunks: List[DocumentChunk], model: SentenceTransformer) -> List[List[float]]:
    """Generate embeddings for all chunks."""
    print(f"\nGenerating embeddings for {len(chunks)} chunks...")

    texts = [chunk.content for chunk in chunks]
    embeddings = model.encode(texts, show_progress_bar=True)

    return embeddings.tolist()


def setup_qdrant_collection(client: QdrantClient):
    """Create or recreate the Qdrant collection."""
    # Check if collection exists
    collections = client.get_collections().collections
    exists = any(c.name == COLLECTION_NAME for c in collections)

    if exists:
        print(f"Deleting existing collection: {COLLECTION_NAME}")
        client.delete_collection(COLLECTION_NAME)

    print(f"Creating collection: {COLLECTION_NAME}")
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=qmodels.VectorParams(
            size=VECTOR_SIZE,
            distance=qmodels.Distance.COSINE
        )
    )


def upload_to_qdrant(
    client: QdrantClient,
    chunks: List[DocumentChunk],
    embeddings: List[List[float]]
):
    """Upload chunks and embeddings to Qdrant."""
    print(f"\nUploading {len(chunks)} vectors to Qdrant...")

    points = [
        qmodels.PointStruct(
            id=i,
            vector=embedding,
            payload={
                "content": chunk.content,
                "source": chunk.source,
                "chapter": chunk.chapter,
                "module": chunk.module,
                "chunk_index": chunk.chunk_index,
                **chunk.metadata
            }
        )
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings))
    ]

    # Upload in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"  Uploaded {min(i + batch_size, len(points))}/{len(points)} vectors")

    print("Upload complete!")


def main():
    """Main ingestion pipeline."""
    print("=" * 60)
    print("Physical AI Documentation Ingestion Pipeline")
    print("=" * 60)

    # Validate environment
    if not QDRANT_URL or not QDRANT_API_KEY:
        print("\nERROR: QDRANT_URL and QDRANT_API_KEY environment variables required")
        print("Set them in backend/.env or export them directly")
        return

    # Check docs directory
    if not DOCS_DIR.exists():
        print(f"\nERROR: Docs directory not found: {DOCS_DIR}")
        return

    # Process documents
    print(f"\n[1/4] Processing documents from: {DOCS_DIR}")
    chunks = process_documents()

    if not chunks:
        print("No chunks created. Check your markdown files.")
        return

    print(f"\nTotal chunks: {len(chunks)}")

    # Load embedding model
    print(f"\n[2/4] Loading embedding model: {EMBEDDING_MODEL}")
    model = SentenceTransformer(EMBEDDING_MODEL)

    # Generate embeddings
    print("\n[3/4] Generating embeddings")
    embeddings = create_embeddings(chunks, model)

    # Upload to Qdrant
    print("\n[4/4] Uploading to Qdrant")
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    setup_qdrant_collection(client)
    upload_to_qdrant(client, chunks, embeddings)

    # Summary
    print("\n" + "=" * 60)
    print("Ingestion Complete!")
    print(f"  - Documents processed: {len(set(c.source for c in chunks))}")
    print(f"  - Chunks created: {len(chunks)}")
    print(f"  - Collection: {COLLECTION_NAME}")
    print("=" * 60)


if __name__ == "__main__":
    main()
