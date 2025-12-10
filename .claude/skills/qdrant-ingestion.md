name: Qdrant Ingestion Pipeline
description: Document ingestion for RAG
Qdrant Ingestion
Python

from qdrant_client import QdrantClient
from sentence_transformers import SentenceTransformer
import frontmatter
from pathlib import Path

class Ingester:
    def __init__(self):
        self.qdrant = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
        self.embedder = SentenceTransformer("all-MiniLM-L6-v2")
    
    def chunk(self, text, size=500, overlap=50):
        chunks = []
        for i in range(0, len(text), size - overlap):
            chunks.append(text[i:i+size])
        return chunks
    
    def ingest(self, docs_path):
        for md in Path(docs_path).rglob("*.md"):
            post = frontmatter.load(md)
            for i, chunk in enumerate(self.chunk(post.content)):
                vector = self.embedder.encode(chunk)
                self.qdrant.upsert("book", [{"id": f"{md}:{i}", "vector": vector, "payload": {"content": chunk}}])

# Run: python ingest.py --docs ./docs
