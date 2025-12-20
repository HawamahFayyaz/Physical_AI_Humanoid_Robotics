#!/usr/bin/env python3
"""Test all service connections before running ingestion.

Usage:
    cd backend
    python -m scripts.test_connections
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import get_settings
from services import embedding, llm, vector_store, database


async def test_voyage_ai() -> bool:
    """Test Voyage AI embedding service."""
    print("\n📦 Testing Voyage AI (embeddings)...")
    try:
        result = await embedding.check_connection()
        if result["status"] == "healthy":
            print(f"   ✅ Connected to Voyage AI")
            print(f"   Model: {result['model']}")
            print(f"   Dimensions: {result['dimensions']}")
            return True
        else:
            print(f"   ❌ Voyage AI unhealthy: {result.get('error', 'Unknown error')}")
            return False
    except Exception as e:
        print(f"   ❌ Voyage AI error: {e}")
        return False


async def test_groq() -> bool:
    """Test Groq LLM service."""
    print("\n🤖 Testing Groq (LLM)...")
    try:
        result = await llm.check_connection()
        if result["status"] == "healthy":
            print(f"   ✅ Connected to Groq")
            print(f"   Model: {result['model']}")
            return True
        else:
            print(f"   ❌ Groq unhealthy: {result.get('error', 'Unknown error')}")
            return False
    except Exception as e:
        print(f"   ❌ Groq error: {e}")
        return False


async def test_qdrant() -> bool:
    """Test Qdrant vector store connection."""
    print("\n🔍 Testing Qdrant (vector store)...")
    try:
        # First initialize the collection if needed
        await vector_store.init_collection()

        result = await vector_store.check_connection()
        if result["status"] == "healthy":
            print(f"   ✅ Connected to Qdrant")
            print(f"   Collection: {result['collection']}")
            print(f"   Vectors: {result.get('vectors_count', 0)}")
            return True
        else:
            print(f"   ❌ Qdrant unhealthy: {result.get('error', 'Unknown error')}")
            return False
    except Exception as e:
        print(f"   ❌ Qdrant error: {e}")
        return False


async def test_neon() -> bool:
    """Test Neon Postgres database connection."""
    print("\n🐘 Testing Neon Postgres (database)...")
    try:
        # Initialize schema first (creates tables if not exist)
        await database.init_schema()
        print("   ✅ Schema initialized")

        result = await database.check_connection()
        if result["status"] == "healthy":
            print(f"   ✅ Connected to Neon Postgres")
            print(f"   Version: {result.get('version', 'Unknown')}")
            return True
        else:
            print(f"   ❌ Neon unhealthy: {result.get('error', 'Unknown error')}")
            return False
    except Exception as e:
        print(f"   ❌ Neon error: {e}")
        return False


async def test_full_pipeline() -> bool:
    """Test full RAG pipeline with a sample query."""
    print("\n🔄 Testing full RAG pipeline...")
    try:
        # Generate a test embedding
        test_query = "What is a humanoid robot?"
        query_embedding = await embedding.embed_text(test_query, input_type="query")
        print(f"   ✅ Generated query embedding ({len(query_embedding)} dims)")

        # Try a vector search (may return empty if no data)
        settings = get_settings()
        results = await vector_store.search_similar_chunks(
            query_embedding=query_embedding,
            top_k=3,
            score_threshold=0.5,
        )
        print(f"   ✅ Vector search returned {len(results)} results")

        # Test LLM with a simple prompt (no context needed)
        test_messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Say 'Hello from Groq!' in exactly 5 words."}
        ]
        response = await llm.generate_response_groq(test_messages)
        print(f"   ✅ LLM response: {response[:50]}...")

        return True
    except Exception as e:
        print(f"   ❌ Pipeline error: {e}")
        return False


async def main():
    """Run all connection tests."""
    print("=" * 60)
    print("🧪 RAG Chatbot Connection Tests")
    print("=" * 60)

    settings = get_settings()
    print(f"\nConfiguration loaded:")
    print(f"  - Voyage model: {settings.voyage_embedding_model}")
    print(f"  - Groq model: {settings.groq_llm_model}")
    print(f"  - Qdrant collection: {settings.qdrant_collection_name}")

    results = {
        "Voyage AI": await test_voyage_ai(),
        "Groq": await test_groq(),
        "Qdrant": await test_qdrant(),
        "Neon Postgres": await test_neon(),
    }

    # Only test pipeline if all services are healthy
    if all(results.values()):
        results["Full Pipeline"] = await test_full_pipeline()

    print("\n" + "=" * 60)
    print("📊 Test Results Summary")
    print("=" * 60)

    all_passed = True
    for service, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {service}: {status}")
        if not passed:
            all_passed = False

    print("\n" + "=" * 60)
    if all_passed:
        print("🎉 All tests passed! Ready for content ingestion.")
        print("   Run: python -m scripts.ingest_content")
    else:
        print("⚠️  Some tests failed. Check configuration and try again.")
    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
