#!/usr/bin/env python3
"""Content ingestion script for RAG Chatbot.

Parses MDX files from website/docs/, chunks content using hybrid strategy
(ADR-003), generates embeddings, and populates Qdrant and Postgres.

Usage:
    python scripts/ingest_content.py [--dry-run] [--clear]

Options:
    --dry-run    Parse and chunk without uploading to vector store
    --clear      Clear existing collection before ingestion
"""

import asyncio
import gc
import hashlib
import logging
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import yaml

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import get_settings
from services.embedding import embed_texts
from services.vector_store import (
    get_qdrant_client,
    init_collection,
    upsert_chunk,
    delete_collection,
)
from services.database import (
    get_db_pool,
    init_schema,
    upsert_chunk_metadata,
    close_db_pool,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Chunking configuration from ADR-003 (aggressive reduction for OOM prevention)
CHUNK_CONFIG = {
    "target_size": 200,      # Target words per chunk (reduced from 600)
    "max_size": 300,         # Maximum words before split (reduced from 800)
    "min_size": 50,          # Minimum chunk size (merge if smaller)
    "overlap": 30,           # Overlap words between chunks (reduced from 100)
}


@dataclass
class Chunk:
    """A chunk of content for embedding."""
    chunk_id: str
    content: str
    chapter: str
    section: str
    file_path: str
    word_count: int


def parse_frontmatter(content: str) -> Tuple[dict, str]:
    """Parse YAML frontmatter from MDX file.

    Args:
        content: Full MDX file content

    Returns:
        Tuple of (frontmatter dict, body without frontmatter)
    """
    if not content.startswith("---"):
        return {}, content

    parts = content.split("---", 2)
    if len(parts) < 3:
        return {}, content

    try:
        frontmatter = yaml.safe_load(parts[1]) or {}
    except yaml.YAMLError:
        frontmatter = {}

    body = parts[2].strip()
    return frontmatter, body


def word_count(text: str) -> int:
    """Count words in text using streaming approach (memory-efficient).

    Does not create intermediate list - counts words character-by-character.
    """
    count = 0
    in_word = False
    for char in text:
        if char.isspace():
            if in_word:
                count += 1
                in_word = False
        else:
            in_word = True
    # Count last word if text doesn't end with whitespace
    if in_word:
        count += 1
    return count


def extract_header(text: str) -> Optional[str]:
    """Extract the first markdown header from text."""
    match = re.search(r"^#{1,6}\s+(.+?)(?:\s*{.*})?$", text, re.MULTILINE)
    if match:
        return match.group(1).strip()
    return None


def split_by_headers(text: str, pattern: str) -> List[str]:
    """Split text by markdown header pattern, keeping the delimiter.

    Args:
        text: Markdown text to split
        pattern: Regex pattern for headers (e.g., r'^## ')

    Returns:
        List of sections, each starting with a header (except possibly first)
    """
    lines = text.split("\n")
    sections = []
    current_section = []

    for line in lines:
        if re.match(pattern, line, re.MULTILINE):
            if current_section:
                sections.append("\n".join(current_section))
            current_section = [line]
        else:
            current_section.append(line)

    if current_section:
        sections.append("\n".join(current_section))

    return sections


def find_code_block_ranges(text: str) -> List[Tuple[int, int]]:
    """Find start and end positions of code blocks in text.

    Returns:
        List of (start, end) character positions of code blocks
    """
    ranges = []
    in_code = False
    start_pos = 0

    lines = text.split("\n")
    pos = 0

    for line in lines:
        if line.startswith("```"):
            if not in_code:
                in_code = True
                start_pos = pos
            else:
                in_code = False
                ranges.append((start_pos, pos + len(line)))
        pos += len(line) + 1  # +1 for newline

    return ranges


def is_in_code_block(position: int, code_ranges: List[Tuple[int, int]]) -> bool:
    """Check if a position is inside a code block."""
    for start, end in code_ranges:
        if start <= position <= end:
            return True
    return False


def fixed_split_with_overlap(
    text: str,
    target_words: int = 600,
    overlap_words: int = 100,
) -> List[str]:
    """Split text into chunks with overlap, preserving code blocks.

    Args:
        text: Text to split
        target_words: Target words per chunk
        overlap_words: Words to overlap between chunks

    Returns:
        List of text chunks
    """
    words = text.split()
    if len(words) <= target_words:
        return [text]

    code_ranges = find_code_block_ranges(text)
    chunks = []
    start_idx = 0

    while start_idx < len(words):
        end_idx = min(start_idx + target_words, len(words))

        # Join words back and find position
        chunk_text = " ".join(words[start_idx:end_idx])

        # Check if we're splitting inside a code block
        char_pos = len(" ".join(words[:end_idx]))
        if is_in_code_block(char_pos, code_ranges) and end_idx < len(words):
            # Extend to end of code block
            while end_idx < len(words):
                end_idx += 1
                extended_text = " ".join(words[start_idx:end_idx])
                if "```" in extended_text.split()[-10:]:
                    # Found end of code block
                    break

        chunk_text = " ".join(words[start_idx:end_idx])
        chunks.append(chunk_text)

        # Move start with overlap
        start_idx = end_idx - overlap_words

        # Prevent infinite loop
        if start_idx <= 0 and len(chunks) > 1:
            break

    return chunks


def generate_chunk_id(file_path: str, chapter: str, section: str, index: int) -> str:
    """Generate a unique chunk ID.

    Args:
        file_path: Source file path
        chapter: Chapter name
        section: Section name
        index: Chunk index within file

    Returns:
        Unique chunk ID string
    """
    content = f"{file_path}:{chapter}:{section}:{index}"
    hash_val = hashlib.md5(content.encode()).hexdigest()[:12]
    return f"chunk_{hash_val}"


def chunk_mdx_content(content: str, file_path: str) -> List[Chunk]:
    """Chunk MDX content using hybrid strategy per ADR-003.

    Args:
        content: Full MDX file content
        file_path: Path to source file

    Returns:
        List of Chunk objects
    """
    chunks = []

    # 1. Parse frontmatter
    frontmatter, body = parse_frontmatter(content)
    chapter_name = frontmatter.get("title", "Unknown Chapter")
    sidebar_label = frontmatter.get("sidebar_label", chapter_name)

    # Skip if minimal content
    if word_count(body) < 50:
        logger.warning(f"Skipping {file_path}: too short ({word_count(body)} words)")
        return []

    # 2. Split by H2 headers first
    h2_sections = split_by_headers(body, r"^## ")

    for section in h2_sections:
        section_name = extract_header(section) or sidebar_label

        # 3. If section > max_size words, split by H3
        if word_count(section) > CHUNK_CONFIG["max_size"]:
            h3_subsections = split_by_headers(section, r"^### ")

            for subsection in h3_subsections:
                subsection_name = extract_header(subsection) or section_name

                # 4. If still > max_size words, fixed split with overlap
                if word_count(subsection) > CHUNK_CONFIG["max_size"]:
                    fixed_chunks = fixed_split_with_overlap(
                        subsection,
                        target_words=CHUNK_CONFIG["target_size"],
                        overlap_words=CHUNK_CONFIG["overlap"],
                    )
                    for text in fixed_chunks:
                        chunk_id = generate_chunk_id(
                            file_path, chapter_name, subsection_name, len(chunks)
                        )
                        chunks.append(
                            Chunk(
                                chunk_id=chunk_id,
                                content=text.strip(),
                                chapter=chapter_name,
                                section=subsection_name,
                                file_path=file_path,
                                word_count=word_count(text),
                            )
                        )
                elif word_count(subsection) >= CHUNK_CONFIG["min_size"]:
                    chunk_id = generate_chunk_id(
                        file_path, chapter_name, subsection_name, len(chunks)
                    )
                    chunks.append(
                        Chunk(
                            chunk_id=chunk_id,
                            content=subsection.strip(),
                            chapter=chapter_name,
                            section=subsection_name,
                            file_path=file_path,
                            word_count=word_count(subsection),
                        )
                    )

        elif word_count(section) >= CHUNK_CONFIG["min_size"]:
            chunk_id = generate_chunk_id(
                file_path, chapter_name, section_name, len(chunks)
            )
            chunks.append(
                Chunk(
                    chunk_id=chunk_id,
                    content=section.strip(),
                    chapter=chapter_name,
                    section=section_name,
                    file_path=file_path,
                    word_count=word_count(section),
                )
            )

    # Handle case where no H2 headers found (treat whole file as one section)
    if not chunks and word_count(body) >= CHUNK_CONFIG["min_size"]:
        if word_count(body) > CHUNK_CONFIG["max_size"]:
            fixed_chunks = fixed_split_with_overlap(
                body,
                target_words=CHUNK_CONFIG["target_size"],
                overlap_words=CHUNK_CONFIG["overlap"],
            )
            for text in fixed_chunks:
                chunk_id = generate_chunk_id(
                    file_path, chapter_name, sidebar_label, len(chunks)
                )
                chunks.append(
                    Chunk(
                        chunk_id=chunk_id,
                        content=text.strip(),
                        chapter=chapter_name,
                        section=sidebar_label,
                        file_path=file_path,
                        word_count=word_count(text),
                    )
                )
        else:
            chunk_id = generate_chunk_id(
                file_path, chapter_name, sidebar_label, 0
            )
            chunks.append(
                Chunk(
                    chunk_id=chunk_id,
                    content=body.strip(),
                    chapter=chapter_name,
                    section=sidebar_label,
                    file_path=file_path,
                    word_count=word_count(body),
                )
            )

    return chunks


def find_mdx_files(docs_dir: Path) -> List[Path]:
    """Find all MDX files in the docs directory.

    Args:
        docs_dir: Path to website/docs directory

    Returns:
        List of MDX file paths
    """
    mdx_files = list(docs_dir.glob("**/*.mdx"))
    md_files = list(docs_dir.glob("**/*.md"))

    # Exclude specific files like category.json descriptions
    all_files = []
    for f in mdx_files + md_files:
        # Skip _category_ files and other metadata
        if "_category_" in f.name:
            continue
        # Skip files in node_modules or build directories
        if "node_modules" in str(f) or ".docusaurus" in str(f):
            continue
        all_files.append(f)

    return sorted(all_files)


def convert_file_path_to_url(file_path: str) -> str:
    """Convert a file path to a Docusaurus URL.

    Args:
        file_path: Path like 'website/docs/03-kinematics/inverse.mdx'

    Returns:
        URL path like '/docs/03-kinematics/inverse'
    """
    # Remove website/docs prefix and file extension
    path = file_path.replace("\\", "/")  # Normalize Windows paths

    if "website/docs/" in path:
        path = path.split("website/docs/")[1]
    elif "docs/" in path:
        path = path.split("docs/")[1]

    # Remove extension
    if path.endswith(".mdx"):
        path = path[:-4]
    elif path.endswith(".md"):
        path = path[:-3]

    # Handle index files
    if path.endswith("/index") or path == "index":
        path = path.replace("/index", "").replace("index", "")

    return f"/docs/{path}" if path else "/docs/"


async def ingest_all_content(
    docs_dir: Path,
    dry_run: bool = False,
    clear_existing: bool = False,
    file_limit: int | None = None,
) -> dict:
    """Ingest all MDX content from docs directory.

    Args:
        docs_dir: Path to website/docs directory
        dry_run: If True, parse and chunk without uploading
        clear_existing: If True, clear existing collection first
        file_limit: If set, limit to processing this many files (for testing)

    Returns:
        Dict with statistics about the ingestion
    """
    settings = get_settings()
    stats = {
        "files_processed": 0,
        "chunks_created": 0,
        "total_words": 0,
        "errors": [],
    }

    # Initialize services
    if not dry_run:
        logger.info("Initializing database schema...")
        await init_schema()

        if clear_existing:
            logger.info("Clearing existing vector collection...")
            try:
                await delete_collection()
            except Exception as e:
                logger.warning(f"Could not delete collection (may not exist): {e}")

        logger.info("Initializing vector collection...")
        await init_collection()

    # Find all MDX files
    mdx_files = find_mdx_files(docs_dir)
    total_files = len(mdx_files)
    logger.info(f"Found {total_files} MDX/MD files to process")

    # Apply file limit for testing
    if file_limit is not None:
        mdx_files = mdx_files[:file_limit]
        logger.info(f"TEST MODE: Limited to {file_limit} files")

    all_chunks: List[Chunk] = []

    # Parse and chunk all files
    for file_idx, file_path in enumerate(mdx_files, 1):
        logger.info(f"[{file_idx}/{len(mdx_files)}] Processing: {file_path.name}")
        try:
            logger.debug(f"  Reading file: {file_path}")
            content = file_path.read_text(encoding="utf-8")
            logger.debug(f"  File size: {len(content)} chars, ~{len(content.split())} words")

            relative_path = str(file_path.relative_to(docs_dir.parent.parent))
            logger.debug(f"  Relative path: {relative_path}")

            logger.debug(f"  Chunking content...")
            chunks = chunk_mdx_content(content, relative_path)
            all_chunks.extend(chunks)

            stats["files_processed"] += 1
            stats["chunks_created"] += len(chunks)
            stats["total_words"] += sum(c.word_count for c in chunks)

            logger.info(f"  -> Created {len(chunks)} chunks ({sum(c.word_count for c in chunks)} words)")

        except Exception as e:
            logger.error(f"ERROR processing {file_path}: {type(e).__name__}: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            stats["errors"].append(str(file_path))

    logger.info(f"Created {len(all_chunks)} chunks from {stats['files_processed']} files")

    if dry_run:
        # Print sample chunks for verification
        logger.info("\n=== DRY RUN - Sample Chunks ===")
        for chunk in all_chunks[:3]:
            logger.info(f"\nChunk ID: {chunk.chunk_id}")
            logger.info(f"Chapter: {chunk.chapter}")
            logger.info(f"Section: {chunk.section}")
            logger.info(f"Words: {chunk.word_count}")
            logger.info(f"File: {chunk.file_path}")
            logger.info(f"Content preview: {chunk.content[:200]}...")
        return stats

    # Generate embeddings in batches
    logger.info("=" * 50)
    logger.info("PHASE: Generating embeddings via Voyage AI")
    logger.info("=" * 50)
    batch_size = 32  # Reduced from 50 for stability
    chunk_contents = [c.content for c in all_chunks]
    logger.info(f"Total chunks to embed: {len(chunk_contents)}")
    logger.info(f"Batch size: {batch_size}")
    logger.info(f"Estimated batches: {(len(chunk_contents) + batch_size - 1) // batch_size}")

    try:
        embeddings = await embed_texts(
            chunk_contents,
            batch_size=batch_size,
            delay_between_batches=0.5,  # Rate limiting
        )
        logger.info(f"Successfully generated {len(embeddings)} embeddings")
    except Exception as e:
        logger.error(f"FATAL: Embedding generation failed: {type(e).__name__}: {e}")
        import traceback
        logger.error(traceback.format_exc())
        raise

    # Upload to vector store and database
    logger.info("Uploading to vector store and database...")
    for i, (chunk, embedding) in enumerate(zip(all_chunks, embeddings)):
        try:
            # Upload to Qdrant
            await upsert_chunk(
                chunk_id=chunk.chunk_id,
                embedding=embedding,
                chapter=chunk.chapter,
                section=chunk.section,
                file_path=chunk.file_path,
                content=chunk.content,
                word_count=chunk.word_count,
            )

            # Upload metadata to Postgres
            await upsert_chunk_metadata(
                chunk_id=chunk.chunk_id,
                chapter=chunk.chapter,
                section=chunk.section,
                file_path=chunk.file_path,
                word_count=chunk.word_count,
            )

            if (i + 1) % 50 == 0:
                logger.info(f"Uploaded {i + 1}/{len(all_chunks)} chunks")

        except Exception as e:
            logger.error(f"Error uploading chunk {chunk.chunk_id}: {e}")
            stats["errors"].append(chunk.chunk_id)

    logger.info("Ingestion complete!")
    return stats


async def main():
    """Main entry point for ingestion script."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Ingest book content into RAG vector store"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and chunk without uploading to vector store",
    )
    parser.add_argument(
        "--clear",
        action="store_true",
        help="Clear existing collection before ingestion",
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        default=None,
        help="Path to docs directory (default: website/docs)",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Limit number of files to process (for testing)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose/debug logging",
    )

    args = parser.parse_args()

    # Set logging level based on verbose flag
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.setLevel(logging.DEBUG)

    # Find docs directory
    if args.docs_dir:
        docs_dir = Path(args.docs_dir)
    else:
        # Try to find docs directory relative to script
        script_dir = Path(__file__).parent
        possible_paths = [
            script_dir.parent.parent / "website" / "docs",
            Path.cwd() / "website" / "docs",
            Path.cwd().parent / "website" / "docs",
        ]
        docs_dir = None
        for path in possible_paths:
            if path.exists():
                docs_dir = path
                break

        if not docs_dir:
            logger.error("Could not find website/docs directory")
            sys.exit(1)

    logger.info(f"Using docs directory: {docs_dir}")

    try:
        stats = await ingest_all_content(
            docs_dir=docs_dir,
            dry_run=args.dry_run,
            clear_existing=args.clear,
            file_limit=args.limit,
        )

        # Print summary
        logger.info("\n=== Ingestion Summary ===")
        logger.info(f"Files processed: {stats['files_processed']}")
        logger.info(f"Chunks created: {stats['chunks_created']}")
        logger.info(f"Total words: {stats['total_words']}")
        if stats['errors']:
            logger.warning(f"Errors: {len(stats['errors'])}")
            for error in stats['errors'][:5]:
                logger.warning(f"  - {error}")

    finally:
        await close_db_pool()


if __name__ == "__main__":
    asyncio.run(main())
