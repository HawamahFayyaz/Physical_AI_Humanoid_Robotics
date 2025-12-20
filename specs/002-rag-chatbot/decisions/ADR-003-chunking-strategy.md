# ADR-003: Content Chunking Strategy

**Status**: Accepted
**Date**: 2025-12-17
**Decision Makers**: Development Team
**Feature**: 002-rag-chatbot

## Context

The book content must be split into chunks for embedding and retrieval. The chunking strategy directly impacts:
- Retrieval accuracy (finding relevant content)
- Context quality (providing coherent information to LLM)
- Response quality (LLM having enough context to answer)
- Storage efficiency (number of vectors and storage size)

The book content is in MDX format with frontmatter, markdown headers, prose, code blocks, and mathematical notation.

## Decision

**Use a hybrid chunking strategy**: Markdown-aware splitting with fixed-size fallback and overlap.

## Options Considered

### Option 1: Fixed-size Chunking
- Split every N words/characters
- Simple implementation
- May split mid-sentence or mid-concept

### Option 2: Semantic Chunking (Paragraph-based)
- Split on paragraph boundaries
- Preserves natural breaks
- Variable chunk sizes (some too small, some too large)

### Option 3: Sliding Window with Overlap
- Fixed-size chunks with N-word overlap
- Ensures context continuity
- Higher storage overhead (50-100% more vectors)

### Option 4: Hybrid (Markdown-aware + Fixed + Overlap) (CHOSEN)
- Primary: Split on markdown headers (##, ###)
- Secondary: Fixed-size split for large sections
- Tertiary: Overlap between chunks
- Preserves document structure while ensuring size limits

### Option 5: RecursiveCharacterTextSplitter (LangChain)
- Standard library approach
- Splits by character types (paragraph, sentence, word)
- Less control over split points

## Rationale

1. **Structure Preservation**: Markdown headers define semantic boundaries in technical documentation. Splitting on headers keeps related content together.

2. **Code Block Integrity**: Technical books contain code examples that must not be split mid-block. The hybrid approach detects and preserves code fences.

3. **Predictable Sizes**: The spec requires 500-800 word chunks. Fixed-size fallback ensures compliance while overlap maintains context across boundaries.

4. **Metadata Extraction**: Splitting by headers allows automatic extraction of chapter/section names for citation metadata.

5. **Retrieval Quality**: Larger, coherent chunks improve retrieval relevance compared to arbitrary splits.

## Implementation

### Algorithm

```python
def chunk_mdx_content(content: str, file_path: str) -> list[Chunk]:
    chunks = []

    # 1. Parse frontmatter for chapter metadata
    frontmatter, body = parse_frontmatter(content)
    chapter_name = frontmatter.get('title', 'Unknown')

    # 2. Split by H2 headers first
    sections = split_by_pattern(body, r'^## ', keep_delimiter=True)

    for section in sections:
        section_name = extract_header(section)

        # 3. If section > 800 words, split by H3
        if word_count(section) > 800:
            subsections = split_by_pattern(section, r'^### ', keep_delimiter=True)

            for subsection in subsections:
                # 4. If still > 800 words, fixed split with overlap
                if word_count(subsection) > 800:
                    fixed_chunks = fixed_split(subsection,
                                               target=600,
                                               overlap=100,
                                               preserve_code_blocks=True)
                    for i, text in enumerate(fixed_chunks):
                        chunks.append(Chunk(
                            content=text,
                            chapter=chapter_name,
                            section=section_name,
                            file_path=file_path,
                            chunk_index=len(chunks)
                        ))
                else:
                    chunks.append(Chunk(
                        content=subsection,
                        chapter=chapter_name,
                        section=extract_header(subsection) or section_name,
                        file_path=file_path,
                        chunk_index=len(chunks)
                    ))
        else:
            chunks.append(Chunk(
                content=section,
                chapter=chapter_name,
                section=section_name,
                file_path=file_path,
                chunk_index=len(chunks)
            ))

    return chunks
```

### Code Block Handling

```python
def preserve_code_blocks(text: str) -> list[tuple[str, bool]]:
    """Split text while marking code blocks as non-splittable."""
    segments = []
    in_code_block = False
    current = []

    for line in text.split('\n'):
        if line.startswith('```'):
            if current:
                segments.append(('\n'.join(current), not in_code_block))
                current = []
            in_code_block = not in_code_block
            current.append(line)
        else:
            current.append(line)

    if current:
        segments.append(('\n'.join(current), not in_code_block))

    return segments
```

## Consequences

### Positive
- Chunks align with document structure (chapters/sections)
- Code blocks remain intact
- Predictable chunk sizes within spec requirements
- Rich metadata for citations

### Negative
- More complex than simple fixed-size splitting
- Some chunks may be slightly smaller than target (short sections)
- Overlap increases storage by ~15%

### Mitigations
- Merge very small chunks (<200 words) with neighbors
- Set minimum chunk size threshold
- Monitor and tune overlap percentage based on retrieval quality

## Configuration

```python
CHUNK_CONFIG = {
    "target_size": 600,      # Target words per chunk
    "max_size": 800,         # Maximum words before split
    "min_size": 200,         # Minimum chunk size (merge if smaller)
    "overlap": 100,          # Overlap words between chunks
    "header_patterns": [      # Markdown headers to split on
        r'^## ',              # H2 first
        r'^### ',             # H3 second
    ]
}
```

## Validation

After ingestion, validate:
- [ ] No chunk exceeds 900 words
- [ ] No chunk is smaller than 100 words (except last chunk)
- [ ] No code block is split across chunks
- [ ] All chunks have chapter and section metadata
- [ ] Total chunks ≈ expected (file_count × avg_chunks_per_file)

## References

- [LangChain Text Splitters](https://python.langchain.com/docs/modules/data_connection/document_transformers/)
- [Chunking Strategies for RAG](https://www.pinecone.io/learn/chunking-strategies/)
- [MDX Specification](https://mdxjs.com/docs/)
