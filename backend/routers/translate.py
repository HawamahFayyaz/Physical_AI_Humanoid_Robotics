"""
Urdu Translation Router

Provides English to Urdu translation for book content:
- Translates chapter content with technical term preservation
- Caches translations for performance
- Supports RTL text direction
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict
import os
import hashlib
import httpx
from datetime import datetime, timedelta

router = APIRouter()

# Configuration
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")
CACHE_TTL_HOURS = 168  # 1 week for translations

# Translation cache
_translation_cache: Dict[str, dict] = {}


class TranslationRequest(BaseModel):
    """Request to translate content."""
    content: str = Field(..., min_length=1, max_length=50000)
    chapter_id: str = Field(..., min_length=1)
    preserve_code: bool = True


class TranslationResponse(BaseModel):
    """Translation response."""
    original_content: str
    translated_content: str
    language: str = "ur"
    direction: str = "rtl"
    cached: bool = False


def get_cache_key(chapter_id: str, content_hash: str) -> str:
    """Generate cache key for translation."""
    return f"ur:{chapter_id}:{content_hash}"


def get_content_hash(content: str) -> str:
    """Get hash of content for caching."""
    return hashlib.md5(content.encode()).hexdigest()[:12]


def get_cached_translation(cache_key: str) -> Optional[str]:
    """Get cached translation if valid."""
    if cache_key in _translation_cache:
        cached = _translation_cache[cache_key]
        if datetime.now() < cached["expires_at"]:
            return cached["translation"]
        else:
            del _translation_cache[cache_key]
    return None


def set_cached_translation(cache_key: str, translation: str):
    """Cache translation result."""
    _translation_cache[cache_key] = {
        "translation": translation,
        "expires_at": datetime.now() + timedelta(hours=CACHE_TTL_HOURS)
    }


async def translate_with_groq(content: str, preserve_code: bool) -> str:
    """Translate content to Urdu using Groq Llama 3."""
    if not GROQ_API_KEY:
        raise HTTPException(
            status_code=503,
            detail="Translation service not configured"
        )

    code_instruction = ""
    if preserve_code:
        code_instruction = """
IMPORTANT: Keep ALL code blocks, variable names, function names, and technical commands in English.
Only translate the explanatory text, not the code itself.
Keep markdown formatting intact."""

    system_prompt = f"""You are an expert translator specializing in technical robotics documentation.
Translate the following content from English to Urdu.

Translation Guidelines:
1. Maintain technical accuracy
2. Keep technical terms in English with Urdu transliteration in parentheses where helpful
3. Examples: ROS2 (آر او ایس 2), LIDAR (لائیڈار), sensor (سینسر)
4. Preserve all markdown formatting, headings, and structure
5. Keep URLs and links unchanged
{code_instruction}

Return ONLY the translated content, maintaining the same structure."""

    user_prompt = f"""Translate to Urdu:

{content}"""

    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.groq.com/openai/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {GROQ_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": "llama-3.1-8b-instant",
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                "max_tokens": 4096,
                "temperature": 0.2
            },
            timeout=60.0
        )

        if response.status_code != 200:
            raise HTTPException(
                status_code=502,
                detail=f"Translation service error: {response.text}"
            )

        result = response.json()
        return result["choices"][0]["message"]["content"]


@router.post("/urdu", response_model=TranslationResponse)
async def translate_to_urdu(request: TranslationRequest):
    """
    Translate chapter content from English to Urdu.

    Preserves:
    - Code blocks and technical commands
    - Technical terminology (with transliteration)
    - Markdown formatting
    - URLs and links
    """
    content_hash = get_content_hash(request.content)
    cache_key = get_cache_key(request.chapter_id, content_hash)

    # Check cache
    cached = get_cached_translation(cache_key)
    if cached:
        return TranslationResponse(
            original_content=request.content,
            translated_content=cached,
            cached=True
        )

    try:
        translated = await translate_with_groq(
            request.content,
            request.preserve_code
        )

        # Cache result
        set_cached_translation(cache_key, translated)

        return TranslationResponse(
            original_content=request.content,
            translated_content=translated,
            cached=False
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/cache/{chapter_id}")
async def clear_translation_cache(chapter_id: str):
    """Clear cached translations for a chapter."""
    keys_to_delete = [
        key for key in _translation_cache.keys()
        if f":{chapter_id}:" in key
    ]
    for key in keys_to_delete:
        del _translation_cache[key]

    return {"cleared": len(keys_to_delete)}


@router.get("/health")
async def translate_health():
    """Check translation service health."""
    return {
        "status": "healthy",
        "groq_configured": bool(GROQ_API_KEY),
        "cache_size": len(_translation_cache),
        "supported_languages": ["ur"]
    }
