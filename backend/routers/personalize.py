"""
Personalization Router - Groq Llama 3

Provides content personalization based on user profile:
- Adapts chapter content to user experience level
- Highlights topics matching user interests
- Caches personalized content for performance
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, Dict
import os
import hashlib
import json
import httpx
from datetime import datetime, timedelta

router = APIRouter()

# Configuration
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")
CACHE_TTL_HOURS = 24

# Simple in-memory cache (use Redis in production)
_personalization_cache: Dict[str, dict] = {}


class PersonalizationRequest(BaseModel):
    """Request to personalize content."""
    content: str = Field(..., min_length=100, max_length=50000)
    chapter_id: str = Field(..., min_length=1)
    user_profile: dict = Field(...)


class PersonalizationResponse(BaseModel):
    """Personalized content response."""
    original_content: str
    personalized_content: str
    adaptations_made: list[str]
    cached: bool = False


def get_cache_key(chapter_id: str, user_profile: dict) -> str:
    """Generate cache key from chapter and user profile."""
    profile_hash = hashlib.md5(
        json.dumps(user_profile, sort_keys=True).encode()
    ).hexdigest()[:8]
    return f"{chapter_id}:{profile_hash}"


def get_cached_personalization(cache_key: str) -> Optional[dict]:
    """Get cached personalization if valid."""
    if cache_key in _personalization_cache:
        cached = _personalization_cache[cache_key]
        if datetime.now() < cached["expires_at"]:
            return cached["data"]
        else:
            del _personalization_cache[cache_key]
    return None


def set_cached_personalization(cache_key: str, data: dict):
    """Cache personalization result."""
    _personalization_cache[cache_key] = {
        "data": data,
        "expires_at": datetime.now() + timedelta(hours=CACHE_TTL_HOURS)
    }


async def personalize_with_groq(
    content: str,
    user_profile: dict
) -> tuple[str, list[str]]:
    """Personalize content using Groq Llama 3."""
    if not GROQ_API_KEY:
        return content, ["No personalization (API not configured)"]

    experience_level = user_profile.get("experience_level", "intermediate")
    interests = user_profile.get("interests", [])

    system_prompt = f"""You are an expert technical writer adapting robotics content for different audiences.

User Profile:
- Experience Level: {experience_level}
- Interests: {', '.join(interests) if interests else 'General robotics'}

Adaptation Guidelines:
- For beginners: Add more explanations, simplify jargon, include analogies
- For intermediate: Balance theory and practice, add context
- For advanced: Focus on implementation details, edge cases, optimizations

Emphasize topics related to user interests where relevant.
Maintain technical accuracy. Keep the same structure and headings.
Return ONLY the adapted content, no meta-commentary."""

    user_prompt = f"""Adapt this chapter content for the user profile above:

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
                "temperature": 0.3
            },
            timeout=60.0
        )

        if response.status_code != 200:
            raise HTTPException(
                status_code=502,
                detail=f"Personalization service error: {response.text}"
            )

        result = response.json()
        personalized = result["choices"][0]["message"]["content"]

        adaptations = [
            f"Adapted for {experience_level} level",
        ]
        if interests:
            adaptations.append(f"Emphasized: {', '.join(interests[:3])}")

        return personalized, adaptations


@router.post("/content", response_model=PersonalizationResponse)
async def personalize_content(request: PersonalizationRequest):
    """
    Personalize chapter content based on user profile.

    Adapts explanations, examples, and emphasis based on:
    - User experience level (beginner/intermediate/advanced)
    - User interests and goals
    """
    # Check cache
    cache_key = get_cache_key(request.chapter_id, request.user_profile)
    cached = get_cached_personalization(cache_key)

    if cached:
        return PersonalizationResponse(
            original_content=request.content,
            personalized_content=cached["personalized_content"],
            adaptations_made=cached["adaptations_made"],
            cached=True
        )

    try:
        personalized_content, adaptations = await personalize_with_groq(
            request.content,
            request.user_profile
        )

        # Cache result
        cache_data = {
            "personalized_content": personalized_content,
            "adaptations_made": adaptations
        }
        set_cached_personalization(cache_key, cache_data)

        return PersonalizationResponse(
            original_content=request.content,
            personalized_content=personalized_content,
            adaptations_made=adaptations,
            cached=False
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/cache/{chapter_id}")
async def clear_chapter_cache(chapter_id: str):
    """Clear cached personalizations for a chapter."""
    keys_to_delete = [
        key for key in _personalization_cache.keys()
        if key.startswith(f"{chapter_id}:")
    ]
    for key in keys_to_delete:
        del _personalization_cache[key]

    return {"cleared": len(keys_to_delete)}


@router.get("/health")
async def personalize_health():
    """Check personalization service health."""
    return {
        "status": "healthy",
        "groq_configured": bool(GROQ_API_KEY),
        "cache_size": len(_personalization_cache)
    }
