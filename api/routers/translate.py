"""
Translation API endpoint for translating content to Urdu.
Uses Gemini API for high-quality contextual translation.
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional
import logging
import google.generativeai as genai

from config import get_settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["translate"])

# Translation prompt template
TRANSLATION_PROMPT = """You are an expert translator specializing in technical content translation from English to Urdu.
Your task is to translate the following educational content about Physical AI and Robotics into Urdu.

Guidelines:
1. Maintain technical accuracy - keep technical terms that don't have good Urdu equivalents in English (like "ROS", "API", "Python", etc.)
2. Use formal Urdu suitable for educational content
3. Preserve any code blocks, commands, or file paths in their original form
4. Keep formatting (headers, bullet points, etc.) intact
5. Make the translation natural and readable for Urdu speakers
6. For concepts that are universally known by English names (like "robot", "sensor", etc.), you may keep the English term with Urdu transliteration in parentheses

Chapter Title: {chapter_title}

Content to translate:
{content}

Provide only the Urdu translation, maintaining the same structure and formatting as the original."""


class TranslateRequest(BaseModel):
    content: str
    target_language: str = "ur"  # Default to Urdu
    chapter_title: Optional[str] = None


class TranslateResponse(BaseModel):
    translated_content: str
    source_language: str
    target_language: str


# Cache for translations to avoid repeated API calls
_translation_cache: dict[str, str] = {}


def get_cache_key(content: str, target_lang: str) -> str:
    """Generate cache key for translation."""
    import hashlib
    content_hash = hashlib.md5(content.encode()).hexdigest()[:16]
    return f"{target_lang}:{content_hash}"


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to the specified language (default: Urdu).
    Uses Gemini API for contextual, high-quality translation.
    """
    if not request.content.strip():
        raise HTTPException(status_code=400, detail="Content cannot be empty")

    # Currently only support Urdu
    if request.target_language != "ur":
        raise HTTPException(
            status_code=400,
            detail="Currently only Urdu (ur) translation is supported"
        )

    # Check cache first
    cache_key = get_cache_key(request.content, request.target_language)
    if cache_key in _translation_cache:
        logger.info(f"Returning cached translation for key: {cache_key[:20]}...")
        return TranslateResponse(
            translated_content=_translation_cache[cache_key],
            source_language="en",
            target_language=request.target_language,
        )

    try:
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)

        model = genai.GenerativeModel(model_name=settings.chat_model)

        prompt = TRANSLATION_PROMPT.format(
            chapter_title=request.chapter_title or "Untitled",
            content=request.content,
        )

        response = model.generate_content(prompt)
        translated_text = response.text

        # Cache the translation
        _translation_cache[cache_key] = translated_text

        logger.info(f"Successfully translated {len(request.content)} chars to Urdu")

        return TranslateResponse(
            translated_content=translated_text,
            source_language="en",
            target_language=request.target_language,
        )

    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to translate content. Please try again."
        )
