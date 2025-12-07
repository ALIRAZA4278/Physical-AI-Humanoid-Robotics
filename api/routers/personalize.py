"""
Personalization API endpoint for adapting content to user's background.
Uses Gemini API to rewrite content based on user's software and hardware experience.
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional
import logging
import google.generativeai as genai

from config import get_settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["personalize"])


# Personalization prompt template
PERSONALIZATION_PROMPT = """You are an expert technical educator. Your task is to adapt the following educational content about Physical AI and Robotics to match the reader's background and experience level.

Reader's Background:
- Software Experience: {software_background}
- Hardware/Robotics Experience: {hardware_background}
- Target Experience Level: {experience_level}

Adaptation Guidelines:

For BEGINNERS (no experience):
- Explain all technical terms when first introduced
- Use analogies to everyday concepts
- Provide step-by-step explanations
- Add "Why this matters" context
- Include more examples

For INTERMEDIATE users (some experience):
- Assume familiarity with basic concepts
- Focus on practical applications
- Connect to related technologies they might know
- Provide optimization tips

For ADVANCED users (professional experience):
- Be concise and technical
- Focus on advanced patterns and best practices
- Include performance considerations
- Reference industry standards
- Add links to deeper resources

Personalization based on background:
- If software background is strong but hardware is weak: Add more hardware context and explanations
- If hardware background is strong but software is weak: Add more programming context
- Balance the explanation to build on their strengths

Chapter Title: {chapter_title}

Original Content:
{content}

Provide the personalized version of this content. Maintain the same overall structure but adapt the explanations, examples, and depth to match the reader's background. Use markdown formatting."""


class PersonalizeRequest(BaseModel):
    content: str
    chapter_title: Optional[str] = None
    user_software_background: str
    user_hardware_background: str
    experience_level: str = "beginner"


class PersonalizeResponse(BaseModel):
    personalized_content: str
    adapted_for: dict


# Cache for personalized content
_personalization_cache: dict[str, str] = {}


def get_cache_key(content: str, sw_bg: str, hw_bg: str, level: str) -> str:
    """Generate cache key for personalization."""
    import hashlib
    content_hash = hashlib.md5(content.encode()).hexdigest()[:12]
    return f"{sw_bg}:{hw_bg}:{level}:{content_hash}"


# Map background selections to descriptive text
BACKGROUND_DESCRIPTIONS = {
    "none": "no prior experience",
    "beginner": "basic understanding, learning fundamentals",
    "hobbyist": "hobbyist level, DIY projects and experimentation",
    "student": "academic coursework and structured learning",
    "intermediate": "solid foundation with practical project experience",
    "professional": "professional work experience",
    "advanced": "extensive professional experience with complex systems",
    "expert": "expert level with deep specialized knowledge",
}


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize content based on user's software and hardware background.
    Adapts explanations, examples, and technical depth to match user's experience.
    """
    if not request.content.strip():
        raise HTTPException(status_code=400, detail="Content cannot be empty")

    # Get descriptive backgrounds
    sw_desc = BACKGROUND_DESCRIPTIONS.get(
        request.user_software_background.lower(),
        request.user_software_background
    )
    hw_desc = BACKGROUND_DESCRIPTIONS.get(
        request.user_hardware_background.lower(),
        request.user_hardware_background
    )

    # Check cache
    cache_key = get_cache_key(
        request.content,
        request.user_software_background,
        request.user_hardware_background,
        request.experience_level,
    )

    if cache_key in _personalization_cache:
        logger.info(f"Returning cached personalization for key: {cache_key[:30]}...")
        return PersonalizeResponse(
            personalized_content=_personalization_cache[cache_key],
            adapted_for={
                "software_background": request.user_software_background,
                "hardware_background": request.user_hardware_background,
                "experience_level": request.experience_level,
            },
        )

    try:
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)

        model = genai.GenerativeModel(model_name=settings.chat_model)

        prompt = PERSONALIZATION_PROMPT.format(
            software_background=sw_desc,
            hardware_background=hw_desc,
            experience_level=request.experience_level,
            chapter_title=request.chapter_title or "Untitled",
            content=request.content,
        )

        response = model.generate_content(prompt)
        personalized_text = response.text

        # Cache the personalization
        _personalization_cache[cache_key] = personalized_text

        logger.info(
            f"Personalized {len(request.content)} chars for "
            f"SW:{request.user_software_background}/HW:{request.user_hardware_background}"
        )

        return PersonalizeResponse(
            personalized_content=personalized_text,
            adapted_for={
                "software_background": request.user_software_background,
                "hardware_background": request.user_hardware_background,
                "experience_level": request.experience_level,
            },
        )

    except Exception as e:
        logger.error(f"Personalization error: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to personalize content. Please try again."
        )
