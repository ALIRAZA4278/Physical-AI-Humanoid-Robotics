"""API routers package."""

from .chat import router as chat_router
from .ingest import router as ingest_router
from .auth import router as auth_router
from .translate import router as translate_router
from .personalize import router as personalize_router

__all__ = [
    "chat_router",
    "ingest_router",
    "auth_router",
    "translate_router",
    "personalize_router",
]
