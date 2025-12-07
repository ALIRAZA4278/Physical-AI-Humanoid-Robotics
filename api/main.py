"""
FastAPI application for RAG chatbot backend.

Physical AI & Humanoid Robotics - Documentation Chatbot API
"""

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import logging
import traceback

from config import get_settings
from models.database import init_db
from models.schemas import HealthResponse
from routers import chat_router, ingest_router, auth_router, translate_router, personalize_router

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events."""
    # Startup
    logger.info("Starting RAG Chatbot API...")
    try:
        init_db()
        logger.info("Database initialized")
    except Exception as e:
        logger.error(f"Failed to initialize database: {e}")
        # Don't fail startup - DB might not be configured yet

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API...")


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics - RAG Chatbot API",
    description="""
    A Retrieval-Augmented Generation (RAG) chatbot API for the Physical AI & Humanoid Robotics documentation.

    ## Features
    - **Chat**: Ask questions about the book content
    - **Context-aware**: Provide selected text for more specific answers
    - **Conversation history**: Maintain context across messages
    - **Source citations**: Get references to relevant documentation sections

    ## Endpoints
    - `POST /api/chat` - Send a message to the chatbot
    - `GET /api/conversations/{id}/messages` - Get conversation history
    - `POST /api/ingest` - Ingest documentation into vector database
    - `GET /api/health` - Check API health status
    """,
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
settings = get_settings()
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router)
app.include_router(ingest_router)
app.include_router(auth_router)
app.include_router(translate_router)
app.include_router(personalize_router)


# Error handling middleware
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler with graceful fallback messages.

    Catches all unhandled exceptions and returns user-friendly error responses
    while logging the full error details for debugging.
    """
    # Log the full error with traceback
    logger.error(f"Unhandled exception on {request.method} {request.url.path}")
    logger.error(f"Error type: {type(exc).__name__}")
    logger.error(f"Error message: {str(exc)}")
    logger.error(f"Traceback: {traceback.format_exc()}")

    # Determine error category and provide user-friendly message
    error_type = type(exc).__name__

    # Map specific errors to user-friendly messages
    error_messages = {
        "ConnectionError": "Unable to connect to external services. Please try again later.",
        "TimeoutError": "The request timed out. Please try again with a shorter question.",
        "RateLimitError": "Too many requests. Please wait a moment before trying again.",
        "AuthenticationError": "API authentication failed. Please contact support.",
        "ValidationError": "Invalid request format. Please check your input.",
    }

    # Get user-friendly message or use default
    user_message = error_messages.get(
        error_type,
        "An unexpected error occurred. Our team has been notified."
    )

    # Return structured error response
    return JSONResponse(
        status_code=500,
        content={
            "error": True,
            "message": user_message,
            "error_type": error_type if settings.environment == "development" else "ServerError",
            "detail": str(exc) if settings.environment == "development" else None,
        },
    )


@app.get("/api/health", response_model=HealthResponse, tags=["health"])
async def health_check():
    """
    Check health status of all API dependencies.

    Returns status of:
    - Qdrant vector database connection
    - PostgreSQL database connection
    - Google Gemini API configuration
    """
    from services.rag import RAGService
    from models.database import get_engine

    rag_health = {"qdrant": False, "gemini": False}
    postgres_status = "disconnected"

    try:
        rag_service = RAGService()
        rag_health = rag_service.health_check()
    except Exception as e:
        logger.error(f"RAG health check failed: {e}")

    try:
        from sqlalchemy import text
        engine = get_engine()
        with engine.connect() as conn:
            conn.execute(text("SELECT 1"))
        postgres_status = "connected"
    except Exception as e:
        logger.error(f"Postgres health check failed: {e}")
        postgres_status = "disconnected"

    overall_status = "healthy" if all([
        rag_health.get("qdrant"),
        rag_health.get("gemini"),
        postgres_status == "connected"
    ]) else "degraded"

    return HealthResponse(
        status=overall_status,
        qdrant="connected" if rag_health.get("qdrant") else "disconnected",
        postgres=postgres_status,
        gemini="configured" if rag_health.get("gemini") else "not configured",
        version="1.0.0"
    )


@app.get("/", tags=["root"])
async def root():
    """Root endpoint with API information."""
    return {
        "name": "Physical AI & Humanoid Robotics - RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health",
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
    )
