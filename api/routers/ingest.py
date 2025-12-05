"""
Document ingestion API endpoint.
Processes markdown files and creates embeddings in Qdrant.
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Optional
import logging
import os

from models.schemas import IngestResponse
from services.embeddings import EmbeddingService
from services.retriever import RetrieverService
from utils.markdown_loader import MarkdownLoader
from utils.chunker import DocumentChunker
from config import get_settings

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["ingest"])

# Track ingestion status
_ingestion_status = {
    "in_progress": False,
    "last_result": None,
}


@router.post("/ingest", response_model=IngestResponse)
async def ingest_documents(
    background_tasks: BackgroundTasks,
    force: bool = False,
    docs_path: Optional[str] = None,
):
    """
    Ingest all documentation into vector database.

    - **force**: If true, clears existing embeddings before ingestion
    - **docs_path**: Optional custom path to docs directory

    This endpoint triggers background ingestion and returns immediately.
    Check /api/ingest/status for progress.
    """
    if _ingestion_status["in_progress"]:
        raise HTTPException(
            status_code=409,
            detail="Ingestion already in progress. Check /api/ingest/status"
        )

    # Determine docs path
    if docs_path is None:
        # Default to ../docs relative to api directory
        api_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        docs_path = os.path.join(os.path.dirname(api_dir), "docs")

    if not os.path.exists(docs_path):
        raise HTTPException(
            status_code=400,
            detail=f"Docs path does not exist: {docs_path}"
        )

    # Start background ingestion
    background_tasks.add_task(run_ingestion, docs_path, force)

    return IngestResponse(
        status="started",
        documents_processed=0,
        chunks_created=0,
        errors=["Ingestion started in background. Check /api/ingest/status"]
    )


@router.get("/ingest/status")
async def get_ingestion_status():
    """Get current ingestion status."""
    return {
        "in_progress": _ingestion_status["in_progress"],
        "last_result": _ingestion_status["last_result"],
    }


def run_ingestion(docs_path: str, force: bool = False):
    """
    Run the full ingestion pipeline.

    Args:
        docs_path: Path to documentation directory
        force: If true, clear existing data first
    """
    global _ingestion_status

    _ingestion_status["in_progress"] = True
    errors = []

    try:
        logger.info(f"Starting ingestion from: {docs_path}")

        # Initialize services
        embedding_service = EmbeddingService()
        retriever = RetrieverService(embedding_service)
        loader = MarkdownLoader(docs_path)
        chunker = DocumentChunker()

        # Ensure collection exists
        retriever.ensure_collection()

        # Clear existing data if force
        if force:
            logger.info("Force flag set - clearing existing data")
            retriever.delete_all()

        # Load documents
        logger.info("Loading documents...")
        documents = loader.load_all_documents()

        if not documents:
            raise ValueError("No documents found to ingest")

        logger.info(f"Loaded {len(documents)} documents")

        # Chunk documents
        logger.info("Chunking documents...")
        chunks = chunker.chunk_documents(documents)
        logger.info(f"Created {len(chunks)} chunks")

        # Generate embeddings
        logger.info("Generating embeddings...")
        chunk_texts = [chunk["content"] for chunk in chunks]
        embeddings = embedding_service.embed_batch(chunk_texts)
        logger.info(f"Generated {len(embeddings)} embeddings")

        # Upsert to Qdrant
        logger.info("Upserting to vector database...")
        retriever.upsert_chunks(chunks, embeddings)

        _ingestion_status["last_result"] = {
            "status": "success",
            "documents_processed": len(documents),
            "chunks_created": len(chunks),
            "errors": errors,
        }

        logger.info("Ingestion completed successfully")

    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        errors.append(str(e))
        _ingestion_status["last_result"] = {
            "status": "failed",
            "documents_processed": 0,
            "chunks_created": 0,
            "errors": errors,
        }

    finally:
        _ingestion_status["in_progress"] = False


@router.post("/ingest/sync", response_model=IngestResponse)
async def ingest_documents_sync(
    force: bool = False,
    docs_path: Optional[str] = None,
):
    """
    Synchronous ingestion endpoint (waits for completion).

    Use this for testing or when you need to wait for ingestion.
    For production, prefer the async /api/ingest endpoint.
    """
    if _ingestion_status["in_progress"]:
        raise HTTPException(
            status_code=409,
            detail="Ingestion already in progress"
        )

    # Determine docs path
    if docs_path is None:
        api_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        docs_path = os.path.join(os.path.dirname(api_dir), "docs")

    if not os.path.exists(docs_path):
        raise HTTPException(
            status_code=400,
            detail=f"Docs path does not exist: {docs_path}"
        )

    # Run synchronously
    run_ingestion(docs_path, force)

    result = _ingestion_status["last_result"]
    return IngestResponse(**result)
