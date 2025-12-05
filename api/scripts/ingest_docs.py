#!/usr/bin/env python3
"""
Standalone script to ingest documentation into vector database.

Usage:
    python scripts/ingest_docs.py [--force] [--docs-path PATH]

Options:
    --force       Clear existing embeddings before ingestion
    --docs-path   Custom path to docs directory (default: ../docs)
"""

import sys
import os
import argparse
import logging

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv
load_dotenv()

from services.embeddings import EmbeddingService
from services.retriever import RetrieverService
from utils.markdown_loader import MarkdownLoader
from utils.chunker import DocumentChunker

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(
        description="Ingest documentation into vector database"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Clear existing embeddings before ingestion"
    )
    parser.add_argument(
        "--docs-path",
        type=str,
        default=None,
        help="Custom path to docs directory"
    )
    args = parser.parse_args()

    # Determine docs path
    if args.docs_path:
        docs_path = args.docs_path
    else:
        # Default: ../../docs relative to this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        api_dir = os.path.dirname(script_dir)
        project_dir = os.path.dirname(api_dir)
        docs_path = os.path.join(project_dir, "docs")

    if not os.path.exists(docs_path):
        logger.error(f"Docs path does not exist: {docs_path}")
        sys.exit(1)

    logger.info(f"Starting ingestion from: {docs_path}")
    logger.info(f"Force mode: {args.force}")

    try:
        # Initialize services
        logger.info("Initializing services...")
        embedding_service = EmbeddingService()
        retriever = RetrieverService(embedding_service)
        loader = MarkdownLoader(docs_path)
        chunker = DocumentChunker()

        # Ensure collection exists
        logger.info("Ensuring vector collection exists...")
        retriever.ensure_collection()

        # Clear existing data if force
        if args.force:
            logger.info("Clearing existing embeddings...")
            retriever.delete_all()

        # Load documents
        logger.info("Loading documents...")
        documents = loader.load_all_documents()

        if not documents:
            logger.error("No documents found to ingest")
            sys.exit(1)

        logger.info(f"Loaded {len(documents)} documents:")
        for doc in documents:
            logger.info(f"  - {doc['source_path']} ({doc['title']})")

        # Chunk documents
        logger.info("Chunking documents...")
        chunks = chunker.chunk_documents(documents)
        logger.info(f"Created {len(chunks)} chunks")

        # Show chunk statistics
        total_tokens = sum(c.get("token_count", 0) for c in chunks)
        avg_tokens = total_tokens / len(chunks) if chunks else 0
        logger.info(f"Total tokens: {total_tokens}, Average per chunk: {avg_tokens:.1f}")

        # Generate embeddings
        logger.info("Generating embeddings (this may take a while)...")
        chunk_texts = [chunk["content"] for chunk in chunks]
        embeddings = embedding_service.embed_batch(chunk_texts)
        logger.info(f"Generated {len(embeddings)} embeddings")

        # Upsert to Qdrant
        logger.info("Upserting to vector database...")
        retriever.upsert_chunks(chunks, embeddings)

        logger.info("=" * 50)
        logger.info("INGESTION COMPLETED SUCCESSFULLY")
        logger.info(f"  Documents processed: {len(documents)}")
        logger.info(f"  Chunks created: {len(chunks)}")
        logger.info(f"  Embeddings generated: {len(embeddings)}")
        logger.info("=" * 50)

    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
