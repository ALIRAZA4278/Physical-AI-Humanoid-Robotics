"""
Embedding service using Google Gemini text-embedding-004.
"""

import warnings
# Suppress deprecation warning for google.generativeai (will migrate to google.genai in future)
warnings.filterwarnings("ignore", message=".*google.generativeai.*", category=FutureWarning)

import google.generativeai as genai
from typing import List
import logging

from config import get_settings

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for generating text embeddings using Google Gemini."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)
        self.model = settings.embedding_model

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        try:
            all_embeddings = []
            # Process in batches of 100 for safety
            batch_size = 100

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                for text in batch:
                    result = genai.embed_content(
                        model=self.model,
                        content=text,
                        task_type="retrieval_document"
                    )
                    all_embeddings.append(result['embedding'])

            return all_embeddings
        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

    def get_embedding_dimension(self) -> int:
        """Return the embedding dimension for the current model."""
        # Gemini text-embedding-004 produces 768-dimensional vectors
        return 768
