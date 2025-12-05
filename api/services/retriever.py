"""
Retriever service using Qdrant vector database.
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
import uuid
import hashlib

from config import get_settings
from .embeddings import EmbeddingService

logger = logging.getLogger(__name__)


def string_to_uuid(s: str) -> str:
    """Convert a string to a deterministic UUID."""
    # Create a hash of the string and use it to generate a UUID
    hash_bytes = hashlib.md5(s.encode()).hexdigest()
    return str(uuid.UUID(hash_bytes))


class RetrieverService:
    """Service for retrieving relevant document chunks from Qdrant."""

    def __init__(self, embedding_service: Optional[EmbeddingService] = None):
        settings = get_settings()
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection
        self.embedding_service = embedding_service or EmbeddingService()
        self.top_k = settings.retrieval_top_k

    def ensure_collection(self):
        """Create collection if it doesn't exist."""
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.embedding_service.get_embedding_dimension(),
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Created collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error ensuring collection: {e}")
            raise

    def upsert_chunks(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]]):
        """
        Insert or update document chunks with embeddings.

        Args:
            chunks: List of chunk dictionaries with metadata
            embeddings: Corresponding embeddings for each chunk
        """
        if len(chunks) != len(embeddings):
            raise ValueError("Number of chunks must match number of embeddings")

        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Convert string ID to UUID for Qdrant compatibility
            chunk_id = chunk.get("id", str(i))
            point_id = string_to_uuid(str(chunk_id))

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": chunk["content"],
                    "source_path": chunk["source_path"],
                    "title": chunk.get("title", ""),
                    "module": chunk.get("module", ""),
                    "heading": chunk.get("heading", ""),
                    "original_id": chunk_id,  # Store original ID for reference
                }
            )
            points.append(point)

        # Upsert in batches of 100
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
            )
            logger.info(f"Upserted batch {i // batch_size + 1} ({len(batch)} points)")

    def retrieve(
        self,
        query: str,
        selected_text: Optional[str] = None,
        top_k: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks for a query.

        Args:
            query: User's question
            selected_text: Optional user-selected text for additional context
            top_k: Number of results to return (defaults to config value)

        Returns:
            List of relevant chunks with metadata and scores
        """
        k = top_k or self.top_k

        # Generate query embedding
        query_embedding = self.embedding_service.embed_text(query)

        # Search for similar chunks using query_points (qdrant-client >= 1.7)
        search_result = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=k,
        )
        results = search_result.points

        # If selected text provided, also search with that
        if selected_text:
            selected_embedding = self.embedding_service.embed_text(selected_text)
            selected_search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=selected_embedding,
                limit=k // 2,  # Get fewer results for selected text
            )
            selected_results = selected_search_result.points
            # Merge results, prioritizing selected text matches
            results = self._merge_results(results, selected_results)

        # Format results
        formatted_results = []
        for result in results[:k]:
            formatted_results.append({
                "content": result.payload.get("content", ""),
                "source_path": result.payload.get("source_path", ""),
                "title": result.payload.get("title", ""),
                "module": result.payload.get("module", ""),
                "heading": result.payload.get("heading", ""),
                "relevance": result.score,
            })

        return formatted_results

    def _merge_results(
        self,
        primary_results: List,
        secondary_results: List
    ) -> List:
        """Merge and deduplicate search results."""
        seen_ids = set()
        merged = []

        # Add secondary (selected text) results first with boosted score
        for result in secondary_results:
            if result.id not in seen_ids:
                result.score = min(result.score * 1.2, 1.0)  # Boost selected text matches
                merged.append(result)
                seen_ids.add(result.id)

        # Add primary results
        for result in primary_results:
            if result.id not in seen_ids:
                merged.append(result)
                seen_ids.add(result.id)

        # Sort by score
        merged.sort(key=lambda x: x.score, reverse=True)
        return merged

    def delete_all(self):
        """Delete all points in the collection (for re-ingestion)."""
        try:
            self.client.delete_collection(self.collection_name)
            self.ensure_collection()
            logger.info(f"Cleared collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error clearing collection: {e}")
            raise

    def health_check(self) -> bool:
        """Check if Qdrant is accessible."""
        try:
            self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False
