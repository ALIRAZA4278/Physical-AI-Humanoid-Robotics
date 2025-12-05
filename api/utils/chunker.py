"""
Document chunking for RAG pipeline.
Splits documents into overlapping chunks optimized for retrieval.
"""

from typing import List, Dict, Any
import tiktoken
import re
import logging
import hashlib

from config import get_settings

logger = logging.getLogger(__name__)


class DocumentChunker:
    """Split documents into overlapping chunks for embedding."""

    def __init__(self):
        settings = get_settings()
        self.chunk_size = settings.chunk_size  # tokens
        self.chunk_overlap = settings.chunk_overlap  # tokens
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def chunk_documents(self, documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Chunk all documents into smaller pieces.

        Args:
            documents: List of document dictionaries

        Returns:
            List of chunk dictionaries with metadata
        """
        all_chunks = []

        for doc in documents:
            chunks = self.chunk_document(doc)
            all_chunks.extend(chunks)

        logger.info(f"Created {len(all_chunks)} chunks from {len(documents)} documents")
        return all_chunks

    def chunk_document(self, document: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Chunk a single document.

        Args:
            document: Document dictionary with content and metadata

        Returns:
            List of chunk dictionaries
        """
        content = document.get("content", "")
        if not content:
            return []

        # Split by paragraphs first
        paragraphs = self._split_into_paragraphs(content)

        # Group paragraphs into chunks
        chunks = self._create_chunks(paragraphs, document)

        return chunks

    def _split_into_paragraphs(self, content: str) -> List[str]:
        """Split content into paragraphs."""
        # Split on double newlines or heading markers
        parts = re.split(r"\n\n+", content)

        # Filter empty parts
        paragraphs = [p.strip() for p in parts if p.strip()]

        return paragraphs

    def _create_chunks(
        self,
        paragraphs: List[str],
        document: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        """
        Create overlapping chunks from paragraphs.

        Args:
            paragraphs: List of paragraph strings
            document: Original document for metadata

        Returns:
            List of chunk dictionaries
        """
        chunks = []
        current_chunk_paragraphs = []
        current_tokens = 0

        for paragraph in paragraphs:
            para_tokens = self._count_tokens(paragraph)

            # If single paragraph exceeds chunk size, split it
            if para_tokens > self.chunk_size:
                # First, save current chunk if any
                if current_chunk_paragraphs:
                    chunk = self._create_chunk_dict(
                        current_chunk_paragraphs, document, len(chunks)
                    )
                    chunks.append(chunk)
                    current_chunk_paragraphs = []
                    current_tokens = 0

                # Split large paragraph
                sub_chunks = self._split_large_paragraph(paragraph, document, len(chunks))
                chunks.extend(sub_chunks)
                continue

            # If adding this paragraph exceeds limit, save current and start new
            if current_tokens + para_tokens > self.chunk_size and current_chunk_paragraphs:
                chunk = self._create_chunk_dict(
                    current_chunk_paragraphs, document, len(chunks)
                )
                chunks.append(chunk)

                # Keep last paragraph(s) for overlap
                overlap_paragraphs = self._get_overlap_paragraphs(
                    current_chunk_paragraphs
                )
                current_chunk_paragraphs = overlap_paragraphs
                current_tokens = sum(self._count_tokens(p) for p in overlap_paragraphs)

            current_chunk_paragraphs.append(paragraph)
            current_tokens += para_tokens

        # Don't forget the last chunk
        if current_chunk_paragraphs:
            chunk = self._create_chunk_dict(
                current_chunk_paragraphs, document, len(chunks)
            )
            chunks.append(chunk)

        return chunks

    def _split_large_paragraph(
        self,
        paragraph: str,
        document: Dict[str, Any],
        chunk_offset: int
    ) -> List[Dict[str, Any]]:
        """Split a large paragraph into smaller chunks."""
        chunks = []
        sentences = re.split(r"(?<=[.!?])\s+", paragraph)

        current_sentences = []
        current_tokens = 0

        for sentence in sentences:
            sent_tokens = self._count_tokens(sentence)

            if current_tokens + sent_tokens > self.chunk_size and current_sentences:
                content = " ".join(current_sentences)
                chunk = self._create_chunk_dict_from_content(
                    content, document, chunk_offset + len(chunks)
                )
                chunks.append(chunk)

                # Overlap: keep last sentence
                current_sentences = current_sentences[-1:] if current_sentences else []
                current_tokens = sum(self._count_tokens(s) for s in current_sentences)

            current_sentences.append(sentence)
            current_tokens += sent_tokens

        if current_sentences:
            content = " ".join(current_sentences)
            chunk = self._create_chunk_dict_from_content(
                content, document, chunk_offset + len(chunks)
            )
            chunks.append(chunk)

        return chunks

    def _get_overlap_paragraphs(self, paragraphs: List[str]) -> List[str]:
        """Get paragraphs for overlap based on token count."""
        overlap_paragraphs = []
        overlap_tokens = 0

        for para in reversed(paragraphs):
            para_tokens = self._count_tokens(para)
            if overlap_tokens + para_tokens <= self.chunk_overlap:
                overlap_paragraphs.insert(0, para)
                overlap_tokens += para_tokens
            else:
                break

        return overlap_paragraphs

    def _create_chunk_dict(
        self,
        paragraphs: List[str],
        document: Dict[str, Any],
        index: int
    ) -> Dict[str, Any]:
        """Create a chunk dictionary from paragraphs."""
        content = "\n\n".join(paragraphs)
        return self._create_chunk_dict_from_content(content, document, index)

    def _create_chunk_dict_from_content(
        self,
        content: str,
        document: Dict[str, Any],
        index: int
    ) -> Dict[str, Any]:
        """Create a chunk dictionary from content string."""
        # Generate unique ID based on content hash
        content_hash = hashlib.md5(content.encode()).hexdigest()[:8]
        chunk_id = f"{document['source_path']}_{index}_{content_hash}"

        # Extract heading if present in chunk
        heading = self._extract_heading(content)

        return {
            "id": chunk_id,
            "content": content,
            "source_path": document["source_path"],
            "title": document.get("title", ""),
            "module": document.get("module", ""),
            "heading": heading,
            "token_count": self._count_tokens(content),
        }

    def _extract_heading(self, content: str) -> str:
        """Extract first heading from content if present."""
        match = re.search(r"^#+\s+(.+)$", content, re.MULTILINE)
        return match.group(1).strip() if match else ""

    def _count_tokens(self, text: str) -> int:
        """Count tokens in text using tiktoken."""
        return len(self.tokenizer.encode(text))
