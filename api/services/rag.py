"""
RAG orchestration service - combines retrieval and generation.
"""

from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session
import logging
import uuid

from .embeddings import EmbeddingService
from .retriever import RetrieverService
from .generator import GeneratorService
from models.database import Conversation, MessageModel
from models.schemas import Source

logger = logging.getLogger(__name__)


class RAGService:
    """Orchestrates the RAG pipeline: retrieve relevant context and generate responses."""

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.retriever = RetrieverService(self.embedding_service)
        self.generator = GeneratorService()

    def chat(
        self,
        query: str,
        session_id: str,
        db: Session,
        conversation_id: Optional[str] = None,
        selected_text: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Process a chat message through the RAG pipeline.

        Args:
            query: User's question
            session_id: Anonymous session identifier
            db: Database session
            conversation_id: Existing conversation ID (optional)
            selected_text: User-selected text context (optional)

        Returns:
            Dictionary with response, sources, and conversation_id
        """
        # Get or create conversation
        conversation = self._get_or_create_conversation(
            db, session_id, conversation_id
        )

        # Get conversation history
        history = self._get_conversation_history(db, conversation.id)

        # Retrieve relevant chunks
        chunks = self.retriever.retrieve(
            query=query,
            selected_text=selected_text,
        )

        # Generate response
        response = self.generator.generate(
            query=query,
            context_chunks=chunks,
            selected_text=selected_text,
            conversation_history=history,
        )

        # Format sources
        sources = [
            Source(
                path=chunk["source_path"],
                title=chunk["title"],
                module=chunk["module"],
                relevance=chunk["relevance"],
                snippet=chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
            )
            for chunk in chunks
        ]

        # Save messages to database
        self._save_message(db, conversation.id, "user", query, {"selected_text": selected_text})
        self._save_message(db, conversation.id, "assistant", response, {"sources": [s.model_dump() for s in sources]})

        return {
            "response": response,
            "sources": sources,
            "conversation_id": str(conversation.id),
        }

    def _get_or_create_conversation(
        self,
        db: Session,
        session_id: str,
        conversation_id: Optional[str] = None,
    ) -> Conversation:
        """Get existing conversation or create new one."""
        if conversation_id:
            try:
                conv_uuid = uuid.UUID(conversation_id)
                conversation = db.query(Conversation).filter(
                    Conversation.id == conv_uuid
                ).first()
                if conversation:
                    return conversation
            except (ValueError, TypeError):
                logger.warning(f"Invalid conversation_id: {conversation_id}")

        # Create new conversation
        conversation = Conversation(session_id=session_id)
        db.add(conversation)
        db.commit()
        db.refresh(conversation)
        return conversation

    def _get_conversation_history(
        self,
        db: Session,
        conversation_id: uuid.UUID,
        limit: int = 10,
    ) -> List[Dict[str, str]]:
        """Get recent messages from conversation."""
        messages = db.query(MessageModel).filter(
            MessageModel.conversation_id == conversation_id
        ).order_by(
            MessageModel.created_at.desc()
        ).limit(limit).all()

        # Reverse to get chronological order
        messages.reverse()

        return [
            {"role": msg.role, "content": msg.content}
            for msg in messages
        ]

    def _save_message(
        self,
        db: Session,
        conversation_id: uuid.UUID,
        role: str,
        content: str,
        context: Optional[Dict] = None,
    ):
        """Save a message to the database."""
        message = MessageModel(
            conversation_id=conversation_id,
            role=role,
            content=content,
            context=context,
        )
        db.add(message)
        db.commit()

    def health_check(self) -> Dict[str, bool]:
        """Check health of all RAG components."""
        return {
            "qdrant": self.retriever.health_check(),
            "gemini": self.generator.health_check(),
        }
