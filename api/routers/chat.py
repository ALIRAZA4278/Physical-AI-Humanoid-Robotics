"""
Chat API endpoint for RAG-powered Q&A.
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
import logging

from models.schemas import ChatRequest, ChatResponse
from models.database import get_db
from services.rag import RAGService

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["chat"])

# Singleton RAG service
_rag_service = None


def get_rag_service() -> RAGService:
    """Get or create RAG service singleton."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: Session = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service),
):
    """
    Send a message to the RAG chatbot.

    - **message**: The user's question
    - **session_id**: Anonymous session identifier for history
    - **conversation_id**: Optional existing conversation to continue
    - **selected_text**: Optional user-selected text for context
    """
    try:
        result = rag_service.chat(
            query=request.message,
            session_id=request.session_id,
            db=db,
            conversation_id=request.conversation_id,
            selected_text=request.selected_text,
        )

        return ChatResponse(
            response=result["response"],
            sources=result["sources"],
            conversation_id=result["conversation_id"],
        )

    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your message. Please try again."
        )


@router.get("/conversations/{conversation_id}/messages")
async def get_conversation_messages(
    conversation_id: str,
    db: Session = Depends(get_db),
):
    """
    Get all messages in a conversation.

    Useful for restoring chat history on page reload.
    """
    from models.database import Conversation, MessageModel
    import uuid

    try:
        conv_uuid = uuid.UUID(conversation_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid conversation ID")

    conversation = db.query(Conversation).filter(
        Conversation.id == conv_uuid
    ).first()

    if not conversation:
        raise HTTPException(status_code=404, detail="Conversation not found")

    messages = db.query(MessageModel).filter(
        MessageModel.conversation_id == conv_uuid
    ).order_by(
        MessageModel.created_at.asc()
    ).all()

    return {
        "conversation_id": conversation_id,
        "messages": [
            {
                "id": str(msg.id),
                "role": msg.role,
                "content": msg.content,
                "context": msg.context,
                "created_at": msg.created_at.isoformat(),
            }
            for msg in messages
        ]
    }
