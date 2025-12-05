"""
Pydantic models for API request/response schemas.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from uuid import UUID


class Source(BaseModel):
    """A source document reference."""
    path: str = Field(..., description="Path to the source document")
    title: str = Field(..., description="Title of the section")
    module: str = Field(..., description="Module name (e.g., 'module-1-ros2')")
    relevance: float = Field(..., ge=0, le=1, description="Relevance score 0-1")
    snippet: Optional[str] = Field(None, description="Text snippet from source")


class ChatRequest(BaseModel):
    """Request body for chat endpoint."""
    message: str = Field(..., min_length=1, max_length=4000, description="User message")
    session_id: str = Field(..., description="Anonymous session identifier")
    conversation_id: Optional[str] = Field(None, description="Existing conversation ID")
    selected_text: Optional[str] = Field(None, max_length=5000, description="User-selected text context")

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is a ROS 2 node?",
                "session_id": "anon-12345",
                "conversation_id": None,
                "selected_text": None
            }
        }


class ChatResponse(BaseModel):
    """Response body for chat endpoint."""
    response: str = Field(..., description="Assistant's response")
    sources: List[Source] = Field(default_factory=list, description="Source references")
    conversation_id: str = Field(..., description="Conversation ID for continuity")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "A ROS 2 node is the fundamental building block...",
                "sources": [
                    {
                        "path": "module-1-ros2/01-architecture",
                        "title": "ROS 2 Architecture",
                        "module": "module-1-ros2",
                        "relevance": 0.95,
                        "snippet": "Nodes are the fundamental..."
                    }
                ],
                "conversation_id": "conv-uuid-here"
            }
        }


class Message(BaseModel):
    """A single message in conversation history."""
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    sources: Optional[List[Source]] = None


class IngestResponse(BaseModel):
    """Response body for ingestion endpoint."""
    status: str = Field(..., description="Status of ingestion")
    documents_processed: int = Field(..., description="Number of documents processed")
    chunks_created: int = Field(..., description="Number of chunks created")
    errors: List[str] = Field(default_factory=list, description="Any errors encountered")


class HealthResponse(BaseModel):
    """Response body for health check endpoint."""
    status: str = Field(..., description="Overall health status")
    qdrant: str = Field(..., description="Qdrant connection status")
    postgres: str = Field(..., description="Postgres connection status")
    gemini: str = Field(..., description="Gemini API configuration status")
    version: str = Field(default="1.0.0", description="API version")
