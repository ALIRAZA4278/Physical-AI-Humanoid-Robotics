"""
Generator service using Google Gemini models.
"""

import google.generativeai as genai
from typing import List, Dict, Any, Optional
import logging

from config import get_settings

logger = logging.getLogger(__name__)

# System prompt for RAG generation
SYSTEM_PROMPT = """You are an expert AI teaching assistant for the "Physical AI & Humanoid Robotics" educational book.
Your goal is to help students understand robotics concepts clearly and thoroughly.

## Your Knowledge Domain:
- ROS 2 (Robot Operating System 2) - architecture, nodes, topics, services, actions
- Gazebo simulation - setting up robot simulations
- NVIDIA Isaac - hardware-accelerated perception and simulation
- VLA (Vision-Language-Action) - voice-controlled robotics
- Humanoid robot development and navigation

## Response Guidelines:

1. **Be Comprehensive**: Provide thorough, educational explanations. Don't just define terms - explain concepts, relationships, and practical applications.

2. **Use the Context Wisely**:
   - When the user asks about a page or topic, explain the broader concept using ALL the retrieved context
   - When selected text is provided, use it as a starting point but expand to cover related concepts
   - If the user asks "explain this page" or similar, provide an overview of the topic covered

3. **Structure Your Responses**:
   - Start with a clear, direct answer
   - Provide supporting details and explanations
   - Include practical examples when relevant
   - Mention related topics the user might want to explore

4. **Citation Format**: Reference sources as [Module Name: Section Title] when citing specific content

5. **Handle Unknowns Gracefully**: If information isn't in the context, say so clearly and suggest where to look

6. **Be Educational**: Remember you're teaching - explain WHY things work, not just WHAT they are

## Response Style:
- Use clear, technical but accessible language
- Break complex topics into digestible parts
- Use bullet points and structure for clarity
- Be thorough but not verbose"""


class GeneratorService:
    """Service for generating responses using Google Gemini models."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel(
            model_name=settings.chat_model,
            system_instruction=SYSTEM_PROMPT
        )

    def generate(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        selected_text: Optional[str] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> str:
        """
        Generate a response using retrieved context.

        Args:
            query: User's question
            context_chunks: Retrieved relevant chunks
            selected_text: Optional user-selected text
            conversation_history: Previous messages in conversation

        Returns:
            Generated response string
        """
        # Build context string from chunks
        context_parts = []
        for i, chunk in enumerate(context_chunks, 1):
            module = chunk.get('module', 'Unknown')
            title = chunk.get('title', 'Section')
            # Format module name nicely
            if module == 'general':
                source_info = f"[{title}]"
            else:
                # Convert module-1-ros2 to "Module 1: ROS 2"
                module_formatted = self._format_module_name(module)
                source_info = f"[{module_formatted}: {title}]"

            context_parts.append(f"Source {i} {source_info}:\n{chunk.get('content', '')}")

        context_string = "\n\n---\n\n".join(context_parts)

        # Build user message with all context
        user_message_parts = []

        if selected_text:
            user_message_parts.append(
                f"The user has selected the following text on the page (use this as context, but provide a comprehensive explanation):\n\"\"\"\n{selected_text}\n\"\"\""
            )

        user_message_parts.append(f"Retrieved documentation context:\n\n{context_string}")
        user_message_parts.append(f"\nUser's question: {query}")
        user_message_parts.append("\nProvide a thorough, educational response based on the context above.")

        user_content = "\n\n".join(user_message_parts)

        # Build chat history for Gemini
        history = []
        if conversation_history:
            for msg in conversation_history[-10:]:
                role = "user" if msg["role"] == "user" else "model"
                history.append({
                    "role": role,
                    "parts": [msg["content"]]
                })

        try:
            # Start chat with history
            chat = self.model.start_chat(history=history)
            response = chat.send_message(user_content)
            return response.text
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

    def _format_module_name(self, module: str) -> str:
        """Format module name for display (e.g., module-1-ros2 -> Module 1: ROS 2)."""
        if not module.startswith('module-'):
            return module

        # Extract number and topic
        parts = module.replace('module-', '').split('-', 1)
        if len(parts) == 2:
            number, topic = parts
            # Format topic nicely
            topic_formatted = topic.upper() if topic in ['ros2', 'vla'] else topic.replace('-', ' ').title()
            return f"Module {number}: {topic_formatted}"
        return module

    def health_check(self) -> bool:
        """Check if Gemini API is accessible."""
        try:
            # Simple test call
            models = genai.list_models()
            return any(True for _ in models)
        except Exception as e:
            logger.error(f"Gemini health check failed: {e}")
            return False
