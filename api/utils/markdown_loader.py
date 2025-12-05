"""
Markdown document loader for processing documentation files.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any, Optional
import frontmatter
import logging

logger = logging.getLogger(__name__)


class MarkdownLoader:
    """Load and parse markdown files from the docs directory."""

    def __init__(self, docs_path: str = "../docs"):
        self.docs_path = Path(docs_path).resolve()

    def load_all_documents(self) -> List[Dict[str, Any]]:
        """
        Load all markdown files from the docs directory.

        Returns:
            List of document dictionaries with content and metadata
        """
        documents = []

        if not self.docs_path.exists():
            logger.error(f"Docs path does not exist: {self.docs_path}")
            return documents

        # Load both .md and .mdx files
        md_files = list(self.docs_path.rglob("*.md")) + list(self.docs_path.rglob("*.mdx"))
        for md_file in md_files:
            try:
                doc = self.load_document(md_file)
                if doc:
                    documents.append(doc)
            except Exception as e:
                logger.error(f"Error loading {md_file}: {e}")

        logger.info(f"Loaded {len(documents)} documents from {self.docs_path}")
        return documents

    def load_document(self, file_path: Path) -> Optional[Dict[str, Any]]:
        """
        Load a single markdown document.

        Args:
            file_path: Path to the markdown file

        Returns:
            Document dictionary with content and metadata
        """
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                post = frontmatter.load(f)

            # Extract relative path for source reference
            relative_path = file_path.relative_to(self.docs_path)
            source_path = str(relative_path).replace("\\", "/").replace(".mdx", "").replace(".md", "")

            # Determine module from path
            module = self._extract_module(relative_path)

            # Extract title from frontmatter or first heading
            title = post.get("title") or self._extract_title(post.content)

            # Clean content (remove frontmatter, keep text)
            content = self._clean_content(post.content)

            if not content.strip():
                logger.warning(f"Empty content in {file_path}")
                return None

            return {
                "source_path": source_path,
                "title": title,
                "module": module,
                "content": content,
                "metadata": dict(post.metadata),
            }

        except Exception as e:
            logger.error(f"Error parsing {file_path}: {e}")
            return None

    def _extract_module(self, relative_path: Path) -> str:
        """Extract module name from file path."""
        parts = relative_path.parts
        for part in parts:
            if part.startswith("module-"):
                return part
        # For files not in a module directory
        return "general"

    def _extract_title(self, content: str) -> str:
        """Extract title from first H1 heading."""
        match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
        if match:
            return match.group(1).strip()
        return "Untitled"

    def _clean_content(self, content: str) -> str:
        """
        Clean markdown content for embedding.

        - Removes code blocks (keep descriptions)
        - Removes excessive whitespace
        - Keeps important structural elements
        """
        # Remove code blocks but keep a marker
        content = re.sub(
            r"```[\s\S]*?```",
            "[code example]",
            content
        )

        # Remove inline code backticks but keep content
        content = re.sub(r"`([^`]+)`", r"\1", content)

        # Remove image references
        content = re.sub(r"!\[.*?\]\(.*?\)", "", content)

        # Simplify links to just text
        content = re.sub(r"\[([^\]]+)\]\([^\)]+\)", r"\1", content)

        # Remove HTML tags
        content = re.sub(r"<[^>]+>", "", content)

        # Normalize whitespace
        content = re.sub(r"\n{3,}", "\n\n", content)
        content = re.sub(r" {2,}", " ", content)

        return content.strip()

    def extract_headings(self, content: str) -> List[Dict[str, Any]]:
        """
        Extract all headings from content with their hierarchy.

        Returns:
            List of heading dictionaries with level, text, and position
        """
        headings = []
        pattern = r"^(#{1,6})\s+(.+)$"

        for match in re.finditer(pattern, content, re.MULTILINE):
            level = len(match.group(1))
            text = match.group(2).strip()
            headings.append({
                "level": level,
                "text": text,
                "position": match.start(),
            })

        return headings
