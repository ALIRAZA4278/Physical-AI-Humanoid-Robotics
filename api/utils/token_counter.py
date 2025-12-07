"""
Token counting utilities using tiktoken.
Provides reusable token counting for RAG pipeline components.
"""

import tiktoken
from typing import List, Union
from functools import lru_cache

# Default encoding for OpenAI models (cl100k_base works for text-embedding-3-small and GPT-4)
DEFAULT_ENCODING = "cl100k_base"


@lru_cache(maxsize=1)
def get_encoding(encoding_name: str = DEFAULT_ENCODING):
    """
    Get a cached tiktoken encoding.

    Args:
        encoding_name: Name of the encoding (default: cl100k_base)

    Returns:
        tiktoken.Encoding instance
    """
    return tiktoken.get_encoding(encoding_name)


def count_tokens(text: str, encoding_name: str = DEFAULT_ENCODING) -> int:
    """
    Count tokens in a text string.

    Args:
        text: Text to count tokens in
        encoding_name: Encoding to use (default: cl100k_base)

    Returns:
        Number of tokens
    """
    if not text:
        return 0

    encoding = get_encoding(encoding_name)
    return len(encoding.encode(text))


def count_tokens_batch(texts: List[str], encoding_name: str = DEFAULT_ENCODING) -> List[int]:
    """
    Count tokens for multiple texts efficiently.

    Args:
        texts: List of texts to count tokens in
        encoding_name: Encoding to use (default: cl100k_base)

    Returns:
        List of token counts
    """
    encoding = get_encoding(encoding_name)
    return [len(encoding.encode(text)) if text else 0 for text in texts]


def truncate_to_tokens(
    text: str,
    max_tokens: int,
    encoding_name: str = DEFAULT_ENCODING,
    suffix: str = "..."
) -> str:
    """
    Truncate text to fit within a token limit.

    Args:
        text: Text to truncate
        max_tokens: Maximum number of tokens
        encoding_name: Encoding to use (default: cl100k_base)
        suffix: Suffix to append if truncated (default: "...")

    Returns:
        Truncated text
    """
    if not text:
        return text

    encoding = get_encoding(encoding_name)
    tokens = encoding.encode(text)

    if len(tokens) <= max_tokens:
        return text

    # Reserve space for suffix
    suffix_tokens = encoding.encode(suffix)
    truncated_tokens = tokens[:max_tokens - len(suffix_tokens)]

    return encoding.decode(truncated_tokens) + suffix


def estimate_cost(
    token_count: int,
    price_per_1k: float = 0.00002  # text-embedding-3-small default
) -> float:
    """
    Estimate cost for embedding tokens.

    Args:
        token_count: Number of tokens
        price_per_1k: Price per 1000 tokens (default: $0.00002 for text-embedding-3-small)

    Returns:
        Estimated cost in dollars
    """
    return (token_count / 1000) * price_per_1k


def validate_chunk_size(
    text: str,
    min_tokens: int = 50,
    max_tokens: int = 2000,
    encoding_name: str = DEFAULT_ENCODING
) -> tuple[bool, int, str]:
    """
    Validate if text meets chunk size requirements.

    Args:
        text: Text to validate
        min_tokens: Minimum token count
        max_tokens: Maximum token count
        encoding_name: Encoding to use

    Returns:
        Tuple of (is_valid, token_count, message)
    """
    token_count = count_tokens(text, encoding_name)

    if token_count < min_tokens:
        return False, token_count, f"Chunk too small: {token_count} tokens (min: {min_tokens})"

    if token_count > max_tokens:
        return False, token_count, f"Chunk too large: {token_count} tokens (max: {max_tokens})"

    return True, token_count, "Valid chunk size"
