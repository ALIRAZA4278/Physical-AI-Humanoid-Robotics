"""
User model for authentication and personalization.
Stores user background information for content personalization.
"""

from sqlalchemy import Column, String, DateTime, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid
import enum

from models.database import Base


class ExperienceLevel(str, enum.Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class PreferredLanguage(str, enum.Enum):
    ENGLISH = "en"
    URDU = "ur"


class User(Base):
    """User model with background information for personalization."""
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    name = Column(String(255), nullable=False)

    # Background information for personalization
    software_background = Column(String(50), nullable=False)
    hardware_background = Column(String(50), nullable=False)

    # User preferences
    preferred_language = Column(String(5), default="en")
    experience_level = Column(String(20), default="beginner")

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    last_login = Column(DateTime, nullable=True)

    def to_dict(self):
        """Convert user to dictionary (excluding password)."""
        return {
            "id": str(self.id),
            "email": self.email,
            "name": self.name,
            "softwareBackground": self.software_background,
            "hardwareBackground": self.hardware_background,
            "preferredLanguage": self.preferred_language,
            "experienceLevel": self.experience_level,
            "createdAt": self.created_at.isoformat() if self.created_at else None,
        }
