"""
Authentication API endpoints.
Handles signup, login, logout, and user preferences.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr
from datetime import datetime, timedelta
import hashlib
import secrets
import logging

from models.database import get_db
from models.user import User

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/auth", tags=["auth"])
security = HTTPBearer(auto_error=False)

# Simple in-memory token store (in production, use Redis or database)
# Format: {token: user_id}
_active_tokens: dict[str, str] = {}


# Request/Response Models
class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    name: str
    software_background: str
    hardware_background: str
    preferred_language: str = "en"
    experience_level: str = "beginner"


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class PreferencesUpdate(BaseModel):
    preferred_language: str | None = None
    experience_level: str | None = None


class AuthResponse(BaseModel):
    token: str
    user: dict


# Helper functions
def hash_password(password: str) -> str:
    """Hash password using SHA-256 with salt."""
    salt = secrets.token_hex(16)
    hashed = hashlib.sha256((password + salt).encode()).hexdigest()
    return f"{salt}:{hashed}"


def verify_password(password: str, stored_hash: str) -> bool:
    """Verify password against stored hash."""
    try:
        salt, hashed = stored_hash.split(":")
        check_hash = hashlib.sha256((password + salt).encode()).hexdigest()
        return check_hash == hashed
    except ValueError:
        return False


def create_token(user_id: str) -> str:
    """Create a new authentication token."""
    token = secrets.token_urlsafe(32)
    _active_tokens[token] = user_id
    return token


def get_user_from_token(token: str, db: Session) -> User | None:
    """Get user from authentication token."""
    user_id = _active_tokens.get(token)
    if not user_id:
        return None
    return db.query(User).filter(User.id == user_id).first()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Depends(security),
    db: Session = Depends(get_db),
) -> User | None:
    """Dependency to get current authenticated user."""
    if not credentials:
        return None
    return get_user_from_token(credentials.credentials, db)


async def require_auth(
    credentials: HTTPAuthorizationCredentials | None = Depends(security),
    db: Session = Depends(get_db),
) -> User:
    """Dependency that requires authentication."""
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
        )
    user = get_user_from_token(credentials.credentials, db)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )
    return user


@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest, db: Session = Depends(get_db)):
    """
    Create a new user account.
    Collects software and hardware background for content personalization.
    """
    # Check if email already exists
    existing_user = db.query(User).filter(User.email == request.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered",
        )

    # Validate password
    if len(request.password) < 8:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Password must be at least 8 characters",
        )

    # Create user
    user = User(
        email=request.email,
        password_hash=hash_password(request.password),
        name=request.name,
        software_background=request.software_background,
        hardware_background=request.hardware_background,
        preferred_language=request.preferred_language,
        experience_level=request.experience_level,
    )

    db.add(user)
    db.commit()
    db.refresh(user)

    # Create token
    token = create_token(str(user.id))

    logger.info(f"New user registered: {user.email}")

    return AuthResponse(token=token, user=user.to_dict())


@router.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    """
    Login with email and password.
    """
    user = db.query(User).filter(User.email == request.email).first()

    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
        )

    # Update last login
    user.last_login = datetime.utcnow()
    db.commit()

    # Create token
    token = create_token(str(user.id))

    logger.info(f"User logged in: {user.email}")

    return AuthResponse(token=token, user=user.to_dict())


@router.post("/logout")
async def logout(
    credentials: HTTPAuthorizationCredentials | None = Depends(security),
):
    """
    Logout current user (invalidate token).
    """
    if credentials and credentials.credentials in _active_tokens:
        del _active_tokens[credentials.credentials]

    return {"message": "Logged out successfully"}


@router.get("/me")
async def get_current_user_info(user: User = Depends(require_auth)):
    """
    Get current user information.
    """
    return user.to_dict()


@router.patch("/preferences")
async def update_preferences(
    preferences: PreferencesUpdate,
    user: User = Depends(require_auth),
    db: Session = Depends(get_db),
):
    """
    Update user preferences.
    """
    if preferences.preferred_language:
        user.preferred_language = preferences.preferred_language
    if preferences.experience_level:
        user.experience_level = preferences.experience_level

    user.updated_at = datetime.utcnow()
    db.commit()
    db.refresh(user)

    return user.to_dict()
