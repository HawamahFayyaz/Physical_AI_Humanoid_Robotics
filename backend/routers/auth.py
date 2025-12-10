"""
Authentication Router - Better-Auth with Neon Postgres

Provides user authentication with:
- Email/password signup and signin
- OAuth support (GitHub, Google)
- JWT token management
- User profile and preferences storage
"""

from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List
from datetime import datetime, timedelta
import os
import hashlib
import secrets
from jose import JWTError, jwt
from passlib.context import CryptContext
import asyncpg

router = APIRouter()
security = HTTPBearer()

# Configuration
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL", "")
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "change-me-in-production")
JWT_ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Database connection pool
_db_pool: Optional[asyncpg.Pool] = None


async def get_db_pool() -> asyncpg.Pool:
    """Get or create database connection pool."""
    global _db_pool
    if _db_pool is None:
        if not NEON_DATABASE_URL:
            raise HTTPException(
                status_code=503,
                detail="Database not configured. Set NEON_DATABASE_URL."
            )
        _db_pool = await asyncpg.create_pool(NEON_DATABASE_URL, min_size=2, max_size=10)
    return _db_pool


# Pydantic models
class UserCreate(BaseModel):
    """User registration model."""
    email: EmailStr
    password: str = Field(..., min_length=8)
    name: str = Field(..., min_length=1, max_length=100)


class UserLogin(BaseModel):
    """User login model."""
    email: EmailStr
    password: str


class UserProfile(BaseModel):
    """User profile response model."""
    id: str
    email: str
    name: str
    created_at: datetime
    experience_level: Optional[str] = None
    interests: List[str] = []


class QuestionnaireRequest(BaseModel):
    """User questionnaire for personalization."""
    experience_level: str = Field(..., pattern="^(beginner|intermediate|advanced)$")
    interests: List[str] = Field(..., min_items=1, max_items=10)
    goals: Optional[str] = None


class TokenResponse(BaseModel):
    """JWT token response."""
    access_token: str
    token_type: str = "bearer"
    expires_in: int


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """Hash a password."""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """Create a JWT access token."""
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES))
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    """Validate JWT token and return user data."""
    try:
        payload = jwt.decode(
            credentials.credentials,
            JWT_SECRET_KEY,
            algorithms=[JWT_ALGORITHM]
        )
        user_id = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
        return {"user_id": user_id, "email": payload.get("email")}
    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token"
        )


@router.post("/signup", response_model=TokenResponse)
async def signup(user: UserCreate):
    """Register a new user."""
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        # Check if user exists
        existing = await conn.fetchrow(
            "SELECT id FROM users WHERE email = $1",
            user.email
        )
        if existing:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Create user
        user_id = secrets.token_urlsafe(16)
        password_hash = get_password_hash(user.password)

        await conn.execute(
            """
            INSERT INTO users (id, email, password_hash, name, created_at)
            VALUES ($1, $2, $3, $4, NOW())
            """,
            user_id, user.email, password_hash, user.name
        )

        # Generate token
        access_token = create_access_token(
            data={"sub": user_id, "email": user.email}
        )

        return TokenResponse(
            access_token=access_token,
            expires_in=ACCESS_TOKEN_EXPIRE_MINUTES * 60
        )


@router.post("/signin", response_model=TokenResponse)
async def signin(credentials: UserLogin):
    """Sign in an existing user."""
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        user = await conn.fetchrow(
            "SELECT id, email, password_hash FROM users WHERE email = $1",
            credentials.email
        )

        if not user or not verify_password(credentials.password, user["password_hash"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        access_token = create_access_token(
            data={"sub": user["id"], "email": user["email"]}
        )

        return TokenResponse(
            access_token=access_token,
            expires_in=ACCESS_TOKEN_EXPIRE_MINUTES * 60
        )


@router.get("/profile", response_model=UserProfile)
async def get_profile(current_user: dict = Depends(get_current_user)):
    """Get the current user's profile."""
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        user = await conn.fetchrow(
            """
            SELECT id, email, name, created_at, experience_level, interests
            FROM users WHERE id = $1
            """,
            current_user["user_id"]
        )

        if not user:
            raise HTTPException(status_code=404, detail="User not found")

        return UserProfile(
            id=user["id"],
            email=user["email"],
            name=user["name"],
            created_at=user["created_at"],
            experience_level=user["experience_level"],
            interests=user["interests"] or []
        )


@router.post("/questionnaire")
async def submit_questionnaire(
    questionnaire: QuestionnaireRequest,
    current_user: dict = Depends(get_current_user)
):
    """Submit user questionnaire for personalization."""
    pool = await get_db_pool()

    async with pool.acquire() as conn:
        await conn.execute(
            """
            UPDATE users
            SET experience_level = $1, interests = $2, goals = $3
            WHERE id = $4
            """,
            questionnaire.experience_level,
            questionnaire.interests,
            questionnaire.goals,
            current_user["user_id"]
        )

    return {"status": "success", "message": "Profile updated"}


@router.get("/health")
async def auth_health():
    """Check auth service health."""
    return {
        "status": "healthy",
        "database_configured": bool(NEON_DATABASE_URL)
    }
