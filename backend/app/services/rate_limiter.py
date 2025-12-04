"""
Rate Limiter Service

Implements token bucket algorithm for rate limiting API requests.
Supports both session-based and IP-based rate limiting.
"""

import time
from typing import Dict, Optional
from collections import defaultdict
from dataclasses import dataclass, field
from app.config import settings
from app.utils.logger import app_logger as logger
from app.utils.exceptions import RateLimitError


@dataclass
class TokenBucket:
    """Token bucket for rate limiting."""
    
    capacity: int
    tokens: float
    refill_rate: float  # tokens per second
    last_refill: float = field(default_factory=time.time)
    
    def refill(self):
        """Refill tokens based on elapsed time."""
        now = time.time()
        elapsed = now - self.last_refill
        self.tokens = min(self.capacity, self.tokens + elapsed * self.refill_rate)
        self.last_refill = now
    
    def consume(self, tokens: int = 1) -> bool:
        """
        Try to consume tokens.
        
        Returns:
            True if tokens were consumed, False if insufficient tokens
        """
        self.refill()
        if self.tokens >= tokens:
            self.tokens -= tokens
            return True
        return False
    
    def time_until_token(self) -> float:
        """Calculate seconds until next token is available."""
        if self.tokens >= 1:
            return 0.0
        return (1 - self.tokens) / self.refill_rate


class RateLimiterService:
    """Service for managing API rate limits."""
    
    def __init__(self):
        """Initialize rate limiter with session and IP buckets."""
        self.session_buckets: Dict[str, TokenBucket] = defaultdict(
            lambda: TokenBucket(
                capacity=settings.rate_limit_per_minute,
                tokens=settings.rate_limit_per_minute,
                refill_rate=settings.rate_limit_per_minute / 60.0  # per second
            )
        )
        
        self.ip_buckets: Dict[str, TokenBucket] = defaultdict(
            lambda: TokenBucket(
                capacity=settings.rate_limit_per_hour,
                tokens=settings.rate_limit_per_hour,
                refill_rate=settings.rate_limit_per_hour / 3600.0  # per second
            )
        )
        
        logger.info(
            f"✅ Rate Limiter initialized: "
            f"{settings.rate_limit_per_minute} req/min per session, "
            f"{settings.rate_limit_per_hour} req/hour per IP"
        )
    
    def check_session_limit(self, session_id: str) -> bool:
        """
        Check if session has rate limit capacity.
        
        Args:
            session_id: Session identifier
        
        Returns:
            True if request is allowed
        
        Raises:
            RateLimitError: If rate limit is exceeded
        """
        bucket = self.session_buckets[session_id]
        
        if not bucket.consume():
            retry_after = int(bucket.time_until_token()) + 1
            logger.warning(
                f"Session rate limit exceeded for {session_id}",
                extra={"session_id": session_id}
            )
            raise RateLimitError(
                f"Rate limit exceeded: {settings.rate_limit_per_minute} requests per minute",
                retry_after=retry_after
            )
        
        return True
    
    def check_ip_limit(self, user_ip: str) -> bool:
        """
        Check if IP has rate limit capacity.
        
        Args:
            user_ip: User IP address
        
        Returns:
            True if request is allowed
        
        Raises:
            RateLimitError: If rate limit is exceeded
        """
        bucket = self.ip_buckets[user_ip]
        
        if not bucket.consume():
            retry_after = int(bucket.time_until_token()) + 1
            logger.warning(
                f"IP rate limit exceeded for {user_ip}",
                extra={"user_ip": user_ip}
            )
            raise RateLimitError(
                f"Rate limit exceeded: {settings.rate_limit_per_hour} requests per hour",
                retry_after=retry_after
            )
        
        return True
    
    def check_limits(self, session_id: str, user_ip: str) -> bool:
        """
        Check both session and IP rate limits.
        
        Args:
            session_id: Session identifier
            user_ip: User IP address
        
        Returns:
            True if request is allowed
        
        Raises:
            RateLimitError: If any rate limit is exceeded
        """
        self.check_session_limit(session_id)
        self.check_ip_limit(user_ip)
        return True
    
    def get_remaining_tokens(self, session_id: str) -> int:
        """
        Get remaining tokens for a session.
        
        Args:
            session_id: Session identifier
        
        Returns:
            Number of remaining tokens
        """
        bucket = self.session_buckets[session_id]
        bucket.refill()
        return int(bucket.tokens)
    
    def cleanup_old_buckets(self, max_age_seconds: int = 3600):
        """
        Remove old inactive buckets to prevent memory leaks.
        
        Args:
            max_age_seconds: Remove buckets inactive for this duration
        """
        now = time.time()
        
        # Clean session buckets
        old_sessions = [
            sid for sid, bucket in self.session_buckets.items()
            if now - bucket.last_refill > max_age_seconds
        ]
        for sid in old_sessions:
            del self.session_buckets[sid]
        
        # Clean IP buckets
        old_ips = [
            ip for ip, bucket in self.ip_buckets.items()
            if now - bucket.last_refill > max_age_seconds
        ]
        for ip in old_ips:
            del self.ip_buckets[ip]
        
        if old_sessions or old_ips:
            logger.info(
                f"Cleaned up {len(old_sessions)} session buckets and {len(old_ips)} IP buckets"
            )


# Global service instance
rate_limiter = RateLimiterService()
