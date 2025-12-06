"""
Rate limiting middleware for authentication endpoints.
"""

import time
from collections import defaultdict
from typing import Dict, Tuple
from fastapi import Request, HTTPException, status


class RateLimiter:
    """
    Simple in-memory rate limiter for auth endpoints.
    
    Tracks requests per IP address with time windows.
    For production, consider using Redis or similar distributed cache.
    """
    
    def __init__(self, requests_per_minute: int = 5):
        """
        Initialize rate limiter.
        
        Args:
            requests_per_minute: Maximum requests allowed per minute per IP
        """
        self.requests_per_minute = requests_per_minute
        self.request_history: Dict[str, list[float]] = defaultdict(list)
    
    def is_rate_limited(self, identifier: str) -> Tuple[bool, int]:
        """
        Check if identifier (IP address) is rate limited.
        
        Args:
            identifier: Unique identifier (IP address)
            
        Returns:
            Tuple of (is_limited: bool, retry_after: int seconds)
        """
        current_time = time.time()
        window_start = current_time - 60  # 1 minute window
        
        # Clean up old requests outside the time window
        self.request_history[identifier] = [
            timestamp for timestamp in self.request_history[identifier]
            if timestamp > window_start
        ]
        
        # Check if limit exceeded
        request_count = len(self.request_history[identifier])
        
        if request_count >= self.requests_per_minute:
            # Calculate retry_after (seconds until oldest request expires)
            oldest_request = self.request_history[identifier][0]
            retry_after = int(60 - (current_time - oldest_request)) + 1
            return True, retry_after
        
        # Record this request
        self.request_history[identifier].append(current_time)
        return False, 0
    
    async def check_rate_limit(self, request: Request):
        """
        Middleware to check rate limit for auth endpoints.
        
        Args:
            request: FastAPI request object
            
        Raises:
            HTTPException: 429 if rate limit exceeded
        """
        # Only apply to auth endpoints
        if not request.url.path.startswith('/v1/auth/'):
            return
        
        # Skip rate limiting for session checks and refresh
        if request.url.path in ['/v1/auth/session', '/v1/auth/refresh', '/v1/auth/logout']:
            return
        
        # Get client IP
        client_ip = request.client.host if request.client else "unknown"
        
        # Check rate limit
        is_limited, retry_after = self.is_rate_limited(client_ip)
        
        if is_limited:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "message": f"Too many requests. Please try again in {retry_after} seconds.",
                    "code": "RATE_LIMIT_EXCEEDED"
                },
                headers={"Retry-After": str(retry_after)}
            )


# Global rate limiter instance
auth_rate_limiter = RateLimiter(requests_per_minute=5)
