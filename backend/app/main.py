"""
FastAPI Application Entry Point

RAG Chatbot backend using OpenAI Agents SDK + Google Gemini via LiteLLM.
"""

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from app.config import settings
from app.api.routes import chat, health
from app.utils.rate_limiter import auth_rate_limiter
import logging
import uuid
import time

# Configure logging
logging.basicConfig(
    level=settings.log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Physical AI & Humanoid Robotics Textbook Assistant",
    version="0.1.0",
    docs_url="/v1/docs",
    redoc_url="/v1/redoc",
    openapi_url="/v1/openapi.json"
)

# Request ID middleware for traceability
@app.middleware("http")
async def add_request_id_middleware(request: Request, call_next):
    """Add unique request ID to all requests and responses."""
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id
    
    # Apply rate limiting to auth endpoints
    await auth_rate_limiter.check_rate_limit(request)
    
    start_time = time.time()
    
    try:
        response = await call_next(request)
    except Exception as e:
        logger.error(
            f"Request failed: {str(e)}",
            extra={"request_id": request_id, "path": request.url.path}
        )
        raise
    
    process_time = (time.time() - start_time) * 1000  # milliseconds
    
    # Add headers
    response.headers["X-Request-ID"] = request_id
    response.headers["X-Process-Time"] = f"{process_time:.2f}ms"
    
    # Log request completion
    logger.info(
        f"Request completed: {request.method} {request.url.path}",
        extra={
            "request_id": request_id,
            "method": request.method,
            "path": request.url.path,
            "status_code": response.status_code,
            "process_time_ms": round(process_time, 2)
        }
    )
    
    return response

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers with /v1 prefix
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(health.router, prefix="/v1", tags=["health"])

# Authentication routes
from app.api.routes import auth
app.include_router(auth.router, tags=["auth"])

# Add validation error handler for better debugging
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Log and format validation errors for debugging."""
    error_details = exc.errors()
    
    # Get request body for debugging
    body = None
    if request.method in ["POST", "PUT", "PATCH"]:
        try:
            body_bytes = await request.body()
            body = body_bytes.decode('utf-8')
        except Exception:
            body = "Could not decode body"
    
    logger.error(
        f"Validation error on {request.method} {request.url.path}",
        extra={
            "errors": error_details,
            "request_body": body
        }
    )
    
    # Also print to console for immediate debugging
    print(f"\n❌ VALIDATION ERROR on {request.method} {request.url.path}")
    print(f"Request Body: {body}")
    print(f"Validation Errors: {error_details}\n")
    
    return JSONResponse(
        status_code=422,
        content={
            "detail": error_details,
            "message": "Request validation failed. Check your input data."
        }
    )

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"CORS Origins: {settings.cors_origins_list}")
    
    # Initialize Postgres connection pool (non-blocking)
    try:
        import asyncio
        from app.services.postgres_service import postgres_service
        # Try to connect with 10 second timeout
        await asyncio.wait_for(postgres_service.connect(), timeout=10.0)
        logger.info("✅ Postgres service initialized")
    except asyncio.TimeoutError:
        logger.warning("⚠️ Postgres connection timed out - conversation history will be disabled")
    except Exception as e:
        logger.warning(f"⚠️ Failed to initialize Postgres: {str(e)} - conversation history will be disabled")
    
    # Initialize other services (they self-initialize on import)
    logger.info("✅ All services initialized")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down RAG Chatbot API...")
    
    # Disconnect Postgres
    try:
        from app.services.postgres_service import postgres_service
        await postgres_service.disconnect()
        logger.info("✅ Postgres disconnected")
    except Exception as e:
        logger.error(f"Error disconnecting Postgres: {str(e)}")
    
    logger.info("Shutdown complete")

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": "0.1.0",
        "docs": "/v1/docs"
    }

# Vercel serverless handler
handler = app
