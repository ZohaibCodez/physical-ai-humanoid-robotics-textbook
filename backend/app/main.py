"""
FastAPI Application Entry Point

RAG Chatbot backend using OpenAI Agents SDK + Google Gemini via LiteLLM.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.api.routes import chat, health
import logging

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

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers with /v1 prefix
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(health.router, prefix="/v1", tags=["health"])

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"CORS Origins: {settings.cors_origins}")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Shutting down RAG Chatbot API...")

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": "0.1.0",
        "docs": "/v1/docs"
    }
