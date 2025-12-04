"""
Chat API Endpoints

Handles question answering and conversation history.
"""

from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import StreamingResponse
from app.models.chat import QuestionRequest, AnswerResponse, ErrorResponse, HistoryResponse
from app.models.textbook import Citation
from app.services.agent_service import agent_service
from app.services.postgres_service import postgres_service
from app.services.rate_limiter import rate_limiter
from app.services.embeddings import google_embedding_service
from app.services.embeddings_local import local_embedding_service
from app.services.vector_store import vector_store_service
from app.services.citation_resolver import citation_resolver
from app.utils.logger import app_logger as logger
from app.utils.exceptions import RateLimitError, AgentError, EmbeddingError, VectorStoreError
from datetime import datetime
import time
import json
from typing import AsyncGenerator

router = APIRouter()


@router.post("/chat/ask", response_model=AnswerResponse, responses={
    429: {"model": ErrorResponse, "description": "Rate limit exceeded"},
    400: {"model": ErrorResponse, "description": "Invalid request"},
    500: {"model": ErrorResponse, "description": "Internal server error"}
})
async def ask_question(request_body: QuestionRequest, request: Request):
    """
    Ask a question and get an answer from the RAG chatbot.
    
    This endpoint uses OpenAI Agents SDK with Google Gemini backend to:
    1. Validate the question
    2. Apply rate limiting
    3. Run the agent with search_textbook tool
    4. Generate answer with citations
    5. Save conversation history
    
    Args:
        request_body: Question request with session_id, question_text, and optional context
        request: FastAPI request object for IP extraction
    
    Returns:
        AnswerResponse with generated answer, citations, and metadata
    """
    start_time = time.time()
    
    try:
        # Get user IP
        user_ip = request.client.host if request.client else "unknown"
        
        # Apply rate limiting
        try:
            rate_limiter.check_limits(request_body.session_id, user_ip)
        except RateLimitError as e:
            logger.warning(
                f"Rate limit exceeded for session {request_body.session_id}",
                extra={"session_id": request_body.session_id, "user_ip": user_ip}
            )
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=e.message,
                headers={"Retry-After": str(e.retry_after)} if e.retry_after else {}
            )
        
        logger.info(
            f"Processing question from session {request_body.session_id}",
            extra={"session_id": request_body.session_id, "context_mode": request_body.context_mode}
        )
        
        # Create or get conversation (with graceful fallback)
        conversation_id = None
        history = []
        
        if postgres_service.pool:
            try:
                conversation_id = await postgres_service.create_or_get_conversation(
                    session_id=request_body.session_id,
                    user_ip=user_ip,
                    context_mode=request_body.context_mode
                )
                
                # Get conversation history for context
                history = await postgres_service.get_conversation_history(
                    session_id=request_body.session_id,
                    limit=10
                )
            except Exception as e:
                logger.warning(f"Failed to access conversation history: {str(e)}")
        else:
            logger.info("Postgres not available - running without conversation history")
        
        # Generate query embedding and search
        try:
            # Try Google embeddings first
            logger.info(f"🔍 Generating embedding for question: '{request_body.question_text[:50]}...'")
            query_embedding = await google_embedding_service.generate_embedding(
                request_body.question_text,
                task_type="RETRIEVAL_QUERY"
            )
            vector_name = "google"
            logger.info(f"✅ Using Google embeddings (768-dim) for search")
        except EmbeddingError as e:
            # Fallback to local embeddings
            logger.warning(f"⚠️ Google embeddings failed: {str(e)}, using local fallback")
            query_embedding = await local_embedding_service.generate_embedding(
                request_body.question_text
            )
            vector_name = "local"
            logger.info(f"✅ Using local embeddings (384-dim) for search")
        
        # Search vector database
        if request_body.context_mode == "selected":
            # Determine filter parameters from either selected_context or selected_metadata
            if request_body.selected_context and request_body.selected_context.metadata:
                chapter = request_body.selected_context.metadata.chapter
                section = request_body.selected_context.metadata.section
            elif request_body.selected_metadata:
                chapter = request_body.selected_metadata.get("chapter")
                section = request_body.selected_metadata.get("section")
            else:
                chapter = None
                section = None
            
            # Filtered search for selected text
            search_results = await vector_store_service.search_filtered(
                query_vector=query_embedding,
                chapter=chapter,
                section=section,
                vector_name=vector_name,
                limit=5,
                score_threshold=0.3
            )
        else:
            # Full textbook search
            search_results = await vector_store_service.search(
                query_vector=query_embedding,
                vector_name=vector_name,
                limit=5,
                score_threshold=0.3
            )
        
        # Run agent with search results
        agent_result = await agent_service.run_agent(
            question=request_body.question_text,
            session_id=request_body.session_id,
            search_results=search_results,
            context_mode=request_body.context_mode,
            conversation_history=history
        )
        
        # Generate citations from search results
        citations = []
        for result in search_results[:3]:  # Top 3 citations
            citation = citation_resolver.create_citation(
                chunk=result,
                relevance_score=result["relevance_score"],
                snippet_length=150
            )
            citations.append(Citation(**citation))
        
        # Calculate confidence score based on search relevance
        confidence_score = search_results[0]["relevance_score"] if search_results else 0.5
        
        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)
        
        # Save question and answer to database (if postgres is available)
        if conversation_id and postgres_service.pool:
            try:
                # Prepare selected_context for storage
                selected_ctx = None
                if request_body.selected_context:
                    selected_ctx = request_body.selected_context.model_dump()
                elif request_body.selected_metadata:
                    selected_ctx = request_body.selected_metadata
                
                question_id = await postgres_service.save_question(
                    conversation_id=conversation_id,
                    question_text=request_body.question_text,
                    selected_context=selected_ctx
                )
                
                await postgres_service.save_answer(
                    question_id=question_id,
                    answer_text=agent_result["answer"],
                    citations=[c.dict() for c in citations],
                    confidence_score=confidence_score,
                    processing_time_ms=processing_time_ms,
                    tokens_used=agent_result["usage"]["total_tokens"]
                )
            except Exception as e:
                logger.warning(f"Failed to save conversation to database: {str(e)}")
        
        logger.info(
            f"Answer generated for session {request_body.session_id}: "
            f"{processing_time_ms}ms, {agent_result['usage']['total_tokens']} tokens",
            extra={
                "session_id": request_body.session_id,
                "processing_time_ms": processing_time_ms,
                "tokens_used": agent_result["usage"]["total_tokens"]
            }
        )
        
        return AnswerResponse(
            answer=agent_result["answer"],
            citations=citations,
            confidence_score=confidence_score,
            processing_time_ms=processing_time_ms,
            context_used=request_body.context_mode,
            tokens_used=agent_result["usage"]["total_tokens"]
        )
        
    except HTTPException:
        raise
    except RateLimitError as e:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=e.message
        )
    except (AgentError, EmbeddingError, VectorStoreError) as e:
        logger.error(f"Service error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Service error: {str(e)}"
        )
    except Exception as e:
        logger.error(f"Unexpected error processing question: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An unexpected error occurred"
        )


@router.post("/chat/ask/stream")
async def ask_question_stream(request_body: QuestionRequest, request: Request):
    """
    Ask a question and stream the answer in real-time using Server-Sent Events (SSE).
    
    This endpoint streams the agent's response as it's generated, providing a more
    interactive user experience. The response format is text/event-stream with
    JSON-encoded events.
    
    Args:
        request_body: Question request with session_id, question_text, and optional context
        request: FastAPI request object for IP extraction
    
    Returns:
        StreamingResponse with SSE formatted events
    """
    async def generate_stream() -> AsyncGenerator[str, None]:
        """Generate SSE formatted stream events."""
        try:
            # Get user IP
            user_ip = request.client.host if request.client else "unknown"
            
            # Apply rate limiting
            try:
                rate_limiter.check_limits(request_body.session_id, user_ip)
            except RateLimitError as e:
                yield f"data: {json.dumps({'error': e.message, 'type': 'rate_limit'})}\n\n"
                return
            
            logger.info(
                f"Starting streamed response for session {request_body.session_id}",
                extra={"session_id": request_body.session_id, "context_mode": request_body.context_mode}
            )
            
            # Create or get conversation (with graceful fallback)
            conversation_id = None
            history = []
            
            if postgres_service.pool:
                try:
                    conversation_id = await postgres_service.create_or_get_conversation(
                        session_id=request_body.session_id,
                        user_ip=user_ip,
                        context_mode=request_body.context_mode
                    )
                    
                    # Get conversation history
                    history = await postgres_service.get_conversation_history(
                        session_id=request_body.session_id,
                        limit=10
                    )
                except Exception as e:
                    logger.warning(f"Failed to access conversation history: {str(e)}")
                    yield f"data: {{\"type\": \"warning\", \"data\": \"Running without conversation history\"}}\n\n"
            else:
                logger.info("Postgres not available - running without conversation history")
                yield f"data: {{\"type\": \"warning\", \"data\": \"Running without conversation history\"}}\n\n"
            
            # Generate query embedding and search
            try:
                query_embedding = await google_embedding_service.generate_embedding(
                    request_body.question_text,
                    task_type="RETRIEVAL_QUERY"
                )
                vector_name = "google"
            except EmbeddingError:
                logger.warning("Using local embeddings fallback")
                query_embedding = await local_embedding_service.generate_embedding(
                    request_body.question_text
                )
                vector_name = "local"
            
            # Search vector database
            if request_body.context_mode == "selected":
                # Determine filter parameters from either selected_context or selected_metadata
                if request_body.selected_context and request_body.selected_context.metadata:
                    chapter = request_body.selected_context.metadata.chapter
                    section = request_body.selected_context.metadata.section
                elif request_body.selected_metadata:
                    chapter = request_body.selected_metadata.get("chapter")
                    section = request_body.selected_metadata.get("section")
                else:
                    chapter = None
                    section = None
                
                search_results = await vector_store_service.search_filtered(
                    query_vector=query_embedding,
                    chapter=chapter,
                    section=section,
                    vector_name=vector_name,
                    limit=5,
                    score_threshold=0.3
                )
            else:
                search_results = await vector_store_service.search(
                    query_vector=query_embedding,
                    vector_name=vector_name,
                    limit=5,
                    score_threshold=0.3
                )
            
            # Stream agent response token-by-token
            full_answer = ""
            async for text_delta in agent_service.run_agent_streamed(
                question=request_body.question_text,
                session_id=request_body.session_id,
                search_results=search_results,
                context_mode=request_body.context_mode,
                conversation_history=history
            ):
                full_answer += text_delta
                yield f"data: {json.dumps({'type': 'delta', 'data': text_delta})}\n\n"
            
            # Save to database (if postgres is available)
            if conversation_id and postgres_service.pool:
                try:
                    # Prepare selected_context for storage
                    selected_ctx = None
                    if request_body.selected_context:
                        selected_ctx = request_body.selected_context.model_dump()
                    elif request_body.selected_metadata:
                        selected_ctx = request_body.selected_metadata
                    
                    question_id = await postgres_service.save_question(
                        conversation_id=conversation_id,
                        question_text=request_body.question_text,
                        selected_context=selected_ctx
                    )
                    
                    await postgres_service.save_answer(
                        question_id=question_id,
                        answer_text=full_answer,
                        citations=[],  # Not tracking citations in streaming mode
                        confidence_score=search_results[0]["relevance_score"] if search_results else 0.5,
                        processing_time_ms=0,  # Not tracked in streaming mode
                        tokens_used=0  # Not tracked in streaming mode
                    )
                except Exception as e:
                    logger.warning(f"Failed to save conversation to database: {str(e)}")
            
            # Send completion event
            yield f"data: {json.dumps({'type': 'done'})}\n\n"
            
        except Exception as e:
            logger.error(f"Streaming error: {str(e)}")
            yield f"data: {json.dumps({'type': 'error', 'data': str(e)})}\n\n"
    
    return StreamingResponse(
        generate_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"
        }
    )


@router.get("/chat/history/{session_id}", response_model=HistoryResponse)
async def get_history(session_id: str):
    """
    Get conversation history for a session.
    
    Args:
        session_id: Session identifier
    
    Returns:
        HistoryResponse with conversation messages
    """
    try:
        logger.info(f"Retrieving history for session {session_id}")
        
        # Check if postgres is available
        if not postgres_service.pool:
            return HistoryResponse(
                session_id=session_id,
                messages=[],
                total_questions=0,
                context_mode="full"
            )
        
        messages = await postgres_service.get_conversation_history(
            session_id=session_id,
            limit=10
        )
        
        # Count questions (user messages)
        total_questions = sum(1 for msg in messages if msg["role"] == "user")
        
        # Get context mode from last conversation (default to full)
        context_mode = "full"  # Default
        
        return HistoryResponse(
            session_id=session_id,
            messages=messages,
            total_questions=total_questions,
            context_mode=context_mode
        )
        
    except Exception as e:
        logger.error(f"Error retrieving history: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve conversation history"
        )
