"""
Function Tools for OpenAI Agents SDK

Defines tools that the agent can use to search the textbook.
"""

from agents import function_tool
from typing import Optional, List, Dict
from app.services.vector_store import vector_store_service
from app.services.embeddings import google_embedding_service
# Removed: local_embedding_service (heavy sentence-transformers - 500MB+)
# For production: Only Google Generative AI (lightweight, API-based)
from app.services.citation_resolver import citation_resolver
from app.utils.logger import app_logger as logger
from app.utils.exceptions import VectorStoreError, EmbeddingError


@function_tool
async def search_textbook(
    query: str,
    selected_context: Optional[str] = None,
    chapter: Optional[int] = None,
    section: Optional[str] = None
) -> List[Dict]:
    """
    Search the textbook for relevant content.
    
    Use this tool to find information from the Physical AI and Humanoid Robotics textbook
    before answering student questions.
    
    Args:
        query: The search query (student's question or key terms)
        selected_context: Optional selected text to search within (for selected mode)
        chapter: Optional chapter number to filter results (1-13)
        section: Optional section name to filter results
    
    Returns:
        List of relevant textbook chunks with citations and relevance scores
    """
    try:
        logger.info(f"Tool: search_textbook called with query: {query[:100]}...")
        
        # Generate query embedding using Google Generative AI (lightweight, API-based)
        # No local models loaded - perfect for free-tier deployment
        query_embedding = await google_embedding_service.generate_embedding(
            query, task_type="RETRIEVAL_QUERY"
        )
        vector_name = "google"
        logger.debug("Using Google embeddings for search")
        
        # Perform search
        if chapter or section:
            # Filtered search for selected-text mode
            results = await vector_store_service.search_filtered(
                query_vector=query_embedding,
                chapter=chapter,
                section=section,
                vector_name=vector_name,
                limit=5,
                score_threshold=0.7
            )
        else:
            # Full textbook search
            results = await vector_store_service.search(
                query_vector=query_embedding,
                vector_name=vector_name,
                limit=5,
                score_threshold=0.7
            )
        
        # Format results with citations
        formatted_results = []
        for result in results:
            citation = citation_resolver.create_citation(
                chunk=result,
                relevance_score=result["relevance_score"],
                snippet_length=200
            )
            
            formatted_results.append({
                "text": result["text"],
                "citation": citation,
                "relevance_score": result["relevance_score"],
                "section": result["section"],
                "chapter": result["chapter"]
            })
        
        logger.info(f"Tool: search_textbook returned {len(formatted_results)} results")
        return formatted_results
        
    except Exception as e:
        logger.error(f"Tool: search_textbook failed: {str(e)}")
        # Return empty results rather than failing
        return []


@function_tool
async def check_topic_coverage(topic: str) -> Dict:
    """
    Check if a topic is covered in the textbook.
    
    Use this tool when you're unsure if a question can be answered from the textbook.
    
    Args:
        topic: The topic or subject to check
    
    Returns:
        Dictionary with coverage information
    """
    try:
        # Perform a search to see if we get any results  
        # Call the underlying function directly, not the decorated version
        from app.services.vector_store import vector_store_service
        from app.services.embeddings import google_embedding_service
        from app.services.embeddings_local import local_embedding_service
        from app.utils.exceptions import EmbeddingError
        
        try:
            query_embedding = await google_embedding_service.generate_embedding(
                topic, task_type="RETRIEVAL_QUERY"
            )
            vector_name = "google"
        except EmbeddingError:
            query_embedding = await local_embedding_service.generate_embedding(topic)
            vector_name = "local"
        
        results = await vector_store_service.search(
            query_vector=query_embedding,
            vector_name=vector_name,
            limit=5,
            score_threshold=0.7
        )
        
        is_covered = len(results) > 0 and results[0]["relevance_score"] > 0.8
        
        return {
            "topic": topic,
            "is_covered": is_covered,
            "num_results": len(results),
            "max_relevance": results[0]["relevance_score"] if results else 0.0,
            "relevant_sections": [r["section"] for r in results[:3]] if results else []
        }
        
    except Exception as e:
        logger.error(f"Tool: check_topic_coverage failed: {str(e)}")
        return {
            "topic": topic,
            "is_covered": False,
            "error": str(e)
        }


# Export tools for agent registration
TEXTBOOK_TOOLS = [search_textbook, check_topic_coverage]
