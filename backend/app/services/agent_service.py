"""
OpenAI Agents SDK Service

Manages RAG agent using OpenAI Agents SDK with Google Gemini backend.
"""

from openai import AsyncOpenAI
from agents import Runner, Agent, OpenAIChatCompletionsModel, set_tracing_disabled, ItemHelpers
from typing import Dict, Any, Optional, List, AsyncGenerator
from openai.types.responses import ResponseTextDeltaEvent
from app.config import settings
from app.utils.logger import app_logger as logger
from app.utils.exceptions import AgentError

# Disable tracing for production
set_tracing_disabled(True)


class AgentService:
    """Service for managing OpenAI Agents SDK with Gemini backend."""
    
    def __init__(self):
        """Initialize agent with external OpenAI-compatible client."""
        try:
            # Create external client pointing to Google Gemini API
            # Using OpenAI SDK's compatibility with other providers
            external_client = AsyncOpenAI(
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
                api_key=settings.google_api_key
            )
            
            # Create OpenAI Chat Completions model wrapper
            self.model = OpenAIChatCompletionsModel(
                openai_client=external_client,
                model=settings.litellm_model  # e.g., "gemini-1.5-flash"
            )
            
            # Create agent with RAG instructions (no model_settings needed)
            self.agent = Agent(
                name="textbook_assistant",
                model=self.model,
                instructions=self._get_agent_instructions()
            )
            
            logger.info(f"✅ Agent Service initialized with {settings.litellm_model}")
            
        except Exception as e:
            logger.error(f"❌ Failed to initialize Agent Service: {str(e)}")
            raise AgentError(f"Agent initialization failed: {str(e)}", agent_name="textbook_assistant")
    
    def _get_agent_instructions(self) -> str:
        """
        Get agent instructions for RAG behavior.
        
        Returns:
            Detailed instructions for the agent
        """
        return """You are a helpful teaching assistant for the Physical AI and Humanoid Robotics textbook.

Your role:
1. Answer student questions based ONLY on the provided textbook content from the search_textbook tool
2. Provide accurate, clear explanations suitable for students learning robotics
3. Always cite the specific textbook sections you reference
4. Keep answers under 500 words while being comprehensive
5. If a question cannot be answered from the textbook, clearly state: "This topic is not covered in the textbook"

Guidelines:
- Use the search_textbook tool to find relevant content before answering
- Cite sources in your answer (e.g., "According to Week 3-5, Section: ROS2 Architecture...")
- If the question is vague, provide a structured overview of related topics
- For out-of-scope questions (not in textbook), politely explain the limitation
- Maintain a helpful, educational tone

Context mode:
- FULL mode: Search entire textbook
- SELECTED mode: Search only within the provided selected text context

Remember: Only use information from the textbook. Do not use external knowledge."""
    
    async def run_agent(
        self,
        question: str,
        session_id: str,
        search_results: Optional[List[Dict]] = None,
        context_mode: str = "full",
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict[str, Any]:  # type: ignore
        """
        Run agent to answer a question.
        
        Args:
            question: User's question
            session_id: Session identifier
            search_results: Pre-fetched search results (optional)
            context_mode: Context mode (full or selected)
            conversation_history: Previous conversation turns
        
        Returns:
            Dictionary with answer, usage info, and metadata
        """
        try:
            # Prepare context for agent
            context = {
                "question": question,
                "session_id": session_id,
                "context_mode": context_mode,
                "search_results": search_results or []
            }
            
            # Build input with context
            input_message = f"Question: {question}\n\n"
            
            if search_results:
                input_message += "Relevant textbook content:\n\n"
                for i, result in enumerate(search_results[:5], 1):
                    input_message += f"{i}. {result.get('section', 'Unknown')} (Relevance: {result.get('relevance_score', 0):.2f})\n"
                    input_message += f"   {result.get('text', '')[:300]}...\n\n"
            
            # Create session object for conversation state
            session_obj = {
                "id": session_id,
                "history": conversation_history or []
            }
            
            # Run agent with Runner
            logger.info(f"Running agent for session {session_id}")
            
            result = await Runner.run(
                starting_agent=self.agent,
                input=input_message
            )
            
            # Extract result
            answer = result.final_output if hasattr(result, 'final_output') else str(result)
            
            # Extract usage information (if available)
            usage = {
                "prompt_tokens": 0,
                "completion_tokens": 0,
                "total_tokens": 0
            }
            
            if hasattr(result, 'usage') and result.usage:
                usage = {
                    "prompt_tokens": getattr(result.usage, 'prompt_tokens', 0),
                    "completion_tokens": getattr(result.usage, 'completion_tokens', 0),
                    "total_tokens": getattr(result.usage, 'total_tokens', 0)
                }
            
            logger.info(
                f"Agent completed for session {session_id}: {usage['total_tokens']} tokens",
                extra={"session_id": session_id, "tokens": usage['total_tokens']}
            )
            
            return {
                "answer": answer,
                "usage": usage,
                "tool_calls": getattr(result, 'tool_calls', []),
                "metadata": {
                    "model": settings.litellm_model,
                    "temperature": 0.3  # Default temperature
                }
            }
            
        except Exception as e:
            logger.error(f"Agent execution failed: {str(e)}")
            raise AgentError(f"Agent run failed: {str(e)}", agent_name="textbook_assistant")
    
    async def run_agent_streamed(
        self,
        question: str,
        session_id: str,
        search_results: Optional[List[Dict]] = None,
        context_mode: str = "full",
        conversation_history: Optional[List[Dict]] = None
    ) -> AsyncGenerator[str, None]:
        """
        Run agent with streaming responses.
        
        Args:
            question: User's question
            session_id: Session identifier
            search_results: Pre-fetched search results (optional)
            context_mode: Context mode (full or selected)
            conversation_history: Previous conversation turns
        
        Yields:
            Text deltas as they are generated
        """
        try:
            # Prepare context for agent
            context = {
                "question": question,
                "session_id": session_id,
                "context_mode": context_mode,
                "search_results": search_results or []
            }
            
            # Build input with context
            input_message = f"Question: {question}\n\n"
            
            if search_results:
                input_message += "Relevant textbook content:\n\n"
                for i, result in enumerate(search_results[:5], 1):
                    input_message += f"{i}. {result.get('section', 'Unknown')} (Relevance: {result.get('relevance_score', 0):.2f})\n"
                    input_message += f"   {result.get('text', '')[:300]}...\n\n"
            
            logger.info(f"Running streamed agent for session {session_id}")
            
            # Run agent with streaming
            stream_result = Runner.run_streamed(
                starting_agent=self.agent,
                input=input_message
            )
            
            # Stream events following OpenAI Agents SDK pattern
            async for event in stream_result.stream_events():
                logger.debug(f"Event type: {event.type}")
                
                # Handle raw response events (actual LLM token deltas)
                if event.type == "raw_response_event":
                    logger.debug(f"Raw response event: {event}")
                    # Try to extract content from the raw response
                    try:
                        if hasattr(event, 'data') and event.data:
                            chunk = event.data
                            logger.debug(f"Chunk: {chunk}")
                            if hasattr(chunk, 'choices') and len(chunk.choices) > 0:
                                delta = chunk.choices[0].delta
                                if hasattr(delta, 'content') and delta.content:
                                    logger.debug(f"Yielding token: {delta.content}")
                                    yield delta.content
                    except Exception as e:
                        logger.debug(f"Error extracting from raw_response_event: {e}")
                # Handle agent updates
                elif event.type == "agent_updated_stream_event":
                    logger.debug(f"Agent updated: {event.new_agent.name}")
                    continue
                # Handle run item stream events (tool calls, outputs, messages)
                elif event.type == "run_item_stream_event":
                    if event.item.type == "tool_call_item":
                        logger.debug("Tool was called")
                    elif event.item.type == "tool_call_output_item":
                        logger.debug(f"Tool output: {event.item.output}")
                    elif event.item.type == "message_output_item":
                        # Fallback: use complete message if raw events don't work
                        message_text = ItemHelpers.text_message_output(event.item)
                        if message_text:
                            logger.info(f"Using message_output_item fallback, yielding: {len(message_text)} chars")
                            # Yield in chunks to simulate streaming
                            for i in range(0, len(message_text), 10):
                                yield message_text[i:i+10]
            
            logger.info(f"Streamed agent completed for session {session_id}")
            
        except Exception as e:
            logger.error(f"Streamed agent execution failed: {str(e)}")
            raise AgentError(f"Streamed agent run failed: {str(e)}", agent_name="textbook_assistant")
    
    async def health_check(self) -> bool:
        """
        Check if agent service is healthy.
        
        Returns:
            True if service is operational
        """
        try:
            # Try a simple query
            result = await self.run_agent(
                question="Test",
                session_id="health_check",
                search_results=[{
                    "text": "This is a test.",
                    "section": "Test",
                    "relevance_score": 1.0
                }]
            )
            return bool(result.get("answer"))
            
        except Exception as e:
            logger.error(f"Agent health check failed: {str(e)}")
            return False


# Global service instance
agent_service = AgentService()
