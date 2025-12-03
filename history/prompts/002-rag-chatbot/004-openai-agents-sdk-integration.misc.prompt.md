---
id: 004
title: OpenAI Agents SDK Integration Clarification
stage: misc
date: 2025-01-20
surface: agent
model: claude-sonnet-4.5
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: human
command: clarification
labels: ["architecture", "openai-agents-sdk", "litellm", "gemini", "planning-update"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-rag-chatbot/research.md
 - specs/002-rag-chatbot/plan.md
 - specs/002-rag-chatbot/tasks.md
tests:
 - none (planning phase update)
---

## Prompt

make sure that agent is built in openai agents sdk, u can also search about openai agents sdk using context7 mcp server

## Response snapshot

The user clarified that the RAG chatbot agent must be built using the **OpenAI Agents SDK** framework, not just direct Gemini API calls. Agent researched OpenAI Agents SDK via Context7 MCP server and discovered full LiteLLM support for Google Gemini:

**Key Findings**:
1. OpenAI Agents SDK supports "various LLMs" via LiteLLM, including Google Gemini
2. Example pattern: `LitellmModel(model="gemini/gemini-1.5-flash", api_key="...")`
3. Agent primitives: `Agent`, `Runner`, `@function_tool`, session management, guardrails, tracing
4. Free tracing available via OpenAI Traces dashboard (works with non-OpenAI models)

**Updates Made**:
1. **research.md**: Added section 11 "Agent Orchestration Framework" with implementation pattern and rationale
2. **plan.md**: 
   - Updated Primary Dependencies to include `openai-agents[litellm]`
   - Added Key Decision #1: "Agent Orchestration with OpenAI Agents SDK + Gemini"
   - Renumbered remaining key decisions (2-7)
   - Updated project structure to use `agent_service.py` and `tools.py` instead of `gemini_service.py`
3. **tasks.md**:
   - Added header note explaining OpenAI Agents SDK integration
   - Updated T002 to include `openai-agents[litellm]` in requirements
   - Rewrote T026 to implement `agent_service.py` with OpenAI Agents SDK + Gemini backend
   - Created new T027 for `tools.py` with `@function_tool` definitions (search_textbook)
   - Updated T031 (chat endpoint) to use `Runner.run(agent, input, session)` instead of direct Gemini calls
   - Added T043 to verify OpenAI Traces dashboard integration

**Architecture Change**:
```
Before: User → FastAPI → Gemini API → Response
After:  User → FastAPI → OpenAI Agent (Gemini backend) → @function_tool → Vector Search → Response
```

**Benefits**:
- Structured agent lifecycle management
- Built-in tool calling orchestration
- Session management primitives
- Future-ready for multi-agent patterns
- Clean separation of agent instructions and tool implementations

## Outcome

- ✅ Impact: Updated all planning documents to reflect OpenAI Agents SDK integration; architecture now uses structured agent framework with Gemini as backend LLM via LiteLLM
- 🧪 Tests: No tests updated (planning phase)
- 📁 Files: Modified research.md (added section 11), plan.md (updated dependencies, key decisions, project structure), tasks.md (updated 5 tasks, added integration note)
- 🔁 Next prompts: Ready to begin implementation (Phase 1: Setup) with OpenAI Agents SDK architecture
- 🧠 Reflection: User's clarification led to discovering a better architectural pattern (OpenAI Agents SDK) that provides structured orchestration, session management, and tool calling abstractions while still using free Gemini backend. This reduces manual session/tool management code and enables future extensibility (guardrails, multi-agent patterns, MCP servers).

## Evaluation notes (flywheel)

- Failure modes observed: Initial planning did not include agent orchestration framework; direct API calls would have required manual session management, tool calling format, and context tracking
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): User prompt was concise ("make sure that agent is built in openai agents sdk"); agent used context7 MCP to research OpenAI Agents SDK and discovered LiteLLM integration
- Next experiment (smallest change to try): During implementation (T026), verify OpenAI Agents SDK + LiteLLM + Gemini integration works with free tier rate limits; test session persistence with Neon Postgres
