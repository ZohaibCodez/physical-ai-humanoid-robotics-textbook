---
id: 002
title: UI/UX redesign technical implementation plan
stage: plan
date: 2025-12-07
surface: agent
model: Claude Sonnet 4.5
feature: ui-ux-redesign
branch: 005-ui-ux-redesign
user: ZohaibCodez
command: /sp.plan
labels: ["ui-ux", "design-system", "accessibility", "mobile-responsive", "docusaurus", "planning"]
links:
  spec: specs/005-ui-ux-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-ui-ux-redesign/plan.md
  - specs/005-ui-ux-redesign/research.md
  - specs/005-ui-ux-redesign/data-model.md
  - specs/005-ui-ux-redesign/contracts/components.md
  - specs/005-ui-ux-redesign/contracts/api.md
  - specs/005-ui-ux-redesign/quickstart.md
tests:
  - Constitution check: All 16 principles validated (PASS)
  - Technical context completeness verified
  - Project structure documented for Docusaurus web application
---

## Prompt

Follow the `/sp.plan` prompt instructions precisely to create a comprehensive technical implementation plan for the UI/UX redesign feature specified in specs/005-ui-ux-redesign/spec.md.

Execute the complete planning workflow:
- Phase 0: Research and resolve all technical unknowns
- Phase 1: Design data models, component contracts, API contracts, and quickstart guide
- Complete plan.md with technical context, constitution check, and project structure
- Run update-agent-context script to add new technologies
- Create PHR documenting the planning process

The feature requires transforming the Physical AI textbook platform from an inconsistent design (blue/purple mix, poor spacing) into a professional, accessible, user-friendly educational platform with:
- Cohesive design system with 50+ design tokens
- Dark/light mode with WCAG 2.1 AA compliance
- Profile dropdown with user info, settings, language switcher (EN/UR), logout
- Collapsible sidebar for mobile responsiveness
- Prominent chatbot feature with floating action button
- Comprehensive content cleanup (remove deprecated week-* folders)
- Landing page aligned with Panaversity hackathon context

Technical constraints:
- Must use Docusaurus 3.x (required framework)
- WCAG 2.1 AA accessibility compliance (constitutional requirement)
- Page load <3 seconds on 3G
- Lighthouse Performance Score ≥80
- Mobile support from 320px minimum width
- No breaking changes to existing Docusaurus functionality

## Response snapshot

**Planning Process Completed**

### Phase 0: Research (research.md)
Resolved 14 critical technical decisions:
1. CSS Architecture: CSS Modules + Global CSS Custom Properties hybrid approach
2. Dark Mode: Leverage Docusaurus built-in `useColorMode` hook with extended tokens
3. Profile Dropdown: Swizzle Navbar component, integrate Feature 003 auth
4. Grid System: 8px base unit with responsive multipliers (1x-6x)
5. Typography: System font stack with modular 1.250 scale
6. Breakpoints: Mobile-first 320px/768px/1024px/1440px
7. Collapsible Sidebar: Extend DocSidebar with collapse state persistence
8. Chatbot FAB: 56x56px floating button, bottom-right Material Design spec
9. Loading States: SkeletonLoader + LoadingButton components
10. Color Palette: 50-900 shades with minimum 4.5:1 contrast ratios
11. Accessibility: ARIA labels, keyboard navigation, semantic HTML, focus indicators
12. Performance: Code splitting, CSS optimization, lazy loading
13. Content Cleanup: @docusaurus/plugin-client-redirects for week-* migration
14. Testing: Jest + RTL + Lighthouse CI + axe-core + manual audits

**Key Insight**: Docusaurus 3.x provides 95% of dark mode infrastructure, allowing focus on design tokens and component extensions rather than reinventing theming.

### Phase 1: Design & Contracts

**data-model.md**: Complete TypeScript interfaces
- DesignTokens: 50+ CSS custom properties structure
- UserPreferences: localStorage schema (theme, sidebar, chatbot, language)
- UserProfile: Backend API schema with timestamps
- Component Props: 6 reusable component interfaces
- API Responses: 8 endpoint request/response formats
- State Management: React Context patterns

**contracts/components.md**: 6 Component Specifications
1. ProfileDropdown: Avatar, name, settings, language, logout
2. LoadingButton: Async spinner, disabled state, aria-busy
3. SkeletonLoader: Shimmer animation, configurable shapes
4. ChatbotFAB: Badge for new messages, pulse animation
5. FeatureCard: Icon, title, description, link
6. Toast: Success/error/info variants, auto-dismiss

Each includes TypeScript interface, behavior contract, accessibility requirements, CSS Module structure, usage examples.

**contracts/api.md**: 8 Backend Endpoints
- Auth: signup, login, logout, session, refresh (Feature 003)
- Profile: GET/PUT user profile
- Chatbot: POST ask endpoint with streaming (Feature 002)
- Health: GET system health check

Documented request/response formats, error handling, rate limits (100/min auth, 20/min chatbot), CORS config.

**quickstart.md**: 5-Phase Implementation Guide
1. Design System Foundation: tokens, dark mode, context, hooks
2. Core Components: ProfileDropdown, LoadingButton, SkeletonLoader, Toast
3. Mobile Responsive: breakpoints, collapsible sidebar, navigation
4. Content Cleanup: remove week-*, redirects, sidebars update
5. Chatbot Integration: FAB, RAG backend, conversation history

Includes code examples, CSS snippets, testing checklist, deployment steps.

**plan.md**: Technical Implementation Plan (Completed)
- Technical Context: JavaScript ES2020+, React 18+, Docusaurus 3.x, CSS Modules
- Constitution Check: ✅ All 16 principles validated (PASS)
- Project Structure: Documented Docusaurus web app structure with NEW/MODIFIED/CLEANUP markers
- Complexity Tracking: No violations detected
- Phase summaries: Research findings, design decisions, contract specifications
- Next steps: Run /sp.tasks for implementation breakdown

### Constitutional Validation
All 16 principles verified:
- **Principles XII-XVI (Primary Goals)**: Visual Consistency, User Experience, Accessibility, Responsive Design, Design System - directly addressed by this feature
- **Principles I-XI**: Educational Excellence, Curriculum Alignment, Technical Depth, Docusaurus Best Practices, AI-Native Design, Modularity, Testing, Performance, Security, Scalability, Documentation - all maintained

### Architectural Decisions
- **Hybrid CSS Strategy**: CSS Modules for component isolation + Global Tokens for consistent theming
- **Docusaurus Extension Pattern**: Leverage built-in theming, extend rather than replace
- **Component-First Architecture**: Clear contracts enable parallel development and testing
- **Accessibility-First**: WCAG 2.1 AA baked into every component from design phase
- **Mobile-First Responsive**: 320px minimum ensures maximum device compatibility
- **Performance Budget**: Code splitting, lazy loading, CSS optimization from start

### Risk Mitigation
- Early accessibility testing prevents late-stage rework
- Incremental implementation reduces integration risk
- Component contracts ensure API consistency
- Lighthouse CI enforces performance budgets
- Redirect plugin prevents 404s from content cleanup

## Outcome

- ✅ Impact: Complete technical implementation plan with 6 planning artifacts ready for task breakdown
- 🧪 Tests: Constitution check passed (16/16 principles), technical decisions validated, project structure documented
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/components.md, contracts/api.md, quickstart.md
- 🔁 Next prompts: Run `/sp.tasks` to break plan into testable implementation tasks, then execute Phase 1 (Design System Foundation)
- 🧠 Reflection: Comprehensive planning phase resolved all technical unknowns, defined clear contracts for 6 components and 8 API endpoints, created actionable implementation guide with testing checklist. Design decisions balance Docusaurus constraints with modern UI/UX requirements while maintaining constitutional compliance.

## Evaluation notes (flywheel)

- Failure modes observed: None - all template sections populated, constitution validated, technical decisions documented
- Graders run and results (PASS/FAIL): Constitution check PASS (16/16), Technical context completeness PASS, Project structure PASS
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (research) and Phase 1 (design, contracts, quickstart)
- Next experiment (smallest change to try): Execute /sp.tasks command to validate plan completeness and break down into atomic testable tasks with acceptance criteria
