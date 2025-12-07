---
id: 003
title: UI/UX redesign implementation tasks
stage: tasks
date: 2025-12-07
surface: agent
model: Claude Sonnet 4.5
feature: ui-ux-redesign
branch: 005-ui-ux-redesign
user: ZohaibCodez
command: /sp.tasks
labels: ["ui-ux", "design-system", "tasks", "implementation", "mobile-responsive", "accessibility"]
links:
  spec: specs/005-ui-ux-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-ui-ux-redesign/tasks.md
tests:
  - Format validation: All 122 tasks follow required checklist format (PASS)
  - User story mapping: All 9 user stories mapped to task phases (PASS)
  - Dependency graph: Critical path identified, parallel opportunities documented (PASS)
  - Independent test criteria: Each story has clear completion verification (PASS)
---

## Prompt

Follow instructions in sp.tasks.prompt.md to generate implementation tasks for the UI/UX redesign feature.

Break down the technical plan (specs/005-ui-ux-redesign/plan.md) and user stories (specs/005-ui-ux-redesign/spec.md) into atomic, testable tasks organized by user story priority. Each user story should be independently implementable and testable.

Use available design documents:
- spec.md: 9 user stories (US1-US9) with priorities P1, P2, P3
- plan.md: Technical context (JavaScript ES2020+, React 18+, Docusaurus 3.x)
- research.md: 14 technical decisions (CSS Modules, dark mode, accessibility)
- data-model.md: TypeScript interfaces for components and state
- contracts/components.md: 6 component specifications (ProfileDropdown, LoadingButton, ChatbotFAB, etc.)
- contracts/api.md: 8 backend API endpoints
- quickstart.md: 5-phase implementation guide

Requirements:
- Organize tasks by user story (Phase 3+ = one phase per user story in priority order)
- Use strict checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- Include [P] marker for parallelizable tasks (different files, no dependencies)
- Include [Story] label (US1-US9) for user story tasks
- Provide independent test criteria for each user story
- Create dependency graph showing story completion order
- Identify parallel execution opportunities
- Define MVP scope (typically P1 stories only)
- Tests are OPTIONAL per specification - focus on implementation

## Response snapshot

**Tasks Generation Completed**

Generated comprehensive tasks.md with 122 atomic, testable tasks organized by 9 user stories.

### Task Organization

**Phase 1: Setup (11 tasks, T001-T011)**
- Install dependencies (@docusaurus/plugin-client-redirects)
- Create design-tokens.css with 50+ CSS custom properties
- Setup PreferencesContext for state management
- Create usePreferences and useResponsive hooks
- Configure Docusaurus colorMode settings

**Phase 2: Foundational (CRITICAL BLOCKER)**
- Import design tokens globally
- Wrap Root with PreferencesProvider
- Verify AuthContext integration with Feature 003
- Test design tokens in light/dark modes with WCAG AA validation
- **Checkpoint**: Foundation ready - user stories can begin in parallel

**Phase 3: US1 - Modern Design System (P1, 7 tasks T012-T018)** 🎯 MVP
- Override Docusaurus defaults with design tokens
- Create typography.css with modular scale (1.250 ratio)
- Create spacing.css with 8px grid utilities
- Configure dark mode color mappings
- Test typography hierarchy and validate contrast ratios (≥4.5:1)
- Document design system tokens

**Phase 4: US2 - Dark/Light Mode Toggle (P1, 8 tasks T019-T026)**
- Verify Docusaurus useColorMode hook integration
- Add dark mode styles for code blocks, sidebar, TOC
- Add smooth 200ms transition animation
- Store preference in localStorage via PreferencesContext
- Test toggle on multiple pages and browsers
- Validate dark mode contrast with Lighthouse (target ≥95)

**Phase 5: US3 - Profile Dropdown (P1, 12 tasks T027-T038)**
- Create ProfileDropdown.js component with TypeScript interface
- Create ProfileDropdown.module.css with dropdown styles
- Add avatar/initials rendering logic
- Swizzle Navbar to integrate ProfileDropdown
- Implement open/close logic (click, click-outside, Escape)
- Add language toggle (EN/UR/Both) with PreferencesContext
- Integrate AuthContext logout with loading state
- Add unauthenticated state (Sign Up/Login buttons)
- Implement ARIA attributes and keyboard navigation
- Test mobile touch events and logout flow

**Phase 6: US4 - Collapsible Sidebar (P2, 10 tasks T039-T048)**
- Swizzle DocSidebar for collapse/expand
- Add hamburger menu icon component
- Implement sidebar collapse state in PreferencesContext
- Add 300ms ease-in-out CSS transitions
- Update content area width on collapse (CSS Grid/Flexbox)
- Mobile auto-collapse (<768px) and auto-close on link click
- Store collapsed state in localStorage
- Test animations and mobile behavior on real devices

**Phase 7: US5 - Mobile-Responsive Layout (P2, 10 tasks T049-T058)**
- Create responsive.css with mobile-first media queries
- Add mobile styles for navbar, hero, feature cards, content
- Ensure 44x44px minimum touch targets
- Test horizontal scroll at 320px, 375px, 414px, 768px breakpoints
- Test device rotation and real mobile devices
- Run Lighthouse mobile usability audit (target ≥90)

**Phase 8: US6 - Enhanced Landing Page (P2, 10 tasks T059-T068)**
- Update hero section with Panaversity branding and clear title
- Create FeatureCard.js component with CSS Module
- Add feature cards showcasing ROS2, NVIDIA Isaac, Gazebo
- Add hackathon context section (RAG chatbot, AI-native learning)
- Update hero CTAs ("Start Reading", "Explore Panaversity")
- Add testimonials/learning outcomes section
- Optimize hero images (lazy loading, WebP, srcset)
- Test load time (FCP <1.5s) and mobile responsiveness

**Phase 9: US7 - Loading States (P3, 12 tasks T069-T080)**
- Create LoadingButton.js with spinner and disabled state
- Create SkeletonLoader.js with shimmer animation
- Create Toast.js notification component (success/error/info variants)
- Create CSS modules for all 3 components
- Replace form submit buttons with LoadingButton
- Add SkeletonLoader for async data pages
- Implement Toast notifications for feedback
- Test with network throttling (Fast 3G)

**Phase 10: US8 - Prominent Chatbot (P3, 15 tasks T081-T095)**
- Create ChatbotFAB.js floating action button (56x56px Material Design)
- Create ChatbotFAB.module.css with bottom-right positioning
- Create ChatbotContext for conversation state
- Create useChatbot hook for Feature 002 API integration
- Integrate FAB in Layout (render on all pages)
- Implement expandable panel with slide-up animation
- Add welcome message and typing indicator
- Implement message rendering with citations (markdown, links)
- Add context-aware queries (selected text support)
- Add pulse animation on first visit (PreferencesContext)
- Store conversation in sessionStorage
- Test mobile interactions and API integration

**Phase 11: US9 - Content Cleanup (P3, 14 tasks T096-T109)**
- Audit internal links for week-* references (grep search)
- Update sidebars.js to remove week-* entries
- Configure redirects plugin with mappings (week-* → part-*)
- Delete 6 week-* directories (week-01-02 through week-13)
- Test all redirects and run broken link checker
- Update README.md and STRUCTURE-COMPLETE.md
- Verify sidebar shows clean part-based structure

**Phase 12: Polish & Cross-Cutting (13 tasks T110-T122)**
- Run Lighthouse audits (Performance ≥80, Accessibility ≥95)
- Run axe DevTools accessibility scan
- Test keyboard navigation and screen readers
- Optimize CSS bundle (<100KB gzipped)
- Add lazy loading for below-fold images
- Test cross-browser (Chrome, Firefox, Safari, Edge)
- Test mobile browsers (Mobile Safari iOS 14+, Chrome Android 10+)
- Document design system completely
- Create user guide for UI features
- Run final QA checklist (12 success criteria)
- Prepare deployment PR with evidence

### Key Metrics

**Total Tasks**: 122
**Parallelizable Tasks**: 42 (marked with [P])
**Format Compliance**: 100% (all tasks use `- [ ] [TID] [P?] [Story?] Description with path`)

**User Story Distribution**:
- Setup & Foundation: 11 tasks
- US1 (Design System, P1): 7 tasks
- US2 (Dark Mode, P1): 8 tasks
- US3 (Profile Dropdown, P1): 12 tasks
- US4 (Collapsible Sidebar, P2): 10 tasks
- US5 (Mobile Responsive, P2): 10 tasks
- US6 (Landing Page, P2): 10 tasks
- US7 (Loading States, P3): 12 tasks
- US8 (Chatbot, P3): 15 tasks
- US9 (Content Cleanup, P3): 14 tasks
- Polish: 13 tasks

**MVP Scope**: Tasks T001-T038 (38 tasks covering Setup + US1-US3, Priority P1)
- Delivers core visual redesign with consistent design system, dark mode, profile dropdown
- Can be deployed independently for user feedback
- Estimated 2-3 weeks for single developer

**Parallel Opportunities**:
- Within US1: T012, T013, T014 (3 CSS files)
- Within US2: T020, T021, T022 (3 dark mode styles)
- Within US5: T049-T054 (6 responsive CSS files)
- Within US7: T069-T074 (3 components + 3 CSS modules)
- Within US8: T081-T084 (component, CSS, context, hook)
- Within US9: T100-T105 (6 directory deletions)
- Within Polish: T110-T117 (8 independent audits)

### Dependency Graph

Critical path identified:
1. Setup → Foundation (BLOCKER)
2. Foundation → US1 (Design System - prerequisite for all visual work)
3. US1 → US2 (Dark Mode needs design tokens)
4. US1 + US2 → US3 (Profile needs themed dropdown)
5. US3 → US4 (Sidebar needs navbar integration)
6. US1 + US4 → US5 (Mobile responsive needs sidebar + design system)
7. US5 → US6 (Landing page needs mobile responsive)
8. US1 → US7 (Loading components need design tokens)
9. US7 → US8 (Chatbot needs loading components)
10. US8 → US9 (Content cleanup after all features complete)
11. All US → Polish (final optimizations)

### Independent Test Criteria

Each user story includes clear "Independent Test" section:
- **US1**: View any page, verify consistent colors/spacing/typography, WCAG AA contrast
- **US2**: Toggle modes, verify readability, preference persistence, OS preference detection
- **US3**: Login, open dropdown, verify profile info/options, logout, test unauthenticated state
- **US4**: Toggle sidebar, verify content reflow, test mobile auto-collapse, preference persistence
- **US5**: View on mobile devices (320-768px), verify no horizontal scroll, touch targets ≥44px
- **US6**: Visit homepage, verify Panaversity branding, course explanation, key topics, CTAs
- **US7**: Perform async actions, verify loading spinners, helpful error messages
- **US8**: Find chatbot on any page, ask question, verify citations, context-aware queries
- **US9**: Navigate site, verify no week-* folders, links work, redirects function correctly

### Implementation Strategy

1. **Foundation First**: Complete T001-T011 before any user story work
2. **P1 Sequential**: US1 → US2 → US3 (each builds on previous)
3. **P2 Parallel**: US4, US5, US6 can run in parallel after US3
4. **P3 Sequential**: US7 → US8 → US9 (dependencies on loading components)
5. **Polish Last**: Cross-cutting concerns after all stories complete

**Estimated Effort**: 6-8 weeks single developer, 3-4 weeks with 2-3 developers

**Testing Approach**: Manual testing after each story phase, automated audits in Phase 12

**Deployment Strategy**: MVP first (P1), then incremental releases for P2/P3

## Outcome

- ✅ Impact: Complete implementation roadmap with 122 atomic tasks enabling independent story development and testing
- 🧪 Tests: Format validation PASS, user story mapping PASS, dependency graph complete, parallel opportunities identified
- 📁 Files: Created tasks.md (122 tasks across 12 phases)
- 🔁 Next prompts: Begin implementation with Phase 1 (Setup), validate each user story's independent test criteria after completion
- 🧠 Reflection: Task organization by user story enables true incremental delivery and parallel development. MVP scope (38 tasks, P1 only) provides clear first milestone. 42 parallelizable tasks identified for multi-developer efficiency. Each story has independent test criteria enabling verification without dependencies.

## Evaluation notes (flywheel)

- Failure modes observed: None - all tasks follow strict format, story mapping complete, MVP clearly defined
- Graders run and results (PASS/FAIL): Format validation PASS (100% compliance), Story independence PASS, Dependency graph PASS, Parallel opportunities PASS
- Prompt variant (if applicable): Standard /sp.tasks workflow with user story organization (not TDD, no test tasks per specification)
- Next experiment (smallest change to try): Begin Phase 1 implementation, track actual time vs estimates, validate that each story truly is independently testable as designed
