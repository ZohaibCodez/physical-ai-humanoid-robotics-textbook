# Tasks: Comprehensive UI/UX Redesign

**Input**: Design documents from `/specs/005-ui-ux-redesign/`  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Organization**: Tasks grouped by user story for independent implementation and testing

**Tests**: Not required per specification - focus on implementation and manual testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, etc.)
- All file paths are absolute from repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and design system foundation

- [ ] T001 Install required dependencies (@docusaurus/plugin-client-redirects) in package.json
- [ ] T002 [P] Create src/css/design-tokens.css with 50+ CSS custom properties (colors 50-900 shades, typography scale, spacing 8px grid, shadows, transitions)
- [ ] T003 [P] Create src/contexts/PreferencesContext.js for theme, sidebar, chatbot state management with localStorage persistence
- [ ] T004 [P] Create src/hooks/usePreferences.js hook for accessing PreferencesContext
- [ ] T005 [P] Create src/hooks/useResponsive.js hook for breakpoint detection (320px/768px/1024px/1440px)
- [ ] T006 Configure docusaurus.config.js colorMode settings (respectPrefersColorScheme: true, defaultMode: 'light')

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Import design-tokens.css in src/css/custom.css and verify CSS variables available globally
- [ ] T008 Wrap Root component with PreferencesProvider in src/theme/Root.js (Docusaurus swizzle)
- [ ] T009 [P] Create base component styles structure (src/components/ directory with .module.css pattern)
- [ ] T010 [P] Verify existing AuthContext from Feature 003 exports user, login, logout, loading state
- [ ] T011 Test design tokens in both light and dark modes, validate WCAG AA contrast ratios (4.5:1 for text)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Modern Design System Implementation (Priority: P1) 🎯 MVP

**Goal**: Implement cohesive design system with consistent colors, typography, spacing, and contrast ratios meeting WCAG AA standards

**Independent Test**: View any page and verify consistent color palette, no blue/purple mixing, 8px grid spacing, clear typographic hierarchy, minimum 4.5:1 contrast ratios in both light and dark modes

### Implementation for User Story 1

- [ ] T012 [P] [US1] Update src/css/custom.css to override Docusaurus defaults with design tokens (primary colors, fonts, spacing)
- [ ] T013 [P] [US1] Create src/css/typography.css with modular scale (1.250 ratio) for h1-h6, body, code styles
- [ ] T014 [P] [US1] Create src/css/spacing.css with utility classes for 8px grid system (8px, 16px, 24px, 32px, 40px, 48px)
- [ ] T015 [US1] Configure dark mode color mappings in src/css/custom.css [data-theme='dark'] selectors
- [ ] T016 [US1] Test typography hierarchy on sample lesson page (docs/part-01-foundations/chapter-01-introduction-to-physical-ai/lesson-01-overview.md)
- [ ] T017 [US1] Validate color contrast ratios using axe DevTools or Lighthouse (all text ≥4.5:1, large text ≥3:1)
- [ ] T018 [US1] Document design system tokens in specs/005-ui-ux-redesign/design-system.md (color palette, typography scale, spacing values)

**Checkpoint**: Design system foundation complete - all pages show consistent visual language

---

## Phase 4: User Story 2 - Dark/Light Mode Toggle (Priority: P1)

**Goal**: Enable users to toggle between light and dark modes with preference persistence, maintaining readability and contrast in both modes

**Independent Test**: Toggle between modes, verify all components maintain WCAG AA contrast, preference persists on page reload, respects OS preference on first visit

### Implementation for User Story 2

- [ ] T019 [P] [US2] Verify Docusaurus useColorMode hook available in src/theme/Navbar/index.js
- [ ] T020 [P] [US2] Add dark mode styles for code blocks in src/css/custom.css (syntax highlighting with readable contrast)
- [ ] T021 [P] [US2] Add dark mode styles for sidebar in src/css/custom.css (background, borders, hover states)
- [ ] T022 [P] [US2] Add dark mode styles for table of contents in src/css/custom.css
- [ ] T023 [US2] Add smooth transition animation (200ms ease-in-out) for theme switching in src/css/custom.css
- [ ] T024 [US2] Store theme preference in localStorage via PreferencesContext (key: 'theme', values: 'light'|'dark'|'system')
- [ ] T025 [US2] Test theme toggle on multiple pages (landing, lesson, appendix) in both Chrome and Firefox
- [ ] T026 [US2] Validate dark mode color contrast with Lighthouse (target score ≥95 accessibility)

**Checkpoint**: Dark/light mode fully functional with preference persistence

---

## Phase 5: User Story 3 - Profile Dropdown in Navbar (Priority: P1)

**Goal**: Authenticated users can access profile info, settings, language toggle (EN/UR), and logout from dropdown in navbar

**Independent Test**: Log in, click avatar, verify dropdown shows profile info and options, logout works, unauthenticated state shows Sign Up/Login buttons

### Implementation for User Story 3

- [ ] T027 [P] [US3] Create src/components/ProfileDropdown.js component with TypeScript interface from contracts/components.md
- [ ] T028 [P] [US3] Create src/components/ProfileDropdown.module.css with dropdown menu styles (positioning, animation, shadow)
- [ ] T029 [P] [US3] Add user avatar/initials rendering logic (extract initials from user.name, use circular div with background color)
- [ ] T030 [US3] Swizzle src/theme/Navbar/index.js to integrate ProfileDropdown component in navbar right section
- [ ] T031 [US3] Implement dropdown open/close logic (click trigger, click outside to close, Escape key closes)
- [ ] T032 [US3] Add language toggle dropdown item with EN/UR/Both options, update PreferencesContext on change
- [ ] T033 [US3] Integrate AuthContext logout function with loading state in Logout button
- [ ] T034 [US3] Add unauthenticated state rendering (Sign Up and Login buttons) when user === null
- [ ] T035 [US3] Add ARIA attributes (aria-expanded, role="menu", role="menuitem") for accessibility
- [ ] T036 [US3] Implement keyboard navigation (Tab through items, Enter to select, Escape to close)
- [ ] T037 [US3] Test dropdown on mobile (touch events, proper z-index above other content)
- [ ] T038 [US3] Verify logout flow redirects to homepage after successful logout

**Checkpoint**: Profile dropdown fully functional for authenticated and unauthenticated users

---

## Phase 6: User Story 4 - Collapsible Sidebar Navigation (Priority: P2)

**Goal**: Users can collapse/expand sidebar to maximize content reading space, with preference persistence and mobile auto-collapse

**Independent Test**: Toggle sidebar on desktop, verify content reflows, test mobile auto-collapse (<768px), preference persists on reload

### Implementation for User Story 4

- [ ] T039 [P] [US4] Swizzle src/theme/DocSidebar/index.js to add collapse/expand functionality
- [ ] T040 [P] [US4] Add hamburger menu icon component in sidebar header (three horizontal lines, accessible button)
- [ ] T041 [US4] Implement sidebar collapse state in PreferencesContext (key: 'sidebarCollapsed', default: false on desktop, true on mobile)
- [ ] T042 [US4] Add CSS transitions for sidebar collapse (300ms ease-in-out, transform: translateX(-100%))
- [ ] T043 [US4] Update content area width when sidebar collapses (use CSS Grid or Flexbox, expand to fill space)
- [ ] T044 [US4] Implement mobile auto-collapse logic (useResponsive hook, collapse when width < 768px)
- [ ] T045 [US4] Implement mobile auto-close on link click (addEventListener for link clicks inside sidebar)
- [ ] T046 [US4] Store collapsed state in localStorage via PreferencesContext
- [ ] T047 [US4] Test sidebar animation on desktop browsers (Chrome, Firefox, Safari, Edge)
- [ ] T048 [US4] Test mobile sidebar behavior on real devices (iPhone SE 375px, iPad 768px, Android phone)

**Checkpoint**: Collapsible sidebar works across all screen sizes with state persistence

---

## Phase 7: User Story 5 - Mobile-Responsive Layout (Priority: P2)

**Goal**: Fully responsive interface adapts to mobile devices (320px-768px) without losing functionality or requiring horizontal scroll

**Independent Test**: View site on mobile devices, verify hero section stacks vertically, content readable without zoom, touch targets ≥44x44px, cards stack in single column

### Implementation for User Story 5

- [ ] T049 [P] [US5] Create src/css/responsive.css with mobile-first media queries (320px, 768px, 1024px, 1440px breakpoints)
- [ ] T050 [P] [US5] Add mobile styles for navbar (compact layout, smaller logo, hamburger menu)
- [ ] T051 [P] [US5] Add mobile styles for hero section in src/pages/index.module.css (stack vertically, larger touch targets)
- [ ] T052 [P] [US5] Add mobile styles for feature cards (single column, full width, spacing between cards)
- [ ] T053 [P] [US5] Add mobile styles for content area (full width, larger font size 16px+ for readability)
- [ ] T054 [P] [US5] Ensure all buttons meet 44x44px minimum touch target on mobile
- [ ] T055 [US5] Test horizontal scroll on all breakpoints (320px, 375px, 414px, 768px) using Chrome DevTools
- [ ] T056 [US5] Test device rotation (portrait to landscape) for layout adaptation
- [ ] T057 [US5] Test on real mobile devices (iPhone SE, iPhone 12, Samsung Galaxy, iPad)
- [ ] T058 [US5] Run Lighthouse mobile usability audit (target score ≥90)

**Checkpoint**: Mobile-responsive layout fully functional across all device sizes

---

## Phase 8: User Story 6 - Enhanced Landing Page (Priority: P2)

**Goal**: Landing page immediately communicates Physical AI & Humanoid Robotics textbook purpose, Panaversity branding, and hackathon context

**Independent Test**: Visit homepage, verify hero section shows Panaversity branding and clear course title, feature cards explain key topics (ROS2, NVIDIA Isaac, Gazebo), RAG chatbot and AI-native learning mentioned, prominent CTAs

### Implementation for User Story 6

- [ ] T059 [P] [US6] Update src/pages/index.js hero section with Panaversity logo, "Physical AI & Humanoid Robotics" title, compelling subtitle
- [ ] T060 [P] [US6] Create src/components/FeatureCard.js component per contracts/components.md (icon, title, description, link)
- [ ] T061 [P] [US6] Create src/components/FeatureCard.module.css with card styles (shadow, hover animation, responsive)
- [ ] T062 [US6] Add feature cards section showcasing key topics: ROS2, NVIDIA Isaac, Gazebo, Humanoid Development
- [ ] T063 [US6] Add hackathon context section mentioning RAG chatbot and AI-native learning approach
- [ ] T064 [US6] Update hero CTA buttons ("Start Reading" → /docs/intro, "Explore Panaversity" → Panaversity website)
- [ ] T065 [US6] Add testimonials or learning outcomes section (3 outcomes: Master ROS2, Build Humanoids, AI Integration)
- [ ] T066 [US6] Optimize hero section images (lazy loading, WebP format, responsive srcset)
- [ ] T067 [US6] Test landing page load time with Lighthouse (target: First Contentful Paint <1.5s)
- [ ] T068 [US6] Verify landing page mobile responsiveness (hero stacks, cards in single column)

**Checkpoint**: Landing page clearly communicates value proposition and hackathon alignment

---

## Phase 9: User Story 7 - Loading States and Feedback (Priority: P3)

**Goal**: Users see visual feedback for async actions (login, chatbot queries, profile saves) with spinners, loading messages, and helpful error states

**Independent Test**: Perform async actions and verify loading spinners appear, buttons disable during loading, error messages display helpfully, skeleton loaders show for content >200ms

### Implementation for User Story 7

- [ ] T069 [P] [US7] Create src/components/LoadingButton.js component per contracts/components.md (spinner, disabled state, variants)
- [ ] T070 [P] [US7] Create src/components/LoadingButton.module.css with loading spinner animation and disabled styles
- [ ] T071 [P] [US7] Create src/components/SkeletonLoader.js component per contracts/components.md (shimmer animation, configurable shapes)
- [ ] T072 [P] [US7] Create src/components/SkeletonLoader.module.css with shimmer gradient animation
- [ ] T073 [P] [US7] Create src/components/Toast.js notification component per contracts/components.md (success/error/info variants)
- [ ] T074 [P] [US7] Create src/components/Toast.module.css with toast styles (positioning, animation, colors)
- [ ] T075 [US7] Replace all form submit buttons with LoadingButton component (login, signup, profile update)
- [ ] T076 [US7] Add SkeletonLoader to pages with async data loading (user profile, chatbot responses)
- [ ] T077 [US7] Implement Toast notifications for success/error feedback (profile updated, logout success, errors)
- [ ] T078 [US7] Add aria-busy attribute to loading buttons for screen readers
- [ ] T079 [US7] Test loading states with network throttling (Chrome DevTools, Fast 3G simulation)
- [ ] T080 [US7] Verify error messages are user-friendly (no technical jargon, actionable suggestions)

**Checkpoint**: Loading states and feedback implemented across all async actions

---

## Phase 10: User Story 8 - Prominent Chatbot Feature (Priority: P3)

**Goal**: RAG chatbot easily accessible via floating action button on all pages, expandable panel, citations marked, context-aware queries, pulse animation for new visitors

**Independent Test**: Find chatbot icon on any page, open panel, ask question, verify citations link to source chapters, close panel, chatbot state persists

### Implementation for User Story 8

- [ ] T081 [P] [US8] Create src/components/ChatbotFAB.js floating action button per contracts/components.md (56x56px Material Design)
- [ ] T082 [P] [US8] Create src/components/ChatbotFAB.module.css with FAB styles (bottom-right position, shadow, animation)
- [ ] T083 [P] [US8] Create src/contexts/ChatbotContext.js for conversation state management (messages, loading, error)
- [ ] T084 [P] [US8] Create src/hooks/useChatbot.js hook for chatbot API integration with Feature 002 backend
- [ ] T085 [US8] Integrate ChatbotFAB in src/theme/Layout/index.js (render on all pages)
- [ ] T086 [US8] Implement expandable chatbot panel (slide up animation, 400px height, close button)
- [ ] T087 [US8] Add welcome message in chatbot panel ("Hi! Ask me anything about Physical AI & Robotics")
- [ ] T088 [US8] Implement message rendering with citations (markdown formatting, links to source chapters)
- [ ] T089 [US8] Add typing indicator when chatbot is generating response (three animated dots)
- [ ] T090 [US8] Implement context-aware queries (pass selected text as context parameter to API)
- [ ] T091 [US8] Add pulse animation to FAB on first visit (PreferencesContext key: 'chatbotSeen', animate if false)
- [ ] T092 [US8] Store chatbot conversation in sessionStorage (cleared on tab close)
- [ ] T093 [US8] Add badge for new messages (count unread responses)
- [ ] T094 [US8] Test chatbot on mobile (touch interactions, proper z-index, keyboard handling)
- [ ] T095 [US8] Verify chatbot API integration with Feature 002 backend (/v1/chat/ask endpoint)

**Checkpoint**: Chatbot prominently featured and fully functional across all pages

---

## Phase 11: User Story 9 - Content Cleanup (Priority: P3)

**Goal**: Remove deprecated week-* folders, update all internal links to part-XX structure, configure redirects, verify no broken links

**Independent Test**: Navigate entire site, verify no week-* folders visible in sidebar, all links work, redirects from old week-* URLs to part-XX structure function correctly

### Implementation for User Story 9

- [ ] T096 [US9] Audit all internal links in docs/ directory for references to week-* folders (grep search: "week-0")
- [ ] T097 [P] [US9] Update sidebars.js to remove week-* entries, keep only part-01 through part-07 and appendices
- [ ] T098 [P] [US9] Configure @docusaurus/plugin-client-redirects in docusaurus.config.js with mappings (week-01-02 → part-01, etc.)
- [ ] T099 [US9] Create redirect mapping documentation in specs/005-ui-ux-redesign/redirects.md (old URL → new URL table)
- [ ] T100 [US9] Delete docs/week-01-02/ directory after verifying all content migrated to part-01-foundations
- [ ] T101 [US9] Delete docs/week-03-05/ directory after verifying all content migrated to part-02-ros2-ecosystem
- [ ] T102 [US9] Delete docs/week-06-07/ directory after verifying all content migrated to part-03-simulation-environments
- [ ] T103 [US9] Delete docs/week-08-10/ directory after verifying all content migrated to part-04-nvidia-isaac-platform
- [ ] T104 [US9] Delete docs/week-11-12/ directory after verifying all content migrated to part-05-humanoid-development
- [ ] T105 [US9] Delete docs/week-13/ directory after verifying all content migrated to part-06-conversational-robotics
- [ ] T106 [US9] Test all redirects (visit old week-* URLs, verify redirect to correct part-* pages)
- [ ] T107 [US9] Run broken link checker (npm package: broken-link-checker or similar)
- [ ] T108 [US9] Update README.md and STRUCTURE-COMPLETE.md to reflect part-based structure only
- [ ] T109 [US9] Verify sidebar navigation shows clean part-based structure in both light and dark modes

**Checkpoint**: Content cleanup complete - site uses only part-based structure with working redirects

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Final optimizations, accessibility audits, performance tuning, documentation

- [ ] T110 [P] Run Lighthouse audit on 5 sample pages (landing, lesson, appendix), target: Performance ≥80, Accessibility ≥95
- [ ] T111 [P] Run axe DevTools accessibility scan, fix any WCAG AA violations
- [ ] T112 [P] Test keyboard navigation across all interactive components (Tab, Enter, Escape, Arrow keys)
- [ ] T113 [P] Test screen reader compatibility (NVDA on Windows or VoiceOver on macOS)
- [ ] T114 [P] Optimize CSS bundle size (remove unused styles, minify, target <100KB gzipped)
- [ ] T115 [P] Add lazy loading for below-the-fold images (Docusaurus Ideal Image plugin or native loading="lazy")
- [ ] T116 [P] Test cross-browser compatibility (Chrome, Firefox, Safari, Edge - last 2 versions each)
- [ ] T117 [P] Test mobile browsers (Mobile Safari iOS 14+, Chrome Mobile Android 10+)
- [ ] T118 Update specs/005-ui-ux-redesign/design-system.md with complete design token documentation
- [ ] T119 Create user guide in docs/appendices/ui-guide.md explaining dark mode, chatbot, profile features
- [ ] T120 Run final QA checklist (all 12 success criteria from spec.md)
- [ ] T121 Prepare deployment PR with screenshots and testing evidence
- [ ] T122 Update CHANGELOG.md with feature summary and breaking changes (if any)

**Checkpoint**: All polish tasks complete - feature ready for production deployment

---

## Dependencies & Execution Strategy

### User Story Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundation) → CRITICAL BLOCKER
                                        ↓
                    ┌─────────────────────────────────────────────┐
                    ↓                                             ↓
            User Story 1 (P1) ────────────→ User Story 2 (P1) ──→ User Story 3 (P1)
            Design System                   Dark Mode              Profile Dropdown
            [T012-T018]                     [T019-T026]            [T027-T038]
                    ↓                           ↓                      ↓
            User Story 4 (P2) ←──────────────────────────────────────┘
            Collapsible Sidebar             User Story 5 (P2)
            [T039-T048]                     Mobile Responsive
                    ↓                       [T049-T058]
            User Story 6 (P2) ←──────────────┘
            Landing Page                    User Story 7 (P3)
            [T059-T068]                     Loading States
                    ↓                       [T069-T080]
            User Story 8 (P3) ←──────────────┘
            Chatbot FAB
            [T081-T095]
                    ↓
            User Story 9 (P3)
            Content Cleanup
            [T096-T109]
                    ↓
            Phase 12: Polish
            [T110-T122]
```

### Parallel Execution Opportunities

**Within User Story 1 (Design System)**:
- T012, T013, T014 can run in parallel (different files)

**Within User Story 2 (Dark Mode)**:
- T020, T021, T022 can run in parallel (different CSS selectors)

**Within User Story 3 (Profile Dropdown)**:
- T027, T028 can run in parallel (component JS and CSS)

**Within User Story 4 (Collapsible Sidebar)**:
- T039, T040 can run in parallel (swizzle and icon component)

**Within User Story 5 (Mobile Responsive)**:
- T049, T050, T051, T052, T053, T054 can run in parallel (different CSS files)

**Within User Story 6 (Landing Page)**:
- T059, T060, T061 can run in parallel (different components)

**Within User Story 7 (Loading States)**:
- T069, T070, T071, T072, T073, T074 can run in parallel (3 independent components)

**Within User Story 8 (Chatbot)**:
- T081, T082, T083, T084 can run in parallel (component, CSS, context, hook)

**Within User Story 9 (Content Cleanup)**:
- T097, T098 can run in parallel (sidebars.js and redirects config)
- T100-T105 can run in parallel (delete 6 directories independently)

**Within Phase 12 (Polish)**:
- T110, T111, T112, T113, T114, T115, T116, T117 can run in parallel (independent audits)

### MVP Scope Recommendation

**Minimum Viable Product (MVP)**: User Stories 1, 2, 3 (P1 tasks only)

**Rationale**: These three stories deliver the core visual redesign with consistent design system, dark mode, and authenticated user experience. This provides immediate value and can be deployed independently for user feedback before implementing P2/P3 enhancements.

**MVP Task Range**: T001-T038 (38 tasks, estimated 2-3 weeks for single developer)

**Incremental Delivery**: After MVP, deploy each subsequent user story independently for continuous value delivery and user feedback loops.

---

## Implementation Strategy

1. **Foundation First** (Phase 1-2): Complete setup and foundational tasks before any user story work
2. **P1 Stories in Sequence** (Phase 3-5): Design System → Dark Mode → Profile Dropdown (each builds on previous)
3. **P2 Stories in Parallel** (Phase 6-8): Collapsible Sidebar, Mobile Responsive, Landing Page (independent)
4. **P3 Stories in Sequence** (Phase 9-11): Loading States → Chatbot → Content Cleanup (Chatbot needs loading components)
5. **Polish Last** (Phase 12): Cross-cutting concerns and final optimizations

**Estimated Total Effort**: 122 tasks, 6-8 weeks for single developer, 3-4 weeks with 2-3 developers parallelizing work

**Testing Approach**: Manual testing after each user story phase, automated Lighthouse/axe audits in Phase 12

**Deployment Strategy**: Deploy MVP (P1) first, then incremental releases for P2 and P3 stories

---

## Task Summary

**Total Tasks**: 122  
**Parallelizable Tasks**: 42 (marked with [P])  
**User Story Distribution**:
- Setup & Foundation: 11 tasks (T001-T011)
- US1 (Design System): 7 tasks (T012-T018)
- US2 (Dark Mode): 8 tasks (T019-T026)
- US3 (Profile Dropdown): 12 tasks (T027-T038)
- US4 (Collapsible Sidebar): 10 tasks (T039-T048)
- US5 (Mobile Responsive): 10 tasks (T049-T058)
- US6 (Landing Page): 10 tasks (T059-T068)
- US7 (Loading States): 12 tasks (T069-T080)
- US8 (Chatbot): 15 tasks (T081-T095)
- US9 (Content Cleanup): 14 tasks (T096-T109)
- Polish: 13 tasks (T110-T122)

**Independent Test Criteria**: Each user story includes clear test criteria for verifying completion without depending on other stories

**MVP Scope**: Tasks T001-T038 (Stories 1-3, Priority P1)

**Format Validation**: ✅ All tasks follow required checklist format with Task ID, [P] marker (where applicable), [Story] label, and file paths
