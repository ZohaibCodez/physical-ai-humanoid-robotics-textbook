# Implementation Plan: Comprehensive UI/UX Redesign

**Branch**: `005-ui-ux-redesign` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)  
**Input**: Feature specification from `/specs/005-ui-ux-redesign/spec.md`

## Summary

Transform the Physical AI textbook platform from an inconsistent, poorly-spaced design into a professional, accessible, and user-friendly educational platform. Implement a cohesive design system with dark/light modes, mobile responsiveness, profile dropdown navigation, collapsible sidebar, prominent chatbot feature, and comprehensive content cleanup. Technical approach leverages Docusaurus 3.x built-in theming with CSS Modules for component isolation and global CSS custom properties for design tokens, ensuring WCAG 2.1 AA accessibility compliance and <3s page load times.

## Technical Context

**Language/Version**: JavaScript (ES2020+), React 18.2+, Node.js 18+  
**Primary Dependencies**: 
- Docusaurus 3.x (static site generator)
- React 18+ (UI framework)
- CSS Modules (component styling)
- @docusaurus/theme-classic (base theme)
- @docusaurus/plugin-client-redirects (URL migrations)

**Storage**: 
- localStorage (user preferences: theme, sidebar state, chatbot seen)
- sessionStorage (chatbot conversation history)
- Backend API (user profile, authentication) - Feature 003

**Testing**: 
- Jest + React Testing Library (component tests)
- Lighthouse CI (accessibility, performance audits)
- axe-core (accessibility validation)
- Manual testing (screen readers, keyboard navigation)

**Target Platform**: Web (modern browsers), GitHub Pages deployment  
**Project Type**: Web frontend (Docusaurus static site)  
**Performance Goals**: 
- Page load time: <3 seconds on 3G
- First Contentful Paint: <1.5s
- Lighthouse Performance Score: ≥80
- Smooth animations at 60fps

**Constraints**: 
- WCAG 2.1 AA accessibility compliance (REQUIRED)
- Color contrast ratios: 4.5:1 for normal text, 3:1 for large text
- Mobile support: 320px minimum width
- No horizontal scrolling on any screen size
- Must not break existing Docusaurus functionality

**Scale/Scope**: 
- 7 Parts + Appendices content structure
- ~50-100 pages/lessons
- 6-8 reusable UI components
- Design system with 50+ design tokens
- Mobile, tablet, desktop breakpoints (320px, 768px, 1024px, 1440px)

## Constitution Check

**Reference**: `.specify/memory/constitution.md` (16 principles)

### Validation Against Constitutional Principles

| Principle | Status | Compliance Notes |
|-----------|--------|------------------|
| I. Educational Excellence | ✅ PASS | Improved content hierarchy and readability directly support learning outcomes |
| II. Curriculum Alignment | ✅ PASS | UI redesign enhances, doesn't alter, pedagogical sequence (Part 01-07) |
| III. Technical Depth | ✅ PASS | Design system depth matches production-grade development practices |
| IV. Docusaurus Best Practices | ✅ PASS | Leverages built-in theming, swizzles components per docs, uses theme API |
| V. AI-Native Design | ✅ PASS | Chatbot prominence (floating action button) central to feature |
| VI. Modularity | ✅ PASS | Component-based architecture with clear contracts (see contracts/components.md) |
| VII. Testing & Quality | ✅ PASS | Testing plan includes accessibility audits, performance testing, component tests |
| VIII. Performance | ✅ PASS | <3s load times, code splitting, optimized CSS, 60fps animations |
| IX. Security | ✅ PASS | Auth integration with Feature 003, no new security surface added |
| X. Scalability | ✅ PASS | Static site scales horizontally, design tokens support future theming |
| XI. Documentation | ✅ PASS | Comprehensive quickstart.md, component contracts with usage examples |
| XII. Visual Consistency | ✅ PASS | **PRIMARY GOAL** - Design system enforces consistent spacing, colors, typography |
| XIII. User Experience | ✅ PASS | **PRIMARY GOAL** - Mobile responsive, loading states, accessible navigation |
| XIV. Accessibility | ✅ PASS | **CRITICAL** - WCAG 2.1 AA compliance, keyboard navigation, screen reader support |
| XV. Responsive Design | ✅ PASS | **PRIMARY GOAL** - Mobile-first, collapsible sidebar, 320px minimum width |
| XVI. Design System | ✅ PASS | **PRIMARY GOAL** - 50+ design tokens, 8px grid, modular typography scale |

**Conclusion**: Full constitutional compliance. Feature directly addresses Principles XII-XVI (visual design) while maintaining all other requirements.

## Project Structure

### Documentation (this feature)

```text
specs/005-ui-ux-redesign/
├── plan.md              # This file - Technical implementation plan
├── spec.md              # Feature specification with user stories
├── research.md          # Phase 0: 14 technical decisions
├── data-model.md        # Phase 1: TypeScript interfaces and data structures
├── quickstart.md        # Phase 1: 5-phase implementation guide
└── contracts/           # Phase 1: Component and API contracts
    ├── components.md    # 6 reusable UI components
    └── api.md           # 8 backend API endpoints
```

### Source Code (repository root)

**Structure Decision**: Docusaurus web application with swizzled components and custom React modules. Follows Docusaurus best practices with `src/` for custom components and `docs/` for markdown content.

```text
physical-ai-humanoid-robotics-textbook/
├── src/
│   ├── components/                    # Reusable UI components (NEW)
│   │   ├── ProfileDropdown.js         # User menu with settings/logout
│   │   ├── ProfileDropdown.module.css
│   │   ├── LoadingButton.js           # Async button with spinner
│   │   ├── LoadingButton.module.css
│   │   ├── ChatbotFAB.js             # Floating chatbot button
│   │   ├── ChatbotFAB.module.css
│   │   ├── SkeletonLoader.js         # Content loading placeholders
│   │   ├── SkeletonLoader.module.css
│   │   ├── FeatureCard.js            # Landing page cards
│   │   ├── FeatureCard.module.css
│   │   ├── Toast.js                  # Notification system
│   │   └── Toast.module.css
│   ├── contexts/                      # React Context providers (NEW)
│   │   ├── PreferencesContext.js     # Theme, sidebar state
│   │   ├── AuthContext.js            # User authentication
│   │   └── ChatbotContext.js         # Chatbot conversation
│   ├── hooks/                         # Custom React hooks (NEW)
│   │   ├── usePreferences.js         # localStorage persistence
│   │   ├── useResponsive.js          # Breakpoint detection
│   │   ├── useChatbot.js             # Chatbot API calls
│   │   └── useAuth.js                # Auth helpers
│   ├── css/                           # Global styles (MODIFIED)
│   │   ├── design-tokens.css         # 50+ CSS custom properties (NEW)
│   │   ├── custom.css                # Theme overrides (MODIFIED)
│   │   └── responsive.css            # Mobile breakpoints (NEW)
│   ├── pages/                         # Custom pages (MODIFIED)
│   │   ├── index.js                  # Landing page redesign
│   │   └── index.module.css
│   └── theme/                         # Docusaurus swizzles (NEW)
│       ├── Navbar/index.js           # Profile dropdown integration
│       ├── DocSidebar/index.js       # Collapsible sidebar
│       └── Layout/index.js           # Dark mode provider
├── docs/                              # Markdown content (CLEANUP)
│   ├── part-01-foundations/          # Keep
│   ├── part-02-ros2-ecosystem/       # Keep
│   ├── part-03-simulation-environments/  # Keep
│   ├── part-04-nvidia-isaac-platform/    # Keep
│   ├── part-05-humanoid-development/ # Keep
│   ├── part-06-conversational-robotics/  # Keep
│   ├── part-07-capstone-project/     # Keep
│   ├── appendices/                   # Keep
│   ├── week-01-02/                   # REMOVE (deprecated)
│   ├── week-03-05/                   # REMOVE (deprecated)
│   ├── week-06-07/                   # REMOVE (deprecated)
│   ├── week-08-10/                   # REMOVE (deprecated)
│   ├── week-11-12/                   # REMOVE (deprecated)
│   └── week-13/                      # REMOVE (deprecated)
├── static/
│   ├── img/                          # Icons, logos
│   └── files/                        # Resources
├── docusaurus.config.js              # MODIFIED - Add redirects plugin
└── sidebars.js                       # MODIFIED - Remove week-* entries
```

**Key Changes**:
- **NEW**: 6 reusable components in `src/components/` with CSS Modules
- **NEW**: 3 React Context providers for state management
- **NEW**: 4 custom hooks for common patterns
- **NEW**: Design tokens system in `src/css/design-tokens.css`
- **NEW**: Swizzled theme components for navbar, sidebar, layout
- **MODIFIED**: Landing page (`src/pages/index.js`) complete redesign
- **CLEANUP**: Remove `docs/week-*` directories (7 folders)
- **MODIFIED**: Add `@docusaurus/plugin-client-redirects` to config

## Complexity Tracking

**Status**: ✅ No violations detected

All design decisions align with constitutional principles. No complexity justifications required.

---

## Phase 0: Research (COMPLETED)

**Output**: [research.md](./research.md)

Resolved 14 technical decisions covering:

1. **CSS Architecture**: CSS Modules + Global CSS Custom Properties hybrid
2. **Dark Mode**: Leverage Docusaurus built-in `useColorMode` hook with extended tokens
3. **Profile Dropdown**: Swizzle Navbar, integrate with Feature 003 auth context
4. **Grid System**: 8px base unit with responsive multipliers
5. **Typography**: System font stack with fallback chain
6. **Breakpoints**: Mobile-first 320px/768px/1024px/1440px
7. **Collapsible Sidebar**: Extend DocSidebar component with collapse state persistence
8. **Chatbot FAB**: Floating action button, bottom-right, 56x56px Material Design spec
9. **Loading States**: SkeletonLoader component + LoadingButton with spinner
10. **Color Palette**: 50-900 shades with 4.5:1+ contrast ratios
11. **Accessibility**: ARIA labels, keyboard navigation, focus indicators, semantic HTML
12. **Performance**: Code splitting, CSS optimization, lazy loading for chatbot
13. **Content Cleanup**: Redirect plugin for week-* folders, automated migration script
14. **Testing Strategy**: Jest + RTL + Lighthouse CI + axe-core + manual audits

**Key Insights**:
- Docusaurus 3.x theming already provides 95% of dark mode infrastructure
- CSS Modules prevent naming collisions while maintaining global token system
- WCAG 2.1 AA compliance requires systematic color contrast validation
- Mobile-first approach starting at 320px ensures maximum device compatibility

---

## Phase 1: Design & Contracts (COMPLETED)

### Data Model

**Output**: [data-model.md](./data-model.md)

Defined complete TypeScript interfaces for:

- **DesignTokens**: 50+ CSS custom properties (colors, typography, spacing, shadows, transitions)
- **UserPreferences**: localStorage schema (theme, sidebar collapsed, chatbot seen, language)
- **UserProfile**: Backend schema (name, email, avatar, role, created/updated timestamps)
- **Component Props**: TypeScript interfaces for 6 reusable components
- **API Responses**: Request/response formats for 8 backend endpoints
- **State Management**: React Context structure for theme, auth, preferences, chatbot

**State Persistence**:
- localStorage: User preferences (persistent across sessions)
- sessionStorage: Chatbot conversation (cleared on tab close)
- Backend API: User profile and authentication tokens

### Component Contracts

**Output**: [contracts/components.md](./contracts/components.md)

Specified 6 reusable UI components with complete contracts:

1. **ProfileDropdown**: User menu with avatar, name, settings, language switcher, logout
2. **LoadingButton**: Async action button with spinner, disabled state, aria-busy
3. **SkeletonLoader**: Content placeholder with shimmer animation, configurable shapes
4. **ChatbotFAB**: Floating action button with badge for new messages, pulse animation
5. **FeatureCard**: Landing page feature showcase with icon, title, description, link
6. **Toast**: Notification system with success/error/info variants, auto-dismiss

Each contract includes:
- TypeScript interface for props
- Behavior specifications (interactions, states, animations)
- Accessibility requirements (ARIA, keyboard, focus management)
- CSS Module structure with BEM naming
- Usage examples with code snippets

### API Contracts

**Output**: [contracts/api.md](./contracts/api.md)

Documented 8 backend API endpoints:

**Authentication** (Feature 003 integration):
- POST `/api/auth/signup` - User registration
- POST `/api/auth/login` - Email/password login
- POST `/api/auth/logout` - Session termination
- GET `/api/auth/session` - Current user session
- POST `/api/auth/refresh` - Token refresh

**Profile Management**:
- GET `/api/profile` - Fetch user profile
- PUT `/api/profile` - Update profile (name, avatar, preferences)

**Chatbot** (Feature 002 integration):
- POST `/api/chatbot/ask` - RAG query with streaming response

Each endpoint specifies:
- Request/response formats (JSON schemas)
- HTTP status codes and error responses
- Rate limiting (100 req/min for auth, 20 req/min for chatbot)
- CORS configuration for GitHub Pages origin
- Authentication requirements (Bearer tokens)

### Implementation Guide

**Output**: [quickstart.md](./quickstart.md)

Created 5-phase step-by-step implementation guide:

**Phase 1: Design System Foundation**
- Set up design-tokens.css with 50+ custom properties
- Configure Docusaurus dark mode in docusaurus.config.js
- Create PreferencesContext for state management
- Implement usePreferences hook with localStorage persistence

**Phase 2: Core Components**
- Build ProfileDropdown with auth integration
- Create LoadingButton with async handling
- Implement SkeletonLoader with shimmer animation
- Add Toast notification system

**Phase 3: Mobile Responsive**
- Implement responsive breakpoints (320px/768px/1024px/1440px)
- Create collapsible sidebar with persistence
- Add mobile navigation menu
- Test across devices and screen sizes

**Phase 4: Content Cleanup**
- Remove docs/week-* directories (7 folders)
- Set up @docusaurus/plugin-client-redirects
- Update sidebars.js to remove week-* references
- Test all internal links for 404s

**Phase 5: Chatbot Integration**
- Build ChatbotFAB floating action button
- Integrate with Feature 002 RAG backend
- Implement conversation history in sessionStorage
- Add loading states and error handling

**Testing Checklist**:
- ✅ Component unit tests (Jest + RTL)
- ✅ Accessibility audits (Lighthouse + axe-core)
- ✅ Performance benchmarks (<3s load time)
- ✅ Mobile device testing (real devices + emulators)
- ✅ Screen reader testing (NVDA, JAWS, VoiceOver)
- ✅ Keyboard navigation (tab order, focus indicators)
- ✅ Cross-browser compatibility (Chrome, Firefox, Safari, Edge)

---

## Planning Summary

**Artifacts Created**:
1. ✅ research.md - 14 technical decisions resolved
2. ✅ data-model.md - Complete TypeScript interfaces
3. ✅ contracts/components.md - 6 component specifications
4. ✅ contracts/api.md - 8 API endpoint contracts
5. ✅ quickstart.md - 5-phase implementation guide
6. ✅ plan.md - This document (technical plan)

**Constitutional Validation**: ✅ PASS (all 16 principles compliant)

**Next Steps**:
1. Run `/sp.tasks` to break plan into testable implementation tasks
2. Execute Phase 1: Design System Foundation
3. Implement and test components incrementally
4. Conduct accessibility and performance audits
5. Deploy to staging for user acceptance testing

**Architectural Decisions**:
- CSS Modules + Global Tokens hybrid for scalable theming
- Docusaurus built-in dark mode extended with custom tokens
- Component-first architecture with clear contracts
- WCAG 2.1 AA compliance baked into all components
- Mobile-first responsive design from 320px up
- Lazy loading for chatbot to optimize initial page load

**Risk Mitigation**:
- Early accessibility testing prevents late-stage rework
- Incremental implementation reduces integration risk
- Component contracts ensure consistent API surface
- Performance budgets enforced via Lighthouse CI
- Redirect plugin prevents 404s from content cleanup
