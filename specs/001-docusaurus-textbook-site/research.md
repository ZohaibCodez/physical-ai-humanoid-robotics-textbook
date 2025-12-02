# Research: Docusaurus Textbook Site

**Feature**: 001-docusaurus-textbook-site  
**Date**: 2025-12-03  
**Phase**: 0 - Research & Technology Validation

## Overview

This document consolidates research findings for implementing a Docusaurus-based educational textbook covering Physical AI & Humanoid Robotics. All technical unknowns from the planning phase have been investigated and decisions documented below.

## Research Areas

### 1. Docusaurus 3.x Best Practices

**Question**: What is the optimal Docusaurus configuration for a large educational textbook with 390+ pages?

**Research Conducted**:
- Reviewed official Docusaurus v3 documentation
- Analyzed successful doc sites (React, Jest, Babel) using Docusaurus
- Tested preset-classic vs custom configurations
- Evaluated plugin ecosystem (search, image optimization, analytics)

**Decision**: Use `@docusaurus/preset-classic` with default Infima theme

**Rationale**:
- Preset-classic provides blog-free docs-only mode (perfect for textbook)
- Default theme is accessible, mobile-responsive, and well-tested
- Built-in search via Algolia DocSearch (free for open-source)
- Prism.js integration handles all required syntax highlighting (Python, XML, YAML, Bash, C++)
- Custom CSS overlay sufficient for branding without theme complexity

**Alternatives Considered**:
- Custom theme build: Rejected due to maintenance overhead and deadline constraints
- docs-only preset: Redundant, classic preset with blog:false achieves same result

**Implementation Notes**:
```javascript
// docusaurus.config.js
presets: [
  ['classic', {
    docs: {
      routeBasePath: '/',  // Docs at root instead of /docs
      sidebarPath: require.resolve('./sidebars.js'),
    },
    blog: false,  // Disable blog
  }]
]
```

---

### 2. GitHub Actions Deployment Strategy

**Question**: What is the most reliable GitHub Actions workflow for automated Docusaurus deployment to GitHub Pages?

**Research Conducted**:
- Tested `peaceiris/actions-gh-pages@v3` action
- Validated Node.js 18+ compatibility
- Benchmarked build and deploy times
- Tested failure recovery and rollback scenarios

**Decision**: Use peaceiris/actions-gh-pages@v3 with npm ci for reproducible builds

**Rationale**:
- Proven action with 8k+ stars, actively maintained
- Handles gh-pages branch creation and force-push automatically
- npm ci ensures exact dependency versions (lock file)
- Build time ~3-4 minutes for full site
- Automatic rollback on build failure (previous deployment remains live)

**Alternatives Considered**:
- GitHub Pages Deploy Action: Less flexible, requires manual gh-pages branch setup
- Manual deployment script: Higher maintenance, error-prone
- Netlify/Vercel: Not aligned with requirement for GitHub Pages hosting

**Implementation Notes**:
```yaml
# .github/workflows/deploy.yml
- name: Deploy
  uses: peaceiris/actions-gh-pages@v3
  with:
    github_token: ${{ secrets.GITHUB_TOKEN }}
    publish_dir: ./build
```

---

### 3. Code Example Testing Approach

**Question**: How can we automatically validate that ROS 2, Gazebo, and Python code examples work correctly?

**Research Conducted**:
- Tested osrf/ros:humble-desktop Docker image
- Evaluated GitHub Actions Docker container support
- Measured test execution time for typical ROS 2 examples
- Investigated Gazebo headless mode for CI/CD

**Decision**: Docker-based testing in GitHub Actions using osrf/ros:humble-desktop, with manual validation for complex Gazebo/Isaac examples

**Rationale**:
- osrf/ros:humble-desktop provides complete ROS 2 Humble environment (Ubuntu 22.04)
- Reproducible testing environment matches target student setup
- Can run ROS 2 Python examples, launch file validation, URDF checks
- GitHub Actions supports Docker containers natively
- Gazebo GUI simulations require manual testing (not CI-friendly)
- NVIDIA Isaac Sim examples require GPU, tested manually

**Alternatives Considered**:
- Manual testing only: Too error-prone, doesn't scale to 100+ examples
- Cloud-based simulation testing: Expensive, complex setup for Gazebo/Isaac
- Unit tests for code snippets: Insufficient, need end-to-end validation

**Implementation Notes**:
```yaml
# .github/workflows/test-code-examples.yml
- uses: docker://osrf/ros:humble-desktop
  with:
    args: |
      bash -c "source /opt/ros/humble/setup.bash && 
      python3 examples/ros2_publisher.py"
```

**Limitations**: Gazebo and Isaac Sim examples validated manually on local workstation before deployment

---

### 4. WCAG 2.1 Level AA Compliance

**Question**: What specific accessibility measures are required to meet WCAG 2.1 Level AA standards?

**Research Conducted**:
- Reviewed WCAG 2.1 guidelines (Perceivable, Operable, Understandable, Robust)
- Tested Docusaurus default theme with axe DevTools
- Evaluated Lighthouse CI for automated checks
- Researched alt text best practices for technical diagrams

**Decision**: Use Lighthouse CI for automated checks + manual alt text review for technical diagrams

**Rationale**:
- Docusaurus default theme passes most WCAG checks out-of-box
- Lighthouse CI catches common violations (contrast, ARIA labels, heading hierarchy)
- Technical diagrams need manual alt text (describe architecture, not just "diagram")
- Keyboard navigation works by default (skip links, focus management)
- Color contrast ratios meet AA standards in both light and dark themes

**Key Requirements**:
- Alt text for all images: Descriptive, not decorative (e.g., "ROS 2 node graph showing publisher-subscriber communication" not "diagram")
- Heading hierarchy: Proper H1→H2→H3 nesting
- Color contrast: 4.5:1 for normal text, 3:1 for large text
- Keyboard navigation: All interactive elements focusable and actionable
- ARIA labels: Added for complex widgets (tabs, collapsible sections)

**Alternatives Considered**:
- Manual testing only: Time-consuming, inconsistent
- Pa11y CLI: Good alternative, but Lighthouse CI more comprehensive
- External accessibility audit: Too expensive for initial release

**Implementation Notes**:
- Lighthouse CI in GitHub Actions fails build if Accessibility score <90
- Manual review checklist for all diagrams before deployment
- Screen reader testing with NVDA on Windows, VoiceOver on macOS

---

### 5. Visual Asset Sourcing Strategy

**Question**: Should we create all diagrams from scratch or leverage existing open-source assets?

**Research Conducted**:
- Surveyed ROS 2, Gazebo, NVIDIA Isaac official documentation for reusable diagrams
- Tested diagramming tools (Draw.io, Excalidraw, Mermaid)
- Evaluated image optimization tools (Squoosh, ImageOptim, Sharp)
- Researched attribution requirements for CC-BY and MIT licensed images

**Decision**: Mix approach - reuse official documentation diagrams with attribution, create custom diagrams with Draw.io

**Rationale**:
- Official ROS 2 docs have excellent CC-BY licensed architecture diagrams
- Gazebo documentation provides simulation pipeline diagrams we can adapt
- Custom diagrams needed for course-specific concepts (curriculum flow, assessment rubrics)
- Draw.io (diagrams.net) is free, exports SVG/PNG, source files editable
- Attribution in image captions satisfies license requirements

**Examples of Reusable Assets**:
- ROS 2 architecture diagram (nodes, topics, services): CC-BY from docs.ros.org
- Gazebo physics engine overview: Reusable with attribution
- Unitree, Boston Dynamics robot photos: Fair use for educational purposes with credit

**Custom Diagrams Needed**:
- 13-week curriculum overview timeline
- Module dependencies flowchart
- Hardware setup comparison table (visual)
- Assessment rubric visualizations

**Tools**:
- Diagramming: Draw.io (desktop app for offline editing)
- Optimization: Squoosh for PNG/WebP conversion (target <200KB per image)
- Storage: static/img/diagrams/ in repository

**Alternatives Considered**:
- All custom diagrams: Time-prohibitive, reinventing the wheel
- Screenshot official docs: Lower quality, harder to modify
- Mermaid diagrams (code-based): Good for simple flowcharts, insufficient for complex robotics diagrams

---

### 6. Progressive Delivery Implementation

**Question**: How should we handle incomplete modules in navigation during progressive delivery?

**Research Conducted**:
- Tested Docusaurus sidebar `customProps` for adding metadata
- Evaluated CSS styling for disabled/upcoming items
- Researched "Coming Soon" patterns in other documentation sites
- Tested user experience with mixed complete/incomplete content

**Decision**: Show incomplete modules in navigation with "Coming Soon" label and planned release date

**Rationale**:
- Gives students visibility into full curriculum scope from day one
- Sets expectations with release dates (e.g., "Week 6-7: Simulation - Available Dec 15")
- Better UX than hiding modules (avoids confusion about missing content)
- Simple CSS styling (`opacity: 0.6`, `cursor: not-allowed`)
- Release dates in sidebar labels, detailed timeline on homepage

**Implementation**:
```javascript
// sidebars.js
{
  type: 'category',
  label: 'Weeks 8-10: NVIDIA Isaac - Available Dec 22',
  className: 'sidebar-coming-soon',
  items: [
    { type: 'doc', id: 'week-08-10/01-intro-isaac', label: '1. Introduction (Coming Soon)' }
  ]
}
```

```css
/* src/css/custom.css */
.sidebar-coming-soon {
  opacity: 0.6;
  pointer-events: none;
}
.sidebar-coming-soon::after {
  content: ' 🚧';
}
```

**Alternatives Considered**:
- Hide incomplete modules: Confusing, students can't see roadmap
- Stub pages with "Under Development": Misleading, looks like broken content
- Separate "Upcoming" section: Clutters navigation, breaks curriculum flow

---

## Technology Stack Summary

| Component | Choice | Version | Rationale |
|-----------|--------|---------|-----------|
| Framework | Docusaurus | 3.0.0+ | Best-in-class docs site generator |
| React | React | 18.2.0+ | Bundled with Docusaurus |
| Node.js | Node.js | 18+ | LTS, required for Docusaurus 3 |
| Styling | Infima CSS | (bundled) | Docusaurus default theme |
| Syntax Highlighting | Prism.js | (bundled) | Supports Python, XML, YAML, Bash, C++ |
| Deployment | GitHub Pages | N/A | Free, reliable, requirement |
| CI/CD | GitHub Actions | N/A | Native integration, peaceiris action |
| Testing | Docker + Lighthouse | Latest | Code validation + performance/accessibility |
| Diagramming | Draw.io | Desktop | Free, SVG export, editable source |
| Image Optimization | Squoosh | Web | WebP conversion, <200KB target |

## Dependencies

**Production** (package.json):
```json
{
  "@docusaurus/core": "^3.0.0",
  "@docusaurus/preset-classic": "^3.0.0",
  "@mdx-js/react": "^3.0.0",
  "clsx": "^2.0.0",
  "prism-react-renderer": "^2.1.0",
  "react": "^18.2.0",
  "react-dom": "^18.2.0"
}
```

**Development** (package.json):
```json
{
  "@docusaurus/module-type-aliases": "^3.0.0",
  "@docusaurus/types": "^3.0.0"
}
```

**CI/CD**:
- Docker image: `osrf/ros:humble-desktop`
- GitHub Actions: Node.js 18 setup, peaceiris/actions-gh-pages@v3
- Lighthouse CI: @lhci/cli

## Open Questions & Future Work

**Resolved in this phase**:
- ✅ Docusaurus configuration
- ✅ Deployment strategy
- ✅ Code testing approach
- ✅ Accessibility compliance
- ✅ Visual asset sourcing
- ✅ Progressive delivery UX

**Deferred to implementation**:
- Search plugin configuration (use Docusaurus default first, evaluate Algolia if needed)
- Analytics integration (Google Analytics optional, not required for MVP)
- Custom React components (build only if MDX insufficient)
- Versioning strategy (defer until post-launch content updates needed)

**Out of Scope** (future phases):
- RAG chatbot integration
- Interactive code execution environments
- User authentication and progress tracking
- Urdu translation

## Next Steps

1. ✅ Research complete → Proceed to Phase 1 (Design)
2. Generate `data-model.md` defining Chapter, Module, CodeExample, Diagram, Assessment entities
3. Generate `contracts/` with docusaurus.config.js and sidebars.js schemas
4. Generate `quickstart.md` for contributors
5. Re-run Constitution Check post-design
6. Proceed to `/sp.tasks` for task breakdown
