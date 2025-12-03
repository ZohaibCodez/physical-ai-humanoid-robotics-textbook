# Tasks: Docusaurus Textbook Site

**Input**: Design documents from `/specs/001-docusaurus-textbook-site/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Dependencies & Execution Order

**Critical Path**:
1. Phase 1 (Setup) → Phase 2 (Foundational) must complete first
2. After Phase 2, all User Stories (3-7) can be implemented in parallel
3. Phase 8 (Polish) can only start after all user stories complete

**User Story Independence**:
- US1 (Core Navigation) - Foundation for all stories, no dependencies
- US2 (Code Examples) - Independent, can run parallel after Phase 2
- US3 (Hardware Setup) - Independent, can run parallel after Phase 2
- US4 (Visual Learning) - Independent, can run parallel after Phase 2
- US5 (Assessment) - Independent, can run parallel after Phase 2

**Parallel Opportunities**:
- Phase 2: T004, T005, T006, T007, T008, T009 can all run in parallel
- Phase 3: All US1 tasks (T010-T018) can run in parallel (different chapters/modules)
- Phase 4-7: All user stories can be implemented concurrently by different developers
- Phase 8: T056-T063 can run in parallel

## Implementation Strategy

**MVP Scope** (User Story 1 only):
- Deliverable: Basic navigable textbook with 2-3 sample chapters
- Timeline: ~2-3 weeks
- Validation: Students can browse chapters, search works, mobile-responsive
- Value: Validates architecture before content creation

**Incremental Delivery**:
1. MVP: User Story 1 (navigation + basic content structure)
2. Sprint 2: Add User Story 2 (code examples with syntax highlighting)
3. Sprint 3: Add User Stories 3+4 (hardware docs + diagrams)
4. Sprint 4: Add User Story 5 (assessments)
5. Sprint 5: Polish, performance optimization, full content

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus structure

- [X] T001 Initialize Docusaurus project with `npx create-docusaurus@latest physical-ai-humanoid-robotics-textbook classic` in repository root
- [X] T002 Install dependencies with `npm install` and verify project structure
- [X] T003 [P] Create .gitignore file to exclude node_modules/, .docusaurus/, build/
- [X] T004 [P] Update package.json with project metadata (name, description, repository, author)
- [X] T005 [P] Configure Git repository settings and branch protection rules for main branch

**Checkpoint**: Basic Docusaurus project structure created, npm start works locally

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core configuration that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Configure docusaurus.config.js with site metadata per contracts/docusaurus-config.md (title, tagline, url, baseUrl, organizationName, projectName)
- [X] T007 [P] Configure docusaurus.config.js preset-classic with docs at root (routeBasePath: '/'), no blog, custom CSS path
- [X] T008 [P] Configure docusaurus.config.js theme with navbar (logo, Textbook link, GitHub link) per contract
- [X] T009 [P] Configure docusaurus.config.js theme with footer (Learning Resources, Community sections) per contract
- [X] T010 [P] Configure docusaurus.config.js Prism syntax highlighting for python, xml, yaml, bash, cpp languages
- [X] T011 [P] Create sidebars.js with tutorialSidebar structure per contracts/sidebars-structure.md
- [X] T012 [P] Add module categories to sidebars.js for weeks 1-2, 3-5, 6-7, 8-10, 11-12, 13, appendix with collapsed states
- [X] T013 [P] Create placeholder logo.svg and favicon.ico in static/img/ directory
- [X] T014 [P] Create custom.css in src/css/ with basic brand colors and typography
- [X] T015 [P] Create directory structure: docs/week-01-02/, docs/week-03-05/, docs/week-06-07/, docs/week-08-10/, docs/week-11-12/, docs/week-13/, docs/appendix/
- [X] T016 [P] Create directory structure: static/img/diagrams/, static/img/screenshots/, static/img/robots/, static/files/
- [X] T017 [P] Create GitHub Actions workflow file .github/workflows/deploy.yml for automated GitHub Pages deployment using peaceiris/actions-gh-pages@v3
- [X] T018 [P] Create GitHub Actions workflow file .github/workflows/test-code-examples.yml for Docker-based code testing (Ubuntu 22.04, ROS 2 Humble)
- [X] T019 Configure GitHub Pages in repository settings to deploy from gh-pages branch
- [X] T020 Test local build with `npm run build` and verify no errors, check output in build/ directory
- [X] T021 Test production serve with `npm run serve` and verify site loads correctly at localhost:3000

**Checkpoint**: Foundation ready - Docusaurus fully configured, builds successfully, GitHub Actions ready

---

## Phase 3: User Story 1 - Core Navigation and Content Access (Priority: P1) 🎯 MVP

**Goal**: Students can navigate through well-organized textbook with clear chapter structure, following 13-week curriculum sequentially

**Independent Test**: Deploy basic Docusaurus site with organized sidebar for all 13 weeks, verify users can browse chapters, use search, access on mobile/desktop

### Implementation for User Story 1

- [X] T022 [P] [US1] Create docs/intro.md as landing page with course overview, learning outcomes, getting started guide
- [X] T023 [P] [US1] Create docs/week-01-02/_category_.json with label "Weeks 1-2: Foundations of Physical AI" and module description
- [X] T024 [P] [US1] Create docs/week-01-02/01-introduction-to-physical-ai.md with learning objectives, introduction section, summary
- [X] T025 [P] [US1] Create docs/week-01-02/02-ai-fundamentals-review.md with learning objectives, content, summary
- [X] T026 [P] [US1] Create docs/week-01-02/03-robotics-hardware-overview.md with learning objectives, content, summary
- [X] T027 [P] [US1] Create docs/week-01-02/04-sensors-and-actuators.md with learning objectives, content, summary
- [X] T028 [P] [US1] Create docs/week-01-02/05-coordinate-systems-and-kinematics.md with learning objectives, content, summary
- [X] T029 [P] [US1] Create docs/week-03-05/_category_.json with label "Weeks 3-5: ROS 2 and Robot Programming"
- [X] T030 [P] [US1] Create docs/week-03-05/01-ros2-architecture.md with frontmatter, learning objectives, sections
- [X] T031 [P] [US1] Create docs/week-03-05/02-nodes-topics-services.md with frontmatter, learning objectives, sections
- [X] T032 [P] [US1] Create docs/week-06-07/_category_.json with label "Weeks 6-7: Perception Systems"
- [X] T033 [P] [US1] Create docs/week-06-07/01-computer-vision-for-robotics.md with frontmatter, learning objectives, sections
- [X] T034 [P] [US1] Create docs/week-08-10/_category_.json with label "Weeks 8-10: Motion Planning and Control"
- [X] T035 [P] [US1] Create docs/week-08-10/01-motion-planning-fundamentals.md with frontmatter, learning objectives, sections
- [X] T036 [P] [US1] Create docs/week-11-12/_category_.json with label "Weeks 11-12: Humanoid Robotics"
- [X] T037 [P] [US1] Create docs/week-11-12/01-humanoid-kinematics.md with frontmatter, learning objectives, sections
- [X] T038 [P] [US1] Create docs/week-13/_category_.json with label "Week 13: Simulation and Integration"
- [X] T039 [P] [US1] Create docs/week-13/01-gazebo-simulation.md with frontmatter, learning objectives, sections
- [X] T040 [P] [US1] Create docs/appendix/_category_.json with label "Appendix" and collapsed: true
- [X] T041 [P] [US1] Add all created chapter references to sidebars.js in correct module categories
- [ ] T042 [US1] Test sidebar navigation - verify all chapters appear, expand/collapse works, breadcrumbs display
- [ ] T043 [US1] Test search functionality with test queries ("ROS 2", "URDF", "humanoid") and verify results
- [ ] T044 [US1] Test mobile responsiveness - verify layout works on 320px, 768px, 1024px widths
- [ ] T045 [US1] Test dark mode toggle - verify all pages render correctly in both themes
- [ ] T046 [US1] Test Next/Previous navigation buttons at bottom of chapters
- [ ] T047 [US1] Deploy to GitHub Pages and verify live site navigation works end-to-end

**Checkpoint**: User Story 1 complete - Students can navigate entire curriculum structure, search works, mobile-responsive

---

## Phase 4: User Story 2 - Code Examples and Practical Learning (Priority: P2)

**Goal**: Students see complete, runnable code examples with syntax highlighting and copy functionality

**Independent Test**: Deploy chapters with code examples, verify syntax highlighting for Python/XML/YAML, copy buttons exist, examples include setup/output

### Implementation for User Story 2

- [X] T048 [P] [US2] Add Python code example to docs/week-03-05/01-ros2-architecture.md with inline comments, expected output
- [X] T049 [P] [US2] Add ROS 2 publisher/subscriber code example to docs/week-03-05/02-nodes-topics-services.md with setup instructions
- [X] T050 [P] [US2] Add XML URDF example to docs/week-03-05/03-robot-state-publisher.md with syntax highlighting and comments
- [X] T051 [P] [US2] Add YAML launch file example to docs/week-03-05/04-tf2-transformations.md with parameter explanations
- [X] T052 [P] [US2] Add Bash installation script example to docs/appendix/software-installation.md with step-by-step comments
- [ ] T053 [P] [US2] Add C++ ROS 2 node example to docs/week-08-10/05-moveit2-integration.md with compilation instructions
- [ ] T054 [P] [US2] Create MDX tabs component example in docs/week-06-07/01-computer-vision-for-robotics.md for Gazebo vs Unity alternatives
- [ ] T055 [P] [US2] Add collapsible code blocks for longer examples (>30 lines) in docs/week-11-12/02-bipedal-locomotion.md
- [X] T056 [P] [US2] Add troubleshooting section with error handling code examples in docs/appendix/troubleshooting.md
- [ ] T057 [US2] Test syntax highlighting for all languages (Python, XML, YAML, Bash, C++) across chapters
- [ ] T058 [US2] Test copy button functionality on all code blocks - verify clipboard works
- [ ] T059 [US2] Test collapsible code sections - verify expand/collapse animations work
- [ ] T060 [US2] Test tabs component - verify switching between alternatives works smoothly
- [X] T061 [US2] Create test script in .github/workflows/test-code-examples.yml to validate Python examples in Docker (osrf/ros:humble-desktop)
- [ ] T062 [US2] Run Docker-based tests on sample ROS 2 code examples and verify they execute without errors
- [ ] T063 [US2] Deploy updated site and verify all code examples display correctly with syntax highlighting

**Checkpoint**: User Story 2 complete - Code examples work, syntax highlighting correct, copy buttons functional

---

## Phase 5: User Story 3 - Hardware Setup and Prerequisites (Priority: P3)

**Goal**: Students have clear documentation about hardware requirements, software installation, and setup options

**Independent Test**: Deploy appendix pages with hardware specs and installation guides, verify students can follow instructions

### Implementation for User Story 3

- [X] T064 [P] [US3] Create docs/appendix/hardware-requirements.md with Economy Jetson Kit specifications and pricing
- [X] T065 [P] [US3] Add Digital Twin Workstation option to docs/appendix/hardware-requirements.md with component list
- [X] T066 [P] [US3] Add Robot Lab tier to docs/appendix/hardware-requirements.md with full setup details
- [X] T067 [P] [US3] Add cloud alternatives section to docs/appendix/hardware-requirements.md (Google Colab, AWS, etc.)
- [X] T068 [P] [US3] Create docs/appendix/software-installation.md with Ubuntu 22.04 LTS installation guide
- [X] T069 [P] [US3] Add ROS 2 Humble installation section to docs/appendix/software-installation.md with apt commands
- [X] T070 [P] [US3] Add Gazebo installation section to docs/appendix/software-installation.md with step-by-step instructions
- [X] T071 [P] [US3] Add NVIDIA Isaac Sim installation section to docs/appendix/software-installation.md with prerequisites
- [X] T072 [P] [US3] Add environment verification section to docs/appendix/software-installation.md with test commands
- [X] T073 [P] [US3] Create docs/appendix/troubleshooting.md with common installation errors organized by OS
- [X] T074 [P] [US3] Add "Hello ROS 2" test program to docs/appendix/software-installation.md for setup verification
- [ ] T075 [US3] Test all installation commands in clean Docker containers (Ubuntu 22.04) to verify accuracy
- [ ] T076 [US3] Review hardware pricing and update if needed for accuracy
- [X] T077 [US3] Add comparison table to docs/appendix/hardware-requirements.md showing cost vs capabilities
- [ ] T078 [US3] Deploy updated appendix and verify all setup documentation is complete and accurate

**Checkpoint**: User Story 3 complete - Students can choose hardware option and install all required software

---

## Phase 6: User Story 4 - Visual Learning Aids (Priority: P4)

**Goal**: Students see diagrams, architecture illustrations, and concept visualizations integrated into chapters

**Independent Test**: Verify chapters include diagrams with proper alt text, images optimized, accessible

### Implementation for User Story 4

- [ ] T079 [P] [US4] Create ROS 2 architecture diagram showing nodes/topics/services and save to static/img/diagrams/ros2-architecture.png
- [ ] T080 [P] [US4] Add ROS 2 architecture diagram to docs/week-03-05/01-ros2-architecture.md with descriptive alt text and caption
- [ ] T081 [P] [US4] Create bipedal locomotion gait cycle diagram and save to static/img/diagrams/gait-cycle.png
- [ ] T082 [P] [US4] Add gait cycle diagram to docs/week-11-12/02-bipedal-locomotion.md with alt text
- [ ] T083 [P] [US4] Create sensor fusion flowchart and save to static/img/diagrams/sensor-fusion.png
- [ ] T084 [P] [US4] Add sensor fusion diagram to docs/week-06-07/05-sensor-fusion.md with detailed alt text
- [ ] T085 [P] [US4] Create coordinate systems illustration and save to static/img/diagrams/coordinate-systems.png
- [ ] T086 [P] [US4] Add coordinate systems diagram to docs/week-01-02/05-coordinate-systems-and-kinematics.md
- [ ] T087 [P] [US4] Source open-source robotics diagrams with proper attribution and save to static/img/diagrams/
- [ ] T088 [P] [US4] Create Isaac Sim workflow diagram and save to static/img/diagrams/isaac-workflow.png
- [ ] T089 [P] [US4] Add Isaac workflow diagram to docs/week-08-10/02-isaac-sim.md with caption
- [ ] T090 [P] [US4] Optimize all images using WebP format with PNG fallbacks for browser compatibility
- [ ] T091 [P] [US4] Configure lazy loading for images in docusaurus.config.js to improve page load performance
- [ ] T092 [US4] Audit all images for alt text completeness - verify descriptive text exists for every image
- [ ] T093 [US4] Test image zoom functionality - verify click-to-zoom works on complex diagrams
- [ ] T094 [US4] Test image loading performance on slow connection (throttle to 1 Mbps) - verify lazy loading works
- [ ] T095 [US4] Test images in dark mode - verify diagrams have appropriate contrast and visibility
- [ ] T096 [US4] Deploy updated site and verify all visual aids display correctly across chapters

**Checkpoint**: User Story 4 complete - Diagrams integrated, optimized, accessible with proper alt text

---

## Phase 7: User Story 5 - Assessment and Project Guidance (Priority: P5)

**Goal**: Students have clear assessment descriptions, project rubrics, and capstone project guidance

**Independent Test**: Deploy assessment pages for each module with rubrics, verify completeness of evaluation criteria

### Implementation for User Story 5

- [X] T097 [P] [US5] Create docs/assessments.md with overview of assessment structure and grading philosophy
- [X] T098 [P] [US5] Add Week 5 (ROS 2) assessment section to docs/assessments.md with project requirements and rubric
- [X] T099 [P] [US5] Add Week 7 (Simulation) assessment section to docs/assessments.md with deliverables and grading criteria
- [X] T100 [P] [US5] Add Week 10 (NVIDIA Isaac) assessment section to docs/assessments.md with evaluation rubric
- [X] T101 [P] [US5] Add Week 12 (Humanoid) assessment section to docs/assessments.md with project requirements
- [X] T102 [P] [US5] Create capstone project description in docs/week-13/05-capstone-project.md with autonomous humanoid requirements
- [X] T103 [P] [US5] Add project timeline to docs/week-13/05-capstone-project.md with milestones and deadlines
- [X] T104 [P] [US5] Add capstone rubric to docs/week-13/05-capstone-project.md with specific grading criteria for each grade level
- [X] T105 [P] [US5] Add implementation tips section to docs/week-13/05-capstone-project.md with suggested approaches (not complete solutions)
- [X] T106 [P] [US5] Add submission guidelines to docs/assessments.md with format requirements and delivery instructions
- [X] T107 [US5] Review all rubrics for clarity and measurability - verify each criterion has specific description
- [X] T108 [US5] Test rubric usability - verify students can self-assess using provided criteria
- [X] T109 [US5] Add assessments.md to sidebars.js navigation for easy access
- [X] T110 [US5] Deploy updated site and verify all assessment documentation is complete and accessible

**Checkpoint**: All user stories complete - Students can navigate, see code examples, set up environment, view diagrams, understand assessments

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Performance optimization, accessibility validation, deployment refinement

- [ ] T111 [P] Run Lighthouse audit on homepage and 5 sample chapters - target >90 all categories
- [ ] T112 [P] Fix any Lighthouse performance issues identified (image optimization, bundle size, caching)
- [ ] T113 [P] Run WCAG 2.1 Level AA accessibility scan with axe-core - fix all violations
- [ ] T114 [P] Test with screen readers (NVDA on Windows, VoiceOver on Mac) - verify all content accessible
- [ ] T115 [P] Add social media preview image (og:image) to static/img/ and configure in docusaurus.config.js
- [ ] T116 [P] Verify sitemap.xml generation in build/ directory for search engine indexing
- [ ] T117 [P] Add robots.txt to static/ directory with appropriate crawl directives
- [ ] T118 [P] Configure meta tags in docusaurus.config.js for SEO (description, keywords, author)
- [ ] T119 [P] Test print CSS - verify chapters are printable with proper page breaks
- [ ] T120 [P] Add deployment status badge to repository README.md showing build status
- [ ] T121 [P] Create CONTRIBUTING.md guide for future content contributors with style guidelines
- [ ] T122 [P] Add link checking to GitHub Actions workflow to detect broken external references
- [ ] T123 [US1] Verify homepage load time <2s on standard broadband (5 Mbps throttle test)
- [ ] T124 [US1] Verify chapter page load time <3s including images and syntax highlighting
- [ ] T125 [US1] Verify search response time <500ms for typical queries
- [ ] T126 Test site on minimum supported screen width (320px) - verify usability
- [ ] T127 Test site on multiple browsers (Chrome, Firefox, Safari, Edge) - verify compatibility
- [ ] T128 Test deployment rollback procedure - verify ability to revert to previous version if needed
- [ ] T129 Run full end-to-end test - verify all acceptance scenarios from spec.md pass
- [ ] T130 Final deployment to production GitHub Pages with all content and optimizations

**Checkpoint**: Site fully polished, performant, accessible, and production-ready

---

## Task Summary

**Total Tasks**: 130

**Task Count by Phase**:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 16 tasks
- Phase 3 (User Story 1 - Navigation): 26 tasks
- Phase 4 (User Story 2 - Code Examples): 16 tasks
- Phase 5 (User Story 3 - Hardware Setup): 15 tasks
- Phase 6 (User Story 4 - Visual Aids): 18 tasks
- Phase 7 (User Story 5 - Assessments): 14 tasks
- Phase 8 (Polish): 20 tasks

**Parallel Opportunities**: 85 tasks marked [P] can run in parallel (65% of total tasks)

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1) = 47 tasks (~36% of total)

**Independent Test Criteria**:
- US1: Navigation works, search functions, mobile-responsive, dark mode toggles
- US2: Code syntax highlighting correct, copy buttons work, tabs/collapse functional
- US3: Installation commands work in clean environment, hardware docs complete
- US4: All images have alt text, diagrams visible in both themes, optimized load times
- US5: All rubrics clear and measurable, submission guidelines complete

**Format Validation**: ✅ All tasks follow checklist format with checkbox, ID, optional [P], required [Story] labels for user story phases, and file paths in descriptions
