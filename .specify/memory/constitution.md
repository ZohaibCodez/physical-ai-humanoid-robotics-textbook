<!--
SYNC IMPACT REPORT
==================
Version Change: 1.1.0 → 1.2.0
Type: MINOR - New UI/UX design principles added

Modified Principles:
- None (existing principles unchanged)

Added Sections:
- Principle XII: Visual Consistency
- Principle XIII: User Experience First
- Principle XIV: Color Psychology
- Principle XV: Professional Polish
- Principle XVI: User Empowerment

Templates Status:
✅ spec-template.md - Compatible (UI/UX requirements can be captured in functional requirements)
✅ plan-template.md - Compatible (Performance Goals and Constraints sections support UX metrics)
✅ tasks-template.md - Compatible (UI/UX tasks can be organized by user story)
⚠ Pending Review: Frontend component consistency audit needed
⚠ Pending Review: Accessibility compliance verification (WCAG 2.1 AA)

Follow-up TODOs:
- Audit existing UI components for visual consistency violations
- Establish color palette constants in theme configuration
- Implement dark mode theming system
- Add accessibility testing to quality gates
- Create UI component library documentation
-->

# Physical AI & Humanoid Robotics Textbook Constitution

**Mission**: Create a comprehensive, AI-native technical textbook for teaching a 13-week Physical AI & Humanoid Robotics course. The textbook will bridge the gap between digital AI and embodied intelligence, enabling students to design, simulate, and deploy humanoid robots.

## Core Principles

### I. Educational Excellence

Content MUST be technically accurate and research-backed. The textbook employs a progressive learning structure from foundations to advanced topics, with a practical, hands-on approach incorporating real-world examples. All explanations must be clear and suitable for students with AI background but new to robotics. Include diagrams, code examples, and visual aids where applicable to enhance comprehension.

**Rationale**: Educational quality is non-negotiable. Students investing time in this course deserve content that is both accurate and pedagogically sound, building confidence through structured progression.

### II. Curriculum Alignment (NON-NEGOTIABLE)

The 13-week course structure MUST be followed strictly. All 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) MUST be covered completely. All specified learning outcomes and assessments MUST be included. The weekly breakdown MUST be preserved exactly as specified with no deviations.

**Rationale**: This textbook serves a specific curriculum with defined learning outcomes. Any deviation compromises the course's integrity and student success metrics.

### III. Technical Depth

Balance theory with practical implementation. Provide working code examples for ROS 2, Python, and relevant frameworks. Include complete setup instructions for required tools and environments. Address hardware requirements and alternatives (cloud vs on-premise solutions).

**Rationale**: Robotics education fails when it is purely theoretical. Students need hands-on skills to succeed in industry roles or research positions.

### IV. Docusaurus Best Practices

Use proper Markdown formatting for optimal rendering. Maintain logical sidebar navigation structure. Leverage MDX features for interactive components where beneficial. Ensure mobile-responsive and accessible design. Optimize for fast page load times and efficient image handling.

**Rationale**: Technical content loses value if presentation is poor. Modern web standards ensure the textbook is accessible across devices and user contexts.

### V. AI-Native Design

Structure content for easy RAG chatbot integration (future phase). Use clear section headings and semantic markup throughout. Maintain consistent terminology across all chapters. Design for future personalization and multi-language translation features.

**Rationale**: AI-assisted learning is the future. Preparing the content structure now enables powerful enhancement features without costly refactoring later.

### VI. Deployment Readiness

Configuration MUST be GitHub Pages compatible. Automated deployment via GitHub Actions is required. Clean, professional URL structure MUST be maintained. Proper SEO and metadata MUST be implemented.

**Rationale**: A textbook that cannot be reliably deployed and accessed fails its primary function. Automation ensures consistency and reduces maintenance burden.

### VII. Code Quality

All code examples MUST be tested and functional before publication. Follow industry best practices and conventions. Include comments and explanations within code blocks. Provide complete setup instructions for every code example.

**Rationale**: Broken code examples destroy student confidence and waste instructional time. Quality code demonstrates professionalism and builds trust.

### VIII. Maintainability

Maintain clear directory structure. Create reusable components and templates. Ensure well-documented configuration. Design for easy content updates and extensions.

**Rationale**: Educational content evolves. A maintainable structure allows updates to keep pace with technological advances in robotics and AI.

### IX. Secure Authentication

Use Better-Auth for modern, secure authentication. Collect user background during signup for personalization. Protect user data and privacy with industry-standard practices.

**Rationale**: Personalized learning requires knowing the learner. Secure authentication enables adaptive content delivery while maintaining user trust through robust privacy protections.

### X. Background Profiling

Ask about software experience (beginner/intermediate/advanced). Ask about hardware access (cloud-only/basic-hardware/full-lab). Store preferences for content adaptation.

**Rationale**: Students have diverse backgrounds and resources. Profiling enables the platform to deliver appropriate content difficulty and hardware-specific guidance, maximizing learning effectiveness.

### XI. Seamless UX

Optional authentication (guest users can still read). Single sign-on ready. Fast, non-intrusive signup flow.

**Rationale**: Friction in authentication drives users away. Guest access lowers barriers to exploration while registered users gain personalization benefits, optimizing for both discovery and engagement.

### XII. Visual Consistency

Use a single cohesive color palette throughout the entire platform. Maintain consistent spacing and typography across all pages and components. Present a professional, modern aesthetic that reflects the technical sophistication of the subject matter. Optimize for dark mode by default to reduce eye strain during extended reading sessions. Implement smooth transitions and animations that enhance rather than distract from the learning experience.

**Rationale**: Visual consistency reduces cognitive load and builds trust. Students focus on learning when the interface is predictable and polished. Dark mode optimization is essential for technical education where users spend hours reading and coding.

### XIII. User Experience First

Prioritize intuitive navigation that allows students to find content effortlessly. Establish clear visual hierarchy to guide attention to key information. Ensure accessibility to all users by meeting WCAG 2.1 AA standards. Optimize for fast loading times to maintain engagement. Design mobile-responsive layouts that work seamlessly across devices from 320px width upward.

**Rationale**: Educational platforms fail when navigation is confusing or content is inaccessible. UX excellence directly impacts learning outcomes—frustrated students disengage, while empowered students thrive.

### XIV. Color Psychology

Select primary colors that convey professionalism and trustworthiness in technical education. Use accent colors that engage attention without causing distraction from content. Choose background colors that are easy on the eyes for extended reading sessions. Ensure text colors provide high contrast for optimal readability. Design interactive elements with clear affordances that signal clickability and state changes.

**Rationale**: Color choices impact learning psychology. Technical content demands colors that reduce fatigue and maintain focus. Proper contrast ensures accessibility and reduces comprehension barriers.

### XV. Professional Polish

Eliminate jarring color transitions that disrupt the reading experience. Maintain consistent button styles, sizes, and behaviors across all interactions. Apply proper spacing and whitespace to improve content scannability. Use professional typography that balances readability with technical aesthetics. Implement smooth micro-interactions that provide feedback and delight without overwhelming users.

**Rationale**: Polish differentiates amateur from professional educational platforms. Students perceive polished interfaces as more credible and are more likely to invest time and effort in the curriculum.

### XVI. User Empowerment

Display clear authentication status so users always know their login state. Provide easy profile access with minimal clicks from any page. Design intuitive logout flows that respect user intent. Ensure the chatbot is always accessible for instant help. Show progress indicators where needed to give students a sense of achievement and direction.

**Rationale**: Empowered users feel in control of their learning journey. Clear system status and easy access to tools reduce anxiety and enable students to focus on mastering robotics concepts rather than fighting the interface.

## Technology Standards

**Platform**: Docusaurus v3.x (REQUIRED)  
**Hosting**: GitHub Pages (REQUIRED)  
**Repository**: Public GitHub repository (REQUIRED)  
**Languages**: Markdown/MDX for content; JavaScript/TypeScript for components  
**Code Examples**: ROS 2, Python 3.10+, relevant robotics frameworks  
**Version Control**: Git with semantic commits and clear history

## Quality & Deployment Standards

**Success Criteria**:
- Complete textbook covering all 13 weeks of curriculum
- Successfully deployed to GitHub Pages
- Professional appearance and intuitive navigation
- All code examples verified and functional
- Submission deadline requirements met (Nov 30, 2025)

**Performance Requirements**:
- Page load times <3 seconds on standard connections
- Mobile-responsive on devices 320px width and above
- WCAG 2.1 Level AA accessibility compliance (REQUIRED)
- SEO score >90 on Lighthouse audits
- Color contrast ratios meeting WCAG AA standards (4.5:1 for normal text, 3:1 for large text)
- Smooth animations at 60fps with no janky transitions
- Consistent UI rendering across modern browsers (Chrome, Firefox, Safari, Edge)

**Testing Gates**:
- All code examples must execute without errors
- Navigation structure must be validated
- Broken links detected and fixed before deployment
- Spelling and grammar checked
- Visual consistency audit (color palette, spacing, typography)
- Accessibility compliance verification (WCAG 2.1 AA automated + manual testing)
- Mobile responsiveness tested on viewport widths 320px-1920px
- Dark mode rendering verified across all pages

## Governance

This constitution supersedes all other project practices and decisions. Any amendments require:
1. Documentation of the proposed change with rationale
2. Impact assessment on existing content and templates
3. Version bump following semantic versioning rules
4. Update of LAST_AMENDED_DATE

**Versioning Policy**:
- MAJOR: Backward incompatible changes (e.g., removing core principles, changing mission)
- MINOR: New principles added or material expansions to guidance
- PATCH: Clarifications, wording improvements, non-semantic refinements

**Compliance**: All content contributions, pull requests, and reviews MUST verify compliance with these principles. Deviations require explicit justification and approval. For runtime development guidance, refer to `.github/copilot-instructions.md`.

**Version**: 1.2.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-07
