# Contributing to Physical AI and Humanoid Robotics Textbook

Thank you for your interest in contributing to this open educational resource! This guide will help you contribute content, fix errors, and improve the learning experience.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How to Contribute](#how-to-contribute)
- [Content Guidelines](#content-guidelines)
- [Style Guide](#style-guide)
- [Development Setup](#development-setup)
- [Submission Process](#submission-process)

---

## Code of Conduct

This project is dedicated to providing an inclusive and respectful learning environment. We expect all contributors to:

- Be respectful and constructive in discussions
- Focus on improving educational content quality
- Welcome newcomers and help them get started
- Provide constructive feedback on contributions

---

## How to Contribute

### Types of Contributions

1. **Content Improvements**
   - Fix typos, grammatical errors, or unclear explanations
   - Add missing code examples or clarify existing ones
   - Improve diagrams and visual aids
   - Update outdated information or broken links

2. **New Content**
   - Add supplementary examples or exercises
   - Create additional diagrams or illustrations
   - Write troubleshooting tips based on real experiences
   - Add references to recent papers or resources

3. **Technical Improvements**
   - Fix website bugs or rendering issues
   - Improve site performance or accessibility
   - Add new features to enhance learning (e.g., interactive demos)
   - Update dependencies and security patches

4. **Documentation**
   - Improve setup instructions
   - Add FAQs based on common questions
   - Document best practices for using the textbook

---

## Content Guidelines

### Educational Quality

- **Accuracy**: All technical content must be technically accurate and up-to-date
- **Clarity**: Explanations should be clear and accessible to the target audience (undergraduate/graduate students)
- **Completeness**: Code examples should be complete and runnable
- **Context**: Provide context for why concepts matter and how they're used in practice

### Target Audience

- Students with basic programming knowledge (Python preferred)
- Familiarity with linear algebra and calculus (undergraduate level)
- Interest in robotics, AI, or mechatronics
- No prior ROS or robotics experience required

### Content Structure

Each chapter should include:
- **Learning Objectives**: Clear goals for what students will learn
- **Conceptual Explanations**: Theory with visual aids
- **Practical Examples**: Working code snippets
- **Exercises**: Practice problems (when applicable)
- **References**: Links to official docs and research papers

---

## Style Guide

### Writing Style

- **Tone**: Professional but conversational, encouraging
- **Person**: Use second person ("you") for instructions, third person for concepts
- **Tense**: Present tense for current state, future tense for upcoming sections
- **Voice**: Active voice preferred over passive

### Markdown Formatting

```markdown
# Chapter Title (H1 - one per file)

## Section Title (H2)

### Subsection Title (H3)

**Bold** for emphasis, *italic* for definitions

- Use bullet lists for unordered items
1. Use numbered lists for sequential steps

`inline code` for commands, variables, file names

\`\`\`python
# Code blocks with language specified
def example():
    pass
\`\`\`
```

### Code Examples

#### Python Style (PEP 8)
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    """A minimal ROS 2 publisher node."""
    
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        """Publish a message every timer tick."""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

#### Code Block Guidelines
- Include necessary imports
- Add docstrings for functions/classes
- Use meaningful variable names
- Add inline comments for non-obvious logic
- Keep examples under 50 lines when possible
- Provide full context (not just snippets)

### URDF/XML Style
```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
</robot>
```

### Command Line Examples
```bash
# Ubuntu 22.04 ROS 2 Humble installation
sudo apt update
sudo apt install ros-humble-desktop

# Source the setup file
source /opt/ros/humble/setup.bash

# Verify installation
ros2 --version
```

### Image and Diagram Guidelines

- **File Format**: PNG for diagrams, SVG for vector graphics, WebP with PNG fallback for photos
- **Resolution**: Minimum 1024px width for technical diagrams
- **Alt Text**: Required for all images, describe content for screen readers
- **Captions**: Use figure captions to explain diagram content
- **Attribution**: Credit sources for external images

Example:
```markdown
![ROS 2 architecture showing nodes communicating via topics and services](../static/img/diagrams/ros2-architecture.svg)

*Figure 1: ROS 2 publish-subscribe architecture with multiple nodes*
```

### Links

- Use descriptive link text (not "click here")
- Link to official documentation when available
- Check links before submitting (use `npm run build` to validate)
- Use relative links for internal pages: `[Setup Guide](../appendix/setup-instructions.md)`

---

## Development Setup

### Prerequisites

- Node.js 18+ and npm
- Git
- Text editor (VS Code recommended)

### Local Development

1. **Fork and Clone**
   ```bash
   git clone https://github.com/YOUR_USERNAME/physical-ai-humanoid-robotics-textbook.git
   cd physical-ai-humanoid-robotics-textbook
   ```

2. **Install Dependencies**
   ```bash
   npm install
   ```

3. **Start Development Server**
   ```bash
   npm start
   ```
   Opens browser at `http://localhost:3000/physical-ai-humanoid-robotics-textbook/`

4. **Build for Production**
   ```bash
   npm run build
   ```
   Generates static files in `build/` directory

5. **Test Build Locally**
   ```bash
   npm run serve
   ```

### Project Structure

```
physical-ai-humanoid-robotics-textbook/
├── docs/                          # All documentation content
│   ├── intro.md                  # Homepage content
│   ├── week-01-02/               # Week 1-2 chapters
│   ├── week-03-05/               # Week 3-5 chapters
│   ├── week-06-07/               # Week 6-7 chapters
│   ├── week-08-10/               # Week 8-10 chapters
│   ├── week-11-12/               # Week 11-12 chapters
│   ├── week-13/                  # Week 13 chapters
│   ├── assessments.md            # Assessment rubrics
│   └── appendix/                 # Setup guides, references
├── static/                        # Static assets
│   ├── img/                      # Images and diagrams
│   └── files/                    # Downloadable resources
├── src/                          # React components (if needed)
├── sidebars.js                   # Navigation structure
├── docusaurus.config.js          # Site configuration
└── package.json                  # Dependencies

```

### Making Changes

1. Create a new branch:
   ```bash
   git checkout -b feature/improve-ros2-chapter
   ```

2. Make your changes in the appropriate `docs/` directory

3. Test locally:
   ```bash
   npm start
   ```

4. Verify build succeeds:
   ```bash
   npm run build
   ```

5. Commit with descriptive message:
   ```bash
   git add .
   git commit -m "Improve ROS 2 publisher example with error handling"
   ```

6. Push to your fork:
   ```bash
   git push origin feature/improve-ros2-chapter
   ```

---

## Submission Process

### Pull Request Guidelines

1. **One Logical Change Per PR**
   - Focus on a single topic or fix
   - Avoid mixing unrelated changes

2. **Descriptive Title**
   - Good: "Add inverse kinematics example to Week 8 manipulation chapter"
   - Bad: "Update docs"

3. **Detailed Description**
   - Explain what changed and why
   - Reference related issues if applicable
   - Include screenshots for visual changes
   - Note any breaking changes

4. **PR Template** (use this format)
   ```markdown
   ## Description
   Brief summary of changes

   ## Type of Change
   - [ ] Content improvement (typo, clarity, examples)
   - [ ] New content (new section, diagram, exercise)
   - [ ] Bug fix (rendering issue, broken link)
   - [ ] Feature (new component, functionality)

   ## Checklist
   - [ ] Content is technically accurate
   - [ ] Code examples run without errors
   - [ ] Links are valid and work
   - [ ] Images have alt text
   - [ ] Build passes (`npm run build`)
   - [ ] Follows style guide

   ## Related Issues
   Closes #123
   ```

### Review Process

1. **Automated Checks**
   - Build must succeed
   - Link checker must pass
   - No ESLint errors

2. **Manual Review**
   - Technical accuracy verified
   - Writing quality assessed
   - Code examples tested
   - Visual consistency checked

3. **Feedback and Iteration**
   - Address reviewer comments
   - Make requested changes
   - Re-request review when ready

4. **Merge**
   - Approved PRs merged by maintainers
   - Automatic deployment to GitHub Pages

---

## Common Tasks

### Adding a New Chapter

1. Create markdown file: `docs/week-XX/YY-chapter-title.md`
2. Add frontmatter:
   ```markdown
   ---
   sidebar_position: YY
   title: Chapter Title
   ---
   ```
3. The chapter will auto-appear in sidebar (via `sidebars.js` autogenerate)

### Adding Images

1. Save to `static/img/diagrams/` or `static/img/screenshots/`
2. Reference in markdown:
   ```markdown
   ![Alt text](../../static/img/diagrams/my-diagram.png)
   ```
3. Add caption:
   ```markdown
   *Figure 1: Description of the diagram*
   ```

### Adding Code Examples

1. Use proper code fences with language:
   ````markdown
   ```python
   # Your code here
   ```
   ````

2. For longer examples, consider collapsible sections:
   ```markdown
   <details>
   <summary>Click to expand full code</summary>
   
   ```python
   # Long code example...
   ```
   
   </details>
   ```

### Updating Dependencies

1. Check for updates:
   ```bash
   npm outdated
   ```

2. Update specific package:
   ```bash
   npm update @docusaurus/core@latest
   ```

3. Test thoroughly after updates

---

## Getting Help

- **Questions**: Open a [GitHub Discussion](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/discussions)
- **Bugs**: File an [Issue](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/issues)
- **Chat**: Join our Discord (link in README)
- **Email**: [maintainer email]

---

## License

By contributing, you agree that your contributions will be licensed under the same license as the project (see LICENSE file).

---

## Recognition

Contributors will be acknowledged in:
- GitHub contributors page
- Project README
- Optional: Contributors section on website

Thank you for helping make robotics education more accessible! 🤖
