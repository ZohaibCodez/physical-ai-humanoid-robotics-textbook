# Data Model: Docusaurus Textbook Site

**Feature**: 001-docusaurus-textbook-site  
**Date**: 2025-12-03  
**Phase**: 1 - Design & Architecture

## Overview

This document defines the key entities and their relationships for the Physical AI & Humanoid Robotics textbook. Since this is a static site, these are logical entities represented as Markdown files with YAML frontmatter, not database records.

## Entity Definitions

### Chapter

**Purpose**: A discrete learning unit covering a specific topic within a module.

**Attributes**:
- `id` (string): Unique identifier, typically the file path (e.g., "week-03-05/01-intro-ros2")
- `title` (string): Chapter display title (e.g., "Introduction to ROS 2")
- `sidebar_position` (number): Order within sidebar (1, 2, 3...)
- `module` (string): Parent module identifier ("week-03-05")
- `learning_objectives` (array[string]): List of measurable learning outcomes
- `content_sections` (array[Section]): Main content blocks (Introduction, Core Concepts, etc.)
- `code_examples` (array[CodeExample]): Embedded or linked code snippets
- `exercises` (array[Exercise]): Hands-on practice activities
- `summary` (array[string]): Key takeaway points
- `further_reading` (array[Link]): External resources and references

**Relationships**:
- Belongs to one Module
- Has many CodeExample (embedded)
- Has many Diagram (embedded)
- Has many Exercise (embedded)

**Validation Rules**:
- MUST have at least 3 learning objectives
- MUST include Introduction and Summary sections
- MUST have at least 1 code example (except intro chapters)
- Title MUST be concise (< 60 characters for sidebar display)
- Sidebar position MUST be unique within module

**File Format** (Markdown with YAML frontmatter):
```markdown
---
sidebar_position: 1
title: Introduction to ROS 2
---

# Introduction to ROS 2

## Learning Objectives
By the end of this chapter, you will be able to:
- Objective 1
- Objective 2

## Content...
```

---

### Module

**Purpose**: A thematic grouping of chapters spanning one or more weeks of curriculum.

**Attributes**:
- `id` (string): Module directory name (e.g., "week-03-05")
- `label` (string): Display name (e.g., "Weeks 3-5: ROS 2 Fundamentals")
- `description` (string): Brief module overview
- `weeks` (array[number]): Week numbers covered ([3, 4, 5])
- `chapters` (array[string]): List of chapter IDs in sequence
- `assessment` (Assessment): Module evaluation criteria
- `coming_soon` (boolean): Whether module is incomplete
- `release_date` (string): Planned availability date (ISO 8601)

**Relationships**:
- Has many Chapter (3-6 typical)
- Has one Assessment

**Validation Rules**:
- MUST have 3-6 chapters for proper depth
- Label MUST include week range
- Assessment MUST be defined for each module

**File Format** (_category_.json in module directory):
```json
{
  "label": "Weeks 3-5: ROS 2 Fundamentals",
  "position": 2,
  "link": {
    "type": "generated-index",
    "description": "Learn the fundamentals of ROS 2, the Robot Operating System 2."
  }
}
```

---

### CodeExample

**Purpose**: A standalone, runnable code snippet demonstrating a concept.

**Attributes**:
- `language` (string): Programming language ("python", "xml", "yaml", "bash", "cpp")
- `code_content` (string): The actual code (embedded in Markdown code fence)
- `description` (string): Brief explanation of what the code does
- `setup_instructions` (string): Prerequisites and environment setup
- `expected_output` (string): What students should see when running the code
- `filename` (string, optional): Suggested filename for students
- `comments_embedded` (boolean): Whether code includes inline comments

**Relationships**:
- Belongs to one Chapter
- May reference Diagram (for architecture explanation)

**Validation Rules**:
- Language MUST be specified for syntax highlighting
- Code MUST include inline comments for complex sections
- Setup instructions MUST be provided if prerequisites exist
- Expected output MUST be shown (as comment or separate block)

**File Format** (embedded in Chapter Markdown):
````markdown
### Example: Simple ROS 2 Publisher

This example creates a basic ROS 2 publisher node that sends messages.

**Prerequisites**: ROS 2 Humble installed, workspace sourced

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [simple_publisher]: Publishing: "Hello World: 0"
[INFO] [simple_publisher]: Publishing: "Hello World: 1"
[INFO] [simple_publisher]: Publishing: "Hello World: 2"
```

**To Run**:
```bash
python3 simple_publisher.py
```
````

---

### Diagram

**Purpose**: Visual aid explaining abstract concepts, architectures, or processes.

**Attributes**:
- `filename` (string): Image file path (e.g., "img/diagrams/ros2-architecture.png")
- `alt_text` (string): Descriptive alternative text for accessibility
- `caption` (string): Brief description or title
- `source_attribution` (string, optional): Credit for open-source diagrams
- `format` (string): Image format ("png", "svg", "webp")
- `chapter_reference` (string): Chapter ID where diagram appears

**Relationships**:
- Belongs to one or more Chapter
- May be referenced by CodeExample

**Validation Rules**:
- Alt text MUST be descriptive (not "diagram" or "image")
- Format MUST be web-optimized (< 500KB file size)
- Attribution MUST be provided for external sources
- Caption MUST explain diagram purpose

**File Format** (embedded in Chapter Markdown):
```markdown
![ROS 2 node graph showing publisher-subscriber communication with message flow arrows](/img/diagrams/ros2-pub-sub.png)
*Figure 3.1: ROS 2 publisher-subscriber pattern. The publisher node sends messages to a topic, and subscriber nodes receive them asynchronously.*
```

**Accessibility Note**:
- Alt text describes the diagram content (architecture, flow, relationships)
- Caption provides additional context
- Complex diagrams also explained in prose for screen reader users

---

### Assessment

**Purpose**: Evaluation criteria for module completion, including project requirements and rubrics.

**Attributes**:
- `module_id` (string): Associated module identifier
- `project_description` (string): Overview of assessment task
- `requirements` (array[Requirement]): Specific deliverables
- `rubric` (object): Grading criteria with point values
- `submission_format` (string): How students submit (GitHub repo, video, report)
- `estimated_time` (number): Expected completion hours
- `due_date_relative` (string): When assessment is due (e.g., "End of Week 5")

**Relationships**:
- Belongs to one Module
- References Chapter (for relevant concepts)

**Validation Rules**:
- Requirements MUST be measurable and specific
- Rubric MUST include criteria for each grade level
- Submission format MUST be clearly defined

**File Format** (section in assessments.md or module page):
```markdown
## Module 2 Assessment: ROS 2 Mini-Project

### Project Description
Create a simple ROS 2 application with two nodes: a publisher and a subscriber. The publisher should send sensor data (simulated), and the subscriber should process and log the data.

### Requirements
1. **Publisher Node** (30 points)
   - Publishes sensor data at 10Hz
   - Uses custom message type
   - Includes proper logging

2. **Subscriber Node** (30 points)
   - Subscribes to sensor topic
   - Processes incoming data
   - Logs processed results

3. **Launch File** (20 points)
   - Starts both nodes
   - Configures parameters
   - Includes comments

4. **Documentation** (20 points)
   - README with setup instructions
   - Code comments
   - Architecture diagram

### Rubric
| Criteria | Excellent (90-100%) | Good (75-89%) | Satisfactory (60-74%) | Needs Improvement (<60%) |
|----------|-------------------|---------------|----------------------|--------------------------|
| Functionality | All requirements met, code runs flawlessly | Minor issues, mostly works | Partial implementation | Major functionality missing |
| Code Quality | Clean, well-documented, follows ROS 2 conventions | Generally clean, adequate comments | Messy but functional | Poor structure, hard to read |
| Documentation | Comprehensive, clear, professional | Good coverage, minor gaps | Basic documentation present | Inadequate or missing |

### Submission
- **Format**: GitHub repository link
- **Due**: End of Week 5
- **Estimated Time**: 6-8 hours
```

---

### Exercise

**Purpose**: Hands-on practice activity within a chapter for skill reinforcement.

**Attributes**:
- `title` (string): Exercise name
- `difficulty` (string): "Beginner", "Intermediate", "Advanced"
- `instructions` (string): Step-by-step guide
- `hints` (array[string], optional): Guidance without full solution
- `estimated_time` (number): Expected completion minutes
- `solution_available` (boolean): Whether solution is provided
- `solution_link` (string, optional): Path to solution file/repo

**Relationships**:
- Belongs to one Chapter
- May reference CodeExample (as starting point)

**Validation Rules**:
- Instructions MUST be clear and numbered
- Difficulty MUST match chapter complexity
- Hints preferred over full solutions

**File Format** (embedded in Chapter Markdown):
```markdown
## Hands-On Exercise: Create Your First ROS 2 Node

**Difficulty**: Beginner  
**Estimated Time**: 15 minutes

### Instructions
1. Create a new Python file called `my_first_node.py`
2. Import the required ROS 2 libraries (`rclpy`, `Node`)
3. Define a class `MyFirstNode` that inherits from `Node`
4. In the constructor, call `super().__init__('my_first_node')`
5. Add a timer that prints "Hello from ROS 2!" every 2 seconds
6. Implement the main function to initialize and spin the node
7. Run your node and verify the output

### Hints
- Use `create_timer(timer_period_sec, callback_function)` to set up periodic execution
- The callback function should use `self.get_logger().info(message)` to print

### Challenge
Modify the node to print the current timestamp with each message.

:::tip
If you get stuck, refer to the Simple Publisher example above.
:::
```

---

## Entity Relationships Diagram

```
Module (1) ----< (many) Chapter
Chapter (1) ----< (many) CodeExample
Chapter (1) ----< (many) Diagram
Chapter (1) ----< (many) Exercise
Module (1) ---- (1) Assessment
```

## File System Mapping

| Entity | Storage Location | Format |
|--------|-----------------|--------|
| Chapter | `docs/week-XX-YY/*.md` | Markdown with YAML frontmatter |
| Module | `docs/week-XX-YY/_category_.json` | JSON configuration |
| CodeExample | Embedded in Chapter Markdown | Fenced code blocks |
| Diagram | `static/img/diagrams/*.{png,svg,webp}` | Image files |
| Assessment | `docs/assessments.md` or module page | Markdown section |
| Exercise | Embedded in Chapter Markdown | Markdown section |

## State Transitions

### Chapter Lifecycle
1. **Draft**: Markdown file created, outline only
2. **Content Complete**: All sections written, examples added
3. **Peer Review**: Domain expert review in progress
4. **Revision**: Incorporating feedback
5. **Approved**: Ready for deployment
6. **Deployed**: Live on GitHub Pages
7. **Maintained**: Periodic updates for accuracy

### Module Lifecycle
1. **Planned**: In sidebar as "Coming Soon"
2. **In Progress**: Some chapters complete, others draft
3. **Complete**: All chapters approved
4. **Deployed**: Module accessible to students
5. **Maintained**: Updates as needed

## Data Validation

**Pre-Deployment Checks**:
- [ ] All chapters have required frontmatter
- [ ] Code examples specify language
- [ ] Diagrams have alt text
- [ ] Assessments have rubrics
- [ ] No broken internal links
- [ ] Sidebar positions unique within module

**Automated Validation** (in CI/CD):
- Lighthouse checks (performance, accessibility)
- Link checker (broken links)
- Markdown linter (formatting consistency)
- Docker tests (code example functionality)

## Scalability Considerations

**Current Scope**: 13 weeks, ~30 chapters, 100+ code examples  
**Storage**: Git repository, ~50-100MB total  
**Build Time**: ~3-4 minutes for full site generation  
**Search Index**: Docusaurus default search handles <500 pages efficiently

**Future Growth**:
- Additional modules: Simply add new week-XX-YY directories
- Translations: Docusaurus i18n plugin supports multiple languages
- Versioning: Docusaurus versioning for major curriculum updates

## Next Steps

1. ✅ Data model complete
2. Generate `contracts/` directory with configuration schemas
3. Generate `quickstart.md` for contributors
4. Update agent context with new technologies
5. Re-run Constitution Check
