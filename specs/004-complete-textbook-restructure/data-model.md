# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: Complete Textbook Restructure & Content Generation  
**Date**: 2025-12-07  
**Purpose**: Define structure and relationships of textbook content entities

## Entity Relationship Diagram

```
Textbook
├── Preface (intro.md)
├── Parts (7) *
│   └── Chapters (27 total, 3-5 per Part) *
│       ├── Chapter Index (index.md)
│       └── Lessons (87 total, 3-5 per Chapter) *
│           ├── Frontmatter
│           ├── Content Sections *
│           ├── Code Examples *
│           ├── Exercises *
│           └── Navigation Links
└── Appendices (5) *
    └── Reference Content

* = one-to-many relationship
```

## Core Entities

### 1. Textbook

**Purpose**: Root container for all educational content

**Properties**:
- `title`: "Physical AI & Humanoid Robotics"
- `subtitle`: "A Comprehensive 13-Week Course"
- `version`: "1.0.0"
- `audience`: ["students", "self-learners", "bootcamp-attendees"]
- `level`: "graduation-level" (undergraduate degree equivalent)
- `duration`: "13 weeks" (195-260 hours total)
- `prerequisites`: ["Python basics", "Linear algebra fundamentals", "Basic calculus"]

**Relationships**:
- Has one `Preface`
- Contains 7 `Part` entities
- Contains 5 `Appendix` entities
- Managed by `docusaurus.config.js` (site-wide configuration)
- Navigation defined in `sidebars.js`

**File Structure**:
```
docs/
├── intro.md (Preface)
├── part-01-foundations/
├── part-02-ros2/
├── part-03-simulation/
├── part-04-nvidia-isaac/
├── part-05-humanoid-development/
├── part-06-conversational-robotics/
├── part-07-capstone/
└── appendices/
```

---

### 2. Preface

**Purpose**: Welcome, course overview, and usage guide

**Properties**:
- `file_path`: "docs/intro.md"
- `sidebar_position`: 0
- `title`: "Welcome to Physical AI"

**Required Sections**:
1. Welcome message
2. Course philosophy (why Physical AI matters)
3. Target audience (who this book is for)
4. Prerequisites (required knowledge)
5. How to use this book (reading strategies)
6. 13-week learning path overview (week-by-week outline)
7. Hardware/software requirements summary
8. Community and support resources

**Word Count**: 800-1200 words

**Validation Rules**:
- Must include all 8 required sections
- Must reference all 7 Parts
- Must provide clear guidance for different learner types (student/self-learner/bootcamp)

---

### 3. Part

**Purpose**: Major division representing a significant learning phase

**Properties**:
- `id`: 1-7 (numeric identifier)
- `slug`: kebab-case name (e.g., "part-01-foundations")
- `title`: Full descriptive title
- `position`: 1-7 (ordering)
- `description`: 1-2 sentence overview
- `duration`: Estimated weeks
- `chapter_count`: 3-5 chapters per part

**Relationships**:
- Contains multiple `Chapter` entities (3-5)
- Parent of `Part`
- Defined by `_category_.json` in part directory

**File Structure**:
```
part-XX-name/
├── _category_.json
├── chapter-01-name/
├── chapter-02-name/
└── chapter-03-name/
```

**_category_.json Format**:
```json
{
  "label": "Part 1: Foundations of Physical AI",
  "position": 1,
  "link": {
    "type": "generated-index",
    "description": "Understanding the fundamentals of Physical AI and embodied intelligence."
  }
}
```

**Validation Rules**:
- Part numbering must be sequential (1-7)
- Part directories must match naming pattern: `part-0X-kebab-case`
- Each part must have valid `_category_.json`
- Chapters within part must have sequential positions

---

### 4. Chapter

**Purpose**: Collection of related lessons on a major topic

**Properties**:
- `id`: Format "CC" where C is chapter number (01-27)
- `slug`: kebab-case name (e.g., "chapter-01-introduction")
- `title`: Full descriptive title
- `position`: Within parent Part (1-5)
- `lesson_count`: Number of lessons (3-5 typical)
- `estimated_time`: Hours to complete (e.g., "8-12 hours")
- `prerequisites`: List of prior chapters/concepts

**Relationships**:
- Parent: One `Part`
- Contains: Multiple `Lesson` entities (3-5)
- Has: One `ChapterIndex`
- Defined by: `_category_.json` in chapter directory

**File Structure**:
```
chapter-XX-name/
├── _category_.json
├── index.md (Chapter Index)
├── 01-lesson-name.md
├── 02-lesson-name.md
└── 03-lesson-name.md
```

**_category_.json Format**:
```json
{
  "label": "Chapter 1: Introduction to Physical AI",
  "position": 1,
  "link": {
    "type": "doc",
    "id": "part-01-foundations/chapter-01-introduction/index"
  }
}
```

**Validation Rules**:
- Chapter numbering sequential within part
- Must have index.md
- Lesson files numbered sequentially (01, 02, 03...)

---

### 5. ChapterIndex

**Purpose**: Overview and navigation hub for chapter

**Properties**:
- `file_path`: "chapter-XX-name/index.md"
- `sidebar_position`: 0
- `title`: Same as parent Chapter

**Required Sections**:
1. Brief overview (1 paragraph)
2. "What You'll Learn" (3-5 key topics bullet list)
3. Prerequisites (prior knowledge required)
4. Estimated Time (hours)
5. Chapter Structure (lesson list with 1-line descriptions)

**Word Count**: 300-500 words

**Markdown Structure**:
```markdown
---
sidebar_position: 0
---

# Chapter X: [Title]

[Brief overview paragraph]

## What You'll Learn

- Key topic 1
- Key topic 2
- Key topic 3

## Prerequisites

- Prerequisite 1
- Prerequisite 2

## Estimated Time

X hours

## Chapter Structure

1. [Lesson 1 title] - [one-line description]
2. [Lesson 2 title] - [one-line description]
...

Let's dive in!
```

**Validation Rules**:
- Must include all 5 required sections
- Lesson list must match actual lesson files in directory
- Prerequisites must reference real content (no forward references)

---

### 6. Lesson

**Purpose**: Atomic learning unit covering specific topic

**Properties**:
- `file_path`: "XX-kebab-case-title.md"
- `sidebar_position`: Sequential number within chapter
- `title`: Descriptive lesson title
- `description`: SEO meta description (<160 chars)
- `word_count`: 1200-1800 words (target range)
- `read_time`: 8-12 minutes
- `difficulty_level`: "beginner" | "intermediate" | "advanced"

**Relationships**:
- Parent: One `Chapter`
- Contains: Multiple `ContentSection` entities
- Contains: Multiple `CodeExample` entities (2-4)
- Contains: Multiple `Exercise` entities (3-5)
- Contains: Multiple `Admonition` entities (2-4 minimum)
- References: Previous/next `Lesson` for navigation

**Frontmatter Structure**:
```yaml
---
sidebar_position: 1
title: "Lesson Title"
description: "Brief description for SEO under 160 characters"
---
```

**Required Content Sections** (in order):
1. **Learning Objectives** (3-5 bullet points)
2. **Introduction** (2-3 engaging paragraphs)
3. **Main Content** (3-5 major sections with subsections)
4. **Hands-On Practice** (at least 1 exercise)
5. **Key Takeaways** (4-6 concise bullet points)
6. **Review Questions** (3-5 questions)
7. **Further Reading** (3-4 authoritative resources)
8. **What's Next** (transition to next lesson with link)

**Validation Rules**:
- Word count within 1200-1800 range
- All required sections present
- At least 2 code examples (for technical lessons)
- At least 1 hands-on exercise
- At least 2 admonitions (tip, warning, note, info, danger)
- Next lesson link valid (resolves without 404)
- Description length ≤ 160 characters
- Title unique within textbook

---

### 7. ContentSection

**Purpose**: Major topical division within lesson

**Properties**:
- `heading_level`: 2 (##) or 3 (###)
- `title`: Section heading text
- `order`: Sequential position within lesson
- `type`: "theory" | "example" | "explanation" | "implementation" | "analysis"

**Content Components**:
- **Paragraphs**: Explanatory text (conversational, professional tone)
- **Mathematical Content**: Equations, derivations (when applicable)
- **Diagrams**: Descriptions of what should be visualized
- **Tables**: Comparison data, specifications, reference info
- **Lists**: Structured information (bullet or numbered)

**Graduation-Level Requirements**:
- **Theory Sections**: Include mathematical formulation + intuition
- **Implementation Sections**: Show algorithm pseudocode + code
- **Analysis Sections**: Compare alternatives with tradeoffs

**Structure Pattern**:
```markdown
## Main Section Title

[Introductory paragraph explaining what and why]

### Subsection Title

[Detailed explanation with examples]

**Key Concept**: [Highlighted important point]

[Additional explanation paragraphs]

:::tip Pro Tip
[Best practice or optimization]
:::

### Subsection Title 2

[Continue logical flow]
```

**Validation Rules**:
- Heading hierarchy valid (no skipping levels: H2 → H4)
- Sections build logically (concepts introduced before used)
- No orphaned content (all paragraphs in a section)

---

### 8. CodeExample

**Purpose**: Complete, runnable code demonstrating technique

**Properties**:
- `language`: "python" | "bash" | "xml" | "yaml" | "cpp" | "markdown"
- `type`: "complete" | "snippet" | "output"
- `purpose`: One-sentence description
- `line_count`: Number of lines
- `has_comments`: boolean (required: true)
- `has_imports`: boolean (for Python/C++)
- `has_output`: boolean (shows expected result)
- `version_info`: Required versions (e.g., "ROS 2 Humble, Python 3.10+")

**Structure Requirements**:

**Complete Example** (primary teaching code):
```python
#!/usr/bin/env python3
"""
Module docstring explaining purpose
"""

# Imports
from typing import List, Optional
import numpy as np

# Configuration
class Config:
    """Configuration parameters."""
    param1: float = 1.0

# Main implementation
class AlgorithmName:
    """
    Detailed docstring with:
    - Purpose
    - Attributes
    - Example usage
    """
    
    def __init__(self, config: Config):
        self.config = config
        
    def compute(self, input_data: np.ndarray) -> np.ndarray:
        """
        Method docstring.
        
        Args:
            input_data: Description
            
        Returns:
            Result description
        """
        # Step 1: Validation
        # Step 2: Processing
        # Step 3: Return
        pass

# Example usage
if __name__ == "__main__":
    # Demonstration with expected output
    pass
```

**Code Snippet** (focused on specific technique):
```python
# Quick example showing specific concept
result = function_call(parameter)
print(f"Result: {result}")
# Output: Result: 42
```

**Graduation-Level Standards**:
- **Type Hints**: All function signatures
- **Docstrings**: Google style (Args, Returns, Raises)
- **Error Handling**: Try-except with logging (for complete examples)
- **Logging**: Appropriate levels (debug/info/warning/error)
- **Testing**: Example usage showing success case
- **Performance**: Specify complexity (time/space) in comments
- **Version**: Specify tested environment

**Validation Rules**:
- Code must be syntactically valid (can be parsed)
- Python code must follow PEP 8 style
- Complete examples must include imports, setup, usage
- Must specify language for syntax highlighting
- Must include comments explaining non-obvious logic
- For algorithms, must cite source (paper, standard, original)

---

### 9. Exercise

**Purpose**: Practical activity for applying concepts

**Properties**:
- `title`: Descriptive name
- `type`: "analytical" | "implementation" | "debugging" | "design"
- `difficulty`: "guided" | "specification" | "algorithm" | "open-ended"
- `estimated_time`: Minutes to complete
- `requires_code`: boolean
- `provides_starter_code`: boolean

**Structure Requirements**:

```markdown
### Exercise: [Title]

[Brief context paragraph explaining scenario]

**Goal**: [One sentence describing what to accomplish]

**Type**: [analytical/implementation/debugging/design]

**Estimated Time**: [X minutes]

**Prerequisites**: 
- [Concept/lesson required]
- [Tool/environment needed]

**Tasks**:
1. [Specific task step 1]
2. [Specific task step 2]
3. [Specific task step 3]

**Starter Code** (if applicable):
```python
# Partial implementation to complete
def function_to_implement():
    # TODO: Implement this
    pass
```

**Expected Output**:
[Show or describe what success looks like]
```
[example output or visualization]
```

**Hints** (optional):
- [Hint 1 for common stuck points]
- [Hint 2]

**Success Criteria**:
- [ ] Criterion 1 (specific, testable)
- [ ] Criterion 2
- [ ] Criterion 3
```

**Exercise Type Definitions**:

| Type | Description | Example | Assessment |
|------|-------------|---------|------------|
| **Analytical** | Calculate/derive using math/formulas | "Calculate Jacobian for given pose" | Correct numerical answer |
| **Implementation** | Code solution from specification | "Implement A* pathfinder" | Code runs, produces correct output |
| **Debugging** | Fix provided buggy code | "Fix race condition in sensor node" | Code runs without errors |
| **Design** | Make architectural decision with justification | "Choose sensors for outdoor nav" | Justified tradeoff analysis |

**Difficulty Progression**:

| Difficulty | Characteristics | Support Provided |
|------------|-----------------|------------------|
| **Guided** | Fill in blanks, heavy scaffolding | Starter code 80% complete, hints |
| **Specification** | Implement from clear requirements | Detailed spec, no code provided |
| **Algorithm** | Implement from pseudocode/paper | Pseudocode, expected behavior |
| **Open-Ended** | Design solution, multiple valid approaches | Problem statement, constraints |

**Validation Rules**:
- Clear, unambiguous goal statement
- Numbered steps (for guided/specification types)
- Expected output specified (enables self-assessment)
- Difficulty appropriate for lesson position
- At least 1 exercise per lesson

---

### 10. Admonition

**Purpose**: Highlight important information, best practices, warnings

**Properties**:
- `type`: "tip" | "warning" | "danger" | "note" | "info"
- `title`: Optional custom heading
- `content`: Markdown text

**Docusaurus Syntax**:
```markdown
:::tip Pro Tip
[Content explaining best practice or optimization]
:::

:::warning Important
[Critical information or common pitfall]
:::

:::danger Safety Critical
[Dangerous operation or data loss risk]
:::

:::note Key Concept
[Important theoretical foundation]
:::

:::info Related Topics
[Additional context or cross-references]
:::
```

**Usage Guidelines**:

| Type | Purpose | When to Use | Example |
|------|---------|-------------|---------|
| **tip** | Best practices, optimizations, pro techniques | Share expert knowledge | "Use `ros2 topic hz` to measure actual publish rate" |
| **warning** | Common pitfalls, errors to avoid, non-obvious issues | Prevent frustration | "QoS mismatch will silently drop messages" |
| **danger** | Critical errors, data loss, hardware damage | Prevent serious problems | "Incorrect joint limits can damage real robot" |
| **note** | Background information, additional context | Enhance understanding | "This algorithm was introduced in Dijkstra (1959)" |
| **info** | Related topics, further reading, alternative approaches | Expand knowledge | "See Chapter 8 for advanced path planning techniques" |

**Frequency Requirements**:
- Minimum 2-4 admonitions per lesson
- At least 1 "tip" or "warning" in technical lessons
- "danger" when discussing real hardware or destructive operations

**Validation Rules**:
- Admonition syntax valid (Docusaurus renders correctly)
- Content provides actionable information (not vague warnings)
- Placement contextually appropriate (near related content)

---

### 11. Appendix

**Purpose**: Supplementary reference material

**Properties**:
- `id`: "A" | "B" | "C" | "D" | "E"
- `title`: Descriptive title
- `purpose`: What information it provides
- `file_path`: "docs/appendices/[name].md"

**Five Required Appendices**:

#### Appendix A: Hardware Shopping Guide
- **Content Requirements**:
  - At least 3 hardware configuration options (budget/mid-range/professional)
  - Specific model recommendations with specifications
  - Approximate pricing with "market fluctuations" disclaimer
  - Selection criteria (help readers choose alternatives)
  - Cloud GPU alternatives (AWS, Google Cloud, Colab)
- **Word Count**: 1500-2500 words

#### Appendix B: Software Installation
- **Content Requirements**:
  - Ubuntu 22.04 LTS installation (primary)
  - ROS 2 Humble installation (step-by-step)
  - Gazebo setup (Classic and Garden)
  - Unity Robotics Hub setup
  - NVIDIA Isaac Sim installation
  - Development tools (VS Code, Git, Python environments)
  - Windows/macOS alternatives (mentioned but not detailed)
- **Word Count**: 2000-3000 words

#### Appendix C: Troubleshooting Guide
- **Content Requirements**:
  - At least 20 common issues with solutions
  - Organized by topic (ROS 2, Gazebo, Isaac, hardware, networking)
  - Diagnostic steps (how to identify the problem)
  - Solutions with commands/code
  - Links to official documentation for complex issues
- **Word Count**: 2500-3500 words

#### Appendix D: Resources and References
- **Content Requirements**:
  - At least 30 curated external resources
  - Official documentation links
  - Research papers (foundational and recent)
  - Video tutorials (vetted quality channels)
  - Online courses (complementary topics)
  - Community resources (forums, Slack/Discord, GitHub)
- **Word Count**: 1000-1500 words

#### Appendix E: Glossary of Terms
- **Content Requirements**:
  - At least 50 technical terms defined
  - Clear, concise definitions (1-3 sentences)
  - Accessible to beginners
  - Alphabetically organized
  - Cross-references to lessons where used
- **Word Count**: 1500-2000 words

**Validation Rules**:
- All 5 appendices present
- Minimum content thresholds met
- External links valid (no 404s)
- Hardware recommendations available for purchase
- Software installation steps tested on Ubuntu 22.04

---

## Content Relationships

### Prerequisite Chains

**Concept Dependency Graph**:
```
Lesson L_n requires concepts from:
- Previous lessons in same chapter: L_{n-1}, L_{n-2}
- Foundation lessons in earlier parts
- Explicitly listed in chapter index "Prerequisites" section
```

**Validation**:
- No forward references (concept used before introduced)
- All prerequisites explicitly stated in chapter index
- Dependency graph is acyclic (no circular dependencies)

### Navigation Flow

**Linear Progression**:
```
Preface → Part 1 → Chapter 1 → Lesson 1.1 → Lesson 1.2 → ... 
       → Chapter 2 → ... → Part 2 → ... → Part 7 → Appendices
```

**Each lesson must have**:
- Link to previous lesson (if not first in chapter)
- Link to next lesson (if not last in chapter)
- Breadcrumb: Part > Chapter > Lesson

**Implementation**:
```markdown
## ➡️ What's Next?

[1-2 sentences teasing the next lesson]

Continue to: [Next Lesson Title](../path/to/next-lesson)
```

---

## Content Generation Specifications

### Per-Lesson Content Distribution

**Word Count Breakdown** (1200-1800 words total):
- **Learning Objectives**: 50-100 words (3-5 items)
- **Introduction**: 200-300 words (2-3 paragraphs)
- **Main Content**: 700-1100 words (3-5 sections)
- **Hands-On Practice**: 150-250 words (exercise description)
- **Key Takeaways**: 75-100 words (4-6 points)
- **Review Questions**: 50-100 words (3-5 questions)
- **Further Reading**: 50-75 words (3-4 links with descriptions)
- **What's Next**: 25-50 words (transition)

**Code Distribution**:
- **Technical Lessons**: 2-4 complete code examples (50-200 lines each)
- **Conceptual Lessons**: 0-2 code examples (as needed)
- **Integration Lessons**: 3-4 examples showing system connections

**Mathematical Content**:
- **Math-Heavy Lessons** (kinematics, control): 4-6 equations with derivations
- **Standard Lessons**: 1-3 equations with explanations
- **Conceptual Lessons**: 0-1 equations (as needed)

### Graduation-Level Rigor Checklist

Each technical lesson must include:
- [ ] **Mathematical Foundation**: Derive or explain 1+ key equations
- [ ] **Algorithm Implementation**: Code algorithm from scratch (not just library calls)
- [ ] **Industry Context**: Reference real-world application (cite company/paper)
- [ ] **Performance Specification**: State expected complexity/latency/accuracy
- [ ] **Research Link**: Reference conference paper or authoritative source (Week 6+)
- [ ] **Debugging Guidance**: Common errors and troubleshooting steps

---

## File Naming Conventions

### Directory Names
- **Parts**: `part-0X-kebab-case` (e.g., `part-01-foundations`)
- **Chapters**: `chapter-0X-kebab-case` (e.g., `chapter-01-introduction`)
- **Appendices**: `appendices/` (single directory)

### File Names
- **Lessons**: `0X-kebab-case.md` (e.g., `01-digital-to-physical.md`)
- **Chapter Index**: `index.md` (in each chapter directory)
- **Appendices**: `kebab-case.md` (e.g., `hardware-guide.md`)
- **Category Files**: `_category_.json` (underscore prefix)

### Slug Generation Rules
- Convert to lowercase
- Replace spaces with hyphens
- Remove special characters (except hyphens)
- No leading/trailing hyphens
- Maximum 50 characters

**Examples**:
- "Introduction to Physical AI" → `introduction-to-physical-ai`
- "ROS 2 Architecture & Communication" → `ros2-architecture-communication`
- "What's Next?" → `whats-next`

---

## Quality Assurance Attributes

### Automated Validation

**Per Lesson**:
- `word_count_valid`: boolean (1200 ≤ count ≤ 1800)
- `sections_complete`: boolean (all 8 required sections present)
- `code_examples_count`: number (≥ 2 for technical lessons)
- `exercises_count`: number (≥ 1)
- `admonitions_count`: number (≥ 2)
- `next_link_valid`: boolean (resolves without 404)
- `description_length_valid`: boolean (≤ 160 chars)

**Per Chapter**:
- `lessons_sequential`: boolean (01, 02, 03... no gaps)
- `index_lessons_match`: boolean (index.md lists match files)
- `prerequisites_valid`: boolean (no forward references)

**Per Part**:
- `chapters_sequential`: boolean
- `category_json_valid`: boolean
- `chapter_count_range`: boolean (3-5 chapters)

**Textbook-Wide**:
- `build_status`: "success" | "error" (Docusaurus build)
- `link_check_status`: number of 404s
- `version_consistency`: boolean (same ROS/Isaac versions throughout)
- `total_completion_percent`: number (0-100)

### Manual Review Checklist

**Technical Accuracy** (per lesson):
- [ ] Code examples run without errors
- [ ] Math derivations are correct
- [ ] Technical claims match official documentation
- [ ] Performance characteristics are realistic

**Pedagogical Effectiveness**:
- [ ] Learning progression is logical
- [ ] Explanations are clear and accessible
- [ ] Examples are relevant and illustrative
- [ ] Exercises test understanding (not just memorization)

**Industry Relevance**:
- [ ] Skills align with job requirements
- [ ] Practices match professional standards
- [ ] Tools and versions are current
- [ ] Real-world applications are accurate

---

**This data model will guide all content generation and validation processes to ensure consistency and quality across 87 lessons.**
