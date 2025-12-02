# Quickstart Guide: Docusaurus Textbook Development

**Feature**: 001-docusaurus-textbook-site  
**Audience**: Content creators, contributors, developers  
**Last Updated**: 2025-12-03

## Prerequisites

- **Node.js**: Version 18.0.0 or higher ([download](https://nodejs.org/))
- **Git**: For version control ([download](https://git-scm.com/))
- **Text Editor**: VS Code recommended ([download](https://code.visualstudio.com/))
- **Optional**: Docker for code example testing

## Quick Start (5 minutes)

### 1. Clone and Install

```bash
# Clone the repository
git clone https://github.com/[username]/physical-ai-humanoid-robotics-textbook.git
cd physical-ai-humanoid-robotics-textbook

# Install dependencies
npm install
```

### 2. Start Development Server

```bash
npm start
```

This opens `http://localhost:3000` in your browser with live reload.

### 3. Create Your First Chapter

Create a new file in the appropriate module directory:

```bash
# Example: Add a new chapter to Week 3-5 (ROS 2 module)
touch docs/week-03-05/06-my-new-chapter.md
```

Add frontmatter and content:

```markdown
---
sidebar_position: 6
title: My New Chapter Title
---

# My New Chapter Title

## Learning Objectives
By the end of this chapter, you will be able to:
- Objective 1
- Objective 2

## Introduction
[Your content here]

## Summary
- Key point 1
- Key point 2
```

Save the file—your changes appear instantly in the browser!

## Project Structure

```
physical-ai-humanoid-robotics-textbook/
├── docs/               # All content (your workspace!)
│   ├── intro.md
│   ├── week-01-02/    # Module directories
│   ├── week-03-05/
│   └── ...
├── static/            # Images, files
│   └── img/
│       └── diagrams/
├── src/               # Custom components, styling
│   └── css/
│       └── custom.css
├── docusaurus.config.js  # Site configuration
├── sidebars.js           # Navigation structure
└── package.json
```

## Common Tasks

### Add a Code Example

Use fenced code blocks with language specification:

````markdown
```python
#!/usr/bin/env python3
# This is a ROS 2 publisher example
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started!')

if __name__ == '__main__':
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
```
````

**Supported Languages**: python, xml, yaml, bash, javascript, cpp, json

### Add a Diagram

1. Place image in `static/img/diagrams/`
2. Reference in Markdown:

```markdown
![Alt text describing the diagram](/img/diagrams/my-diagram.png)
*Figure 2.1: Caption explaining the diagram.*
```

**Image Guidelines**:
- Format: PNG, SVG, or WebP
- Size: < 500KB (use [Squoosh](https://squoosh.app/) to compress)
- Alt text: Descriptive, not just "diagram"

### Use Admonitions

```markdown
:::tip Pro Tip
This is helpful additional information.
:::

:::warning Important
Pay attention to this critical detail.
:::

:::danger Caution
This could cause errors if not done correctly.
:::

:::info
Informational callout.
:::
```

### Add Tabs (for alternatives)

```mdx
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="gazebo" label="Gazebo" default>

Use Gazebo for physics-based simulation...

  </TabItem>
  <TabItem value="unity" label="Unity">

Use Unity for high-fidelity visualization...

  </TabItem>
</Tabs>
```

## Building and Testing

### Local Build

Test the production build locally:

```bash
npm run build   # Generate static files in build/
npm run serve   # Serve on http://localhost:3000
```

### Check for Errors

```bash
# Docusaurus will report:
# - Broken links
# - Missing files
# - Configuration errors
```

Fix any errors before committing!

## Deployment

### Automatic Deployment

Push to `main` branch → GitHub Actions automatically deploys to GitHub Pages (~5 minutes).

```bash
git add .
git commit -m "docs: add chapter on X"
git push origin main
```

Check deployment status: GitHub → Actions tab

### Manual Deployment (if needed)

```bash
npm run deploy
```

## Content Guidelines

### Chapter Structure

Every chapter should follow this template:

1. **Frontmatter** (sidebar_position, title)
2. **Title** (H1, matches frontmatter title)
3. **Learning Objectives** (H2, bullet list)
4. **Introduction** (H2, context and motivation)
5. **Main Content** (H2 sections)
6. **Code Examples** (with comments and expected output)
7. **Hands-On Exercise** (H2, practical activity)
8. **Summary** (H2, key takeaways)
9. **Further Reading** (H2, external links)

### Code Examples Best Practices

- ✅ Include setup instructions
- ✅ Add inline comments explaining key concepts
- ✅ Show expected output
- ✅ Keep examples focused and runnable
- ✅ Test before committing

### Writing Style

- Use **clear, concise language**
- Define technical terms on first use
- Include examples for abstract concepts
- Use active voice
- Target audience: Students with AI/ML background, new to robotics

## Troubleshooting

### Development Server Won't Start

```bash
# Clear cache and reinstall
rm -rf node_modules package-lock.json
npm install
npm start
```

### Broken Links

- Check file paths are relative to `docs/`
- Use `.md` extension in links: `[Link](./other-chapter.md)`
- Docusaurus auto-converts to URLs

### Images Not Showing

- Verify image is in `static/img/`
- Use absolute path from static: `/img/diagrams/example.png`
- Check file extension matches (case-sensitive on Linux)

### Build Fails

- Read error message carefully
- Common issues: broken Markdown syntax, missing frontmatter, invalid config
- Test locally before pushing: `npm run build`

## Getting Help

- **Docusaurus Docs**: https://docusaurus.io/docs
- **Project Issues**: GitHub Issues tab
- **ROS 2 Docs**: https://docs.ros.org/en/humble/
- **Gazebo Docs**: https://gazebosim.org/docs
- **NVIDIA Isaac Docs**: https://docs.omniverse.nvidia.com/isaacsim/

## Next Steps

1. ✅ Read this quickstart
2. Familiarize yourself with project structure
3. Review an existing chapter to understand style
4. Create a new chapter or enhance an existing one
5. Test locally before committing
6. Submit pull request for review

## Contribution Workflow

1. **Create feature branch**: `git checkout -b add-chapter-X`
2. **Make changes**: Edit/create Markdown files
3. **Test locally**: `npm start` and `npm run build`
4. **Commit**: `git commit -m "docs: add chapter X on topic Y"`
5. **Push**: `git push origin add-chapter-X`
6. **Pull Request**: Submit PR for peer review
7. **Address feedback**: Make revisions if needed
8. **Merge**: Once approved, merge to main → auto-deploy!

---

**Happy Contributing! 🚀**
