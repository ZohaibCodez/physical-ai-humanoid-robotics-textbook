# Physical AI & Humanoid Robotics Textbook

[![Deploy to GitHub Pages](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/actions/workflows/deploy.yml)
[![Website](https://img.shields.io/website?url=https%3A%2F%2Fzohaibcodez.github.io%2Fphysical-ai-humanoid-robotics-textbook%2F)](https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/)

A comprehensive, graduation-level textbook for Physical AI and Humanoid Robotics with 87 interactive lessons across 7 parts, built with Docusaurus.

## 🎯 Textbook Structure

**7 Parts | 22 Chapters | 87 Lessons | 5 Appendices**

- **Part 1**: Foundations of Physical AI (10 lessons)
- **Part 2**: ROS 2 Ecosystem (17 lessons)
- **Part 3**: Simulation Environments (12 lessons)
- **Part 4**: NVIDIA Isaac Platform (16 lessons)
- **Part 5**: Humanoid Development (16 lessons)
- **Part 6**: Conversational Robotics (12 lessons)
- **Part 7**: Capstone Project (8 lessons)

**Current Progress**: 1/87 lessons complete (1.2%) | Structure: 100% complete

## 🚀 Quick Start

```bash
# Install dependencies
npm install

# Start local development server
npm start

# Visit http://localhost:3000
```

## 📖 For Content Contributors

### Generate New Lessons

See comprehensive guides:
- **[QUICK-START.md](QUICK-START.md)** - 5-minute lesson generation guide
- **[LESSON-GENERATION-GUIDE.md](LESSON-GENERATION-GUIDE.md)** - Complete instructions with prompt templates
- **[STRUCTURE-COMPLETE.md](STRUCTURE-COMPLETE.md)** - Overview of completed structure

### Example Lesson

Study the complete lesson example:
```
docs/part-01-foundations/chapter-01-introduction-to-physical-ai/01-digital-to-physical.md
```
(3500 words, 7 admonitions, 1 Mermaid diagram, 6 code examples, highly interactive)

## 📁 Project Structure

```
docs/
├── part-01-foundations/          # Foundational concepts (10 lessons)
├── part-02-ros2-ecosystem/        # ROS 2 programming (17 lessons)
├── part-03-simulation-environments/ # Gazebo, Unity, URDF (12 lessons)
├── part-04-nvidia-isaac-platform/  # Isaac Sim & ROS (16 lessons)
├── part-05-humanoid-development/   # Balance, IK, control (16 lessons)
├── part-06-conversational-robotics/ # NLP, VLM, gestures (12 lessons)
├── part-07-capstone-project/      # Final project guide (8 lessons)
└── appendices/                    # Reference materials (5 docs)

specs/004-complete-textbook-restructure/  # Project specification
scripts/                          # Validation & generation scripts
templates/                        # Lesson & chapter templates
static/                          # Images, diagrams, resources
src/                            # Custom components, styling
```

## Development

- `npm start` - Start local development server
- `npm run build` - Build static site for production
- `npm run serve` - Serve production build locally

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines on how to contribute content, report issues, and submit pull requests.

Quick links:
- [Contributor Guide](CONTRIBUTING.md)
- [Project Specification](specs/001-docusaurus-textbook-site/spec.md)
- [Development Quickstart](specs/001-docusaurus-textbook-site/quickstart.md)

## License

MIT

## 🤖 RAG Chatbot Feature

This textbook includes an AI-powered chatbot that answers questions about the content!

**Features:**
- 💬 Ask questions about any topic in the textbook
- 📚 Get answers with citations to specific sections
- 📝 Select text and ask focused questions
- 🔄 Conversation history across sessions

**Try it:** Click the 💬 button in the bottom-right corner on any page!

**Learn More:**
- [Chatbot Specification](specs/002-rag-chatbot/spec.md)
- [Technical Implementation](specs/002-rag-chatbot/plan.md)
- [Deployment Guide](DEPLOYMENT.md) - Deploy your own chatbot for FREE!

## Deployment

**Frontend:** Automatically deployed to GitHub Pages on every push to main branch  
**Backend:** See [DEPLOYMENT.md](DEPLOYMENT.md) for complete FREE deployment guide (Railway + Qdrant + Neon)

**Live Site:** https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/
