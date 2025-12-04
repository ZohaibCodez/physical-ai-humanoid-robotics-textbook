# Physical AI & Humanoid Robotics Textbook

[![Deploy to GitHub Pages](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/ZohaibCodez/physical-ai-humanoid-robotics-textbook/actions/workflows/deploy.yml)
[![Website](https://img.shields.io/website?url=https%3A%2F%2Fzohaibcodez.github.io%2Fphysical-ai-humanoid-robotics-textbook%2F)](https://zohaibcodez.github.io/physical-ai-humanoid-robotics-textbook/)

A comprehensive 13-week textbook for teaching Physical AI and Humanoid Robotics, built with Docusaurus.

## Quick Start

```bash
npm install
npm start
```

Visit `http://localhost:3000` to view the textbook locally.

## Project Structure

- `docs/` - All textbook content organized by week/module
- `static/` - Images, diagrams, and downloadable resources
- `src/` - Custom components and styling
- `docusaurus.config.js` - Main configuration
- `sidebars.js` - Navigation structure

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
