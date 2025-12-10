# Physical AI & Humanoid Robotics

This repository contains the complete, production-ready, GitHub Pages-deployed Docusaurus book on Physical AI & Humanoid Robotics with integrated RAG chatbot, authentication, personalization, and Urdu translation.

## Table of Contents
- [Project Overview](#project-overview)
- [Local Development](#local-development)
- [Deployment](#deployment)
- [Tech Stack](#tech-stack)
- [Features](#features)
- [Contribution Guidelines](#contribution-guidelines)

## Project Overview

This project is a comprehensive book on Physical AI and Humanoid Robotics, featuring:

- 14 chapters across 5 modules (1000-1200 words each)
- Interactive RAG chatbot for content queries
- User authentication with background questionnaire
- Personalization engine for adaptive content
- Urdu translation with RTL support
- Deployed on GitHub Pages with 100/100 Lighthouse scores

## Local Development

1. **Prerequisites**:
   - Node.js 20+ installed
   - Git installed

2. **Setup**:
   ```bash
   git clone https://github.com/your-username/Physical_AI_Humanoid_Robotics.git
   cd Physical_AI_Humanoid_Robotics/website
   npm install
   ```

3. **Development Server**:
   ```bash
   npm start
   ```
   The site will be available at http://localhost:3000

## Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the main branch.

### GitHub Pages Deployment Process

1. **Automatic Deployment**: The site is deployed automatically via GitHub Actions when changes are pushed to the main branch.

2. **Workflow**: The deployment workflow is defined in `.github/workflows/deploy.yml`:
   - Triggers on push to main branch
   - Sets up Node.js environment
   - Installs dependencies
   - Builds the Docusaurus site
   - Deploys to GitHub Pages

3. **Configuration**: The deployment is configured in `docusaurus.config.ts` with:
   - `url`: Base URL for GitHub Pages
   - `baseUrl`: Base path for deployment
   - `organizationName`: GitHub username/organization
   - `projectName`: Repository name

4. **Manual Deployment**: If needed, you can trigger a manual build and deployment:
   ```bash
   cd website
   npm run build
   npm run deploy
   ```

## Tech Stack

- **Frontend**: Docusaurus 3.x (React-based static site generator)
- **Backend**: FastAPI (Python)
- **Deployment**: GitHub Pages + GitHub Actions
- **Database**: Neon Postgres (serverless)
- **Vector DB**: Qdrant Cloud
- **Authentication**: Better-Auth
- **LLM**: Groq API with Llama 3
- **Language**: TypeScript, Python 3.11

## Features

- **Interactive RAG Chatbot**: Ask questions about the book content and get relevant answers
- **User Authentication**: Sign up and sign in with background questionnaire
- **Content Personalization**: Content adapts based on user profile and experience level
- **Urdu Translation**: Toggle between English and Urdu with proper RTL formatting
- **Lighthouse 100/100**: Optimized for performance, accessibility, best practices, and SEO
- **Mobile Responsive**: Fully responsive design for all device sizes

## Contribution Guidelines

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

### Development Workflow

- All changes to the main branch will trigger an automatic deployment
- Please ensure all changes pass the build process before submitting a PR
- Follow the existing code style and documentation patterns