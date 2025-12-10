<!-- SYNC IMPACT REPORT:
Version change: N/A → 1.0.0 (initial creation)
Modified principles: N/A (new project)
Added sections: All sections created for new project
Removed sections: N/A
Templates requiring updates: N/A (project starting)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Excellence in Content Creation
All content, tone, layout, depth, emoji usage, mermaid diagrams, callout boxes, and writing style must be professional, exciting, and technically rigorous — identical to the highest-quality modern AI engineering books. This includes creating visually distinct "Key Takeaways" sections, "Further Reading", and "Assessment" sections with quizzes, coding exercises, or mini-projects at the end of every chapter.

### II. Reusable Intelligence and Automation
Maximum reuse of sub-agents and skills must be implemented, with all agents and skills placed in .claude/agents and .claude/skills for future hackathon use. Every component must be designed for reusability and exportability, supporting the goal of creating 8+ clean agents and skills in .claude/ directory.

### III. Full-Featured Implementation (100% Bonus Features)
100% of bonus features must be implemented: Better-Auth signup/signin with background questionnaire, per-chapter Personalize button, per-chapter Urdu translation button. No feature should be left incomplete or partially implemented.

### IV. RAG Chatbot Excellence
The RAG chatbot must support both full-book queries and selected-text-only queries with perfect functionality. The system must be robust, responsive, and provide accurate information retrieval across the entire book content.

### V. Deployment Perfection
The final site must deploy perfectly on GitHub Pages with zero errors and achieve 100/100 Lighthouse scores. All console errors must be eliminated, and the site must function flawlessly in production environment.

### VI. Technical Standards Compliance
All markdown files must go into /docs/ with clean numbering (01-, 02-, etc.) and proper folder hierarchy. Docusaurus config, sidebars.js, and theme must be flawless and auto-generated. Backend must use FastAPI + Neon Serverless Postgres + Qdrant Cloud (free tier) + proper ingestion script. Code components in /src/components must be clean React + TypeScript. Every generated file must be ready to commit — no placeholders, no TODOs.

## Additional Constraints and Requirements

The project must operate within strict time constraints of ≤10 hours total working time. Only free-tier services must be used to ensure cost-effectiveness. All agents and skills must be exportable for future hackathons. The backend architecture must include FastAPI, Neon Serverless Postgres, and Qdrant Cloud for vector storage. The frontend must include proper authentication, personalization, and Urdu translation capabilities.

## Development Workflow and Quality Standards

All development must follow Spec-Driven Development (SDD) principles with proper artifact creation (spec, plan, tasks). Every user input must be recorded as a Prompt History Record (PHR). Architecturally significant decisions must be documented as Architecture Decision Records (ADRs). All code must be testable, maintainable, and production-ready. Implementation must follow the Red-Green-Refactor cycle with comprehensive testing at all levels.

## Governance

This constitution supersedes all other development practices for this project. All implementation work must verify compliance with these principles. All pull requests and code reviews must ensure adherence to these standards. The project must maintain focus on delivering a complete, production-ready, GitHub Pages-deployed Docusaurus book on Physical AI & Humanoid Robotics with integrated RAG chatbot, authentication, personalization, and Urdu translation. All features must work perfectly as specified in the success criteria.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09