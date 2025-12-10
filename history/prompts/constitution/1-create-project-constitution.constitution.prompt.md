---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: claude
command: /sp.constitution
labels: [constitution,setup,initialization]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
Project: Physical AI & Humanoid Robotics – Unified Book + Deployed Docusaurus Site + Embedded RAG Chatbot

Core Objective:
Deliver a complete, production-ready, GitHub Pages-deployed Docusaurus book on Physical AI & Humanoid Robotics with integrated RAG chatbot, authentication, personalization, and Urdu translation — all generated using Spec-Kit Plus and Claude Code.

Core Principles:
- Tone, layout, depth, emoji usage, mermaid diagrams, callout boxes, and writing style must be professional, exciting, and technically rigorous — identical to the highest-quality modern AI engineering books.
- Every chapter must end with a visually distinct "Key Takeaways" section, "Further Reading", and an "Assessment" section containing quizzes, coding exercises, or mini-projects.
- Maximum reuse of sub-agents and skills (all agents and skills must be reusable and placed in .claude/agents and .claude/skills).
- 100% bonus features implemented: Better-Auth signup/signin with background questionnaire, per-chapter Personalize button, per-chapter Urdu translation button.
- RAG chatbot must support both full-book queries and selected-text-only queries.
- Final site must deploy perfectly on GitHub Pages with zero errors.

Key Standards:
- All markdown files go into /docs/ with clean numbering (01-, 02-, etc.) and proper folder hierarchy.
- Docusaurus config, sidebars.js, and theme must be flawless and auto-generated.
- Backend: FastAPI + Neon Serverless Postgres + Qdrant Cloud (free tier) + proper ingestion script.
- Code components in /src/components must be clean React + TypeScript.
- Every generated file must be ready to commit — no placeholders, no TODOs.

Constraints:
- Total working time: ≤10 hours from now.
- Must use only free-tier services.
- All agents and skills must be exportable for future hackathons.
- Zero console errors on final deployed site.

Success Criteria (strictly judged):
- Book is fully generated and beautifully formatted
- RAG chatbot works perfectly (tested with selected text)
- Auth + background questionnaire works
- Personalize button changes content based on user profile
- Urdu button translates technical content accurately
- GitHub Pages site is live and scoring 100/100 Lighthouse
- Reusable intelligence: 8+ clean agents and skills in .claude/

Failure is not an option.
We ship perfection.
```

## Response snapshot

Created a comprehensive constitution file for the Physical AI & Humanoid Robotics project that captures all the core principles and requirements. The constitution includes six core principles: Excellence in Content Creation, Reusable Intelligence and Automation, Full-Featured Implementation, RAG Chatbot Excellence, Deployment Perfection, and Technical Standards Compliance. It also includes sections on additional constraints, development workflow, and governance that align with the project's ambitious goals.

## Outcome

- ✅ Impact: Created initial project constitution with all core principles defined
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive project constitution
- 🔁 Next prompts: Continue with spec, plan, and task creation for the project
- 🧠 Reflection: Constitution provides clear guidance for all project implementation work

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A