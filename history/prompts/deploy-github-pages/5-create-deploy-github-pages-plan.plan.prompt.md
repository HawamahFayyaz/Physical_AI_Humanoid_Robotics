---
id: 5
title: Create Deploy GitHub Pages Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: deploy-github-pages
branch: 001-deploy-github-pages
user: claude
command: /sp.plan
labels: [plan,deployment,github-pages,lighthouse,optimization]
links:
  spec: specs/001-deploy-github-pages/spec.md
  plan: specs/001-deploy-github-pages/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-deploy-github-pages/plan.md
 - specs/001-deploy-github-pages/research.md
 - specs/001-deploy-github-pages/data-model.md
 - specs/001-deploy-github-pages/quickstart.md
 - specs/001-deploy-github-pages/contracts/deployment-contract.md
tests:
 - none
---

## Prompt

```
# Physical AI & Humanoid Robotics - Implementation Plan

## Project Context
- 14 chapters across 5 modules (1000-1200 words each)
- 4 bonus features: RAG chatbot, Auth, Personalization, Urdu translation
- All free-tier services
- Deploy to GitHub Pages

---

## Phase 1: Foundation
### Tasks:
- Initialize Docusaurus 3.x project
- Create folder structure (docs/, src/components/, backend/, .claude/)
- Configure docusaurus.config.js for GitHub Pages
- Set up sidebars.js with auto-generation
- Create FastAPI backend scaffold
- Create .env.example

### Validation:
- [ ] `npm run start` works without errors
- [ ] All folders exist
- [ ] Backend runs with `uvicorn main:app`

---

## Phase 2: Content Generation
### Tasks:
- Write Module 0: 2 chapters (Introduction)
- Write Module 1: 3 chapters (ROS 2)
- Write Module 2: 3 chapters (Digital Twin)
- Write Module 3: 3 chapters (Isaac)
- Write Module 4: 3 chapters (VLA + Capstone)

### Validation:
- [ ] 14 markdown files exist in docs/
- [ ] Each chapter: 1000-1200 words
- [ ] Each chapter: 2+ Mermaid diagrams
- [ ] Each chapter: Key Takeaways + Assessment
- [ ] `npm run build` succeeds

---

## Phase 3: RAG Chatbot
### Tasks:
- Set up Qdrant Cloud collection
- Create document ingestion script (chunk all chapters)
- Implement embeddings with sentence-transformers/all-MiniLM-L6-v2
- Build FastAPI /api/chat endpoint
- Build React ChatBot component
- Implement selected-text mode

### Validation:
- [ ] Ingestion script processes all 14 chapters
- [ ] Chat returns relevant answers
- [ ] Selected-text mode works
- [ ] Response time < 3 seconds

---

## Phase 4: Authentication
### Tasks:
- Configure Better-Auth
- Create Neon Postgres tables (users, profiles)
- Build signup flow with questionnaire
- Build signin flow
- Build UserMenu component

### Validation:
- [ ] Signup → Questionnaire → Profile saved
- [ ] Signin works
- [ ] Session persists on refresh

---

## Phase 5: Personalization
### Tasks:
- Build FastAPI /api/personalize endpoint
- Integrate Groq API (Llama 3)
- Build PersonalizeButton component
- Implement caching in Neon

### Validation:
- [ ] Button appears on each chapter
- [ ] Content changes based on user profile
- [ ] Cached version loads faster

---

## Phase 6: Urdu Translation
### Tasks:
- Build FastAPI /api/translate endpoint
- Configure RTL CSS
- Build UrduToggle component
- Implement translation caching

### Validation:
- [ ] Toggle switches to Urdu
- [ ] RTL layout renders correctly
- [ ] Code blocks stay English
- [ ] Technical terms: "اردو (English)" format

---

## Phase 7: Deployment
### Tasks:
- Create .github/workflows/deploy.yml
- Configure secrets in GitHub
- Optimize for Lighthouse 100
- Deploy to GitHub Pages

### Validation:
- [ ] GitHub Actions succeeds
- [ ] Site is live
- [ ] Lighthouse: 100/100 all categories
- [ ] Zero console errors

---

## Architecture Decisions

| Decision | Options | Choice | Rationale |
|----------|---------|--------|-----------|
| Embeddings | OpenAI / HuggingFace | sentence-transformers/all-MiniLM-L6-v2 | Free, local, fast |
| LLM | GPT-4 / Claude / Groq | Groq (Llama 3) | Free tier, fast inference |
| Vector DB | Pinecone / Qdrant / Chroma | Qdrant Cloud | Free tier, good docs |
| User DB | Supabase / Neon / PlanetScale | Neon Postgres | Free tier, serverless |
| Auth | NextAuth / Clerk / Better-Auth | Better-Auth | Simple, self-hosted |

---

## Testing Strategy

### Unit Tests:
- Embedding generation
- Chunk splitting
- API endpoints

### Integration Tests:
- RAG pipeline end-to-end
- Auth flow end-to-end
- Personalization with real user profile

### Manual Validation:
- Read each chapter for quality
- Test chat with 10 sample questions
- Test Urdu toggle on all chapters
- Lighthouse audit

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Groq rate limits | Cache all LLM responses |
| Qdrant free tier limits | Efficient chunking, 500 token chunks |
| Build failures | Test locally before push |
| Slow personalization | Pre-compute for common profiles |

---

## Success Metrics

| Metric | Target |
|--------|--------|
| Chapters complete | 14/14 |
| Words per chapter | 1000-1200 |
| Mermaid diagrams | 28+ (2 per chapter) |
| RAG response time | < 3s |
| Lighthouse score | 100/100 |
| Console errors | 0 |

---
```

## Response snapshot

Created comprehensive implementation plan for GitHub Pages deployment including technical context, constitution check, project structure, research, data model, quickstart guide, and deployment contracts. The plan covers automated GitHub Actions deployment, Lighthouse 100/100 optimization, and documentation requirements with all necessary artifacts generated.

## Outcome

- ✅ Impact: Created complete implementation plan for GitHub Pages deployment
- 🧪 Tests: none
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, and contracts/ in specs/001-deploy-github-pages/
- 🔁 Next prompts: Create tasks and implement the deployment features
- 🧠 Reflection: Plan addresses all requirements with proper architecture and validation steps

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A