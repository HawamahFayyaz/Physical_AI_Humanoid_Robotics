---
id: 3
title: Create Core Features Spec
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: core-features
branch: 001-core-features
user: claude
command: /sp.specify
labels: [spec,rag-chatbot,auth,personalization,urdu-translation]
links:
  spec: specs/001-core-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-core-features/spec.md
 - specs/001-core-features/checklists/requirements.md
tests:
 - none
---

## Prompt

```
# Features Implementation

## Feature 1: RAG Chatbot
Use agent: rag-chatbot-developer
Use skill: 02-build-rag-chatbot

Build:
- FastAPI backend at /backend/
- Qdrant Cloud integration (free tier)
- Embeddings: sentence-transformers/all-MiniLM-L6-v2 (local, free)
- Two modes: full-book query + selected-text query
- React chat component at /src/components/ChatBot/
- Chunking: 500 tokens, 50 overlap

---

## Feature 2: Better-Auth + Questionnaire
Use agent: auth-specialist
Use skill: 04-implement-auth

Build:
- Better-Auth setup
- Neon Postgres schema (users + profiles)
- Questionnaire: experience_level, gpu_type, ram_gb, linux_exp, ros_exp, interest, role
- React components: SignIn, SignUp, Questionnaire, UserMenu

---

## Feature 3: Personalization
Use agent: personalization-engine
Use skill: 05-personalize-content

Build:
- "Personalize for me" button per chapter
- FastAPI endpoint /api/personalize
- LLM: Groq API with Llama 3 (free tier)
- Rules: Beginner=more analogies, Advanced=edge cases
- Cache personalized versions in Neon

---

## Feature 4: Urdu Translation
Use agent: urdu-translator
Use skill: 06-translate-to-urdu

Build:
- "اردو میں دیکھیں" button per chapter
- FastAPI endpoint /api/translate/urdu
- Rules: code stays English, terms = "اردو (English)"
- RTL CSS with Noto Nastaliq Urdu font
- Cache translations

---

Implement all 4 features now. Production-ready, no placeholders.
```

## Response snapshot

Created comprehensive specification for core features implementation including RAG Chatbot, Better-Auth + Questionnaire, Personalization, and Urdu Translation. The spec includes four prioritized user stories, 22 functional requirements, 5 key entities, and 7 measurable success criteria. Quality checklist was created and specification was validated against all criteria.

## Outcome

- ✅ Impact: Created complete specification for all four core features
- 🧪 Tests: none
- 📁 Files: Created spec.md and requirements checklist in specs/001-core-features/
- 🔁 Next prompts: Plan and implement the four features
- 🧠 Reflection: Specification covers all required features with clear acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A