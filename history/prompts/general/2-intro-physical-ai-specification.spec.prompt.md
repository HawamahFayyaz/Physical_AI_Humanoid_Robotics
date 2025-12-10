---
id: 2
title: "intro-physical-ai-specification"
stage: "spec"
date_iso: "2025-12-09"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "intro-physical-ai"
branch: "2-intro-physical-ai"
user: "user"
command: "/sp.specify"
labels: ["specification", "chapters", "physical-ai"]
links:
  spec: "specs/2-intro-physical-ai/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/2-intro-physical-ai/spec.md
  - specs/2-intro-physical-ai/checklists/requirements.md
tests_yaml: |
  - Specification validation completed
  - Quality checklist created and verified
prompt_text: |
  # Module 0: Introduction to Physical AI (2 Chapters)

  Use agent: physical-ai-robotics-author
  Use skill: 01-write-chapter

  ## Chapter 01: what-is-physical-ai.md
  Location: docs/00-introduction/01-what-is-physical-ai.md
  Words: 1000-1200

  Topics:
  - What is Physical AI vs traditional AI
  - Why humanoid robots now (Tesla Optimus, Figure, Unitree)
  - The shift from digital to embodied intelligence
  - Course roadmap overview

  Mermaid: Physical AI ecosystem mindmap
  Code: None (conceptual)

  ---

  ## Chapter 02: hardware-and-sensors.md
  Location: docs/00-introduction/02-hardware-and-sensors.md
  Words: 1000-1200

  Topics:
  - Sensors: LiDAR, depth cameras (RealSense D435i), IMU
  - Compute: Cloud → Workstation → Edge (Jetson Orin)
  - Hardware requirements: RTX 4070+, 64GB RAM, Ubuntu 22.04
  - The $700 Jetson Student Kit breakdown

  Mermaid: Hardware architecture diagram
  Code: Python dataclass for SensorReading

  ---

  ## Chapter Template (apply to both)
  - Opening hook (2 sentences)
  - 4 learning objectives
  - Content with mermaid + code + callouts (NOTE, TIP, WARNING)
  - Key Takeaways (7 emoji bullets)
  - Further Reading (3 links)
  - Assessment: 4 MCQs + 2 exercises + 1 mini-project (hidden solutions)

  Generate both chapters now.
response_text: |
  Created specification for Introduction to Physical AI chapters.
  - Generated feature specification with user stories, requirements, and success criteria
  - Created quality checklist to validate specification completeness
  - Organized requirements around chapter creation, template structure, and assessment content
outcome: "Specification created successfully with all required sections completed"
evaluation: "Specification meets quality standards with testable requirements and measurable success criteria"
---
