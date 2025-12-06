
---
id: 2
title: Specify Physical AI & Humanoid Robotics Course Book
stage: spec
date: 2025-12-05
surface: agent
model: claude-opus-4-5-20251101
feature: 1-robotics-course-book
branch: 1-robotics-course-book
user: user
command: /sp.specify
labels: ["specification", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-robotics-course-book/spec.md
  - specs/1-robotics-course-book/checklists/requirements.md
tests:
  - No tests run for specification
---

## Prompt

```text
Physical AI & Humanoid Robotics course book for beginners.

- 4 modules:
  1) Physical AI & humanoid basics
  2) ROS 2 “robot nervous system”
  3) Simulation (Gazebo, Unity, NVIDIA Isaac)
  4) VLA + capstone (voice → ROS 2 actions)

- Based on a 13‑week course → about 20–26 chapters following week order.
- ROS 2 only, no hardware assembly, no advanced math.
- One chapter for recommended hardware (RTX workstation + Jetson Orin Nano + depth camera), concept only.
- Docusaurus left sidebar should show the 4 modules as top‑level sections.
```

## Response snapshot

The specification and quality checklist have been created and validated for the Physical AI & Humanoid Robotics Course Book. The feature branch `1-robotics-course-book` has been created, and `specs/1-robotics-course-book/spec.md` and `specs/1-robotics-course-book/checklists/requirements.md` files have been successfully written.

## Outcome

- ✅ Impact: Project specification created successfully.
- 🧪 Tests: No tests were run for this specification.
- 📁 Files: Two files were created: `specs/1-robotics-course-book/spec.md` and `specs/1-robotics-course-book/checklists/requirements.md`.
- 🔁 Next prompts: The feature is ready for planning or further clarification.
- 🧠 Reflection: The specification outlines the course book structure, user scenarios, functional requirements, key entities, and success criteria, adhering to the provided constraints.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
