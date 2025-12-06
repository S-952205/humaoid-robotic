# Tasks: Physical AI & Humanoid Robotics Course Book Docusaurus Site

**Input**: Design documents from `/specs/1-robotics-course-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md

**Tests**: The feature specification does not explicitly request test tasks, but rather independent test criteria for each user story. Therefore, direct test implementation tasks will not be generated, but verification steps will be included in the implementation tasks.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths shown below assume Docusaurus project structure at the repository root as defined in plan.md.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus structure

- [X] T001 Initialize a new Docusaurus v3 project in the repository root (e.g., `npx create-docusaurus@latest . classic --typescript`)
- [X] T002 Configure `docusaurus.config.js` to enable docs-only mode if not default, and set `baseUrl` for GitHub Pages deployment.
- [X] T003 [P] Create `docs/` and `src/` directories at the root, if not created by Docusaurus init.
- [X] T004 [P] Create `static/` directory for static assets, if not created by Docusaurus init.
- [X] T005 [P] Update `package.json` with appropriate project name and description.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Docusaurus configuration and content structure that MUST be complete before ANY user story content can be added.

**⚠️ CRITICAL**: No user story content creation can begin until this phase is complete

- [X] T006 Create `docs/01-intro/` directory for Module 1.
- [X] T007 Create `docs/02-ros2/` directory for Module 2.
- [X] T008 Create `docs/03-simulation/` directory for Module 3.
- [X] T009 Create `docs/04-isaac/` directory for Module 4, as outlined in plan.md.
- [X] T010 Create `docs/05-vla/` directory for Module 5, as outlined in plan.md.
- [X] T011 Create `docs/hardware-guide/` directory for the hardware chapter.
- [X] T012 Create `docs/capstone/` directory for the capstone project.
- [X] T013 Create placeholder markdown files for 13 weeks (2-3 chapters per week, approx 20-26 chapters) in respective module directories (e.g., `docs/01-intro/week-01-chapter-1.md`).
- [X] T014 Create placeholder `docs/hardware-guide/recommended-hardware.md`.
- [X] T015 Create placeholder `docs/capstone/capstone-project.md`.
- [X] T016 Configure `sidebar.js` to define the 4 modules as top-level categories, with chapters ordered to follow the 13-week course progression within each module. Include hardware guide and capstone as distinct entries.

**Checkpoint**: Foundation ready - user story content creation can now begin in parallel

---

## Phase 3: User Story 1 - Learn Physical AI & Humanoid Basics (Priority: P1) 🎯 MVP

**Goal**: A beginner learner wants to understand the fundamental concepts of physical AI and humanoid robotics.

**Independent Test**: A user can read through the module's chapters and comprehend the core concepts without prior knowledge.

### Implementation for User Story 1

- [ ] T017 [US1] Populate `docs/01-intro/week-01-chapter-1.md` with beginner-friendly content on physical AI basics.
- [ ] T018 [US1] Populate `docs/01-intro/week-02-chapter-2.md` with beginner-friendly content on humanoid robotics basics.
- [ ] T019 [US1] Verify sidebar navigation for the "01-intro" module in the development server (`npm run start`).

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Master ROS 2 Fundamentals (Priority: P1)

**Goal**: A learner wants to understand and apply ROS 2 as the “robot nervous system” for controlling robots in simulation.

**Independent Test**: A user can follow the ROS 2 chapters, run the provided code examples in simulation, and understand how ROS 2 nodes communicate.

### Implementation for User Story 2

- [ ] T020 [US2] Populate `docs/02-ros2/week-03-chapter-3.md` with content on ROS 2 basics and node communication.
- [ ] T021 [US2] Populate `docs/02-ros2/week-04-chapter-4.md` with content on creating and running simple ROS 2 nodes in simulation.
- [ ] T022 [US2] Verify sidebar navigation for the "02-ros2" module in the development server (`npm run start`).

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Explore Robot Simulation (Priority: P2)

**Goal**: A learner wants to gain practical experience with different robotics simulation platforms like Gazebo, Unity, and NVIDIA Isaac.

**Independent Test**: A user can launch and interact with robot simulations in Gazebo, Unity, and NVIDIA Isaac based on the provided examples.

### Implementation for User Story 3

- [ ] T023 [US3] Populate `docs/03-simulation/week-05-chapter-5.md` with content on Gazebo simulation setup and interaction.
- [ ] T024 [US3] Populate `docs/03-simulation/week-06-chapter-6.md` with content on Unity and NVIDIA Isaac simulation basics.
- [ ] T025 [US3] Populate `docs/04-isaac/week-07-chapter-7.md` with content specific to NVIDIA Isaac simulation.
- [ ] T026 [US3] Populate `docs/04-isaac/week-08-chapter-8.md` with advanced topics or examples for NVIDIA Isaac.
- [ ] T027 [US3] Verify sidebar navigation for the "03-simulation" and "04-isaac" modules in the development server (`npm run start`).

**Checkpoint**: All user stories up to US3 should now be independently functional

---

## Phase 6: User Story 4 - Implement VLA + Capstone Project (Priority: P2)

**Goal**: A learner wants to implement a capstone project using Vision-Language-Action (VLA) models to control ROS 2 actions via voice commands.

**Independent Test**: A user can complete the capstone project, successfully issue voice commands, and observe the robot performing the corresponding ROS 2 actions in simulation.

### Implementation for User Story 4

- [ ] T028 [US4] Populate `docs/05-vla/week-09-chapter-9.md` with content on Vision-Language-Action (VLA) models.
- [ ] T029 [US4] Populate `docs/05-vla/week-10-chapter-10.md` with content on integrating VLA with ROS 2 actions.
- [ ] T030 [US4] Populate `docs/capstone/capstone-project.md` with the full capstone project details and instructions.
- [ ] T031 [US4] Verify sidebar navigation for the "05-vla" module and "capstone" in the development server (`npm run start`).

**Checkpoint**: All user stories up to US4 should now be independently functional

---

## Phase 7: User Story 5 - Understand Recommended Hardware (Priority: P3)

**Goal**: A user wants to understand the recommended hardware (RTX workstation, Jetson Orin Nano, depth camera) for physical AI and humanoid robotics, even without hands-on assembly.

**Independent Test**: A user can read the hardware chapter and understand the purpose and role of each recommended component.

### Implementation for User Story 5

- [ ] T032 [US5] Populate `docs/hardware-guide/recommended-hardware.md` with conceptual descriptions of the recommended hardware components.
- [ ] T033 [US5] Verify sidebar navigation for the "hardware-guide" in the development server (`npm run start`).

**Checkpoint**: All user stories should now be independently functional

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T034 Configure GitHub Actions workflow (`.github/workflows/deploy.yml`) for automated deployment to GitHub Pages.
- [ ] T035 Run `npm run build` to ensure no build errors exist.
- [ ] T036 Run `npm run start` and perform a comprehensive review of sidebar, navigation, internal links, and overall site presentation.
- [ ] T037 Address edge case: Add a "Troubleshooting" section or guide to installation instructions for required software (ROS 2, simulation platforms) in an appropriate `docs/` file.
- [ ] T038 Address edge case: Add a "Version Compatibility" section or guide within `docs/` to provide guidance on ROS 2 or simulation platform version mismatches.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories content creation
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Content creation for chapters.
- Verification of sidebar navigation in development server.

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T003, T004, T005)
- Once Foundational phase completes, content creation for different user stories can start in parallel (if team capacity allows).
- Verification tasks within each user story can run in parallel with content creation if separate developers.

---

## Parallel Example: Content Creation for User Stories

```bash
# Once Foundational Phase is complete, multiple developers can work on different user stories in parallel:

# Developer A:
Task: "Populate docs/01-intro/week-01-chapter-1.md with beginner-friendly content on physical AI basics."
Task: "Populate docs/01-intro/week-02-chapter-2.md with beginner-friendly content on humanoid robotics basics."
Task: "Verify sidebar navigation for the '01-intro' module in the development server (`npm run start`)."

# Developer B:
Task: "Populate docs/02-ros2/week-03-chapter-3.md with content on ROS 2 basics and node communication."
Task: "Populate docs/02-ros2/week-04-chapter-4.md with content on creating and running simple ROS 2 nodes in simulation."
Task: "Verify sidebar navigation for the '02-ros2' module in the development server (`npm run start`)."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Verify sidebar navigation and content for User Story 1
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Verify independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Verify independently → Deploy/Demo
4. Add User Story 3 → Verify independently → Deploy/Demo
5. Add User Story 4 → Verify independently → Deploy/Demo
6. Add User Story 5 → Verify independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
   - Developer E: User Story 5
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
