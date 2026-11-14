# Contributing to UET_RE_AMR

This document outlines the branching strategy, workflow, and coding standards for all contributors to the UET_RE_AMR project.

---

## Branching Strategy

The repository follows a simplified Git Flow model:

```
master        <-- Stable (the robot runs this; only updated via Pull Request)
feature/*     <-- New features or refactors
fix/*         <-- Bug fixes
```

### Branch Naming Conventions

Prefix your branch name with `feature/` or `fix/`, followed by the targeted component and a short description:

```
feature/navigation-slam-mapping
feature/firmware-motor-pid-tuning
feature/perception-people-mask
fix/odom-drift
```

---

## Development Workflow

### 1. Repository Setup
```bash
git clone https://github.com/UET-RE/UET_RE_AMR.git
```

### 2. Create a Feature Branch
```bash
git checkout -b feature/<component>-<description>
# Example:
git checkout -b feature/navigation-global-planner
```

### 3. Commit Guidelines

Commits must follow the Conventional Commits specification:

```
<type>(<scope>): <subject>

Types:   feat | fix | docs | style | refactor | test | chore
Scopes:  nav | hw | fw | perception | maps | control | teleop | msgs | docker | scripts | docs
         Tuning-only changes: tune(<scope>): <what and why>, one concern per commit

Example:
feat(nav): add keepout filter to local costmap
fix(fw): correct encoder overflow handling at max RPM
docs(hw): update pinout diagram for base controller v2
```

### 4. Submitting a Pull Request
```bash
git push origin feature/navigation-global-planner
# Create a Pull Request (PR) into 'master' on GitHub
```

---

## Pull Request Rules

- PRs must include a clear, descriptive title and summary of changes.
- CI (lint) must pass.
- At least one code review and approval from a team member is required before merging.
- Direct commits to `master` are prohibited.
- Do not commit generated build files or IDE configs (e.g., `build/`, `install/`, `.pio/`).

---

## Component Ownership

To minimize conflicts, modifications should be restricted to the directory assigned to your team:

| Team / Role | Target Directory | Restricted Directory |
|---|---|---|
| Navigation | `ros2/src/uet_amr_navigation/` | `firmware/` |
| Perception / Maps | `ros2/src/uet_amr_perception/`, `ros2/src/uet_amr_maps/` | `firmware/` |
| Hardware Interface | `ros2/src/uet_amr_hardware/` | - |
| Firmware | `firmware/` | `ros2/src/uet_amr_navigation/` |
| Hardware Design | `hardware/` | `ros2/`, `firmware/` |

If you need to make changes outside your targeted directory, open an issue and coordinate with the respective team before making modifications.

---

## Coding Standards

### ROS2 (C++)
- Adhere to the [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- Use `clang-format` (`.clang-format`, 4 spaces) to format source code before committing.

### ROS2 (Python)
- Follow PEP 8 guidelines.
- Code should pass `flake8` (config in `.flake8`: 130 columns).

### Firmware (C/C++)
- Use `snake_case` for function and variable names, and `UPPER_CASE` for macro definitions.
- Provide descriptive inline documentation for critical logic and driver configurations.

### Tuning changes
Field-test numbers and the history of a value ("was X") belong in `docs/tuning_log.md`, not in YAML comments; keep the comment to *why* the value is what it is.
