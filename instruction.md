# Project Instructions

## Purpose

This document describes the project's preferred comment styles and provides templates you can customize at the top of new source files.

## Customize Comments (start here)

Edit the block below to set the project's default comment header. Replace the placeholder values.

```
/*
 * Project : rc_remote_controller
 * File    : <FILENAME>
 * Author  : Philippe Peurichard
 * Date    : <YYYY-MM-DD>
 * Brief   : Short file summary (one line)
 * License : Same license as other file
 */
```

Place the block at the top of every new C source/header file. For header files, add include guards or `#pragma once` under the header.

## Scope for Automated Changes

Automated scripts or bulk-edit operations MUST only modify files under the `app/src/` directory by default. Do not apply automated header insertion or mass edits to other folders (for example `app/ui/`) unless explicitly approved in a separate change request.

## Inline and Block Comment Conventions

- Do not use and remove single-line comments:  `//`
- Always use block comments: use `/* ... */` for all comments but also for file headers and longer multi-line explanations.
- Keep comments concise and focused on "why", not "what" (prefer intent over restating code).

## Function Documentation Template (Doxygen-style)

```
/**
 * @brief Short description of the function.
 *
 * Detailed description (optional).
 *
 * @param[in]  arg  description
 * @param[out] out  description
 * @return status or value
 */
```

Use this template above function definitions in `.c` files and in header declarations for the public API.

## TODO/NOTE Tags

- Mark work items with `TODO:` and add an owner/issue reference when possible.
- Use `FIXME:` for known bugs that need fixing.
- Use `NOTE:` for non-obvious design decisions.

Example:

```c
// TODO(philippe): Replace with power-managed API (issue #123)
```

## Commit Message Guidance

- First line: short summary (<= 72 chars).
- Blank line.
- Body: explain the why and the impact (wrap at ~72 chars).

## How to Update This File

Adjust the header template at the top of this file to match project metadata, then use it as the starting point for new files.

---
Created to help standardize comments across the Zephyr-based `rc-remote-controller` project.
