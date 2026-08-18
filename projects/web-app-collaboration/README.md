# Web App Collaboration

**Project:** Web Application  
**Assumed GitHub Repo:** https://github.com/PawanRandeepa12/web-app  
**Branch for collaboration:** `dev/cw`  
**Base branch:** `main`

## Overview

This folder contains starter assets and collaboration notes for building a modern web application.

This is a **shared collaboration workspace**. 

- Primary development will happen in the dedicated repo: `PawanRandeepa12/web-app`
- Use this directory for local prototypes, shared docs, and initial scaffolding before pushing to the remote repo.
- All work on this project should target the `dev/cw` branch (to be created/pushed in a dedicated session).

## Setup Instructions (for new repo)

1. Create the GitHub repository:
   - Go to: https://github.com/new
   - Repository name: `web-app`
   - Description: Web application (collaboration project)
   - Visibility: Public (or Private as preferred)
   - **Do NOT** initialize with README (we will push existing content)

2. After creation, in a **new session**:
   ```bash
   git clone https://github.com/PawanRandeepa12/web-app.git
   cd web-app
   git checkout -b dev/cw
   # Copy or init content from this collaboration folder
   git add .
   git commit -m "feat: initial web app scaffolding + collaboration setup"
   git push -u origin dev/cw
   ```

## Current Local Structure (in PawanRandeepa12 profile)

```
projects/web-app-collaboration/
├── README.md          # This file
├── index.html         # Basic starter HTML
├── style.css          # Basic styles
├── app.js             # Basic JS interactivity
└── package.json       # Placeholder for future Node/dependencies
```

## Starter Files

The following files provide a minimal viable web app skeleton:

- **index.html** — Responsive landing page with hero + features
- **style.css** — Clean modern styling (mobile friendly)
- **app.js** — Simple interactivity (e.g. button actions, dynamic content)

## Next Steps / Collaboration Notes

- [ ] Finalize tech stack (React / Vanilla / Next.js / etc.)
- [ ] Set up CI/CD (GitHub Actions)
- [ ] Add authentication / backend (Node / Python / etc.)
- [ ] Define UI/UX requirements
- [ ] Create issues & project board on the new repo

## Contact / Contributors

- Primary: Pawan Jayarathna
- Collaboration branch owner: `dev/cw`

---

**Status:** Ready for repo creation + `dev/cw` push from new session.

*Last updated: 2026-08-18*
