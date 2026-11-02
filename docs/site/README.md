# Documentation site

The repository's Markdown as a website (VitePress): search, dark mode, the PDFs to download.
The Markdown stays the one source; the site is built from it and published to GitHub Pages by
[`.github/workflows/docs.yml`](../../.github/workflows/docs.yml).

```bash
cd docs/site
npm ci
npm run dev       # http://localhost:5173, rebuilt on save (run `npm run sync` after editing a source page)
npm run build     # .vitepress/dist; DOCS_BASE=/<repo>/ for the Pages path
```

| File                         | Holds                                                                                                |
| ---------------------------- | ---------------------------------------------------------------------------------------------------- |
| `site.config.mjs`            | this repository's site: title, colours, the sections and which Markdown file is which page, the PDFs |
| `index.md`, `public/`        | the home page and its logo                                                                           |
| `scripts/sync.mjs`           | copies the listed pages into `content/` (git-ignored), fixing links, images and admonitions          |
| `.vitepress/config.mts`      | the VitePress configuration built from `site.config.mjs` (generic)                                   |
| `.vitepress/theme/style.css` | the shared look: figures, numbered guides, home cards                                                |

`scripts/sync.mjs`, `.vitepress/` and the workflow are the same in every product made from this
template; a product changes `site.config.mjs`, `index.md` and `public/`. Links between listed
pages become site links, links to other files of the repository go to GitHub, and a page edited
on the site opens its source file.

**Publishing.** Once per repository: Settings → Pages → Source: **GitHub Actions**. Until then the
workflow builds the site on every docs change and only reports that Pages is off.
