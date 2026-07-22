# docs/ — the GitHub Pages site

This folder holds the published project notes. GitHub builds it with Jekyll.

## Publishing

In the repository on GitHub: **Settings → Pages → Build and deployment**

- Source: **Deploy from a branch**
- Branch: **main**
- Folder: **/docs**

The site then appears at `https://<your-username>.github.io/swarm_robotics/`.

## Before you publish

Open `_config.yml` and check `baseurl`:

| Where the site is published | Correct `baseurl` |
|---|---|
| `username.github.io/swarm_robotics/` | `"/swarm_robotics"` |
| `username.github.io/` (a user site repo) | `""` |

If `baseurl` is wrong, the CSS and the images will not load.

## Local preview (optional)

```bash
cd docs
bundle install
bundle exec jekyll serve --baseurl ""
# open http://localhost:4000
```

## Layout

```text
docs/
├── _config.yml           # site settings and the sidebar navigation order
├── _layouts/
│   └── default.html      # the shell: sidebar, header, footer, KaTeX
├── assets/
│   ├── css/style.css     # all styling
│   ├── images/           # 15 result plots + 4 simulator screenshots
│   └── video/            # 2 clips of the Phase 4b forest run, plus posters
├── index.md              # overview and the full phase plan
├── phase-0.md            # setup and dynamics
├── phase-1.md            # control and locomotion
├── phase-2.md            # perception
├── phase-3.md            # the decentralized communication protocol
├── phase-4.md            # navigation and search
├── phase-5.md            # planned work
└── glossary.md           # every term, one meaning each
```

## Adding a new phase page

1. Copy an existing page, for example `phase-5.md`.
2. Set the `permalink` in the front matter, for example `/phase-6/`.
3. Add an entry to the `nav:` list in `_config.yml`. The `url` must match the
   `permalink` exactly, including both slashes.

## Tabs

Each page is divided into tabs. A tab starts at a marker div:

```html
<div class="tabmark" data-tab="Results"></div>
```

Everything from one marker to the next marker becomes one tab panel. The JavaScript in
`_layouts/default.html` finds these markers and builds the sticky tab bar. A page with
fewer than two markers renders as one continuous page, with no tab bar.

Keep between three and six tabs on a page. More than six makes the bar wrap.

To link to a specific tab, use the slug of the tab label: lower case, with each run of
non-alphanumeric characters replaced by one hyphen. The label `1A — PID control` becomes
`/phase-1/#1a-pid-control`.

## Video

Videos live in `assets/video/` and appear on the overview page, in the "Simulation video"
tab. Each one needs two files: the `.mp4` and a `.jpg` poster frame.

Keep every file under 50 MB. GitHub rejects any file over 100 MB. Re-encode a raw screen
recording before you add it:

```bash
ffmpeg -i raw.mp4 -vf "scale=1280:-2" -c:v libx264 -crf 26 -preset fast \
  -pix_fmt yuv420p -movflags +faststart -an assets/video/new-clip.mp4
ffmpeg -ss 10 -i assets/video/new-clip.mp4 -frames:v 1 -q:v 3 \
  assets/video/poster-new-clip.jpg
```

Raise `-crf` for a smaller file, or lower it for better quality. The useful range is 23
to 30. The `preload="none"` attribute on each `<video>` tag means the browser downloads
only the poster image until the reader selects play.

## Theme

The site follows the operating system setting by default. The control at the foot of the
sidebar cycles through system, light, and dark. The choice is stored in `localStorage`
under the key `sr-theme`.

To change the colours, edit the two variable blocks at the top of `assets/css/style.css`:
`[data-theme="light"]` and `[data-theme="dark"]`. The `prefers-color-scheme` block below
them repeats the dark values and must be kept in step.

## Writing standard

The pages use ASD-STE100 Simplified Technical English: short sentences, the active
voice, and one meaning for each word.
