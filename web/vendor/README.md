# vendor/

Third-party JS bundled locally so the field tool needs no internet at runtime
(see CLAUDE.md — "No CDN fetches"). These are pinned snapshots; update only
with a deliberate bump.

| file            | upstream                                         | version | license |
|-----------------|--------------------------------------------------|---------|---------|
| deck.min.js     | https://unpkg.com/deck.gl                        | 9.0.0   | MIT     |
| roslib.min.js   | https://cdn.jsdelivr.net/npm/roslib              | 1.4.1   | BSD-3   |

Both are UMD bundles exposing globals (`deck`, `ROSLIB`).
