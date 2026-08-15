# Mock data for frontend development

Fixtures served by the Vite dev server so the frontend can be developed without flashing an
ESP32 or running the firmware. The middleware that serves them is in `vite.config.ts`.

## Known broken

**The four endpoints below currently return 500, so `npm run dev` does not work offline.**
This directory holds `.bin` files; `vite.config.ts`'s `mockDataMap` still names `.json`
ones. See the comment on that map before changing either side.

| endpoint | file |
|---|---|
| `GET /status` | `status.bin` |
| `GET /configuration` | `configuration.bin` |
| `GET /machine-definition` | `machine-definition.bin` |
| `GET /routines` | `routines.bin` |

`GET /shots` is unaffected and deliberately points elsewhere — at
`fixtures/shot_log_list.bin`, which the schema exporter generates. Those are the same bytes
the round-trip test checks against Rust, so that mock cannot drift from the wire format.

The middleware picks its `Content-Type` by file extension, because the two kinds are not
interchangeable: postcard bodies are read as bytes by `fetchPostcard` and would choke on a
JSON content type.
