# Persistent Camera Identification Plan

## Goal

Make Linux UVC cameras with duplicate or missing USB serials persistently
selectable by their physical USB port. The identity intentionally follows the
port: reconnecting a camera to the same port preserves selection, while moving
it to another port changes its identity.

## Current Finding

Three Innomaker U20CAM-1080p-S1 cameras report the same USB serial, `SN0001`.
Their `/dev/v4l/by-id` names collide and cannot identify individual units.
Their `/dev/v4l/by-path` names are distinct and identify their USB topology.

## Design

1. Resolve Linux UVC USB identity from sysfs and `/dev/v4l/by-path`.
   - Capture USB VID, PID, serial, physical devpath, canonical `ID_PATH`, and
     the capture node's by-path selector.
   - Prefer the normal `usb-` by-path alias over the `usbvN-` alias.

2. Preserve serial identity where it is valid.
   - A camera with a unique serial continues to use its by-id selector.
   - Same-model cameras sharing a serial use their by-path selector.
   - If no by-path selector is available, retain index fallback and mark the
     identity unstable.

3. Make by-path selectors openable through the default OmniCamera backend.
   - Resolve the by-path link to the current video index immediately before
     open, then open that bare index.
   - Continue rejecting ambiguous by-id selectors rather than silently opening
     whichever unit currently owns the symlink.

4. Keep identity stable through user-facing workflows.
   - Discovery metadata exposes serial, VID/PID, physical port, by-id, and
     by-path information.
   - Interactive selection displays the physical port for colliding cameras.
   - Config export/import preserves the configured by-path string unchanged.
   - SharedCamera derives a stable service token from the configured by-path,
     not the currently assigned `/dev/videoN`.

5. Extend setup validation and documentation.
   - USB diagnostics accept both raw video nodes and by-path selectors.
   - Docs explain that by-id identifies a physical unit only when its serial is
     unique, while by-path identifies a fixed USB socket.

## Verification

- Unit tests cover unique serials, duplicate serials, missing by-path links,
  by-path resolution, and ambiguous by-id rejection.
- Config and SharedCamera tests prove that by-path strings and service names do
  not change when `/dev/videoN` changes.
- Hardware acceptance: record selectors, reboot or replug into the same ports,
  and verify each selector reconnects to the intended port.
