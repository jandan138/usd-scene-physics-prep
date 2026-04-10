---
title: "Local VPS Proxy Toolkit"
code_reference:
  - "/root/.bashrc"
  - "/root/.local/bin/vps-proxy-toolkit.sh"
  - "/root/.local/share/zhuzihou-cli/vps_proxy_tunnel.py"
  - "/root/.local/bin/vps_proxy_tunnel_start"
  - "/root/.local/bin/vps_proxy_tunnel_status"
  - "/root/.local/bin/vps_proxy_tunnel_stop"
  - "/root/.config/claude-providers/vps-http-proxy.env"
  - "/root/.config/claude-providers/vps-direct-proxy.env"
  - "/root/.config/claude-providers/vps-static-tunnel.env"
  - "/root/.config/claude-providers/codex-openai-tunnel.env"
  - "/root/.local/bin/codex_vps"
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: "GitHub Copilot"
status: "completed"
---

# Local VPS Proxy Toolkit

## Summary

The local machine now exposes a unified `vps_proxy` shell interface aligned with the external `proxy-setup-complete.md` workflow.

Implemented modes:

- `vps_proxy d` / `vps_proxy direct` – clear proxy environment variables
- `vps_proxy s` / `vps_proxy static` – direct HTTP proxy through `45.32.95.77:18443` (VPS -> static residential)
- `vps_proxy v` / `vps_proxy vps` – direct HTTP proxy through `45.32.95.77:18444` (VPS native exit)
- `vps_proxy t` / `vps_proxy tunnel` – local SSH tunnel `127.0.0.1:28790 -> VPS 127.0.0.1:18443`
- `vps_proxy tv` / `vps_proxy tunnel-vps` – local SSH tunnel `127.0.0.1:28791 -> VPS 127.0.0.1:18444`

Tunnel lifecycle commands:

- `vps_proxy_tunnel_start <port>`
- `vps_proxy_tunnel_status <port>`
- `vps_proxy_tunnel_stop <port>`

## Local Implementation Details

### Shell integration

`/root/.bashrc` now defines:

- `vps_proxy()` – sources `/root/.local/bin/vps-proxy-toolkit.sh`
- `unset_proxy()` – calls `vps_proxy direct`

This means proxy switching affects the current interactive shell session instead of a short-lived subprocess.

### Proxy env files

- `vps-http-proxy.env` – `18443` static residential route
- `vps-direct-proxy.env` – `18444` VPS direct route

### Tunnel env files

- `vps-static-tunnel.env` – `28790 -> 18443`
- `codex-openai-tunnel.env` – `28791 -> 18444`

### Tunnel helper

`/root/.local/share/zhuzihou-cli/vps_proxy_tunnel.py` is the generic password-based SSH tunnel manager.

It maps local ports to root-only env files and can:

- start the expected SSH forward
- report running/stopped state
- stop the matching SSH process
- detect and replace stale or wrong-target SSH forwards on the same local port

## Codex Alignment

`codex_vps` has been aligned to the same `28791 -> 18444` path so the machine no longer has two conflicting meanings for port `28791`.

Current Codex path:

```text
codex_vps
  -> 127.0.0.1:28791
  -> SSH forward to VPS 127.0.0.1:18444
  -> VPS direct OpenAI / ChatGPT egress
```

## Validation Results

Validated on 2026-04-10:

1. `type vps_proxy` reports a shell function from `/root/.bashrc`
2. `vps_proxy` without args prints the documented `d / v / s / t / tv` modes
3. `vps_proxy d` clears proxy env vars
4. `vps_proxy s` sets `HTTP_PROXY` to `45.32.95.77:18443`
5. `vps_proxy v` sets `HTTP_PROXY` to `45.32.95.77:18444`
6. `vps_proxy t` starts `127.0.0.1:28790`; `curl https://api.ipify.org` through that shell env returned `HTTP 200`
7. `vps_proxy tv` starts `127.0.0.1:28791`; `curl https://api.openai.com/v1/models` through that shell env returned `HTTP 401`
8. `codex_vps --version` returned `codex-cli 0.118.0`
9. `codex_vps exec --color never --output-last-message ... "Reply with exactly: ok"` returned `ok`
10. `vps_proxy_tunnel_stop 28790` and `codex_vps_tunnel_stop` successfully terminate their tunnels and status becomes `stopped`

## Known Divergence From The External Doc

The external doc mentions SSH key-based local files such as:

- `~/.ssh/vps_proxy_ed25519`
- `~/.ssh/config`

This machine intentionally keeps the already-working password-based SSH tunnel approach instead. Credentials are stored in root-only env files under `/root/.config/claude-providers/`.
