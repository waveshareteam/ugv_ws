# Experimental

Optional features — **voice**, **Ollama LLM**, and **Web AI**. Not required for the core tutorial path ([Mapping](mapping.md) → [Navigation](navigation.md)).

For the browser control UI (teleop, camera, mapping, navigation), see [Web App](web_app.md).

Packages: **`ugv_voice`**, **`ugv_chat_ai`**, plus **`ugv_tools`** **`behavior_ctrl`** for Web AI motion.

For the suggested reading order, see [index](index.md#suggested-reading-order).

---

## Prerequisites

1. **Build and source** **`ugv_ws`** ([Installation](installation.md)).
2. Set **`UGV_MODEL`** and **`LDLIDAR_MODEL`** when the robot must move ([environment variables](index.md#product-names-vs-environment-variables)).
3. **Microphone** (and speakers) for voice nodes.
4. **Ollama** on the network for [Voice chat](#voice-chat) and [Web AI](#web-ai) — [Ollama](https://github.com/ollama/ollama), model **`qwen3:8b`**.

!!! warning "Safety"
    [Web AI](#web-ai) drives the chassis via **`behavior_ctrl`** → **`/cmd_vel`**. Clear the area before use.

  Emergency stop:

  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --once
  ```

---

## Overview

### Before you start

| Feature | Needs bringup | Conflicts with |
|---------|---------------|----------------|
| [Voice control](#voice-control) | No | — |
| [Voice chat](#voice-chat) | No | — |
| [Web AI](#web-ai) | Yes — **`/odom`** | [Teleoperation](teleoperation.md), [Nav2](navigation.md), [LiDAR](lidar.md) / [Vision](vision.md) motion |

Pick **one** motion source at a time when **`behavior_ctrl`** is active — see [Teleoperation — One motion source at a time](teleoperation.md#one-motion-source-at-a-time).

### Features

| Feature | Executable | Section |
|---------|------------|---------|
| Voice control (KWS / ASR / TTS) | `voice_ctrl` | [Voice control](#voice-control) |
| Voice + Ollama chat | `voice_chat` | [Voice chat](#voice-chat) |
| LLM web UI + motion | `behavior_ctrl` + `app` | [Web AI](#web-ai) |

Source: `src/ugv_main/ugv_voice/`, `ugv_chat_ai/`, `ugv_tools/behavior_ctrl.py`.

---

## Voice

Argument: **`language`** — **`zh`** or **`en`**.

### Voice control

Keyword spotting, speech recognition, and text-to-speech. Does **not** publish **`/cmd_vel`**.

**Launch:**

```bash
ros2 run ugv_voice voice_ctrl --ros-args -p language:=en
```

Control sub-features via topics:

| Feature | Start | Stop |
|---------|-------|------|
| KWS | `ros2 topic pub /kws std_msgs/Bool "{data: true}" --once` | `data: false` |
| ASR | `ros2 topic pub /asr std_msgs/Bool "{data: true}" --once` | `data: false` |
| TTS | `ros2 topic pub /tts std_msgs/String "{data: 'Hello robot'}" --once` | — |

Chinese: `-p language:=zh`.

If the program no longer needs to run, press **`Ctrl+C`**.

---

### Voice chat

Spoken dialog through Ollama. Does **not** drive the chassis by default.

**Launch:**

```bash
ros2 run ugv_voice voice_chat \
  --ros-args \
  -p language:=en \
  -p server_url:=http://<ollama-ip>:11434/api/chat
```

Requires **`qwen3:8b`** (or compatible model) on the Ollama server.

---

## Web AI

Browser chat that sends motion commands to the **`/behavior`** action. **`behavior_ctrl`** executes open-loop moves and publishes **`/cmd_vel`**.

CLI command reference, prerequisites, and all JSON types (including map points / Nav2): **[Behavior Command Control](behavior_ctrl.md)**.

Supported behavior types in the default Web AI prompt: **`drive_on_heading`**, **`back_up`**, **`spin`**, **`stop`**.

Example LLM JSON: `{"T": 1, "type": "drive_on_heading", "data": 2}`

### Workflow

| Role | What to run |
|----------|-------------|
| **T0** | **`ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true`** — skip if [Mapping](mapping.md) / [Navigation](navigation.md) already includes bringup |
| **T1** | **`ros2 run ugv_tools behavior_ctrl`** |
| **T2** | **`ros2 run ugv_chat_ai app --ros-args -p server_url:=http://<ollama-ip>:11434/api/chat`** |

Open **`http://<robot-ip>:5000`** in a browser (factory images).

Stop [teleoperation](teleoperation.md) and other **`/cmd_vel`** publishers before **T1**.

Not the same as [Web App](web_app.md) — Web AI uses port **5000**; Vizanti (Web App) defaults to **5100**. Run **one** web stack at a time if unsure.

### behavior_ctrl

Action server on **`/behavior`**. Subscribes to **`/odom`**.

```bash
ros2 run ugv_tools behavior_ctrl
```

Leave running in **T1** while using the chat web app.

### Chat web app

```bash
ros2 run ugv_chat_ai app --ros-args -p server_url:=http://<ollama-ip>:11434/api/chat
```

Requires Ollama with **`qwen3:8b`**. Parsed JSON from the LLM is sent to **`behavior_ctrl`**.

---

## Troubleshooting

| Symptom | Likely cause | What to try |
|---------|--------------|-------------|
| Voice node silent | KWS / ASR not enabled | Publish **`/kws`** or **`/asr`** **`true`** |
| Voice chat / Web AI fails | Ollama unreachable | Check **`server_url`**, firewall, **`qwen3:8b`** pulled |
| Web AI no motion | **`behavior_ctrl`** not running | Start **T1** before **T2** |
| Robot moves unexpectedly | Teleop / Nav2 still up | Stop other **`/cmd_vel`** sources |
| Port **5000** busy | [Web App](web_app.md) or old session | Stop the other web app (Web App defaults to **5100**) |

---

## Related Tutorials

| Chapter | What it adds |
|---------|----------------|
| [Web App](web_app.md) | Browser control & visualization (separate chapter) |
| [Mapping](mapping.md) | SLAM stack |
| [Navigation](navigation.md) | Nav2 (do not run with Web AI motion) |
| [Keyboard & Gamepad Control](teleoperation.md) | Manual drive (stop before Web AI) |
| [Hardware Driver](bringup.md) | **T0** for Web AI |
| [Gazebo](gazebo.md) | Simulation |

**Next:** [Gazebo](gazebo.md).
