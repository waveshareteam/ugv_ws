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

Package **`ugv_voice`**. Neither node publishes **`/cmd_vel`**.

| Node | Default `language` | What it does |
|------|--------------------|--------------|
| **`voice_ctrl`** | **`en`** | Manual KWS / ASR / TTS over topics. Wake word and recognition only **print to the terminal** — they do not start a dialog or move the robot. |
| **`voice_chat`** | **`zh`** | KWS starts on launch. After a wake word, the node replies, listens (~10 s), sends the transcript to Ollama, and speaks the answer. |

Parameter: **`language`** — **`zh`** or **`en`**. That chooses the wake-word list and the ASR model. Chinese TTS uses the VITS model; English TTS uses **pyttsx3** / eSpeak.

Capture device is hardcoded to **`plughw:2,0`** (`arecord`). Check with **`arecord -l`**. Do not run **`voice_ctrl`** and **`voice_chat`** at the same time — both open the mic.

ASR/TTS weights are Git LFS files. Pull them **on the host when you clone** (`git lfs install` then `git lfs pull`) — not as root in Docker. If `language:=zh` fails on a missing **`encoder_jit_trace-pnnx.ncnn.bin`**, see [Installation](installation.md#clone-on-the-host-git-lfs).

### Wake words

Say **one** of the phrases below (same list for **`voice_ctrl`** and **`voice_chat`**). Match **`language`**.

#### Chinese (`language:=zh`)

| Say | Notes |
|-----|--------|
| **小爱同学** | Default example |
| **小薇小薇** | Repeat the name |
| **小艺小艺** | Repeat the name |
| **张伟张伟** | Repeat the name |

#### English (`language:=en`)

| Say |
|-----|
| **hello world** |
| **hi google** |
| **hey siri** |
| **alexa** |
| **love and peace** |
| **play music** |
| **go home** |
| **happy new year** |
| **merry christmas** |

Readable copies: **`keywords_raw.txt`** next to each model. The node loads only the encoded **`keywords.txt`**. Editing **`keywords_raw.txt`** by itself does nothing.

### Change the wake-word list

`sherpa-onnx-cli` is installed with the **`sherpa_onnx`** pip package (`build_first.sh` / `requirements.txt`). With **`--symlink-install`**, edit the files under **`src/`** and **restart the node** — no rebuild.

**1. Edit the raw list** (one phrase per line). Chinese: no spaces in the phrase; you can append **`@显示名`**. Optional **`:score`** (boost) and **`#threshold`** (trigger):

```text
小爱同学
你好小车 @你好小车
小薇小薇 :2.0 #0.25 @小薇小薇
```

English (uppercase is typical for this BPE model):

```text
HELLO WORLD
HEY ROBOT
```

**2. Encode into `keywords.txt`.**

Chinese (`language:=zh`), **ppinyin**:

```bash
cd /home/ws/ugv_ws/src/ugv_main/ugv_voice/ugv_voice/models/kws/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01

sherpa-onnx-cli text2token \
  --tokens tokens.txt \
  --tokens-type ppinyin \
  keywords_raw.txt keywords.txt
```

English (`language:=en`), **bpe** (needs **`bpe.model`**):

```bash
cd /home/ws/ugv_ws/src/ugv_main/ugv_voice/ugv_voice/models/kws/sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01

sherpa-onnx-cli text2token \
  --tokens tokens.txt \
  --tokens-type bpe \
  --bpe-model bpe.model \
  keywords_raw.txt keywords.txt
```

**3. Restart** **`voice_ctrl`** or **`voice_chat`**.

Do not hand-edit **`keywords.txt`** unless you know the token alphabet. The node also applies **`keywords_score=2.0`** and **`keywords_threshold=0.25`** in `kws_sherpa_onnx.py`; if a phrase false-triggers or never fires, change that line's **`:score`** / **`#threshold`** or those two arguments, then restart.

Upstream: [sherpa-onnx KWS](https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html).

---

### Voice control

Keyword spotting, speech recognition, and text-to-speech as **separate** switches. Useful to test the mic and models. Does **not** call the LLM and does **not** turn a wake word into ASR automatically.

**Launch:**

```bash
ros2 run ugv_voice voice_ctrl --ros-args -p language:=en
```

Chinese: `-p language:=zh`.

KWS and ASR are **off** until you publish:

| Feature | Start | Stop | When it fires |
|---------|-------|------|----------------|
| KWS | `ros2 topic pub /kws std_msgs/Bool "{data: true}" --once` | `data: false` | Wake word → terminal: `Keyword Spotting` |
| ASR | `ros2 topic pub /asr std_msgs/Bool "{data: true}" --once` | `data: false` | Speech → terminal: `voice reconize …` |
| TTS | `ros2 topic pub /tts std_msgs/String "{data: 'Hello robot'}" --once` | — | Speaks the string |

Typical test: enable **`/kws`**, say a [wake word](#wake-words), confirm the log line; then enable **`/asr`** and speak a sentence.

Press **`Ctrl+C`** to stop.

---

### Voice chat

Wake word → listen → Ollama → speak. Does **not** drive the chassis.

**Launch:**

```bash
ros2 run ugv_voice voice_chat \
  --ros-args \
  -p language:=zh \
  -p server_url:=http://<ollama-ip>:11434/api/chat
```

Requires **`qwen3:8b`** (or set **`llm_model`**) on the Ollama server. Optional: **`prompt_file`** (default `ugv_voice/prompt.txt`).

Flow after launch (KWS is already on):

1. Say a [wake word](#wake-words). You can also publish **`/kws`** `{data: true}` once.
2. Robot speaks **「我在，你说」** (zh) or **I'm listening** (en).
3. Speak a sentence. ASR waits up to **10 s**.
4. Transcript goes to Ollama; the reply is spoken.
5. If nothing is recognized: **「没有听清，请再说一遍」** / **Sorry, I didn't catch that**, then KWS starts again.

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
| Voice node silent | **`voice_ctrl`**: KWS / ASR still off | Publish **`/kws`** or **`/asr`** **`true`** |
| Wake word ignored | Wrong **`language`**, or phrase not in the [list](#wake-words) | Use **`zh`** vs **`en`** phrases; say the full phrase clearly |
| `encoder_jit_….bin does not exist` | Git LFS weights not pulled on the host | On the **host**: `git lfs install` and `git lfs pull` ([Installation](installation.md#clone-on-the-host-git-lfs)) |
| `arecord` / no capture | Mic is not **`plughw:2,0`** | `arecord -l`; USB audio device index may differ |
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
