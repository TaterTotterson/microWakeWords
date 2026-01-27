## 🗣️ Request a New Wake Word

You can request a new microWakeWord model by opening a GitHub issue.

### ✅ How to request a word

1. Go to the **Issues** tab  
2. Click **New issue**
3. Set the **title** to:

mww: your wake word here

Examples:
```
mww: hey tater  
mww: tater totterson  
mww: hello computer  
```
That’s it — no labels, no templates, no body text required.

---

### 🔄 What happens next
- The `.tflite` and `.json` files are added to the repository
- The issue is labeled, commented on, and closed when complete

---

### ⚠️ Notes

- **Test your wake word with TTS first.**  
  Make sure your text-to-speech engine pronounces the phrase the way you expect.  
  You may need to spell it *phonetically* or a little “funny” so TTS says it correctly — the trainer uses the same pronunciation.
- Please request **one wake word per issue**
- Avoid punctuation or emojis in the title
- Training runs sequentially if multiple requests are open

---

## 🗣️ Set Up Your Custom Wake Word on Home Assistant Voice

⚠️ **Important:**  
voicePE-TaterTimer.yaml is for **Voice PE**, but the same structure and steps apply to *any* Home Assistant voice device.  
You can **mimic these instructions** for your own hardware by updating the equivalent file for your device.

All of the settings below are located **at the very top of the YAML file** inside the `substitutions:` section.  
You no longer need to hunt for line numbers — everything commonly edited lives in one place.

Open `voicePE-TaterTimer.yaml` (or your device’s YAML) and edit the `substitutions:` block.

---

### 🧾 Device Name & Friendly Name
Change how the device appears in ESPHome and Home Assistant:
```
device_name: tatervpe
friendly_name: TaterVPE
```
---

### 📡 Wi-Fi & Network Settings
Set your Wi-Fi credentials (or use secrets) and optionally pin the device to a Home Assistant Voice IP:
```
wifi_ssid: !secret wifi_ssid
wifi_password: !secret wifi_password
ha_voice_ip: "127.0.0.1"
```
If you don’t want a fixed IP, simply remove `ha_voice_ip` and the device will use DHCP.

---

### 🎙️ Wake Word Model
Choose the wake word model and give it a matching ID:
```
wake_word_name: hey_tater
wake_word_model_url: https://raw.githubusercontent.com/TaterTotterson/microWakeWords/refs/heads/main/microWakeWords/hey_tater.json
```
The `wake_word_name` **must match** the model ID used internally.

---

### 🎚️ Wake Word Sensitivity
Tune how sensitive the wake word detection is:
```
wake_cutoff_slight: "250"     # Slightly sensitive (very strict)
wake_cutoff_moderate: "245"   # Balanced
wake_cutoff_very: "222"       # Very sensitive
```
Lower numbers = more sensitive  
Higher numbers = fewer false activations

---

### 🔔 Optional – Change the Wake Sound
You can customize the sound played when the wake word is detected.

Edit the wake sound URL in the substitutions section:
```
wake_word_triggered_sound_file: https://github.com/esphome/home-assistant-voice-pe/raw/dev/sounds/wake_word_triggered.flac
```
You can point this to any compatible `.mp3` or `.flac` file hosted online.

---

### ✅ Final Notes
• These values are read throughout the config automatically  
• No other parts of the YAML need to be edited  
• Test your wake word in **TTS first** to ensure it’s pronounced correctly  
  (you may need to spell it creatively for best results)
