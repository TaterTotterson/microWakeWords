## Create a issue to request a word!
---
edit home-assistant-voice.yaml to use the wake word you want to use and paste it in ESPhome and update your Home Assistant Voice!

edit lines 32 and 33 to the name you want
```
  name: tatervpe
  friendly_name: TaterVPE
```

edit lines 77-79 to your wifi or use secrets, change ip address to your ha voice ip or remove this line
```
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  use_address: 10.4.20.60
```

edit lines 1596-1597 to the url of the wake work you want to use and edit id to match
```
  models:
    - model: https://raw.githubusercontent.com/TaterTotterson/microWakeWords/refs/heads/main/microWakeWords/hey_tater.json
      id: hey_tater
```

edit lines 1659-1666 change "hey_tater" to the name of the wake word, same as the id on line 1597
```
      lambda: |-
        if (x == "Slightly sensitive") {
          id(hey_tater).set_probability_cutoff(250);    // 0.98 -> 0.000 FAPH
        } else if (x == "Moderately sensitive") {
          id(hey_tater).set_probability_cutoff(245);    // 0.96 -> 0.187 FAPH
        } else if (x == "Very sensitive") {
          id(hey_tater).set_probability_cutoff(222);    // 0.87 -> 0.375 FAPH
        }
```
---
## Optional - Change the Wake Sound
edit line 25 with the url to your wake sound
```
wake_word_triggered_sound_file: https://github.com/esphome/home-assistant-voice-pe/raw/dev/sounds/wake_word_triggered.flac
```
