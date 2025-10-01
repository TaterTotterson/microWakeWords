edit home-assistant-voice.yaml to use the wake word you want to use and paste it in ESPhome and update your Home Assistant Voice!

edit lines 32 and 33 to the name you want
```
  name: tatervpe
  friendly_name: TaterVPE
```

edit lines 77-79 to your wifi or use secrect, change ip address to you ha voice ip or remove this line
```
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  use_address: 10.4.20.60
```

edit lines 1596-1597 to the url of the wake work you want to use and name
```
  models:
    - model: https://git.phooey.cloud/mastaful/microWakeWord-Trainer-AppleSilicon/raw/branch/main/mww_tater/hey_tater.json
      id: hey_tater
```

edit lines 1659-1666 change "hey_tater" to the name of the wake word, same as line 1597
```
      lambda: |-
        if (x == "Slightly sensitive") {
          id(hey_tater).set_probability_cutoff(217);    // 0.85 -> 0.000 FAPH on DipCo (Manifest's default)
        } else if (x == "Moderately sensitive") {
          id(hey_tater).set_probability_cutoff(176);    // 0.69 -> 0.376 FAPH on DipCo
        } else if (x == "Very sensitive") {
          id(hey_tater).set_probability_cutoff(143);    // 0.56 -> 0.751 FAPH on DipCo
        }
```
