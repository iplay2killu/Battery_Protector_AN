# ✅ ToDo - Battery Protector Project

This file tracks the progress of hardware and firmware tasks. Please update it before each pull request!

---

## 📅 Immediate Tasks

- [ ] **Test all peripherals on PCB Rev 1.0.0** _(Blocks ordering PCB Rev 2.0.0)_
  - [x] Accelerometer ✅
  - [ ] Piezo sensor amplifier circuit
    - Must order 7BB-20-3 piezoelectric transducer
    - Solder to board before testing
  - [x] Buzzer ✅
  - [ ] USB Micro ORing issue investigation
    - 5V buck converter remains disabled after 5V_MCU removal (Micro USB unplug)
  - [ ] Noise issue investigation
    - Audible humming from inductor + buzzer > 40V input
    - Ideas: increase switching frequency? Move buzzer away from high-noise areas
  - [ ] Gate Driver functionality test
  - [ ] Wifi Testing 

- [ ] **Schematic design migration to KiCAD**
  - Clean up symbols/footprints
  - Update based on Rev 1.0.0 testing outcomes

---

## 🚧 In Progress

_(Move tasks here if you're currently working on them)_

---

## ✅ Completed

_(Keep a record of major finished items)_
- [x] Accelerometer testing
- [x] Buzzer testing

---

## 📌 Notes

- **Next milestone:** Complete peripheral validation to enable PCB Rev 2.0.0 order
- Assign initials or GitHub handles to tasks if needed (e.g., `@username`)

---
