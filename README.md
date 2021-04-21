
**SE4130 Real-Time Embedded Systems**

**Course Project, Spring 2021**

**Team #3 - Team Nuclear Football**

**Tim Collier, Austin FitzGerald, Brandon Krcmar**

---

This repository contains our codebase. All documentation and non-release deliverables are in Microsoft Teams.

---

**Setting up the IDE to use code outside of main.c**

1. In the workspace explorer, right-click on the project and choose properties
2. Choose "C/C++ General" -> "Paths and Symbols"
3. In the "Includes" tab, click "Add"
4. Type "${ProjDirPath}/Inc", click "Ok" and "Apply"
5. Switch to the "Source Location" tab and click "Add Folder..."
6. Choose both "Inc" and "Src", click "Ok" and "Apply and Close"