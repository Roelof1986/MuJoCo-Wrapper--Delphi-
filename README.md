Absolutely — here’s your complete, fully English README.md for the MuJoCo-Wrapper (Delphi) project, in one clean code block, ready for GitHub.
It’s professional, technically accurate, and written in the same style as top-tier open-source repos.


---

# MuJoCo-Wrapper (Delphi)

A lightweight **C++ → Delphi bridge** for the [MuJoCo](https://mujoco.org) physics engine, providing real-time simulation, GLFW visualization, and native mouse-camera control (orbit / pan / zoom).  
This wrapper exposes a compact `MJW_*` API so Delphi applications can run **physics, rendering, and joint control** directly through a C++ DLL.

<p align="center">
  <img src="docs/screenshot_humanoid.png" width="600" alt="Humanoid simulation with MuJoCo-Wrapper (Delphi)">
</p>

---

## ✨ Features

- 🔌 **C++ DLL bridge** for Delphi (`mujoco_delphi_wrapper.dll`)
- 🧠 Real-time **MuJoCo physics** + **GLFW** visualization
- 🖱️ **Native mouse camera**:  
  - Left drag → Orbit  
  - Right drag → Pan  
  - Mouse wheel → Zoom  
  - Adjustable sensitivity (`MJW_SetMouseCamParams`)
- 🦾 Full **joint introspection and torque control**
- 🧪 Clean `MJW_*` API, compatible with any Pascal compiler
- 🧾 **MIT License** (wrapper only; MuJoCo & GLFW retain their own)

---

## 📁 Repository structure

MuJoCo-Wrapper-Delphi/ 
├─ CMakeLists.txt 
├─ src/ 
│   └─ mujoco_delphi_wrapper.cpp      # main C++ wrapper 
├─ examples/ 
│   └─ delphi/Unit1.pas               # Delphi example using FMX 
├─ models/ 
│   └─ humanoid.xml                   # test model 
├─ docs/ 
│   └─ screenshot_humanoid.png 
├─ LICENSE.txt └─ THIRD_PARTY_NOTICES.txt

---

## ⚙️ Build Instructions (CMake)

### Requirements
- [MuJoCo](https://github.com/google-deepmind/mujoco) (headers + binaries)
- [GLFW](https://www.glfw.org) (fetched automatically via CMake)
- **CMake ≥ 3.16**
- **Compiler:** MSVC (Windows) / Clang or GCC (Linux)

> **Windows:** Visual Studio 2022 recommended  
> **Linux:** Install OpenGL & GLFW dev packages or use FetchContent

---

### 🪟 Windows / Visual Studio build

Open *Developer PowerShell for VS* in the repo root:

```powershell
# 1) Configure the project
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DMUJOCO_DIR="%CD%/third_party/mujoco"

# 2) Build the Release DLL
cmake --build build --config Release

The resulting DLL will be here:

build/Release/mujoco_delphi_wrapper.dll


---

🐧 Linux build (Makefiles)

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DMUJOCO_DIR="$PWD/third_party/mujoco"
cmake --build build -j

> Tip: You can make the path portable by using:

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco")
include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/bin)

Then just place the MuJoCo SDK under third_party/mujoco/.




---

🧩 Using the DLL from Delphi

1️⃣ Define the externals

Create a MujocoWrapper.pas (or inline in your Unit1.pas):

type
  TMJWHandle = Pointer;

function  MJW_Create(modelXML: PAnsiChar; width, height: LongInt): TMJWHandle; cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Destroy(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Step(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_Render(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_PollEvents(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ShouldClose(h: TMJWHandle): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_SetTimestep(h: TMJWHandle; dt: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_GetSimTime(h: TMJWHandle): Double; cdecl; external 'mujoco_delphi_wrapper.dll';

function  MJW_JointNameToDof(h: TMJWHandle; jointName: PAnsiChar; var dofIndex, ndof: LongInt): LongInt; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ReadQpos(h: TMJWHandle; idx: LongInt): Double; cdecl; external 'mujoco_delphi_wrapper.dll';
function  MJW_ReadQvel(h: TMJWHandle; idx: LongInt): Double; cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_ApplyDof(h: TMJWHandle; dofIndex: LongInt; value: Double); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_ClearApplied(h: TMJWHandle); cdecl; external 'mujoco_delphi_wrapper.dll';

procedure MJW_EnableMouseCam(h: TMJWHandle; enable: LongInt); cdecl; external 'mujoco_delphi_wrapper.dll';
procedure MJW_SetMouseCamParams(h: TMJWHandle; rotScale, panScale, zoomScale: Double); cdecl; external 'mujoco_delphi_wrapper.dll';


---

2️⃣ Minimal Delphi example

var
  H: TMJWHandle;
begin
  H := MJW_Create(PAnsiChar(UTF8String('models\humanoid.xml')), 1200, 900);
  if H = nil then Halt(1);

  MJW_SetTimestep(H, 1/240.0);
  MJW_EnableMouseCam(H, 1);
  // Optional: stronger zoom
  // MJW_SetMouseCamParams(H, -1, -1, 0.01);

  while MJW_ShouldClose(H) = 0 do
  begin
    MJW_ClearApplied(H);
    // Apply torques or run control logic:
    // MJW_ApplyDof(H, dofIndex, torque);
    MJW_Step(H);
    MJW_Render(H);
    MJW_PollEvents(H);
  end;

  MJW_Destroy(H);
end.


---

🖱️ Mouse Controls

Action	Input

Orbit	Left mouse drag
Pan	Right mouse drag
Zoom	Mouse wheel
Sensitivity	MJW_SetMouseCamParams(h, rot, pan, zoom)



---

🧠 Integration notes

Works seamlessly with Delphi FMX and console builds

Can be linked into C#, Python, or C as well

Designed for neural simulation, e.g. integrating CReST or reinforcement controllers

All joint sensors (angles/velocities) can be read via MJW_ReadQpos / MJW_ReadQvel



---

⚠️ Troubleshooting

Issue	Fix

Black window	Ensure mjv_updateScene → mjr_render → glfwSwapBuffers loop runs
Mouse not working	Call MJW_EnableMouseCam(H, 1) after MJW_Create
“DLL not found”	Place mujoco_delphi_wrapper.dll next to your EXE
Wheel too weak	Increase zoomScale (e.g. 0.01)
Model error	Check path or syntax in humanoid.xml



---

📜 License

MuJoCo-Wrapper (Delphi) — © 2025 Roelof Emmerink
Released under the MIT License.
See LICENSE.txt.

This project depends on third-party software licensed separately:

MuJoCo — Apache 2.0

GLFW — Zlib/MIT
For details, see THIRD_PARTY_NOTICES.txt.



---

🙌 Acknowledgements

MuJoCo — Multi-Joint dynamics with Contact

GLFW — OpenGL window and input

Google DeepMind — for open-sourcing MuJoCo

Community examples, especially the simulate reference app



---

Created by Roelof Emmerink — 2025

---
