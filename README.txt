GeoGlobe Project (VTK 9.5.2)
============================

Requirements:
- Windows 10
- Visual Studio 2022 (Community Edition)
- CMake 3.30 or newer
- VTK 9.5.2 (downloaded and extracted to C:\VTK)

Setup Instructions:
1. Extract this folder anywhere (e.g. C:\GeoGlobe).
2. Open "CMake GUI".
3. Source folder: C:/GeoGlobe
   Build folder:  C:/GeoGlobe/build
4. Click "Configure" → choose "Visual Studio 17 2022 (Win64)".
5. When asked for VTK_DIR, set:
   C:/VTK/lib/cmake/vtk-9.5
6. Click "Generate" → "Open Project".
7. In Visual Studio: Build → Build Solution.
8. Run → Start Without Debugging (Ctrl + F5).



