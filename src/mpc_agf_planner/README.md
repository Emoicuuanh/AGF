# mpc_agf_planner

Local planner plugin for move_base_flex. Path tracking uses an LTV-MPC; see the
[MPC setup and tuning guide](docs/mpc_setup_guide.md) for installation, model
identification, configuration and tuning.

# Format C++ code

```json
{
  // Add this line to ~/.config/Code/User/settings.json
  "C_Cpp.clang_format_fallbackStyle": " {ReflowComments: false, ColumnLimit: 80, BreakBeforeBraces: Allman, SpacesBeforeTrailingComments: 2, AlignTrailingComments: true, PointerAlignment: Left, }"
}
```
