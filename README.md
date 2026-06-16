# Bridging MVH DR

## Layout

- `src/`, `include/`: implementation
- `tests/`: solver comparison tests
- `resources/synthetic_graph/`: small 5-objective fixture

## Build

```powershell
cmake -S . -B build
cmake --build build
```

## Test

```powershell
ctest --test-dir build --output-on-failure
```

## Example

```powershell
.\build\MultivaluedHeuristicSearch.exe --map .\resources\synthetic_graph --start 1 --goal 20 --algorithm NAMOA_DR --objectives 0 1 2 3 4 --cutoffTime 5
```
