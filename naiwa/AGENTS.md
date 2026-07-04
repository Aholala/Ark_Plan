# Project Coding Rules

This project uses a layered embedded architecture. Future code should follow
these rules unless the user explicitly asks for a different design.

## Architecture

- Keep hardware details separated from reusable logic.
- Prefer these layers:
  - `User/Bsp`: chip, HAL, pin, peripheral, DMA, interrupt, and board-specific code.
  - `User/Module`: reusable drivers or middleware that depend on abstract interfaces.
  - `User/App`: product behavior, task orchestration, state machines, and feature logic.
  - `User/Lib`: chip-independent algorithms, containers, filters, math, protocol helpers.
- Code in `User/Module`, `User/App`, and `User/Lib` should be portable enough that
  changing MCU/HAL only requires replacing the BSP adapter.
- Avoid calling STM32 HAL APIs directly from App or Lib code. Use a BSP interface or
  function-pointer operations table when practical.

## Design Style

- Write code like a senior embedded engineer: clear ownership, small interfaces,
  predictable state, and explicit error handling.
- Prefer encapsulation with structs that hold state and configuration.
- Prefer function pointers for hardware operations, callbacks, polymorphic behavior,
  and replaceable backends.
- Use C-style "inheritance" only where it improves reuse, for example a base ops
  structure embedded in a concrete driver object.
- Avoid global mutable state when a context struct would make the dependency clearer.
- Keep public APIs small and stable; put private helpers in `.c` files as `static`.

## Non-Blocking Rule

- Default to non-blocking code.
- Do not use `HAL_Delay()` or long busy-wait loops in libraries, modules, or app tasks.
- Use state machines, periodic task functions, interrupts, DMA, callbacks, or RTOS
  synchronization instead.
- If blocking is unavoidable, make it explicit in the function name or comment and keep
  it out of reusable libraries unless the user approves.

## Comments

- Add useful comments for public APIs, hardware assumptions, timing requirements,
  state machines, interrupt interactions, and non-obvious logic.
- Avoid noisy comments that repeat the code.
- Prefer ASCII comments in source files unless the file already uses a stable Chinese
  encoding and the toolchain displays it correctly.

## CubeMX

- Keep CubeMX generated files inside user-code regions when editing them manually.
- Prefer putting custom code under `User/` and adding it through CMake rather than
  modifying generated code directly.
- When enabling middleware such as FreeRTOS, configure it in `.ioc` and let CubeMX
  generate the middleware files.
