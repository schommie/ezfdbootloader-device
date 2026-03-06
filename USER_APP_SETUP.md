# Configuring an STM32 Application to Work with ezfd-bootloader

This guide explains the changes required to a vanilla STM32CubeIDE/CubeMX project (STM32H533) to run as a user application loaded by the ezfd-bootloader.

## Memory Layout

The bootloader occupies the first 32KB of flash. The user application starts at `0x08008000`:

```
0x08000000 ┌──────────────────────┐
           │  Bootloader (32KB)   │
0x08008000 ├──────────────────────┤
           │  User Application    │
           │       (480KB)        │
0x08080000 └──────────────────────┘
```

## 1. Linker Script (.ld file)

Change the FLASH origin and length to account for the bootloader region.

**Default (vanilla project):**
```
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 272K
  FLASH    (rx)    : ORIGIN = 0x08000000,  LENGTH = 512K
}
```

**Modified for bootloader:**
```
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 272K
  FLASH    (rx)    : ORIGIN = 0x08008000,  LENGTH = 480K
}
```

- `ORIGIN` moves from `0x08000000` to `0x08008000` (32KB offset)
- `LENGTH` reduces from `512K` to `480K` (512K minus 32K bootloader)

## 2. Vector Table Offset (VECT_TAB_OFFSET)

The application's `SystemInit()` sets `SCB->VTOR` on startup. By default it points to `0x08000000`, which is the bootloader — not the application's vector table. This must be updated.

**File:** `Core/Src/system_stm32h5xx.c`

**Default:**
```c
#define VECT_TAB_OFFSET  0x00U
```

**Modified for bootloader:**
```c
#define VECT_TAB_OFFSET  0x8000U
```

This ensures `SCB->VTOR = FLASH_BASE + 0x8000 = 0x08008000`, matching where the linker placed `.isr_vector`.

**Why this matters:** Without this change, the bootloader correctly sets VTOR to `0x08008000` before jumping, but then the app's own `SystemInit()` immediately overwrites it back to `0x08000000`. Any interrupt then vectors into bootloader memory instead of the application's handlers.

## Summary

| File | Change | Value |
|---|---|---|
| `STM32H533xx_FLASH.ld` | FLASH ORIGIN | `0x08000000` → `0x08008000` |
| `STM32H533xx_FLASH.ld` | FLASH LENGTH | `512K` → `480K` |
| `system_stm32h5xx.c` | VECT_TAB_OFFSET | `0x00U` → `0x8000U` |

No other changes are needed. The application's clock init, peripheral init, and startup code work as-is.
